#include "slgw_node.hpp"
#include <iostream>
#include <zmq.h>
#include <vector>
#include <cstring>
#include "ipc_messages.pb.h"
#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace operators {

SlgwNode::SlgwNode(std::shared_ptr<core::MP10Space> mp10,
                   std::shared_ptr<core::MP11Manager> mp11,
                   std::string zmq_endpoint,
                   bool enable_ml_worker)
    : mp10_(std::move(mp10)),
      mp11_(std::move(mp11)),
      enable_kalman_(true),
      enable_depth_filter_(true),
      zmq_endpoint_(std::move(zmq_endpoint)),
      is_running_(true),
      enable_ml_worker_(enable_ml_worker),
      frames_processed_(0),
      frames_sent_to_ml_(0),
      frames_dropped_by_ml_(0) {

    // Инициализация менеджера воксельной сетки MP10
    octo_mgr_ = std::make_unique<core::OctoMapManager>(mp10_);

    // Инициализация сенсорных фильтров (Калман для SE3 + Bilateral Depth)
    kalman_filter_ = std::make_unique<core::KalmanFilterSE3>(0.02f, 0.5f);
    kalman_filter_->set_orientation_gain(1.0f); // Прямое отслеживание ориентации без углового запаздывания
    depth_filter_ = std::make_unique<core::DepthFilter>(5, 0.04, 4.0);

    // Инициализация ZeroMQ
    zmq_ctx_ = zmq_ctx_new();
    init_zmq_socket();

    // Запуск фонового потока ML-коммуникации ("Drop Latest")
    if (enable_ml_worker_) {
        ml_worker_thread_ = std::thread(&SlgwNode::ml_worker_loop, this);
    }
}

SlgwNode::~SlgwNode() {
    is_running_.store(false);
    buffer_cv_.notify_all();

    if (ml_worker_thread_.joinable()) {
        ml_worker_thread_.join();
    }

    close_zmq_socket();
    if (zmq_ctx_) {
        zmq_ctx_destroy(zmq_ctx_);
        zmq_ctx_ = nullptr;
    }
}

void SlgwNode::init_zmq_socket() {
    std::lock_guard<std::recursive_mutex> lock(zmq_mtx_);
    if (!zmq_ctx_) return;

    zmq_req_socket_ = zmq_socket(zmq_ctx_, ZMQ_REQ);
    int timeout = 200; // 200 мс таймаут
    int linger = 0;
    zmq_setsockopt(zmq_req_socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(zmq_req_socket_, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
    zmq_setsockopt(zmq_req_socket_, ZMQ_LINGER, &linger, sizeof(linger));

    zmq_connect(zmq_req_socket_, zmq_endpoint_.c_str());
}

void SlgwNode::close_zmq_socket() {
    std::lock_guard<std::recursive_mutex> lock(zmq_mtx_);
    if (zmq_req_socket_) {
        zmq_close(zmq_req_socket_);
        zmq_req_socket_ = nullptr;
    }
}

void SlgwNode::reset_zmq_socket() {
    std::lock_guard<std::recursive_mutex> lock(zmq_mtx_);
    close_zmq_socket();
    init_zmq_socket();
}

void SlgwNode::process_frame(double timestamp,
                             const cv::Mat& rgb,
                             const cv::Mat& depth,
                             const Eigen::Matrix4f& pose) {
    // 1. Фильтрация позы агента через Фильтр Калмана (SE3)
    Eigen::Matrix4f filtered_pose = pose;
    if (enable_kalman_ && kalman_filter_) {
        filtered_pose = kalman_filter_->update(timestamp, pose);
    }

    // Обновление сглаженной позы в MP10
    if (mp10_) {
        mp10_->update_pose(filtered_pose);
    }

    // 2. Двусторонняя фильтрация карты глубины (Bilateral Depth Filter + Flying Pixels)
    cv::Mat processed_depth = depth;
    if (enable_depth_filter_ && depth_filter_ && !depth.empty()) {
        processed_depth = depth_filter_->filter(depth);
    }

    // 3. Вокселизация очищенного облака глубины в OctoMap (< 33 мс)
    if (octo_mgr_) {
        octo_mgr_->update_from_depth(processed_depth, filtered_pose);
    }

    // 4. Неблокирующая передача кадра в буфер "Drop Latest" (емкость 1)
    if (enable_ml_worker_ && !rgb.empty()) {
        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);
            if (pending_frame_.has_value()) {
                frames_dropped_by_ml_++;
            }
            pending_frame_ = FrameBufferItem{timestamp, rgb.clone(), processed_depth.clone(), filtered_pose};
        }
        buffer_cv_.notify_one();
    }

    frames_processed_++;
}

bool SlgwNode::send_to_cnn_sync(double timestamp, const cv::Mat& rgb, const cv::Mat& depth, const Eigen::Matrix4f& pose) {
    if (rgb.empty()) return false;

    std::vector<uchar> img_buffer;
    cv::imencode(".jpg", rgb, img_buffer);

    gtr1::ipc::DetectionRequest req;
    req.mutable_meta()->set_timestamp(timestamp);
    req.mutable_meta()->set_frame_id(frames_processed_.load());
    req.set_rgb_image_data(img_buffer.data(), img_buffer.size());

    if (!depth.empty()) {
        std::vector<uchar> depth_buffer;
        cv::imencode(".png", depth, depth_buffer);
        req.set_depth_image_data(depth_buffer.data(), depth_buffer.size());
    }

    std::string serialized_req;
    req.SerializeToString(&serialized_req);

    std::lock_guard<std::recursive_mutex> lock(zmq_mtx_);
    if (!zmq_req_socket_) return false;

    zmq_msg_t msg;
    zmq_msg_init_size(&msg, serialized_req.size());
    memcpy(zmq_msg_data(&msg), serialized_req.c_str(), serialized_req.size());

    int bytes_sent = zmq_msg_send(&msg, zmq_req_socket_, 0);
    zmq_msg_close(&msg);

    if (bytes_sent < 0) {
        reset_zmq_socket();
        return false;
    }

    zmq_msg_t reply;
    zmq_msg_init(&reply);
    int bytes_received = zmq_msg_recv(&reply, zmq_req_socket_, 0);

    bool success = false;
    if (bytes_received > 0) {
        gtr1::ipc::DetectionResponse resp;
        if (resp.ParseFromArray(zmq_msg_data(&reply), bytes_received)) {
            // Transform bounding boxes and spatial anchors from Camera Optical frame to World frame SE(3)
            Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
            Eigen::Vector3f t = pose.block<3, 1>(0, 3);
            for (int i = 0; i < resp.detected_objects_size(); ++i) {
                auto* proto_obj = resp.mutable_detected_objects(i);
                Eigen::Vector3f p_cam(
                    proto_obj->bounding_box().center().x(),
                    proto_obj->bounding_box().center().y(),
                    proto_obj->bounding_box().center().z()
                );
                Eigen::Vector3f p_world = R * p_cam + t;

                proto_obj->mutable_bounding_box()->mutable_center()->set_x(p_world.x());
                proto_obj->mutable_bounding_box()->mutable_center()->set_y(p_world.y());
                proto_obj->mutable_bounding_box()->mutable_center()->set_z(p_world.z());

                proto_obj->mutable_spatial_anchor()->set_x(p_world.x());
                proto_obj->mutable_spatial_anchor()->set_y(p_world.y());
                proto_obj->mutable_spatial_anchor()->set_z(p_world.z());
            }

            if (mp11_) {
                mp11_->process_detections(resp);
            }
            success = true;
        }
    } else {
        // Таймаут или ошибка: Lazy Pirate сброс сокета
        reset_zmq_socket();
    }

    zmq_msg_close(&reply);
    return success;
}

void SlgwNode::ml_worker_loop() {
    while (is_running_.load()) {
        FrameBufferItem item;
        {
            std::unique_lock<std::mutex> lock(buffer_mtx_);
            buffer_cv_.wait(lock, [this]() {
                return !is_running_.load() || pending_frame_.has_value();
            });

            if (!is_running_.load()) break;

            item = std::move(*pending_frame_);
            pending_frame_.reset();
        }

        frames_sent_to_ml_++;
        send_to_cnn_sync(item.timestamp, item.rgb, item.depth, item.pose);
    }
}

} // namespace operators
} // namespace gtr1
