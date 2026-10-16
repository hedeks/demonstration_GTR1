#include "sim_stream_receiver.hpp"
#include "ipc_messages.pb.h"
#include <zmq.h>
#include <iostream>

namespace gtr1 {
namespace infra {

SimStreamReceiver::SimStreamReceiver(const std::string& sub_endpoint,
                                     const std::string& pub_endpoint)
    : sub_endpoint_(sub_endpoint), pub_endpoint_(pub_endpoint) {
    zmq_ctx_ = zmq_ctx_new();
    if (!zmq_ctx_) {
        std::cerr << "[SimReceiver ERROR] Failed to create ZMQ context.\n";
        return;
    }

    // 1. Настройка SUB-сокета (прием сенсорного потока от PyBullet)
    zmq_sub_ = zmq_socket(zmq_ctx_, ZMQ_SUB);
    int rcvtimeo = 100; // 100 ms таймаут
    zmq_setsockopt(zmq_sub_, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));
    zmq_setsockopt(zmq_sub_, ZMQ_SUBSCRIBE, "", 0);

    int rc = zmq_connect(zmq_sub_, sub_endpoint_.c_str());
    if (rc != 0) {
        std::cerr << "[SimReceiver ERROR] Failed to connect SUB to " << sub_endpoint_ << "\n";
        return;
    }

    // 2. Настройка PUB-сокета (отправка команд управления в PyBullet)
    zmq_pub_ = zmq_socket(zmq_ctx_, ZMQ_PUB);
    rc = zmq_bind(zmq_pub_, pub_endpoint_.c_str());
    if (rc != 0) {
        std::cerr << "[SimReceiver WARN] Could not bind PUB to " << pub_endpoint_ << " (errno: " << errno << ")\n";
    }

    is_connected_ = true;
    std::cout << "[SimReceiver] Connected to PyBullet sensor stream: " << sub_endpoint_
              << " | Command pub: " << pub_endpoint_ << "\n";
}

SimStreamReceiver::~SimStreamReceiver() {
    is_connected_ = false;
    if (zmq_sub_) {
        zmq_close(zmq_sub_);
        zmq_sub_ = nullptr;
    }
    if (zmq_pub_) {
        zmq_close(zmq_pub_);
        zmq_pub_ = nullptr;
    }
    if (zmq_ctx_) {
        zmq_ctx_term(zmq_ctx_);
        zmq_ctx_ = nullptr;
    }
}

bool SimStreamReceiver::poll_frame(cv::Mat& out_rgb,
                                   cv::Mat& out_depth,
                                   Eigen::Matrix4f& out_pose,
                                   double& out_timestamp,
                                   std::string& out_trace_id,
                                   bool& out_is_clean,
                                   int timeout_ms) {
    uint64_t dummy_frame_id = 0;
    return poll_frame(out_rgb, out_depth, out_pose, out_timestamp, out_trace_id, out_is_clean, dummy_frame_id, timeout_ms);
}

bool SimStreamReceiver::poll_frame(cv::Mat& out_rgb,
                                   cv::Mat& out_depth,
                                   Eigen::Matrix4f& out_pose,
                                   double& out_timestamp,
                                   std::string& out_trace_id,
                                   bool& out_is_clean,
                                   uint64_t& out_frame_id,
                                   int timeout_ms) {
    if (!zmq_sub_) return false;

    int rcvtimeo = timeout_ms;
    zmq_setsockopt(zmq_sub_, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

    zmq_msg_t msg;
    zmq_msg_init(&msg);

    int bytes = zmq_msg_recv(&msg, zmq_sub_, 0);
    if (bytes < 0) {
        zmq_msg_close(&msg);
        return false;
    }

    gtr1::ipc::SensorFrame frame_proto;
    bool parsed = frame_proto.ParseFromArray(zmq_msg_data(&msg), bytes);
    zmq_msg_close(&msg);

    if (!parsed) {
        std::cerr << "[SimReceiver WARN] Failed to parse SensorFrame protobuf.\n";
        return false;
    }

    out_timestamp = frame_proto.meta().timestamp();
    out_trace_id = frame_proto.meta().trace_id();
    out_is_clean = frame_proto.is_clean_mode();
    out_frame_id = frame_proto.meta().frame_id();

    // 1. Декодирование RGB (JPEG -> cv::Mat BGR)
    std::vector<uchar> rgb_buf(frame_proto.rgb_image_data().begin(), frame_proto.rgb_image_data().end());
    out_rgb = cv::imdecode(rgb_buf, cv::IMREAD_COLOR);

    // 2. Декодирование Depth (PNG -> 16-бит cv::Mat CV_16UC1)
    std::vector<uchar> depth_buf(frame_proto.depth_image_data().begin(), frame_proto.depth_image_data().end());
    out_depth = cv::imdecode(depth_buf, cv::IMREAD_UNCHANGED);

    // 3. Восстановление 4x4 матрицы позы SE(3)
    out_pose = Eigen::Matrix4f::Identity();
    if (frame_proto.camera_pose().data_size() == 16) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                out_pose(i, j) = frame_proto.camera_pose().data(i * 4 + j);
            }
        }
    }

    return (!out_rgb.empty() && !out_depth.empty());
}

void SimStreamReceiver::send_command(float vx, float vy, float wz, const std::string& trace_id, uint64_t frame_id) {
    if (!zmq_pub_) return;

    gtr1::ipc::KinematicCommand cmd;
    cmd.mutable_meta()->set_trace_id(trace_id);
    cmd.mutable_meta()->set_frame_id(frame_id);
    cmd.mutable_velocity()->set_x(vx);
    cmd.mutable_velocity()->set_y(vy);
    cmd.mutable_velocity()->set_z(0.0f);
    cmd.mutable_angular_velocity()->set_z(wz);

    std::string payload;
    cmd.SerializeToString(&payload);

    zmq_send(zmq_pub_, payload.data(), payload.size(), ZMQ_DONTWAIT);
}

} // namespace infra
} // namespace gtr1
