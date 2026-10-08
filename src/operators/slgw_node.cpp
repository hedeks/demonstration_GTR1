#include "slgw_node.hpp"
#include <iostream>
#include <zmq.h>
#include <vector>
// Подключаем сгенерированные protobuf классы
#include "ipc_messages.pb.h"
#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace operators {

SlgwNode::SlgwNode(std::shared_ptr<core::MP10Space> mp10, std::shared_ptr<core::MP11Manager> mp11) 
    : mp10_(std::move(mp10)), mp11_(std::move(mp11)) {
    // Инициализация OctoMapManager
    octo_mgr_ = std::make_unique<core::OctoMapManager>(mp10_);
    
    // Инициализация ZeroMQ (REQ socket)
    zmq_ctx_ = zmq_ctx_new();
    zmq_req_socket_ = zmq_socket(zmq_ctx_, ZMQ_REQ);
    
    // NFR 2.4: Убираем таймаут, чтобы REQ/REP стейт-машина не ломалась (иначе EFSM ошибка)
    // int timeout = 200;
    // zmq_setsockopt(zmq_req_socket_, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    // zmq_setsockopt(zmq_req_socket_, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Подключаемся к Python ML сервису
    zmq_connect(zmq_req_socket_, "tcp://127.0.0.1:5555");
    std::cout << "[SLGW] ZMQ REQ socket initialized and connected to CNN1.\n";
}

SlgwNode::~SlgwNode() {
    zmq_close(zmq_req_socket_);
    zmq_ctx_destroy(zmq_ctx_);
}

void SlgwNode::process_frame(double timestamp, const cv::Mat& rgb, const cv::Mat& depth, const Eigen::Matrix4f& pose) {
    // 1. Шаг SLAM: Использование Ground Truth позы (Эмуляция идеального SLAM)
    Eigen::Matrix4f current_pose = pose; 
    
    // 2. Обновление позы субъекта в пространстве MP10
    mp10_->update_pose(current_pose);

    // 3. Передача облака точек в OctoMapManager для обновления сетки препятствий
    octo_mgr_->update_from_depth(depth, current_pose);
    
    // 4. Отправка RGB кадра в CNN1 через ZeroMQ (Drop Latest)
    send_to_cnn(timestamp, rgb);
}

void SlgwNode::send_to_cnn(double timestamp, const cv::Mat& rgb) {
    // Кодируем изображение в JPEG для передачи
    std::vector<uchar> img_buffer;
    cv::imencode(".jpg", rgb, img_buffer);

    // Формируем Protobuf сообщение
    gtr1::ipc::DetectionRequest req;
    req.mutable_meta()->set_timestamp(timestamp);
    req.mutable_meta()->set_frame_id(0); // TODO: frame counter
    req.set_rgb_image_data(img_buffer.data(), img_buffer.size());

    std::string serialized_req;
    req.SerializeToString(&serialized_req);

    // Отправляем запрос (REQ)
    zmq_msg_t msg;
    zmq_msg_init_size(&msg, serialized_req.size());
    memcpy(zmq_msg_data(&msg), serialized_req.c_str(), serialized_req.size());
    
    int bytes_sent = zmq_msg_send(&msg, zmq_req_socket_, 0); // Блокирующий send
    zmq_msg_close(&msg);

    if (bytes_sent < 0) {
        std::cerr << "WARN: CNN1 Send Failed!\n";
        return;
    }

    // Ожидаем ответ (с таймаутом 200мс)
    zmq_msg_t reply;
    zmq_msg_init(&reply);
    int bytes_received = zmq_msg_recv(&reply, zmq_req_socket_, 0);
    
    if (bytes_received > 0) {
        gtr1::ipc::DetectionResponse resp;
        resp.ParseFromArray(zmq_msg_data(&reply), bytes_received);
        
        // Передача инвариантов в MP11 Manager для слияния
        if (mp11_) {
            mp11_->process_detections(resp);
        }
    } else {
        // std::cerr << "WARN: CNN1 Timeout (200ms)\n";
    }
    zmq_msg_close(&reply);
}

} // namespace operators
} // namespace gtr1
