#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <zmq.hpp>

#include "core/include/mp10_space.hpp"
#include "core/include/mp11_manager.hpp"
#include "core/include/mtv1_motivator.hpp"
#include "operators/slgw_node.hpp"
#include "operators/prd1_predictor.hpp"

#include "../07_infrastructure/dataset_player.hpp"

class TelemetryPublisher {
private:
    zmq::context_t ctx_;
    zmq::socket_t pub_;
public:
    TelemetryPublisher() : ctx_(1), pub_(ctx_, zmq::socket_type::pub) {
        pub_.bind("tcp://*:5556");
    }
    void send_log(const std::string& source, const std::string& msg) {
        std::string json = "{\"type\":\"TRACE_LOG\",\"source\":\"" + source + "\",\"message\":\"" + msg + "\"}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
    void send_path(const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
        std::string json = "{\"type\":\"PATH\",\"points\":[{\"x\":" + std::to_string(start.x()) + ",\"y\":" + std::to_string(start.y()) + ",\"z\":" + std::to_string(start.z()) + "},{\"x\":" + std::to_string(end.x()) + ",\"y\":" + std::to_string(end.y()) + ",\"z\":" + std::to_string(end.z()) + "}]}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
    void send_voxel(float x, float y, float z) {
        std::string json = "{\"type\":\"VOXEL\",\"x\":" + std::to_string(x) + ",\"y\":" + std::to_string(y) + ",\"z\":" + std::to_string(z) + "}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
    void send_map(const std::vector<Eigen::Vector3f>& voxels) {
        std::string json = "{\"type\":\"MAP\",\"voxels\":[";
        for (size_t i = 0; i < voxels.size(); ++i) {
            json += "{\"x\":" + std::to_string(voxels[i].x()) + ",\"y\":" + std::to_string(voxels[i].y()) + ",\"z\":" + std::to_string(voxels[i].z()) + "}";
            if (i < voxels.size() - 1) json += ",";
        }
        json += "]}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
    void send_pose(const Eigen::Matrix4f& pose) {
        Eigen::Vector3f pos = pose.block<3,1>(0,3);
        Eigen::Quaternionf q(pose.block<3,3>(0,0));
        std::string json = "{\"type\":\"POSE\",\"x\":" + std::to_string(pos.x()) + ",\"y\":" + std::to_string(pos.y()) + ",\"z\":" + std::to_string(pos.z()) + 
            ",\"qx\":" + std::to_string(q.x()) + ",\"qy\":" + std::to_string(q.y()) + ",\"qz\":" + std::to_string(q.z()) + ",\"qw\":" + std::to_string(q.w()) + "}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
};

int main() {
    std::cout << "[SYSTEM] Initializing GTR1 MVP Core...\n";

    // 1. Создание структур памяти
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("base_map_1");
    auto mp11 = std::make_shared<gtr1::core::MP11Manager>();
    
    std::cout << "[SYSTEM] MP10 and MP11 Memory initialized.\n";

    // 2. Инициализация операторов
    auto mtv1 = std::make_shared<gtr1::core::Mtv1Motivator>();
    auto slgw = std::make_shared<gtr1::operators::SlgwNode>(mp10, mp11);
    auto prd1 = std::make_shared<gtr1::operators::Prd1Predictor>(mp10);

    std::cout << "[SYSTEM] Operators (MTV1, SLGW, PRD1) initialized.\n";

    // 3. Инжект начальной цели
    mtv1->inject_goal("cup", 0.9f);
    std::cout << "[SYSTEM] MTV1 Goal injected: FIND_CUP\n";

    try {
        std::cout << "[DEBUG] Try block entered\n";
        // Инициализация ZMQ Telemetry
        TelemetryPublisher telemetry;
        std::cout << "[DEBUG] Telemetry initialized\n";
        telemetry.send_log("SYS", "GTR1 Core Booting Up...");

        // 4. Инициализация и запуск чтения датасета
        std::string dataset_path = "07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
        std::cout << "[DEBUG] Creating player...\n";
        gtr1::infra::DatasetPlayer player(dataset_path, slgw, 30.0);
        std::cout << "[DEBUG] Loading associations...\n";
        
        if (player.load_associations("associations_gt.txt")) {
            std::cout << "[DEBUG] Associations loaded!\n";
            std::cout << "[SYSTEM] Dataset loaded. Starting ingestion pipeline...\n";
            telemetry.send_log("SYS", "Dataset loaded. Pipeline starting...");
        
        // Обратный ход (RTR) вызывается каждый кадр
        int frame_counter = 0;
        auto on_frame = [&]() {
            frame_counter++;
            
            if (frame_counter % 5 == 0) {
                telemetry.send_pose(mp10->current_pose);
            }
            
            // Отправляем карту раз в 30 кадров (~ 1 раз в секунду)
            if (frame_counter % 30 == 0) {
                telemetry.send_map(mp10->get_occupied_voxels());
            }

            // Ищем цель в памяти MP11
            const auto& invariants = mp11->get_active_invariants();
            for (const auto& inv : invariants) {
                if (inv.class_label_id() == mtv1->get_target_class()) {
                    telemetry.send_log("MP11", "Target '" + inv.class_label_id() + "' is actively tracked in memory.");
                    
                    // Эмуляция робота в (0,0,0)
                    Eigen::Vector3f start_pos(0,0,0); 
                    Eigen::Vector3f target_pos(inv.bounding_box().center().x(), 
                                               inv.bounding_box().center().y(), 
                                               inv.bounding_box().center().z());
                                               
                    std::vector<std::shared_ptr<gtr1::models::MP12Oprn>> splices; // Пустые для теста A*
                    
                    telemetry.send_log("PRD1", "Planning path to target...");
                    auto blom = prd1->build_event_chain(inv.base().uuid(), start_pos, target_pos, splices);
                    
                    if (blom->is_complete()) {
                        telemetry.send_log("PRD1", "A* Path Generated with " + std::to_string(blom->oprn_sequence_size()) + " waypoints.");
                        telemetry.send_path(start_pos, target_pos);
                        telemetry.send_voxel(target_pos.x(), target_pos.y(), target_pos.z());
                    } else {
                        telemetry.send_log("PRD1", "A* Failed to find path.");
                    }
                    break; // Нашли цель, дальше не ищем
                }
            }
        };

        player.play(on_frame);
    } else {
        std::cout << "[SYSTEM] WARNING: TUM dataset not found at " << dataset_path << "\n";
        std::cout << "[SYSTEM] To run the full simulation, please mount the dataset.\n";
    }
    } catch (const std::exception& e) {
        std::cerr << "\n[CRASH FATAL] Exception: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "\n[CRASH FATAL] Unknown exception!\n";
    }

    std::cout << "[SYSTEM] GTR1 MVP Core shutdown.\n";
    return 0;
}
