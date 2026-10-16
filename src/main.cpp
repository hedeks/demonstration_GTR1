#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <csignal>
#include <atomic>
#include <cmath>
#include <zmq.hpp>

#include "core/include/mp10_space.hpp"
#include "core/include/mp11_manager.hpp"
#include "core/include/mtv1_motivator.hpp"
#include "core/include/trl1_log.hpp"
#include "core/include/trace_logger.hpp"
#include "operators/slgw_node.hpp"
#include "operators/spl1_splicer.hpp"
#include "operators/prd1_predictor.hpp"

#include "../07_infrastructure/dataset_player.hpp"
#include "../07_infrastructure/sim_stream_receiver.hpp"

namespace {
std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int) {
    g_shutdown_requested.store(true);
}

float normalize_angle(float a) {
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}
} // namespace

class TelemetryPublisher {
private:
    zmq::context_t ctx_;
    zmq::socket_t pub_;
    std::string mode_{"SIM"};
public:
    explicit TelemetryPublisher(std::string mode = "SIM")
        : ctx_(1), pub_(ctx_, zmq::socket_type::pub), mode_(std::move(mode)) {
        pub_.bind("tcp://*:5556");
    }
    void set_mode(const std::string& mode) { mode_ = mode; }

    void send_log(const std::string& source, const std::string& msg) {
        std::string json = "{\"type\":\"TRACE_LOG\",\"source\":\"" + source + "\",\"message\":\"" + msg + "\"}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    void send_path(const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
        std::string json = "{\"type\":\"PATH\",\"mode\":\"" + mode_ + "\",\"points\":[{\"x\":" + std::to_string(start.x()) + ",\"y\":" + std::to_string(start.y()) + ",\"z\":" + std::to_string(start.z()) + "},{\"x\":" + std::to_string(end.x()) + ",\"y\":" + std::to_string(end.y()) + ",\"z\":" + std::to_string(end.z()) + "}]}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    void send_voxel(float x, float y, float z) {
        std::string json = "{\"type\":\"VOXEL\",\"mode\":\"" + mode_ + "\",\"x\":" + std::to_string(x) + ",\"y\":" + std::to_string(y) + ",\"z\":" + std::to_string(z) + "}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    void send_map(const std::vector<Eigen::Vector3f>& voxels) {
        if (voxels.empty()) return;
        std::string json;
        json.reserve(voxels.size() * 26 + 64);
        json = "{\"type\":\"MAP\",\"mode\":\"" + mode_ + "\",\"voxels\":[";
        char buf[64];
        for (size_t i = 0; i < voxels.size(); ++i) {
            int len = snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                               voxels[i].x(), voxels[i].y(), voxels[i].z());
            json.append(buf, len);
            if (i + 1 < voxels.size()) json.push_back(',');
        }
        json += "]}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    void send_pose(const Eigen::Matrix4f& pose) {
        Eigen::Vector3f pos = pose.block<3,1>(0,3);
        Eigen::Quaternionf q(pose.block<3,3>(0,0));
        float yaw = (mode_ == "SIM") ? std::atan2(pose(1,2), pose(0,2)) : 0.0f;
        float ground_z = (mode_ == "SIM") ? 0.0f : pos.z();
        std::string json = "{\"type\":\"POSE\",\"mode\":\"" + mode_ + 
            "\",\"x\":" + std::to_string(pos.x()) + 
            ",\"y\":" + std::to_string(pos.y()) + 
            ",\"z\":" + std::to_string(ground_z) + 
            ",\"cam_z\":" + std::to_string(pos.z()) + 
            ",\"yaw\":" + std::to_string(yaw) + 
            ",\"qx\":" + std::to_string(q.x()) + 
            ",\"qy\":" + std::to_string(q.y()) + 
            ",\"qz\":" + std::to_string(q.z()) + 
            ",\"qw\":" + std::to_string(q.w()) + "}";
        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    void send_rtr_debug(
        const std::vector<gtr1::core::MP11Object>& invariants,
        const std::string& target_class,
        const Eigen::Vector3f& agent_pos,
        std::shared_ptr<gtr1::core::MP13Blom> active_blom,
        const std::vector<gtr1::core::MP13Blom>& cached_bloms,
        bool goal_reached) {
        
        std::string json = "{\"type\":\"RTR_DEBUG\",\"mode\":\"" + mode_ + "\",";
        
        // active_goal
        json += "\"active_goal\":{";
        json += "\"target_class\":\"" + target_class + "\",";
        json += "\"goal_reached\":" + std::string(goal_reached ? "true" : "false") + ",";
        float active_dist = -1.0f;
        bool found_target = false;
        for (const auto& inv : invariants) {
            if (inv.class_label_id == target_class) {
                active_dist = (inv.bounding_box.center - agent_pos).norm();
                json += "\"pos\":{\"x\":" + std::to_string(inv.bounding_box.center.x()) +
                        ",\"y\":" + std::to_string(inv.bounding_box.center.y()) +
                        ",\"z\":" + std::to_string(inv.bounding_box.center.z()) + "},";
                json += "\"dims\":{\"dx\":" + std::to_string(inv.bounding_box.size.x()) +
                        ",\"dy\":" + std::to_string(inv.bounding_box.size.y()) +
                        ",\"dz\":" + std::to_string(inv.bounding_box.size.z()) + "},";
                found_target = true;
                break;
            }
        }
        if (!found_target) {
            json += "\"pos\":null,\"dims\":null,";
        }
        json += "\"distance\":" + std::to_string(active_dist) + "},";

        // goals_tree
        json += "\"goals_tree\":[";
        for (size_t i = 0; i < invariants.size(); ++i) {
            const auto& inv = invariants[i];
            float d = (inv.bounding_box.center - agent_pos).norm();
            bool is_target = (!target_class.empty() && inv.class_label_id == target_class);
            json += "{\"uuid\":\"" + inv.uuid() + "\",";
            json += "\"class_label\":\"" + inv.class_label_id + "\",";
            json += "\"is_target\":" + std::string(is_target ? "true" : "false") + ",";
            json += "\"distance\":" + std::to_string(d) + ",";
            json += "\"pos\":{\"x\":" + std::to_string(inv.bounding_box.center.x()) +
                    ",\"y\":" + std::to_string(inv.bounding_box.center.y()) +
                    ",\"z\":" + std::to_string(inv.bounding_box.center.z()) + "},";
            json += "\"dims\":{\"dx\":" + std::to_string(inv.bounding_box.size.x()) +
                    ",\"dy\":" + std::to_string(inv.bounding_box.size.y()) +
                    ",\"dz\":" + std::to_string(inv.bounding_box.size.z()) + "}}";
            if (i + 1 < invariants.size()) json += ",";
        }
        json += "],";

        // active_blom
        json += "\"active_blom\":";
        if (active_blom) {
            json += "{\"uuid\":\"" + active_blom->uuid() + "\",";
            json += "\"is_complete\":" + std::string(active_blom->is_complete ? "true" : "false") + ",";
            json += "\"outcome_marker\":" + std::to_string(active_blom->outcome_marker) + ",";
            json += "\"oprn_count\":" + std::to_string(active_blom->oprn_sequence.size()) + ",";
            json += "\"oprn_nodes\":[";
            Eigen::Vector3f step_start = agent_pos;
            if (!active_blom->oprn_items.empty()) {
                for (size_t j = 0; j < active_blom->oprn_items.size(); ++j) {
                    const auto& o = active_blom->oprn_items[j];
                    if (!o) continue;
                    Eigen::Vector3f step_end = step_start + o->b_vector;
                    float b_len = o->b_vector.norm();
                    std::string st_str = "PENDING";
                    if (o->status == gtr1::core::OprnStatus::VALIDATED) st_str = "VALIDATED";
                    else if (o->status == gtr1::core::OprnStatus::COLLISION) st_str = "COLLISION";
                    else if (o->status == gtr1::core::OprnStatus::EXECUTED) st_str = "EXECUTED";
                    
                    json += "{\"index\":" + std::to_string(j + 1) + ",";
                    json += "\"uuid\":\"" + o->uuid() + "\",";
                    json += "\"subject_ref\":\"" + o->subject_ref + "\",";
                    json += "\"target_ref\":\"" + o->target_ref + "\",";
                    json += "\"status\":\"" + st_str + "\",";
                    json += "\"b_length\":" + std::to_string(b_len) + ",";
                    json += "\"start\":{\"x\":" + std::to_string(step_start.x()) + ",\"y\":" + std::to_string(step_start.y()) + ",\"z\":" + std::to_string(step_start.z()) + "},";
                    json += "\"end\":{\"x\":" + std::to_string(step_end.x()) + ",\"y\":" + std::to_string(step_end.y()) + ",\"z\":" + std::to_string(step_end.z()) + "},";
                    json += "\"b_vector\":{\"x\":" + std::to_string(o->b_vector.x()) +
                            ",\"y\":" + std::to_string(o->b_vector.y()) +
                            ",\"z\":" + std::to_string(o->b_vector.z()) + "}}";
                    if (j + 1 < active_blom->oprn_items.size()) json += ",";
                    step_start = step_end;
                }
            } else {
                for (size_t j = 0; j < active_blom->oprn_sequence.size(); ++j) {
                    json += "{\"index\":" + std::to_string(j + 1) + ",";
                    json += "\"uuid\":\"" + active_blom->oprn_sequence[j] + "\",";
                    json += "\"subject_ref\":\"agent_base\",";
                    json += "\"target_ref\":\"" + target_class + "\",";
                    json += "\"status\":\"VALIDATED\",";
                    json += "\"b_length\":0.0,";
                    json += "\"b_vector\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}";
                    if (j + 1 < active_blom->oprn_sequence.size()) json += ",";
                }
            }
            json += "]}";
        } else {
            json += "null";
        }
        json += ",";

        // cached_bloms
        json += "\"cached_bloms\":[";
        for (size_t k = 0; k < cached_bloms.size(); ++k) {
            const auto& cb = cached_bloms[k];
            json += "{\"uuid\":\"" + cb.uuid() + "\",";
            json += "\"is_complete\":" + std::string(cb.is_complete ? "true" : "false") + ",";
            json += "\"outcome_marker\":" + std::to_string(cb.outcome_marker) + ",";
            json += "\"oprn_count\":" + std::to_string(cb.oprn_sequence.size()) + "}";
            if (k + 1 < cached_bloms.size()) json += ",";
        }
        json += "]}";

        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }

    /**
     * @brief Единый атомарный пакет тика (Unified Tick Bundle).
     * Синхронизирует кадр, позу, карту вокселей, путь и статус целеполагания за один такт.
     */
    void send_tick_bundle(
        uint64_t tick_id,
        double timestamp,
        const Eigen::Matrix4f& pose,
        const std::vector<Eigen::Vector3f>& voxels,
        const Eigen::Vector3f& path_start,
        const Eigen::Vector3f& path_end,
        bool has_path,
        const std::vector<gtr1::core::MP11Object>& invariants,
        const std::string& target_class,
        const Eigen::Vector3f& agent_pos,
        std::shared_ptr<gtr1::core::MP13Blom> active_blom,
        const std::vector<gtr1::core::MP13Blom>& cached_bloms,
        bool goal_reached) {
        
        Eigen::Vector3f pos = pose.block<3,1>(0,3);
        Eigen::Quaternionf q(pose.block<3,3>(0,0));
        float yaw = (mode_ == "SIM") ? std::atan2(pose(1,2), pose(0,2)) : 0.0f;
        float ground_z = (mode_ == "SIM") ? 0.0f : pos.z();

        // Синхронизация: поза и путь шлются каждый тик (60 FPS), воксели карты — каждые 5 тиков
        bool has_map_update = (tick_id % 5 == 0) && !voxels.empty();

        std::string json;
        if (has_map_update) {
            json.reserve(voxels.size() * 26 + 2048);
        } else {
            json.reserve(2048);
        }

        json = "{\"type\":\"TICK_BUNDLE\",\"mode\":\"" + mode_ + "\",";
        json += "\"tick_id\":" + std::to_string(tick_id) + ",";
        json += "\"timestamp\":" + std::to_string(timestamp) + ",";
        json += "\"has_map_update\":" + std::string(has_map_update ? "true" : "false") + ",";

        // 1. Pose (обновляется строго синхронно на каждом тике)
        json += "\"pose\":{";
        json += "\"x\":" + std::to_string(pos.x()) + ",";
        json += "\"y\":" + std::to_string(pos.y()) + ",";
        json += "\"z\":" + std::to_string(ground_z) + ",";
        json += "\"cam_z\":" + std::to_string(pos.z()) + ",";
        json += "\"yaw\":" + std::to_string(yaw) + ",";
        json += "\"qx\":" + std::to_string(q.x()) + ",";
        json += "\"qy\":" + std::to_string(q.y()) + ",";
        json += "\"qz\":" + std::to_string(q.z()) + ",";
        json += "\"qw\":" + std::to_string(q.w()) + "},";

        // 2. Voxels (передаются только при has_map_update == true)
        char buf[64];
        json += "\"voxels\":[";
        if (has_map_update) {
            for (size_t i = 0; i < voxels.size(); ++i) {
                int len = snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                                   voxels[i].x(), voxels[i].y(), voxels[i].z());
                json.append(buf, len);
                if (i + 1 < voxels.size()) json.push_back(',');
            }
        }
        json += "],";

        // 3. Path
        json += "\"path\":[";
        if (has_path) {
            int len1 = snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},",
                                path_start.x(), path_start.y(), path_start.z());
            json.append(buf, len1);
            int len2 = snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                                path_end.x(), path_end.y(), path_end.z());
            json.append(buf, len2);
        }
        json += "],";

        // 4. Active Goal
        json += "\"active_goal\":{";
        json += "\"target_class\":\"" + target_class + "\",";
        json += "\"goal_reached\":" + std::string(goal_reached ? "true" : "false") + ",";
        float active_dist = -1.0f;
        bool found_target = false;
        for (const auto& inv : invariants) {
            if (inv.class_label_id == target_class) {
                active_dist = (inv.bounding_box.center - agent_pos).norm();
                json += "\"pos\":{\"x\":" + std::to_string(inv.bounding_box.center.x()) +
                        ",\"y\":" + std::to_string(inv.bounding_box.center.y()) +
                        ",\"z\":" + std::to_string(inv.bounding_box.center.z()) + "},";
                json += "\"dims\":{\"dx\":" + std::to_string(inv.bounding_box.size.x()) +
                        ",\"dy\":" + std::to_string(inv.bounding_box.size.y()) +
                        ",\"dz\":" + std::to_string(inv.bounding_box.size.z()) + "},";
                found_target = true;
                break;
            }
        }
        if (!found_target) {
            json += "\"pos\":null,\"dims\":null,";
        }
        json += "\"distance\":" + std::to_string(active_dist) + "},";

        // 5. Goals Tree
        json += "\"goals_tree\":[";
        for (size_t i = 0; i < invariants.size(); ++i) {
            const auto& inv = invariants[i];
            float d = (inv.bounding_box.center - agent_pos).norm();
            bool is_target = (!target_class.empty() && inv.class_label_id == target_class);
            json += "{\"uuid\":\"" + inv.uuid() + "\",";
            json += "\"class_label\":\"" + inv.class_label_id + "\",";
            json += "\"is_target\":" + std::string(is_target ? "true" : "false") + ",";
            json += "\"distance\":" + std::to_string(d) + ",";
            json += "\"pos\":{\"x\":" + std::to_string(inv.bounding_box.center.x()) +
                    ",\"y\":" + std::to_string(inv.bounding_box.center.y()) +
                    ",\"z\":" + std::to_string(inv.bounding_box.center.z()) + "},";
            json += "\"dims\":{\"dx\":" + std::to_string(inv.bounding_box.size.x()) +
                    ",\"dy\":" + std::to_string(inv.bounding_box.size.y()) +
                    ",\"dz\":" + std::to_string(inv.bounding_box.size.z()) + "}}";
            if (i + 1 < invariants.size()) json += ",";
        }
        json += "],";

        // 6. Active BLOM
        json += "\"active_blom\":";
        if (active_blom) {
            json += "{\"uuid\":\"" + active_blom->uuid() + "\",";
            json += "\"is_complete\":" + std::string(active_blom->is_complete ? "true" : "false") + ",";
            json += "\"outcome_marker\":" + std::to_string(active_blom->outcome_marker) + ",";
            json += "\"oprn_count\":" + std::to_string(active_blom->oprn_sequence.size()) + ",";
            json += "\"oprn_nodes\":[";
            Eigen::Vector3f step_start = agent_pos;
            if (!active_blom->oprn_items.empty()) {
                for (size_t j = 0; j < active_blom->oprn_items.size(); ++j) {
                    const auto& o = active_blom->oprn_items[j];
                    if (!o) continue;
                    Eigen::Vector3f step_end = step_start + o->b_vector;
                    float b_len = o->b_vector.norm();
                    std::string st_str = "PENDING";
                    if (o->status == gtr1::core::OprnStatus::VALIDATED) st_str = "VALIDATED";
                    else if (o->status == gtr1::core::OprnStatus::COLLISION) st_str = "COLLISION";
                    else if (o->status == gtr1::core::OprnStatus::EXECUTED) st_str = "EXECUTED";

                    json += "{\"index\":" + std::to_string(j + 1) + ",";
                    json += "\"uuid\":\"" + o->uuid() + "\",";
                    json += "\"subject_ref\":\"" + o->subject_ref + "\",";
                    json += "\"target_ref\":\"" + o->target_ref + "\",";
                    json += "\"status\":\"" + st_str + "\",";
                    json += "\"b_length\":" + std::to_string(b_len) + ",";
                    json += "\"start\":{\"x\":" + std::to_string(step_start.x()) + ",\"y\":" + std::to_string(step_start.y()) + ",\"z\":" + std::to_string(step_start.z()) + "},";
                    json += "\"end\":{\"x\":" + std::to_string(step_end.x()) + ",\"y\":" + std::to_string(step_end.y()) + ",\"z\":" + std::to_string(step_end.z()) + "},";
                    json += "\"b_vector\":{\"x\":" + std::to_string(o->b_vector.x()) +
                            ",\"y\":" + std::to_string(o->b_vector.y()) +
                            ",\"z\":" + std::to_string(o->b_vector.z()) + "}}";
                    if (j + 1 < active_blom->oprn_items.size()) json += ",";
                    step_start = step_end;
                }
            } else {
                for (size_t j = 0; j < active_blom->oprn_sequence.size(); ++j) {
                    json += "{\"index\":" + std::to_string(j + 1) + ",";
                    json += "\"uuid\":\"" + active_blom->oprn_sequence[j] + "\",";
                    json += "\"subject_ref\":\"agent_base\",";
                    json += "\"target_ref\":\"" + target_class + "\",";
                    json += "\"status\":\"VALIDATED\",";
                    json += "\"b_length\":0.0,";
                    json += "\"b_vector\":{\"x\":0.0,\"y\":0.0,\"z\":0.0}}";
                    if (j + 1 < active_blom->oprn_sequence.size()) json += ",";
                }
            }
            json += "]}";
        } else {
            json += "null";
        }
        json += ",";

        // 7. Cached BLOMs
        json += "\"cached_bloms\":[";
        for (size_t k = 0; k < cached_bloms.size(); ++k) {
            const auto& cb = cached_bloms[k];
            json += "{\"uuid\":\"" + cb.uuid() + "\",";
            json += "\"is_complete\":" + std::string(cb.is_complete ? "true" : "false") + ",";
            json += "\"outcome_marker\":" + std::to_string(cb.outcome_marker) + ",";
            json += "\"oprn_count\":" + std::to_string(cb.oprn_sequence.size()) + "}";
            if (k + 1 < cached_bloms.size()) json += ",";
        }
        json += "]}";

        zmq::message_t zmsg(json.begin(), json.end());
        pub_.send(zmsg, zmq::send_flags::none);
    }
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    bool use_sim = true;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--dataset") use_sim = false;
        if (arg == "--sim") use_sim = true;
    }

    std::cout << "========================================================\n";
    std::cout << "      🚀 GTR1 Neuro-Symbolic Cognitive Core             \n";
    std::cout << "      Mode: " << (use_sim ? "LIVE PYBULLET SIMULATION (Closed-Loop)" : "TUM RGB-D DATASET (Open-Loop)") << "\n";
    std::cout << "========================================================\n";

    // 1. Инициализация памяти GTR1
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("gtr1_space_01", 0.05);
    auto mp11 = std::make_shared<gtr1::core::MP11Manager>();
    auto trl1 = std::make_shared<gtr1::core::TRL1Log>();
    std::cout << "[SYSTEM] Domain Memory Models (MP10, MP11, TRL1) initialized.\n";

    // 2. Инициализация операторов каскадов DTR и RTR
    auto mtv1 = std::make_shared<gtr1::core::Mtv1Motivator>();
    auto slgw = std::make_shared<gtr1::operators::SlgwNode>(mp10, mp11, "tcp://127.0.0.1:5555", true);
    auto splicer = std::make_shared<gtr1::operators::Spl1Splicer>(mp10);
    auto prd1 = std::make_shared<gtr1::operators::Prd1Predictor>(mp10, trl1);
    std::cout << "[SYSTEM] Operators (SLGW, SPL1, PRD1, MTV1) initialized.\n";

    // 3. Инжекция целевой потребности в Мотиватор (MTV1)
    mtv1->inject_goal("cup", 0.95f);
    std::cout << "[SYSTEM] MTV1 Goal Injected: class='cup', urgency=0.95\n";

    try {
        TelemetryPublisher telemetry(use_sim ? "SIM" : "DATASET");
        gtr1::core::TraceLogger::get_instance().set_telemetry_sink(
            [&telemetry](const std::string& tid, const std::string& comp, const std::string& msg) {
                telemetry.send_log(comp, "[" + tid + "] " + msg);
            }
        );

        GTR_TRACE("INIT", "SYS", "GTR1 Stack Online. ZeroMQ Telemetry active on port 5556.");

        if (use_sim) {
            // === РЕЖИМ 1: Живой симулятор PyBullet (Closed-Loop RTR контур) ===
            std::cout << "[SYSTEM] Connecting to PyBullet ZMQ stream (SUB: 5560, PUB: 5561)...\n";
            gtr1::infra::SimStreamReceiver receiver("tcp://127.0.0.1:5560", "tcp://127.0.0.1:5561");

            // Инициализация сокета для получения команд с дашборда (настройка цели)
            void* goal_ctx = zmq_ctx_new();
            void* goal_sub = zmq_socket(goal_ctx, ZMQ_SUB);
            zmq_connect(goal_sub, "tcp://127.0.0.1:5562");
            zmq_setsockopt(goal_sub, ZMQ_SUBSCRIBE, "", 0);
            int goal_timeout = 0; // Неблокирующий
            zmq_setsockopt(goal_sub, ZMQ_RCVTIMEO, &goal_timeout, sizeof(goal_timeout));

            cv::Mat rgb, depth;
            Eigen::Matrix4f pose;
            double ts = 0.0;
            std::string trace_id;
            bool is_clean = true;
            uint64_t rx_frame_id = 0;
            uint64_t frame_counter = 0;
            bool goal_reached = false;
            std::shared_ptr<gtr1::core::MP13Blom> sim_active_blom = nullptr;
            bool is_manual_stopped = false;
            bool has_manual_nav_target = false;
            Eigen::Vector3f manual_nav_pos{0.0f, 0.0f, 0.20f};
            float manual_nav_yaw{0.0f};
            Eigen::Vector3f active_blom_origin{0.0f, 0.0f, 0.20f};
            std::string active_blom_target_key = "";

            GTR_TRACE("INIT", "SIM", "Closed-loop PyBullet pipeline started.");

            while (!g_shutdown_requested.load()) {
                if (!receiver.poll_frame(rgb, depth, pose, ts, trace_id, is_clean, rx_frame_id, 200)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                frame_counter++;
                uint64_t active_tick_id = (rx_frame_id > 0) ? rx_frame_id : frame_counter;

                // 0. Опрос динамической цели из Дашборда (порт 5562)
                char goal_buf[256];
                int goal_len = zmq_recv(goal_sub, goal_buf, sizeof(goal_buf) - 1, ZMQ_DONTWAIT);
                if (goal_len > 0) {
                    goal_buf[goal_len] = '\0';
                    std::string goal_str(goal_buf);
                    if (goal_str.rfind("TARGET:", 0) == 0) {
                        std::string target_name = goal_str.substr(7);
                        while (!target_name.empty() && (target_name.back() == '\r' || target_name.back() == '\n' || target_name.back() == ' ')) {
                            target_name.pop_back();
                        }
                        has_manual_nav_target = false;
                        if (target_name == "NONE" || target_name == "none" || target_name == "EXPLORE" || target_name.empty()) {
                            mtv1->zero_goal();
                            goal_reached = false;
                            GTR_TRACE(trace_id, "MTV1", "Target cleared. Switching to AUTONOMOUS AREA EXPLORATION.");
                            std::cout << "[MTV1] Goal cleared -> Autonomous Area Exploration mode.\n";
                        } else {
                            mtv1->inject_goal(target_name, 0.95f);
                            goal_reached = false;
                            GTR_TRACE(trace_id, "MTV1", "New target injected from UI: '" + target_name + "'");
                            std::cout << "[MTV1] Dynamic Goal Injected from Web UI: '" << target_name << "'\n";
                        }
                    } else if (goal_str.rfind("CMD:", 0) == 0) {
                        std::string cmd = goal_str.substr(4);
                        if (cmd.find("STOP") != std::string::npos) {
                            is_manual_stopped = true;
                            receiver.send_command(0.0f, 0.0f, 0.0f, trace_id, active_tick_id);
                            GTR_TRACE(trace_id, "UI", "Manual emergency STOP triggered from dashboard.");
                            std::cout << "[UI] Manual STOP triggered.\n";
                        } else if (cmd.find("RESUME") != std::string::npos) {
                            is_manual_stopped = false;
                            GTR_TRACE(trace_id, "UI", "Manual motion RESUMED from dashboard.");
                            std::cout << "[UI] Motion RESUMED.\n";
                        }
                    } else if (goal_str.rfind("NAV_POSE:", 0) == 0) {
                        std::string payload = goal_str.substr(9);
                        float nx = 0.0f, ny = 0.0f, nyaw = 0.0f;
                        if (sscanf(payload.c_str(), "%f,%f,%f", &nx, &ny, &nyaw) >= 2) {
                            manual_nav_pos = Eigen::Vector3f(nx, ny, 0.20f);
                            manual_nav_yaw = nyaw * M_PI / 180.0f;
                            has_manual_nav_target = true;
                            goal_reached = false;
                            mtv1->zero_goal();
                            active_blom_target_key = "";
                            GTR_TRACE(trace_id, "UI", "Interactive Goal received: target=[" + 
                                      std::to_string(nx).substr(0,4) + ", " + std::to_string(ny).substr(0,4) + 
                                      "], yaw=" + std::to_string(nyaw).substr(0,4) + " deg");
                            std::cout << "[MTV1] Navigating to manual waypoint [" << nx << ", " << ny << "] yaw=" << (nyaw*180.0f/M_PI) << " deg\n";
                        }
                    }
                }

                // 1. DTR Прямой ход: Вокселизация в MP10 и передача в ML-воркер каскада CD11
                slgw->process_frame(ts, rgb, depth, pose);

                // 2. Сбор состояния для RTR и телеметрии
                Eigen::Vector3f cam_pos = mp10->get_pose().block<3,1>(0,3);
                Eigen::Vector3f agent_pos = cam_pos;
                if (use_sim) {
                    agent_pos.z() = 0.20f; // Якорная точка шасси робота на высоте пола (0.20м)
                }
                auto invariants = mp11->get_domain_invariants();
                auto current_voxels = mp10->get_occupied_voxels();

                bool has_path = false;
                Eigen::Vector3f path_start = agent_pos;
                Eigen::Vector3f path_end = agent_pos;

                float vx = 0.0f;
                float wz = 0.0f;

                // 3. RTR Обратный ход: ручная остановка, целевые координаты или семантические инварианты
                if (is_manual_stopped) {
                    receiver.send_command(0.0f, 0.0f, 0.0f, trace_id, active_tick_id);
                    telemetry.send_tick_bundle(active_tick_id, ts, mp10->get_pose(), current_voxels,
                                               path_start, path_end, has_path, invariants,
                                               mtv1->get_target_class(), agent_pos, sim_active_blom,
                                               (prd1 && prd1->get_log()) ? prd1->get_log()->get_all_trajectories() : std::vector<gtr1::core::MP13Blom>{},
                                               goal_reached);
                    continue;
                }

                if (has_manual_nav_target) {
                    float dist_final = (manual_nav_pos.head<2>() - agent_pos.head<2>()).norm();
                    float current_yaw = std::atan2(pose(1,2), pose(0,2));

                    if (active_blom_target_key != "manual_waypoint") {
                        active_blom_origin = agent_pos;
                        active_blom_target_key = "manual_waypoint";
                    }

                    gtr1::core::MP11Object subject("agent_base");
                    subject.bounding_box.center = agent_pos;
                    gtr1::core::MP11Object target_obj("manual_waypoint");
                    target_obj.bounding_box.center = manual_nav_pos;
                    auto direct_splice = splicer->form_splice(subject, target_obj);
                    std::vector<std::shared_ptr<gtr1::core::MP12Oprn>> splices = { direct_splice };
                    auto blom = prd1->build_event_chain("manual_waypoint", agent_pos, manual_nav_pos, splices, "waypoint");
                    sim_active_blom = blom;

                    if (sim_active_blom) {
                        sim_active_blom->update_oprn_progress(agent_pos, active_blom_origin, 0.25f);
                    }

                    // Определяем цель руления: следующий незавершенный waypoint в цепочке OPRN
                    Eigen::Vector3f steer_target = manual_nav_pos;
                    if (blom && !blom->oprn_items.empty()) {
                        Eigen::Vector3f wp_accum = active_blom_origin;
                        for (const auto& item : blom->oprn_items) {
                            if (!item) continue;
                            wp_accum += item->b_vector;
                            if (item->status != gtr1::core::OprnStatus::EXECUTED &&
                                (wp_accum.head<2>() - agent_pos.head<2>()).norm() > 0.22f) {
                                steer_target = wp_accum;
                                break;
                            }
                        }
                    }

                    has_path = true;
                    path_start = agent_pos;
                    path_end = manual_nav_pos;

                    if (dist_final > 0.20f) {
                        float dx = steer_target.x() - agent_pos.x();
                        float dy = steer_target.y() - agent_pos.y();
                        float target_yaw = std::atan2(dy, dx);
                        float yaw_err = normalize_angle(target_yaw - current_yaw);

                        wz = std::clamp(1.6f * yaw_err, -0.9f, 0.9f);
                        if (std::abs(yaw_err) < 0.35f) {
                            vx = std::clamp(0.4f * dist_final, 0.08f, 0.28f);
                        } else if (std::abs(yaw_err) < 0.70f) {
                            vx = 0.05f;
                        } else {
                            vx = 0.0f; // чистый разворот на месте
                        }
                        receiver.send_command(vx, 0.0f, wz, trace_id, active_tick_id);
                    } else {
                        float final_yaw_err = normalize_angle(manual_nav_yaw - current_yaw);
                        if (std::abs(final_yaw_err) > 0.08f) {
                            wz = std::clamp(1.5f * final_yaw_err, -0.6f, 0.6f);
                            vx = 0.0f;
                            receiver.send_command(0.0f, 0.0f, wz, trace_id, active_tick_id);
                        } else {
                            vx = 0.0f;
                            wz = 0.0f;
                            receiver.send_command(0.0f, 0.0f, 0.0f, trace_id, active_tick_id);
                            has_manual_nav_target = false;
                            if (sim_active_blom) {
                                sim_active_blom->close_as_successful(1.0f);
                                prd1->save_to_log(sim_active_blom, "manual_waypoint");
                                GTR_TRACE(trace_id, "TRL1", "Manual waypoint reached! BLOM persisted in TRL1Log (outcome_marker=1.0).");
                            }
                            GTR_TRACE(trace_id, "OP13", "Target coordinate and orientation achieved! Holding position.");
                        }
                    }

                    telemetry.send_tick_bundle(active_tick_id, ts, mp10->get_pose(), current_voxels,
                                               path_start, path_end, has_path, invariants,
                                               mtv1->get_target_class(), agent_pos, sim_active_blom,
                                               (prd1 && prd1->get_log()) ? prd1->get_log()->get_all_trajectories() : std::vector<gtr1::core::MP13Blom>{},
                                               goal_reached);
                    continue;
                }

                bool found_target = false;
                if (mtv1->has_active_goal()) {
                    for (const auto& inv : invariants) {
                        if (inv.class_label_id == mtv1->get_target_class()) {
                            found_target = true;
                            mtv1->set_target_position(inv.bounding_box.center);

                            float dist = mtv1->calculate_distance_to_zero(agent_pos);

                            // Формирование сплайса
                            gtr1::core::MP11Object subject("agent_base");
                            subject.bounding_box.center = agent_pos;
                            auto direct_splice = splicer->form_splice(subject, inv);

                            // Планирование A* траектории
                            std::vector<std::shared_ptr<gtr1::core::MP12Oprn>> splices = { direct_splice };
                            auto blom = prd1->build_event_chain(inv.uuid(), agent_pos, inv.bounding_box.center, splices, mtv1->get_target_class());
                            sim_active_blom = blom;

                            if (active_blom_target_key != inv.uuid()) {
                                active_blom_origin = agent_pos;
                                active_blom_target_key = inv.uuid();
                            }

                            if (blom && blom->is_complete) {
                                // Пошаговый FSM-трекинг OPRN
                                blom->update_oprn_progress(agent_pos, active_blom_origin, 0.25f);

                                has_path = true;
                                path_start = agent_pos;
                                path_end = inv.bounding_box.center;

                                // Определяем цель руления: следующий незавершенный waypoint в цепочке OPRN
                                Eigen::Vector3f steer_target = inv.bounding_box.center;
                                if (!blom->oprn_items.empty()) {
                                    Eigen::Vector3f wp_accum = active_blom_origin;
                                    for (const auto& item : blom->oprn_items) {
                                        if (!item) continue;
                                        wp_accum += item->b_vector;
                                        if (item->status != gtr1::core::OprnStatus::EXECUTED &&
                                            (wp_accum.head<2>() - agent_pos.head<2>()).norm() > 0.25f) {
                                            steer_target = wp_accum;
                                            break;
                                        }
                                    }
                                }

                                // Динамическое перепланирование при препятствии перед активным путем
                                Eigen::Vector3f hit_pt;
                                if (!mp10->raycast(agent_pos, steer_target, hit_pt) && (steer_target - agent_pos).norm() > 0.35f) {
                                    GTR_TRACE(trace_id, "PRD1", "Dynamic obstacle ahead! Local replanning from current waypoint...");
                                    auto replanned = prd1->replan_from_current(blom, agent_pos, inv.bounding_box.center, inv.uuid(), mtv1->get_target_class());
                                    if (replanned && replanned->is_complete) {
                                        blom = replanned;
                                        sim_active_blom = blom;
                                        active_blom_origin = agent_pos;
                                        GTR_TRACE(trace_id, "PRD1", "Replanning from current position complete. Continuity preserved.");
                                    }
                                }

                                // Расчет замкнутого кинематического управления (Closed-Loop Kinematics)
                                if (dist > 0.60f && !goal_reached) {
                                    float dx = steer_target.x() - agent_pos.x();
                                    float dy = steer_target.y() - agent_pos.y();
                                    float target_yaw = std::atan2(dy, dx);

                                    float current_yaw = std::atan2(pose(1,2), pose(0,2));
                                    float yaw_err = normalize_angle(target_yaw - current_yaw);

                                    wz = std::clamp(1.6f * yaw_err, -0.9f, 0.9f);
                                    if (std::abs(yaw_err) < 0.35f) {
                                        vx = std::clamp(0.4f * dist, 0.08f, 0.28f);
                                    } else if (std::abs(yaw_err) < 0.70f) {
                                        vx = 0.05f;
                                    } else {
                                        vx = 0.0f; // чистый разворот на месте
                                    }

                                    receiver.send_command(vx, 0.0f, wz, trace_id, active_tick_id);

                                    if (frame_counter % 5 == 0) {
                                        GTR_TRACE(trace_id, "OP13", "Navigating to " + inv.class_label_id + " via OPRN waypoints (dist: " + 
                                                  std::to_string(dist).substr(0,4) + "m, vx=" + std::to_string(vx).substr(0,4) + 
                                                  ", wz=" + std::to_string(wz).substr(0,4) + ")");
                                    }
                                } else {
                                    // Цель достигнута! Остановка.
                                    vx = 0.0f;
                                    wz = 0.0f;
                                    receiver.send_command(0.0f, 0.0f, 0.0f, trace_id, active_tick_id);
                                    if (!goal_reached) {
                                        goal_reached = true;
                                        GTR_TRACE(trace_id, "MTV1", "GOAL CONTACT ACHIEVED! Reached target object '" + inv.class_label_id + "'.");
                                        blom->close_as_successful(1.0f);
                                        prd1->save_to_log(blom, mtv1->get_target_class());
                                        GTR_TRACE(trace_id, "TRL1", "Goal BLOM persisted in TRL1Log with outcome_marker=1.0.");
                                        mtv1->zero_goal();
                                    }
                                }
                            }
                            break;
                        }
                    }
                }

                // 4. Активное исследование неизведанной области (Frontier Exploration) при отсутствии цели или ее поиске
                if (!found_target && !goal_reached) {
                    float current_yaw = std::atan2(pose(1,2), pose(0,2));
                    Eigen::Vector3f fwd(std::cos(current_yaw), std::sin(current_yaw), 0.0f);
                    Eigen::Vector3f left(std::cos(current_yaw + 0.65f), std::sin(current_yaw + 0.65f), 0.0f);  // ~+37 deg
                    Eigen::Vector3f right(std::cos(current_yaw - 0.65f), std::sin(current_yaw - 0.65f), 0.0f); // ~-37 deg

                    Eigen::Vector3f p_center(agent_pos.x(), agent_pos.y(), 0.30f);
                    Eigen::Vector3f hit_c, hit_l, hit_r;

                    // Передний луч (0.75м)
                    bool blocked_front = !mp10->raycast(p_center, p_center + fwd * 0.75f, hit_c) ||
                                         mp10->is_near_obstacle(p_center + fwd * 0.45f, 0.30f);

                    // Боковые лучи (0.65м)
                    bool blocked_left  = !mp10->raycast(p_center, p_center + left * 0.65f, hit_l) ||
                                         mp10->is_near_obstacle(p_center + left * 0.40f, 0.25f);

                    bool blocked_right = !mp10->raycast(p_center, p_center + right * 0.65f, hit_r) ||
                                         mp10->is_near_obstacle(p_center + right * 0.40f, 0.25f);

                    // Критическая близость к углу / стене в упор (< 0.35м)
                    bool stuck_close = mp10->is_near_obstacle(p_center + fwd * 0.32f, 0.22f) ||
                                       (agent_pos.x() > 3.20f || agent_pos.x() < -1.20f ||
                                        agent_pos.y() > 2.20f || agent_pos.y() < -2.20f);

                    // Границы стен арены (запас безопасности)
                    bool wall_ahead = (p_center.x() + fwd.x()*0.7f > 3.1f || p_center.x() + fwd.x()*0.7f < -1.1f ||
                                       p_center.y() + fwd.y()*0.7f > 2.1f || p_center.y() + fwd.y()*0.7f < -2.1f);

                    if (stuck_close || (blocked_front && blocked_left && blocked_right)) {
                        // УГОЛ / ТУПИК: обе стороны и перед заблокированы!
                        vx = -0.12f;
                        wz = blocked_left ? -0.55f : 0.55f;
                        if (frame_counter % 10 == 0) {
                            GTR_TRACE(trace_id, "OP13", "Corner/Trap detected! Backing up (-0.12 m/s) with escape rotation.");
                        }
                    } else if (blocked_front || wall_ahead) {
                        // Препятствие или стена строго впереди: ЧИСТЫЙ РАЗВОРОТ НА МЕСТЕ (vx = 0.0)
                        vx = 0.0f;
                        if (blocked_left && !blocked_right) {
                            wz = -0.65f; // вправо
                        } else if (blocked_right && !blocked_left) {
                            wz = 0.65f;  // влево
                        } else {
                            // Отворачиваем к центру арены (0, 0)
                            float to_center_yaw = std::atan2(-agent_pos.y(), 1.0f - agent_pos.x());
                            float yaw_diff = normalize_angle(to_center_yaw - current_yaw);
                            wz = (yaw_diff >= 0.0f) ? 0.65f : -0.65f;
                        }
                        if (frame_counter % 10 == 0) {
                            GTR_TRACE(trace_id, "OP13", "Obstacle ahead. Rotating on spot toward open sector (wz=" + 
                                      std::to_string(wz).substr(0,4) + ")");
                        }
                    } else if (blocked_left) {
                        vx = 0.14f;
                        wz = -0.45f;
                    } else if (blocked_right) {
                        vx = 0.14f;
                        wz = 0.45f;
                    } else {
                        vx = 0.20f;
                        wz = 0.05f;
                        if (frame_counter % 10 == 0) {
                            std::string mode_str = mtv1->has_active_goal() ? 
                                ("Searching for target '" + mtv1->get_target_class() + "' in room") : 
                                "Autonomous Frontier Exploration (No target)";
                            GTR_TRACE(trace_id, "OP13", mode_str + " (vx=" + std::to_string(vx).substr(0,4) + 
                                      ", pos=[" + std::to_string(agent_pos.x()).substr(0,4) + ", " + 
                                      std::to_string(agent_pos.y()).substr(0,4) + "])");
                        }
                    }

                    receiver.send_command(vx, 0.0f, wz, trace_id, active_tick_id);
                }

                // 5. Синхронная публикация единого пакета тика (TICK_BUNDLE)
                std::vector<gtr1::core::MP13Blom> cached;
                if (prd1 && prd1->get_log()) {
                    cached = prd1->get_log()->get_all_trajectories();
                }
                telemetry.send_tick_bundle(active_tick_id, ts, mp10->get_pose(), current_voxels,
                                           path_start, path_end, has_path, invariants,
                                           mtv1->get_target_class(), agent_pos, sim_active_blom,
                                           cached, goal_reached);

                if (frame_counter % 30 == 0) {
                    GTR_TRACE(trace_id, "TICK", "Tick #" + std::to_string(active_tick_id) + " processed. Voxels: " + 
                              std::to_string(current_voxels.size()) + ". Sensor: " + (is_clean ? "CLEAN" : "EMULATED_NOISE"));
                }
            }

            // Остановка симулятора при выходе
            receiver.send_command(0.0f, 0.0f, 0.0f, "SHUTDOWN", 0);

        } else {
            // === РЕЖИМ 2: Воспроизведение датасета TUM RGB-D ===
            std::string dataset_path = "07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
            std::ifstream check_ds(dataset_path + "/associations_gt.txt");
            if (!check_ds.is_open()) {
                dataset_path = "../07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
            }

            gtr1::infra::DatasetPlayer player(dataset_path, slgw, 30.0);
            if (!player.load_associations("associations_gt.txt")) {
                std::cerr << "[ERROR] Could not load dataset associations.\n";
                return 1;
            }

            std::cout << "[SYSTEM] Playing TUM RGB-D stream at 30 FPS. Invariant recognition and RTR loop active...\n";
            GTR_TRACE("INIT", "DATASET", "Synchronized RGB-D playback started.");

            size_t frame_counter = 0;
            std::shared_ptr<gtr1::core::MP13Blom> dataset_active_blom = nullptr;
            player.play(0, true, [&](size_t f_idx, double ts) {
                if (g_shutdown_requested.load()) {
                    player.stop();
                    return;
                }

                frame_counter++;
                std::string trace_id = gtr1::core::TraceLogger::generate_trace_id(f_idx);

                auto ds_voxels = mp10->get_occupied_voxels();
                auto invariants = mp11->get_domain_invariants();
                Eigen::Vector3f agent_pos = mp10->get_pose().block<3,1>(0,3);

                bool has_path = false;
                Eigen::Vector3f path_start = agent_pos;
                Eigen::Vector3f path_end = agent_pos;

                for (const auto& inv : invariants) {
                    if (inv.class_label_id == mtv1->get_target_class()) {
                        mtv1->set_target_position(inv.bounding_box.center);

                        float dist = mtv1->calculate_distance_to_zero(agent_pos);

                        gtr1::core::MP11Object subject("agent_base");
                        subject.bounding_box.center = agent_pos;
                        auto direct_splice = splicer->form_splice(subject, inv);

                        std::vector<std::shared_ptr<gtr1::core::MP12Oprn>> splices = { direct_splice };
                        auto blom = prd1->build_event_chain(inv.uuid(), agent_pos, inv.bounding_box.center, splices, mtv1->get_target_class());
                        dataset_active_blom = blom;

                        if (blom && blom->is_complete) {
                            has_path = true;
                            path_start = agent_pos;
                            path_end = inv.bounding_box.center;

                            if (frame_counter % 15 == 0) {
                                GTR_TRACE(trace_id, "RTR", "Target '" + inv.class_label_id + "' tracked. Distance to zero: " +
                                          std::to_string(dist).substr(0,4) + "m. BLOM links: " + std::to_string(blom->oprn_sequence.size()));
                            }

                            if (mtv1->is_goal_achieved(agent_pos, 0.3f)) {
                                GTR_TRACE(trace_id, "MTV1", "GOAL CONTACT ACHIEVED! Zeroing goal vector.");
                                blom->set_outcome_marker(0.95f);
                                prd1->save_to_log(blom, mtv1->get_target_class());
                                mtv1->zero_goal();
                            }
                        }
                        break;
                    }
                }

                std::vector<gtr1::core::MP13Blom> cached;
                if (prd1 && prd1->get_log()) {
                    cached = prd1->get_log()->get_all_trajectories();
                }
                telemetry.send_tick_bundle(f_idx, ts, mp10->get_pose(), ds_voxels,
                                           path_start, path_end, has_path, invariants,
                                           mtv1->get_target_class(), agent_pos, dataset_active_blom,
                                           cached, mtv1->is_goal_achieved(agent_pos, 0.3f));
            });
        }

    } catch (const std::exception& e) {
        std::cerr << "[CRASH FATAL] Exception: " << e.what() << "\n";
    }

    std::cout << "[SYSTEM] GTR1 Core Shutdown cleanly.\n";
    return 0;
}
