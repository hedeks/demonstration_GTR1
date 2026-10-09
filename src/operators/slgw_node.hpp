#pragma once

#include "../core/include/mp10_space.hpp"
#include "../core/include/octomap_manager.hpp"
#include "../core/include/mp11_manager.hpp"
#include "../core/include/kalman_filter_se3.hpp"
#include "../core/include/depth_filter.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <optional>
#include <string>

namespace gtr1 {
namespace operators {

/**
 * @brief SLGW (Sensor-Level Gateway)
 * Шлюз базового каскада. Отвечает за Visual SLAM и сенсорную интеграцию.
 * Включает:
 * 1. Фильтр Калмана для позы агента (KalmanFilterSE3) - устраняет дрейф и джиттер SE(3).
 * 2. Двусторонний фильтр глубины (DepthFilter) - сглаживает шум и удаляет летающие пиксели.
 * 3. Вокселизацию в OctoMap (MP10).
 * 4. Неблокирующий буфер "Drop Latest" (емкость 1) для передачи в ML-сервис.
 */
class SlgwNode {
private:
    std::shared_ptr<core::MP10Space> mp10_;
    std::shared_ptr<core::MP11Manager> mp11_;
    std::unique_ptr<core::OctoMapManager> octo_mgr_;

    // Фильтры сенсорных данных
    std::unique_ptr<core::KalmanFilterSE3> kalman_filter_;
    std::unique_ptr<core::DepthFilter> depth_filter_;
    bool enable_kalman_{true};
    bool enable_depth_filter_{true};

    // ZeroMQ контекст и REQ-сокет
    void* zmq_ctx_{nullptr};
    void* zmq_req_socket_{nullptr};
    std::string zmq_endpoint_{"tcp://127.0.0.1:5555"};
    std::recursive_mutex zmq_mtx_;

    // Асинхронный кольцевой буфер емкостью 1 ("Drop Latest")
    struct FrameBufferItem {
        double timestamp{0.0};
        cv::Mat rgb;
        cv::Mat depth;
        Eigen::Matrix4f pose{Eigen::Matrix4f::Identity()};
    };

    std::optional<FrameBufferItem> pending_frame_;
    std::mutex buffer_mtx_;
    std::condition_variable buffer_cv_;
    std::atomic<bool> is_running_{true};
    std::thread ml_worker_thread_;
    bool enable_ml_worker_{true};

    // Метрики производительности
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_sent_to_ml_{0};
    std::atomic<uint64_t> frames_dropped_by_ml_{0};

public:
    explicit SlgwNode(std::shared_ptr<core::MP10Space> mp10,
                      std::shared_ptr<core::MP11Manager> mp11 = nullptr,
                      std::string zmq_endpoint = "tcp://127.0.0.1:5555",
                      bool enable_ml_worker = true);
    ~SlgwNode();

    /**
     * @brief Синхронная обработка кадра на частоте 30 FPS.
     * Применяет фильтр Калмана к позе, фильтрует карту глубины,
     * обновляет позу в MP10, вокселизирует глубину в OctoMap и помещает RGB-D в буфер Drop Latest.
     */
    void process_frame(double timestamp,
                       const cv::Mat& rgb,
                       const cv::Mat& depth,
                       const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

    /**
     * @brief Синхронная отправка кадра в CNN1 (для тестов ZMQ таймаута и верификации IPC).
     */
    bool send_to_cnn_sync(double timestamp, const cv::Mat& rgb, const cv::Mat& depth = cv::Mat(), const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

    // Управление фильтрами
    void set_enable_kalman(bool enable) { enable_kalman_ = enable; }
    void set_enable_depth_filter(bool enable) { enable_depth_filter_ = enable; }
    bool is_kalman_enabled() const { return enable_kalman_; }
    bool is_depth_filter_enabled() const { return enable_depth_filter_; }

    core::KalmanFilterSE3* get_kalman_filter() { return kalman_filter_.get(); }
    core::DepthFilter* get_depth_filter() { return depth_filter_.get(); }

    uint64_t get_frames_processed() const { return frames_processed_.load(); }
    uint64_t get_frames_sent_to_ml() const { return frames_sent_to_ml_.load(); }
    uint64_t get_frames_dropped_by_ml() const { return frames_dropped_by_ml_.load(); }

    core::OctoMapManager* get_octomap_manager() { return octo_mgr_.get(); }
    std::shared_ptr<core::MP11Manager> get_mp11_manager() const { return mp11_; }

private:
    void ml_worker_loop();
    void init_zmq_socket();
    void close_zmq_socket();
    void reset_zmq_socket(); // Паттерн Lazy Pirate
};

} // namespace operators
} // namespace gtr1
