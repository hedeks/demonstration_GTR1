#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <memory>
#include <atomic>
#include <cstdint>

namespace gtr1 {
namespace infra {

/**
 * @brief SimStreamReceiver
 * Шлюз связи C++ ядра GTR1 со средой симуляции PyBullet по ZeroMQ.
 * 1. Принимает поток кадров SensorFrame (RGB, Depth, Pose, Tick/Frame ID) на порту 5560.
 * 2. Отправляет команды управления KinematicCommand (vx, vy, wz, Tick/Frame ID) на порт 5561.
 */
class SimStreamReceiver {
private:
    void* zmq_ctx_{nullptr};
    void* zmq_sub_{nullptr}; // Порт 5560 (входной стрим)
    void* zmq_pub_{nullptr}; // Порт 5561 (выходные команды)

    std::string sub_endpoint_;
    std::string pub_endpoint_;
    std::atomic<bool> is_connected_{false};

public:
    explicit SimStreamReceiver(const std::string& sub_endpoint = "tcp://127.0.0.1:5560",
                               const std::string& pub_endpoint = "tcp://127.0.0.1:5561");
    ~SimStreamReceiver();

    /**
     * @brief Опрос следующего кадра с таймаутом (базовый API).
     * @return true если получен валидный кадр, false при таймауте.
     */
    bool poll_frame(cv::Mat& out_rgb,
                    cv::Mat& out_depth,
                    Eigen::Matrix4f& out_pose,
                    double& out_timestamp,
                    std::string& out_trace_id,
                    bool& out_is_clean,
                    int timeout_ms = 100);

    /**
     * @brief Опрос следующего кадра с извлечением frame_id / tick_id (синхронизированный API).
     */
    bool poll_frame(cv::Mat& out_rgb,
                    cv::Mat& out_depth,
                    Eigen::Matrix4f& out_pose,
                    double& out_timestamp,
                    std::string& out_trace_id,
                    bool& out_is_clean,
                    uint64_t& out_frame_id,
                    int timeout_ms = 100);

    /**
     * @brief Отправка кинематической команды скорости в PyBullet с привязкой к tick_id / frame_id.
     */
    void send_command(float vx, float vy, float wz, const std::string& trace_id = "", uint64_t frame_id = 0);

    bool is_connected() const { return is_connected_.load(); }
};

} // namespace infra
} // namespace gtr1
