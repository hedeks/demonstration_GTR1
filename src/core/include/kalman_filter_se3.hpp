#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>

namespace gtr1 {
namespace core {

/**
 * @brief KalmanFilterSE3
 * Фильтр Калмана для оценки 6-DOF позы и скоростей агента в пространстве SE(3).
 * Состояние x = [p_x, p_y, p_z, v_x, v_y, v_z]^T (6D).
 * Ориентация сглаживается через кватернионное геодезическое интерполирование (SLERP).
 */
class KalmanFilterSE3 {
private:
    bool is_initialized_{false};
    double last_timestamp_{0.0};

    // Вектор состояния 6x1: [pos(3), vel(3)]
    Eigen::Matrix<float, 6, 1> state_;
    // Ковариационная матрица ошибки 6x6
    Eigen::Matrix<float, 6, 6> P_;
    // Матрица шума процесса Q (6x6)
    Eigen::Matrix<float, 6, 6> Q_;
    // Матрица шума измерений R (3x3)
    Eigen::Matrix<float, 3, 3> R_;

    // Текущая сглаженная ориентация
    Eigen::Quaternionf current_orientation_;
    float orientation_gain_{0.35f}; // Коэффициент доверия к измерению ориентации (SLERP)

    // Параметры шумов
    float process_noise_acc_{0.5f}; // Шум ускорения (м/с^2)
    float measurement_noise_pos_{0.02f}; // Погрешность сенсора (2 см)

    std::mutex mtx_;

public:
    explicit KalmanFilterSE3(float measurement_noise = 0.02f, float process_noise = 0.5f);

    void reset();

    /**
     * @brief Полный шаг фильтрации Калмана (Predict + Update) по новому измерению позы.
     * @param timestamp Метка времени кадра (сек)
     * @param measured_pose Измеренная матрица 4x4 SE(3)
     * @return Сглаженная матрица 4x4 SE(3)
     */
    Eigen::Matrix4f update(double timestamp, const Eigen::Matrix4f& measured_pose);

    /**
     * @brief Получить текущую сглаженную позицию.
     */
    Eigen::Vector3f get_position() const;

    /**
     * @brief Получить текущую оценку линейной скорости агента (м/с).
     */
    Eigen::Vector3f get_velocity() const;

    /**
     * @brief Получить текущую сглаженную ориентацию.
     */
    Eigen::Quaternionf get_orientation() const;

    bool is_initialized() const { return is_initialized_; }

    void set_measurement_noise(float noise_m);
    void set_orientation_gain(float gain);
};

} // namespace core
} // namespace gtr1
