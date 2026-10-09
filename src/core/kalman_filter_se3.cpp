#include "kalman_filter_se3.hpp"
#include <algorithm>

namespace gtr1 {
namespace core {

KalmanFilterSE3::KalmanFilterSE3(float measurement_noise, float process_noise)
    : process_noise_acc_(process_noise),
      measurement_noise_pos_(measurement_noise) {
    reset();
}

void KalmanFilterSE3::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    is_initialized_ = false;
    last_timestamp_ = 0.0;
    state_.setZero();

    // Начальная ковариация неопределенности
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 0.1f;
    P_.block<3, 3>(3, 3) *= 1.0f;

    // Шум измерения R
    R_.setIdentity();
    R_ *= (measurement_noise_pos_ * measurement_noise_pos_);

    current_orientation_ = Eigen::Quaternionf::Identity();
}

void KalmanFilterSE3::set_measurement_noise(float noise_m) {
    std::lock_guard<std::mutex> lock(mtx_);
    measurement_noise_pos_ = noise_m;
    R_.setIdentity();
    R_ *= (measurement_noise_pos_ * measurement_noise_pos_);
}

void KalmanFilterSE3::set_orientation_gain(float gain) {
    std::lock_guard<std::mutex> lock(mtx_);
    orientation_gain_ = std::clamp(gain, 0.01f, 1.0f);
}

Eigen::Matrix4f KalmanFilterSE3::update(double timestamp, const Eigen::Matrix4f& measured_pose) {
    std::lock_guard<std::mutex> lock(mtx_);

    Eigen::Vector3f z_pos = measured_pose.block<3, 1>(0, 3);
    Eigen::Quaternionf z_rot(measured_pose.block<3, 3>(0, 0));
    z_rot.normalize();

    if (!is_initialized_) {
        state_.block<3, 1>(0, 0) = z_pos;
        state_.block<3, 1>(3, 0).setZero();
        current_orientation_ = z_rot;
        last_timestamp_ = timestamp;
        is_initialized_ = true;
        return measured_pose;
    }

    double dt = timestamp - last_timestamp_;
    if (dt <= 1e-4) {
        dt = 1.0 / 30.0; // Значение по умолчанию при идентичных метках времени
    }
    if (dt > 1.0) {
        // Слишком большой разрыв во времени: мягкий сброс состояния
        dt = 0.033;
    }
    float dt_f = static_cast<float>(dt);
    last_timestamp_ = timestamp;

    // --- 1. PREDICT (Экстраполяция состояния) ---
    // Модель постоянной скорости:
    // x_{k|k-1} = x_{k-1} + v_{k-1} * dt
    // v_{k|k-1} = v_{k-1}
    Eigen::Matrix<float, 6, 6> F = Eigen::Matrix<float, 6, 6>::Identity();
    F(0, 3) = dt_f;
    F(1, 4) = dt_f;
    F(2, 5) = dt_f;

    // Матрица ковариации шума процесса Q (Piecewise Constant White Acceleration Model)
    float q_var = process_noise_acc_ * process_noise_acc_;
    float dt2 = dt_f * dt_f;
    float dt3 = dt2 * dt_f;
    float dt4 = dt3 * dt_f;

    Q_.setZero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * (0.25f * dt4 * q_var);
    Q_.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * (0.50f * dt3 * q_var);
    Q_.block<3, 3>(3, 0) = Eigen::Matrix3f::Identity() * (0.50f * dt3 * q_var);
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * (dt2 * q_var);

    // Экстраполяция
    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q_;

    // --- 2. UPDATE (Коррекция по сенсорному измерению позиции) ---
    // Матрица наблюдения H: измеряем только позицию (3x6)
    Eigen::Matrix<float, 3, 6> H;
    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();

    // Невязка измерения (инновация)
    Eigen::Vector3f y = z_pos - state_.block<3, 1>(0, 0);

    // Ковариация инновации S = H * P * H^T + R
    Eigen::Matrix3f S = H * P_ * H.transpose() + R_;

    // Коэффициент усиления Калмана K = P * H^T * S^-1
    Eigen::Matrix<float, 6, 3> K = P_ * H.transpose() * S.inverse();

    // Обновление состояния и ковариации ошибки
    state_ = state_ + K * y;
    Eigen::Matrix<float, 6, 6> I = Eigen::Matrix<float, 6, 6>::Identity();
    P_ = (I - K * H) * P_;

    // --- 3. Сглаживание ориентации через SLERP ---
    current_orientation_ = current_orientation_.slerp(orientation_gain_, z_rot);
    current_orientation_.normalize();

    // Формирование итоговой матрицы SE(3)
    Eigen::Matrix4f filtered_pose = Eigen::Matrix4f::Identity();
    filtered_pose.block<3, 3>(0, 0) = current_orientation_.toRotationMatrix();
    filtered_pose.block<3, 1>(0, 3) = state_.block<3, 1>(0, 0);

    return filtered_pose;
}

Eigen::Vector3f KalmanFilterSE3::get_position() const {
    return state_.block<3, 1>(0, 0);
}

Eigen::Vector3f KalmanFilterSE3::get_velocity() const {
    return state_.block<3, 1>(3, 0);
}

Eigen::Quaternionf KalmanFilterSE3::get_orientation() const {
    return current_orientation_;
}

} // namespace core
} // namespace gtr1
