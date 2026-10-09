#pragma once

#include "mp10_space.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>

namespace gtr1 {
namespace core {

/**
 * @brief Менеджер построения 3D воксельной карты OctoMap.
 * Выполняет калиброванную репроекцию карты глубины (TUM RGB-D интринсики) в 3D облако точек,
 * фильтрует сенсорный шум и обновляет воксельную сетку с разрешением 0.05м (NFR: < 33 мс на кадр).
 */
class OctoMapManager {
private:
    std::shared_ptr<MP10Space> mp10_;
    // Интринсики камеры TUM Freiburg 1
    float fx_{517.3f};
    float fy_{516.5f};
    float cx_{318.6f};
    float cy_{255.3f};
    float depth_scale_{5000.0f};

    // Параметры фильтрации шума
    float min_depth_{0.4f}; // Минимальная глубина (слепая зона сенсора 40 см)
    float max_depth_{4.0f}; // Максимальная дальность надежного отсчета (4 м)
    int downsample_step_{3}; // Шаг прореживания пикселей для удержания задержки < 33 мс

    // Физическая привязка ровного пола (для наземного колесного робота)
    bool ground_plane_snapping_{true};
    float ground_plane_z_{0.0f};
    float ground_snap_threshold_{0.04f}; // Допуск привязки пола (+- 4 см)
    float ground_cutoff_min_{-0.05f};    // Отсечение подповерхностного шума (< -5 см)

    double last_update_latency_ms_{0.0};

public:
    explicit OctoMapManager(std::shared_ptr<MP10Space> mp10,
                            float fx = 517.3f, float fy = 516.5f,
                            float cx = 318.6f, float cy = 255.3f,
                            float depth_scale = 5000.0f,
                            int downsample_step = 3);

    /**
     * @brief Обновляет воксельную сетку MP10 на основе кадра глубины и позы камеры SE(3).
     */
    void update_from_depth(const cv::Mat& depth, const Eigen::Matrix4f& pose);

    double get_last_latency_ms() const { return last_update_latency_ms_; }
    void set_downsample_step(int step) { downsample_step_ = std::max(1, step); }
    void set_depth_range(float min_d, float max_d) { min_depth_ = min_d; max_depth_ = max_d; }
    void set_ground_plane_snapping(bool enable, float ground_z = 0.0f, float threshold = 0.04f, float cutoff_min = -0.05f) {
        ground_plane_snapping_ = enable;
        ground_plane_z_ = ground_z;
        ground_snap_threshold_ = threshold;
        ground_cutoff_min_ = cutoff_min;
    }
    bool is_ground_plane_snapping_enabled() const { return ground_plane_snapping_; }
    void clear_map();
};

} // namespace core
} // namespace gtr1