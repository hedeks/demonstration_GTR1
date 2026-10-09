#include "include/octomap_manager.hpp"
#include <iostream>
#include <chrono>

namespace gtr1 {
namespace core {

OctoMapManager::OctoMapManager(std::shared_ptr<MP10Space> mp10,
                               float fx, float fy, float cx, float cy,
                               float depth_scale, int downsample_step)
    : mp10_(std::move(mp10)),
      fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      depth_scale_(depth_scale),
      min_depth_(0.4f),
      max_depth_(4.0f),
      downsample_step_(downsample_step),
      last_update_latency_ms_(0.0) {
    if (mp10_ && !mp10_->occupancy_grid) {
        // Каноническое разрешение 0.05м (5 см) в соответствии со Sprint 2 DoD
        mp10_->occupancy_grid = std::make_shared<octomap::OcTree>(0.05);
    }
}

void OctoMapManager::update_from_depth(const cv::Mat& depth, const Eigen::Matrix4f& pose) {
    if (depth.empty() || !mp10_ || !mp10_->occupancy_grid) return;

    auto start_time = std::chrono::steady_clock::now();

    octomap::Pointcloud octo_cloud;

    // Репроекция глубины в 3D точки (координаты камеры -> мировые координаты SE(3))
    // Формат TUM RGB-D: 16-битный PNG, scale factor 5000 (5000 единиц = 1.0 метр)
    const int rows = depth.rows;
    const int cols = depth.cols;
    const int step = std::max(1, downsample_step_);

    Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
    Eigen::Vector3f translation = pose.block<3, 1>(0, 3);

    for (int v = 0; v < rows; v += step) {
        const uint16_t* row_ptr = depth.ptr<uint16_t>(v);
        for (int u = 0; u < cols; u += step) {
            uint16_t raw_depth = row_ptr[u];
            if (raw_depth == 0) continue;

            float z = static_cast<float>(raw_depth) / depth_scale_;

            // Шумоподавление: отсечение слепой зоны сенсора (< 0.4м) и шума на дальней границе (> 4.0м)
            if (z < min_depth_ || z > max_depth_) continue;

            float x = (static_cast<float>(u) - cx_) * z / fx_;
            float y = (static_cast<float>(v) - cy_) * z / fy_;

            Eigen::Vector3f p_cam(x, y, z);
            Eigen::Vector3f p_world = rotation * p_cam + translation;

            // Физическая привязка ровного пола и подавление подповерхностного шума:
            // Наземный колесный робот физически не способен двигаться вне плоскости пола.
            if (ground_plane_snapping_) {
                if (p_world.z() < ground_cutoff_min_) continue; // отсечение шума под полом
                if (std::abs(p_world.z() - ground_plane_z_) <= ground_snap_threshold_) {
                    p_world.z() = ground_plane_z_; // каноническая привязка к плоскости пола Z = 0
                }
            }

            octo_cloud.push_back(p_world.x(), p_world.y(), p_world.z());
        }
    }

    // Вставка облака точек в OctoMap из позиции сенсора
    octomap::point3d sensor_origin(translation.x(), translation.y(), translation.z());
    mp10_->occupancy_grid->insertPointCloud(octo_cloud, sensor_origin);

    auto end_time = std::chrono::steady_clock::now();
    last_update_latency_ms_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

void OctoMapManager::clear_map() {
    if (mp10_ && mp10_->occupancy_grid) {
        mp10_->occupancy_grid->clear();
    }
}

} // namespace core
} // namespace gtr1