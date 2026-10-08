#include "../include/octomap_manager.hpp"
#include <iostream>

namespace gtr1 {
namespace core {

OctoMapManager::OctoMapManager(std::shared_ptr<MP10Space> mp10, float fx, float fy, float cx, float cy)
    : mp10_(std::move(mp10)), fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
    if (!mp10_->occupancy_grid) {
        // Инициализация OctoMap (разрешение 0.1м = 10см воксель для баланса скорости/детализации)
        mp10_->occupancy_grid = std::make_shared<octomap::OcTree>(0.1);
    }
}

void OctoMapManager::update_from_depth(const cv::Mat& depth, const Eigen::Matrix4f& pose) {
    if (depth.empty()) return;

    octomap::Pointcloud octo_cloud;
    
    // Перевод пикселей глубины в 3D точки (координатная система камеры)
    // Формат TUM: depth scale factor is 5000 (16-bit PNG, mm to m)
    for (int v = 0; v < depth.rows; v += 4) { // Downsampling (x4) для скорости (NFR: 33ms)
        for (int u = 0; u < depth.cols; u += 4) {
            uint16_t d = depth.at<uint16_t>(v, u);
            if (d == 0 || d > 20000) continue; // Нет данных или слишком далеко
            
            float z = d / 5000.0f; 
            float x = (u - cx_) * z / fx_;
            float y = (v - cy_) * z / fy_;
            
            // Точка в координатах камеры
            Eigen::Vector4f point_camera(x, y, z, 1.0f);
            // Трансформация в мировую систему координат MP10
            Eigen::Vector4f point_world = pose * point_camera;
            
            octo_cloud.push_back(point_world.x(), point_world.y(), point_world.z());
        }
    }
    
    // Вставка облака точек в OctoMap
    octomap::point3d sensor_origin(pose(0,3), pose(1,3), pose(2,3));
    mp10_->occupancy_grid->insertPointCloud(octo_cloud, sensor_origin);
}

} // namespace core
} // namespace gtr1
