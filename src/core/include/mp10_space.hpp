#pragma once

#include <string>
#include <memory>
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace gtr1 {
namespace core {

/**
 * @brief MP10 Space: Пространство базы. 
 * Хранит воксельную карту препятствий и текущую локализацию субъекта.
 */
class MP10Space {
public:
    std::string map_id;
    std::shared_ptr<octomap::OcTree> occupancy_grid;
    Eigen::Matrix4f current_pose;

    explicit MP10Space(std::string id) : map_id(std::move(id)) {
        occupancy_grid = std::make_shared<octomap::OcTree>(0.1); // Resolution 0.1m
    }

    // Helper to get active voxels for visualization
    std::vector<Eigen::Vector3f> get_occupied_voxels() const {
        std::vector<Eigen::Vector3f> voxels;
        if (!occupancy_grid) return voxels;
        for (auto it = occupancy_grid->begin_leafs(), end = occupancy_grid->end_leafs(); it != end; ++it) {
            if (occupancy_grid->isNodeOccupied(*it)) {
                voxels.emplace_back(it.getX(), it.getY(), it.getZ());
            }
        }
        return voxels;
    }

    /**
     * @brief Выполняет бросание луча (raycast) для проверки проходимости b-вектора
     * @return true если путь свободен, false если луч упирается в препятствие
     */
    bool raycast(const Eigen::Vector3f& origin, 
                 const Eigen::Vector3f& target, 
                 Eigen::Vector3f& hit_point) const {
        if (!occupancy_grid) return true;
        
        octomap::point3d octo_origin(origin.x(), origin.y(), origin.z());
        Eigen::Vector3f dir = target - origin;
        octomap::point3d octo_dir(dir.x(), dir.y(), dir.z());
        octomap::point3d end;
        
        bool hit = occupancy_grid->castRay(octo_origin, octo_dir, end, true, dir.norm());
        if (hit) {
            hit_point = Eigen::Vector3f(end.x(), end.y(), end.z());
            return false; // Путь прегражден
        }
        return true; 
    }

    /**
     * @brief Проверяет, свободен ли конкретный воксель в пространстве (для A*)
     */
    bool is_node_free(const Eigen::Vector3f& pos) const {
        if (!occupancy_grid) return true;
        octomap::point3d query(pos.x(), pos.y(), pos.z());
        octomap::OcTreeNode* node = occupancy_grid->search(query);
        if (node != nullptr) {
            return !occupancy_grid->isNodeOccupied(node);
        }
        // Неизвестное пространство (unknown) считаем свободным
        return true; 
    }

    void update_pose(const Eigen::Matrix4f& pose) {
        current_pose = pose;
    }
};

} // namespace core
} // namespace gtr1
