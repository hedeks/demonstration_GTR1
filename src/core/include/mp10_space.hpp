#pragma once

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <Eigen/Dense>
#include <memory>
#include <shared_mutex>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace core {

/**
 * @brief MP10 Space: Карта трехмерного пространства на базе воксельной сетки OctoMap.
 * Хранит геометрию сцены, выполняет бросание лучей (Raycast) для проверки проходимости b-векторов,
 * поддерживает проверку Safety Margin (0.3м) и отслеживает текущую позу агента в SE(3).
 * Потокобезопасен (std::shared_mutex).
 */
class MP10Space {
public:
    std::string map_id;
    std::shared_ptr<octomap::OcTree> occupancy_grid;
    Eigen::Matrix4f current_pose;

private:
    mutable std::shared_mutex mtx_;

public:
    explicit MP10Space(std::string id = "default_space", double resolution = 0.05)
        : map_id(std::move(id)),
          occupancy_grid(std::make_shared<octomap::OcTree>(resolution)),
          current_pose(Eigen::Matrix4f::Identity()) {}

    MP10Space(const MP10Space& other) {
        std::shared_lock<std::shared_mutex> lock(other.mtx_);
        map_id = other.map_id;
        current_pose = other.current_pose;
        if (other.occupancy_grid) {
            occupancy_grid = std::make_shared<octomap::OcTree>(*other.occupancy_grid);
        } else {
            occupancy_grid = std::make_shared<octomap::OcTree>(0.05);
        }
    }

    MP10Space& operator=(const MP10Space& other) {
        if (this != &other) {
            std::scoped_lock lock(mtx_, other.mtx_);
            map_id = other.map_id;
            current_pose = other.current_pose;
            if (other.occupancy_grid) {
                occupancy_grid = std::make_shared<octomap::OcTree>(*other.occupancy_grid);
            }
        }
        return *this;
    }

    /**
     * @brief Устанавливает статус занятости вокселя (для тестов и геометрических преград).
     */
    void set_occupied(const Eigen::Vector3f& pos, bool occupied = true) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        if (occupancy_grid) {
            occupancy_grid->updateNode(pos.x(), pos.y(), pos.z(), occupied);
        }
    }

    /**
     * @brief Получает список координат занятых вокселей для визуализации.
     */
    std::vector<Eigen::Vector3f> get_occupied_voxels() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
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
     * @brief Выполняет бросание луча (raycast) для проверки проходимости b-вектора.
     * @param origin Начальная точка луча (субъект).
     * @param target Конечная точка луча (цель).
     * @param hit_point Точка первого препятствия (заполняется при столкновении).
     * @return true если путь свободен, false если обнаружена преграда.
     */
    bool raycast(const Eigen::Vector3f& origin, 
                 const Eigen::Vector3f& target, 
                 Eigen::Vector3f& hit_point) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        if (!occupancy_grid) return true;

        octomap::point3d octo_origin(origin.x(), origin.y(), origin.z());
        Eigen::Vector3f dir = target - origin;
        float dist = dir.norm();
        if (dist < 1e-6f) return true;

        octomap::point3d octo_dir(dir.x(), dir.y(), dir.z());
        octomap::point3d end;

        bool hit = occupancy_grid->castRay(octo_origin, octo_dir, end, true, dist);
        if (hit) {
            hit_point = Eigen::Vector3f(end.x(), end.y(), end.z());
            return false; // Препятствие найдено
        }
        return true; // Свободно
    }

    /**
     * @brief Проверяет, свободен ли воксель в пространстве (для планировщика A*).
     */
    bool is_node_free(const Eigen::Vector3f& pos) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        if (!occupancy_grid) return true;

        octomap::point3d query(pos.x(), pos.y(), pos.z());
        octomap::OcTreeNode* node = occupancy_grid->search(query);
        if (node != nullptr) {
            return !occupancy_grid->isNodeOccupied(node);
        }
        // Неразмеченное пространство считаем свободным
        return true;
    }

    /**
     * @brief Проверяет наличие занятых вокселей в окрестности Safety Margin (по умолчанию 0.3м).
     */
    bool is_near_obstacle(const Eigen::Vector3f& pos, float margin = 0.3f) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        if (!occupancy_grid) return false;

        // Зондирование контрольных точек сферы вокруг позиции
        const float d = margin;
        const float d_diag = margin * 0.7071f;
        std::array<Eigen::Vector3f, 14> probes = {{
            {pos.x() + d, pos.y(), pos.z()}, {pos.x() - d, pos.y(), pos.z()},
            {pos.x(), pos.y() + d, pos.z()}, {pos.x(), pos.y() - d, pos.z()},
            {pos.x(), pos.y(), pos.z() + d}, {pos.x(), pos.y(), pos.z() - d},
            {pos.x() + d_diag, pos.y() + d_diag, pos.z()},
            {pos.x() - d_diag, pos.y() + d_diag, pos.z()},
            {pos.x() + d_diag, pos.y() - d_diag, pos.z()},
            {pos.x() - d_diag, pos.y() - d_diag, pos.z()},
            {pos.x() + d_diag, pos.y(), pos.z() + d_diag},
            {pos.x() - d_diag, pos.y(), pos.z() + d_diag},
            {pos.x(), pos.y() + d_diag, pos.z() + d_diag},
            {pos.x(), pos.y() - d_diag, pos.z() + d_diag}
        }};

        for (const auto& probe : probes) {
            octomap::point3d query(probe.x(), probe.y(), probe.z());
            octomap::OcTreeNode* node = occupancy_grid->search(query);
            if (node != nullptr && occupancy_grid->isNodeOccupied(node)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Потокобезопасное обновление текущей позы SE(3).
     */
    void update_pose(const Eigen::Matrix4f& pose) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        current_pose = pose;
    }

    /**
     * @brief Получение текущей позы SE(3).
     */
    Eigen::Matrix4f get_pose() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return current_pose;
    }

    /**
     * @brief Сериализация в Protobuf DTO MP10Space.
     */
    void to_proto(gtr1::models::MP10Space& proto) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        proto.set_map_id(map_id);

        if (occupancy_grid) {
            std::stringstream ss;
            occupancy_grid->writeBinary(ss);
            proto.set_occupancy_grid_data(ss.str());
        }

        auto* pose_proto = proto.mutable_current_pose();
        pose_proto->clear_data();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                pose_proto->add_data(current_pose(r, c));
            }
        }
    }

    /**
     * @brief Десериализация из Protobuf DTO MP10Space.
     */
    void from_proto(const gtr1::models::MP10Space& proto) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        map_id = proto.map_id();

        if (!proto.occupancy_grid_data().empty()) {
            std::stringstream ss(proto.occupancy_grid_data());
            occupancy_grid = std::make_shared<octomap::OcTree>(0.05);
            occupancy_grid->readBinary(ss);
        }

        if (proto.has_current_pose() && proto.current_pose().data_size() == 16) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    current_pose(r, c) = proto.current_pose().data(r * 4 + c);
                }
            }
        }
    }
};

} // namespace core
} // namespace gtr1