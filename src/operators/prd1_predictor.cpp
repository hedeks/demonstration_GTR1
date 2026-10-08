#include "prd1_predictor.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace gtr1 {
namespace operators {

Prd1Predictor::Prd1Predictor(std::shared_ptr<core::MP10Space> mp10) : mp10_(std::move(mp10)) {}

struct AStarNode {
    Eigen::Vector3f pos;
    float g_cost;
    float h_cost;
    std::shared_ptr<AStarNode> parent;

    float f_cost() const { return g_cost + h_cost; }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        return a->f_cost() > b->f_cost(); // Мин-куча
    }
};

// Хэш для Eigen::Vector3f (квантование 10см для сетки A*)
struct Vector3fHash {
    std::size_t operator()(const Eigen::Vector3f& v) const {
        int x = std::round(v.x() / 0.1f);
        int y = std::round(v.y() / 0.1f);
        int z = std::round(v.z() / 0.1f);
        return std::hash<int>()(x) ^ (std::hash<int>()(y) << 1) ^ (std::hash<int>()(z) << 2);
    }
};

struct Vector3fEqual {
    bool operator()(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const {
        return (a - b).norm() < 0.05f;
    }
};

std::vector<Eigen::Vector3f> Prd1Predictor::run_astar(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) {
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, NodeCompare> open_set;
    std::unordered_set<Eigen::Vector3f, Vector3fHash, Vector3fEqual> closed_set;

    auto start_node = std::make_shared<AStarNode>();
    start_node->pos = start;
    start_node->g_cost = 0;
    start_node->h_cost = (start - goal).norm();
    open_set.push(start_node);

    float step = 0.2f; // Шаг алгоритма (20 см)
    
    // Возможные направления движения в 3D (6 соседей)
    std::vector<Eigen::Vector3f> directions = {
        {step, 0, 0}, {-step, 0, 0},
        {0, step, 0}, {0, -step, 0},
        {0, 0, step}, {0, 0, -step}
    };

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        // Пропускаем дубликаты узлов с большим G-cost, которые могли накопиться в очереди
        if (closed_set.find(current->pos) != closed_set.end()) {
            continue;
        }

        // Достигли ли мы радиуса цели?
        if ((current->pos - goal).norm() <= step * 1.5f) {
            std::vector<Eigen::Vector3f> path;
            auto curr = current;
            while (curr != nullptr) {
                path.push_back(curr->pos);
                curr = curr->parent;
            }
            std::reverse(path.begin(), path.end());
            path.push_back(goal);
            return path;
        }

        closed_set.insert(current->pos);

        for (const auto& dir : directions) {
            Eigen::Vector3f neighbor_pos = current->pos + dir;

            if (closed_set.find(neighbor_pos) != closed_set.end()) {
                continue; // Уже посещали этот воксель
            }

            // REQ-03: Проверка на препятствие в OctoMap
            if (!mp10_->is_node_free(neighbor_pos)) {
                continue; // Столкновение, воксель занят
            }

            auto neighbor = std::make_shared<AStarNode>();
            neighbor->pos = neighbor_pos;
            neighbor->g_cost = current->g_cost + step;
            neighbor->h_cost = (neighbor_pos - goal).norm();
            neighbor->parent = current;

            open_set.push(neighbor);
        }
        
        // Предохранитель (лимит итераций для MVP)
        if (closed_set.size() > 2000) break;
    }

    return {}; // Путь не найден (или лимит)
}

std::shared_ptr<gtr1::models::MP13Blom> Prd1Predictor::build_event_chain(
        const std::string& target_uuid, 
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<gtr1::models::MP12Oprn>>& available_splices) {
        
    auto blom = std::make_shared<gtr1::models::MP13Blom>();
    blom->mutable_base()->set_uuid("blom_route_to_" + target_uuid);

    // 1. Проверяем, есть ли уже прямой VALIDATED сплайс
    for (const auto& splice : available_splices) {
        if (splice->target_ref() == target_uuid && splice->status() == gtr1::models::VALIDATED) {
            blom->add_oprn_sequence(splice->base().uuid());
            blom->set_is_complete(true);
            return blom; // Возвращаем прямой путь
        }
    }

    // 2. Прямого пути нет (Raycast уперся в стену). Запускаем A* (REQ-03)!
    std::vector<Eigen::Vector3f> path = run_astar(start_pos, target_pos);

    if (!path.empty()) {
        // Превращаем точки пути в цепочку сплайсов (MP12)
        for (size_t i = 1; i < path.size(); ++i) {
            // В реальной системе мы создаем транзитные MP12Oprn (Шаги),
            // для MVP мы добавляем сгенерированные UUID в граф MP13
            std::string waypoint_uuid = "waypoint_" + std::to_string(i);
            blom->add_oprn_sequence(waypoint_uuid);
        }
        blom->set_is_complete(true);
    } else {
        blom->set_is_complete(false);
    }

    return blom;
}

void Prd1Predictor::save_to_log(std::shared_ptr<gtr1::models::MP13Blom> completed_blom) {
    trl1_log_.push_back(std::move(completed_blom));
}

} // namespace operators
} // namespace gtr1
