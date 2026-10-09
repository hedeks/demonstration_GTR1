#include "prd1_predictor.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace gtr1 {
namespace operators {

Prd1Predictor::Prd1Predictor(std::shared_ptr<core::MP10Space> mp10,
                             std::shared_ptr<core::TRL1Log> trl1_log)
    : mp10_(std::move(mp10)),
      trl1_log_(trl1_log ? std::move(trl1_log) : std::make_shared<core::TRL1Log>()) {}

struct AStarNode {
    Eigen::Vector3f pos;
    float g_cost{0.0f};
    float h_cost{0.0f};
    std::shared_ptr<AStarNode> parent{nullptr};

    float f_cost() const { return g_cost + h_cost; }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        return a->f_cost() > b->f_cost(); // Мин-куча
    }
};

struct Vector3fHash {
    std::size_t operator()(const Eigen::Vector3f& v) const {
        int x = static_cast<int>(std::round(v.x() / 0.1f));
        int y = static_cast<int>(std::round(v.y() / 0.1f));
        int z = static_cast<int>(std::round(v.z() / 0.1f));
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
    start_node->g_cost = 0.0f;
    start_node->h_cost = (start - goal).norm();
    open_set.push(start_node);

    const float step = 0.2f; // Шаг алгоритма (20 см)

    // Направления перемещения: 6 ортогональных + 4 диагональных в плоскости XY
    std::vector<Eigen::Vector3f> directions = {
        {step, 0.0f, 0.0f}, {-step, 0.0f, 0.0f},
        {0.0f, step, 0.0f}, {0.0f, -step, 0.0f},
        {0.0f, 0.0f, step}, {0.0f, 0.0f, -step},
        {step, step, 0.0f}, {-step, step, 0.0f},
        {step, -step, 0.0f}, {-step, -step, 0.0f}
    };

    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();

        if (closed_set.find(current->pos) != closed_set.end()) {
            continue;
        }

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
                continue;
            }

            // Проверка проходимости вокселя в MP10
            if (mp10_ && !mp10_->is_node_free(neighbor_pos)) {
                continue; // Препятствие
            }

            // Расчет стоимости g(n): перемещение + штраф за безопасность (Safety Margin 0.3м)
            float move_cost = dir.norm();
            float safety_penalty = 0.0f;
            if (mp10_ && mp10_->is_near_obstacle(neighbor_pos, 0.3f)) {
                safety_penalty = 50.0f; // Штраф за опасную близость к препятствию
            }

            auto neighbor = std::make_shared<AStarNode>();
            neighbor->pos = neighbor_pos;
            neighbor->g_cost = current->g_cost + move_cost + safety_penalty;
            neighbor->h_cost = (neighbor_pos - goal).norm();
            neighbor->parent = current;

            open_set.push(neighbor);
        }

        if (closed_set.size() > 5000) break;
    }

    return {};
}

std::shared_ptr<core::MP13Blom> Prd1Predictor::build_event_chain(
        const std::string& target_uuid,
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<core::MP12Oprn>>& available_splices,
        const std::string& goal_class) {

    // 0. Запрос в TRL1Log: поиск готовой цепочки с outcome_marker > 0.8 (выборка опыта)
    if (trl1_log_) {
        std::string search_key = !goal_class.empty() ? goal_class : target_uuid;
        auto cached = trl1_log_->find_trajectories_by_goal(search_key, 0.8f);
        if (!cached.empty()) {
            return std::make_shared<core::MP13Blom>(cached.front());
        }
        auto direct_by_id = trl1_log_->get_trajectory(target_uuid);
        if (direct_by_id && direct_by_id->get_outcome_marker() > 0.8f) {
            return std::make_shared<core::MP13Blom>(*direct_by_id);
        }
    }

    auto blom = std::make_shared<core::MP13Blom>("blom_route_to_" + target_uuid);

    // 1. Проверяем наличие прямого валидированного сплайса (без преград)
    for (const auto& splice : available_splices) {
        if (splice && splice->target_ref == target_uuid && splice->status == core::OprnStatus::VALIDATED) {
            blom->append_oprn(splice);
            blom->is_complete = true;
            blom->set_outcome_marker(1.0f);
            return blom;
        }
    }

    // 2. Прямой сплайс в коллизии или отсутствует — обратный ход (RTR) через A* поиск
    std::vector<Eigen::Vector3f> path = run_astar(start_pos, target_pos);

    if (!path.empty()) {
        for (size_t i = 1; i < path.size(); ++i) {
            std::string step_id = "oprn_step_" + std::to_string(i);
            std::string sub = (i == 1) ? "agent_base" : ("waypoint_" + std::to_string(i - 1));
            std::string tgt = (i == path.size() - 1) ? target_uuid : ("waypoint_" + std::to_string(i));

            auto step_oprn = std::make_shared<core::MP12Oprn>(step_id, sub, tgt);
            step_oprn->b_vector = path[i] - path[i - 1];
            step_oprn->transition_to(core::OprnStatus::VALIDATED);
            blom->append_oprn(step_oprn);
        }
        blom->is_complete = true;
        blom->set_outcome_marker(0.9f);
    } else {
        for (const auto& splice : available_splices) {
            if (splice) {
                blom->append_oprn(splice);
            }
        }
        blom->is_complete = false;
        blom->set_outcome_marker(0.0f);
    }

    return blom;
}

std::shared_ptr<gtr1::models::MP13Blom> Prd1Predictor::build_event_chain(
        const std::string& target_uuid,
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<gtr1::models::MP12Oprn>>& available_splices,
        const std::string& goal_class) {

    // Преобразуем DTO сплайсы в доменные модели
    std::vector<std::shared_ptr<core::MP12Oprn>> domain_splices;
    domain_splices.reserve(available_splices.size());
    for (const auto& dto : available_splices) {
        if (dto) {
            domain_splices.push_back(std::make_shared<core::MP12Oprn>(core::MP12Oprn::from_proto(*dto)));
        }
    }

    auto domain_blom = build_event_chain(target_uuid, start_pos, target_pos, domain_splices, goal_class);

    auto proto_blom = std::make_shared<gtr1::models::MP13Blom>();
    domain_blom->to_proto(*proto_blom);
    return proto_blom;
}

void Prd1Predictor::save_to_log(std::shared_ptr<core::MP13Blom> completed_blom, const std::string& goal_class) {
    if (completed_blom && trl1_log_) {
        trl1_log_->record_trajectory(*completed_blom, goal_class);
    }
}

void Prd1Predictor::save_to_log(std::shared_ptr<gtr1::models::MP13Blom> completed_blom, const std::string& goal_class) {
    if (completed_blom && trl1_log_) {
        core::MP13Blom domain_blom = core::MP13Blom::from_proto(*completed_blom);
        trl1_log_->record_trajectory(domain_blom, goal_class);
    }
}

} // namespace operators
} // namespace gtr1