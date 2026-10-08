#pragma once
#include "../core/include/mp10_space.hpp"
#include "gtr1_structures.pb.h"
#include <memory>
#include <vector>
#include <string>

namespace gtr1 {
namespace operators {

class Prd1Predictor {
private:
    std::shared_ptr<core::MP10Space> mp10_;
    std::vector<std::shared_ptr<gtr1::models::MP13Blom>> trl1_log_;

public:
    Prd1Predictor(std::shared_ptr<core::MP10Space> mp10);

    /**
     * @brief Запускает обратный ход (RTR) - планирует маршрут к цели.
     * Если прямой сплайс COLLISION, запускает A* для построения цепочки.
     */
    std::shared_ptr<gtr1::models::MP13Blom> build_event_chain(
        const std::string& target_uuid, 
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<gtr1::models::MP12Oprn>>& available_splices);

    void save_to_log(std::shared_ptr<gtr1::models::MP13Blom> completed_blom);

private:
    // Вспомогательный метод поиска A*
    std::vector<Eigen::Vector3f> run_astar(const Eigen::Vector3f& start, const Eigen::Vector3f& goal);
};

} // namespace operators
} // namespace gtr1
