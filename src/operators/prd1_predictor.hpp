#pragma once

#include "../core/include/mp10_space.hpp"
#include "../core/include/mp12_oprn.hpp"
#include "../core/include/mp13_blom.hpp"
#include "../core/include/trl1_log.hpp"
#include "gtr1_structures.pb.h"
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace gtr1 {
namespace operators {

/**
 * @brief PRD1: Семантический предиктор (Predictor).
 * Отвечает за обратный ход (RTR): планирование оптимального поведенческого графа (MP13Blom)
 * для достижения целевого инварианта.
 * Выполняет поиск A* с учетом Safety Margin (0.3м) и штрафом за близость к препятствиям (+50.0).
 * Кэширует успешные цепочки в TRL1Log и мгновенно извлекает их при Outcome Marker > 0.8.
 */
class Prd1Predictor {
private:
    std::shared_ptr<core::MP10Space> mp10_;
    std::shared_ptr<core::TRL1Log> trl1_log_;
    float robot_radius_{0.22f}; // Физический радиус цилиндра шасси (22 см)
    float robot_height_{0.75f}; // Полная высота робота от пола (75 см)

public:
    explicit Prd1Predictor(std::shared_ptr<core::MP10Space> mp10,
                           std::shared_ptr<core::TRL1Log> trl1_log = nullptr,
                           float robot_radius = 0.22f,
                           float robot_height = 0.75f);

    void set_robot_dimensions(float radius, float height) {
        robot_radius_ = radius;
        robot_height_ = height;
    }
    float get_robot_radius() const { return robot_radius_; }
    float get_robot_height() const { return robot_height_; }

    /**
     * @brief Доменный метод: планирование цепочки сплайсов (MP13Blom).
     * @param target_uuid UUID целевого объекта в MP11.
     * @param start_pos Текущая позиция агента (субъекта).
     * @param target_pos 3D позиция цели.
     * @param available_splices Набор предрассчитанных сплайсов от SPL1.
     * @param goal_class Имя класса цели (например, "cup") для выборки из TRL1Log.
     */
    std::shared_ptr<core::MP13Blom> build_event_chain(
        const std::string& target_uuid,
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<core::MP12Oprn>>& available_splices,
        const std::string& goal_class = "");

    /**
     * @brief Адаптер для Protobuf DTO (обратная совместимость).
     */
    /**
     * @brief Динамическое локальное перепланирование от текущей достигнутой точки.
     * Сохраняет историю ранее выполненных OPRN (со статусом EXECUTED)
     * и строит обход препятствия через A* от текущего положения агента.
     */
    std::shared_ptr<core::MP13Blom> replan_from_current(
        std::shared_ptr<core::MP13Blom> current_blom,
        const Eigen::Vector3f& current_pos,
        const Eigen::Vector3f& target_pos,
        const std::string& target_uuid,
        const std::string& goal_class = "");

    std::shared_ptr<gtr1::models::MP13Blom> build_event_chain(
        const std::string& target_uuid,
        const Eigen::Vector3f& start_pos,
        const Eigen::Vector3f& target_pos,
        const std::vector<std::shared_ptr<gtr1::models::MP12Oprn>>& available_splices,
        const std::string& goal_class = "");

    /**
     * @brief Сохраняет выполненный маршрут в лог опыта TRL1.
     */
    void save_to_log(std::shared_ptr<core::MP13Blom> completed_blom, const std::string& goal_class = "");
    void save_to_log(std::shared_ptr<gtr1::models::MP13Blom> completed_blom, const std::string& goal_class = "");

    std::shared_ptr<core::TRL1Log> get_log() const { return trl1_log_; }

private:
    std::vector<Eigen::Vector3f> run_astar(const Eigen::Vector3f& start, const Eigen::Vector3f& goal);
};

} // namespace operators
} // namespace gtr1