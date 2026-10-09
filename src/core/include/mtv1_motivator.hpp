#pragma once

#include "gtr1_structures.pb.h"
#include <string>
#include <shared_mutex>
#include <Eigen/Dense>

namespace gtr1 {
namespace core {

/**
 * @brief MTV1 Motivator: Мотиватор агента (инициатор обратного хода RTR).
 * Отвечает за инжекцию целевых потребностей (класс объекта, 3D координаты),
 * расчет вектора невязки и дистанции обнуления целевого вектора,
 * а также сброс (обнуление) цели при достижении физического контакта.
 * Потокобезопасен (std::shared_mutex).
 */
class Mtv1Motivator {
private:
    gtr1::models::MTV1Goal current_goal_;
    std::string target_object_class_;
    Eigen::Vector3f target_position_{0.0f, 0.0f, 0.0f};
    bool has_active_goal_{false};
    mutable std::shared_mutex mtx_;

public:
    Mtv1Motivator();

    /**
     * @brief Инжекция цели по имени класса объекта (например, "cup").
     */
    void inject_goal(const std::string& object_class, float urgency = 1.0f);

    /**
     * @brief Инжекция цели с известными 3D координатами объекта.
     */
    void inject_goal(const std::string& object_class, const Eigen::Vector3f& target_pos, float urgency = 1.0f);

    /**
     * @brief Установка целевых 3D координат объекта.
     */
    void set_target_position(const Eigen::Vector3f& target_pos);

    /**
     * @brief Получение целевых координат.
     */
    Eigen::Vector3f get_target_position() const;

    /**
     * @brief Расчет текущей дистанции до цели и обновление целевого вектора в Protobuf DTO.
     * @return Евклидово расстояние между текущей позицией агента и целью.
     */
    float calculate_distance_to_zero(const Eigen::Vector3f& current_agent_pos);

    /**
     * @brief Проверяет, достигнута ли цель (дистанция меньше порога касания, по умолчанию 0.2м).
     */
    bool is_goal_achieved(const Eigen::Vector3f& current_agent_pos, float threshold = 0.2f) const;

    /**
     * @brief Обнуление цели (завершение RTR цикла): сброс срочности и вектора невязки.
     */
    void zero_goal();

    /**
     * @brief Проверка наличия активной цели.
     */
    bool has_active_goal() const;

    /**
     * @brief Текущий уровень срочности цели [0.0, 1.0].
     */
    float get_urgency() const;

    /**
     * @brief Имя целевого класса объектов.
     */
    std::string get_target_class() const;

    /**
     * @brief Получение Protobuf DTO текущей цели.
     */
    gtr1::models::MTV1Goal get_active_goal() const;

    /**
     * @brief Сериализация в Protobuf DTO.
     */
    void to_proto(gtr1::models::MTV1Goal& proto) const;

    /**
     * @brief Десериализация из Protobuf DTO.
     */
    void from_proto(const gtr1::models::MTV1Goal& proto);
};

} // namespace core
} // namespace gtr1