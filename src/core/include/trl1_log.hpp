#pragma once

#include "mp13_blom.hpp"
#include "gtr1_structures.pb.h"
#include <unordered_map>
#include <vector>
#include <string>
#include <optional>
#include <shared_mutex>
#include <mutex>
#include <algorithm>

namespace gtr1 {
namespace core {

/**
 * @brief TRL1 Log (Trajectory Log): Хранилище траекторного опыта агента.
 * In-memory база данных поведенческих маршрутов (MP13Blom).
 * Позволяет кэшировать проверенные цепочки действий, индексировать по цели
 * и находить маршруты с высоким маркером исхода (> 0.8) для ускорения RTR.
 * Потокобезопасен (std::shared_mutex).
 */
class TRL1Log {
public:
    struct Entry {
        MP13Blom blom;
        std::string goal_class;
        double timestamp{0.0};
        float outcome_marker{0.0f};
    };

private:
    std::unordered_map<std::string, Entry> storage_;
    std::unordered_map<std::string, std::vector<std::string>> goal_index_;
    mutable std::shared_mutex mtx_;

public:
    TRL1Log() = default;

    /**
     * @brief Записывает завершенную или спланированную цепочку в лог опыта.
     */
    void record_trajectory(const MP13Blom& blom, const std::string& goal_class = "", double timestamp = 0.0);

    /**
     * @brief Запись траектории из Protobuf DTO TRL1Entry.
     */
    void record_trajectory(const gtr1::models::TRL1Entry& entry);

    /**
     * @brief Ищет лучшую траекторию с маркером исхода выше заданного порога.
     * @param min_outcome Минимально допустимый маркер (по умолчанию 0.8f).
     * @return Траектория с максимальным маркером, либо std::nullopt.
     */
    std::optional<MP13Blom> find_best_trajectory(float min_outcome = 0.8f) const;

    /**
     * @brief Ищет траектории для конкретного класса цели (например, "cup").
     */
    std::vector<MP13Blom> find_trajectories_by_goal(const std::string& goal_class, float min_outcome = 0.0f) const;

    /**
     * @brief Получает траекторию по UUID.
     */
    std::optional<MP13Blom> get_trajectory(const std::string& uuid) const;

    /**
     * @brief Обновляет маркер исхода существующей траектории по результатам выполнения.
     */
    bool update_outcome(const std::string& uuid, float outcome_marker);

    /**
     * @brief Сериализует запись в Protobuf DTO TRL1Entry.
     */
    bool to_proto(const std::string& uuid, gtr1::models::TRL1Entry& proto) const;

    /**
     * @brief Количество сохраненных траекторий.
     */
    size_t size() const;

    /**
     * @brief Очистить лог.
     */
    void clear();

    /**
     * @brief Возвращает все сохраненные траектории.
     */
    std::vector<MP13Blom> get_all_trajectories() const;
};

} // namespace core
} // namespace gtr1