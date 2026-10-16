#pragma once

#include "vali.hpp"
#include "mp12_oprn.hpp"
#include <memory>
#include <vector>
#include <algorithm>
#include <string>

#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace core {

/**
 * @brief MP13 BLOM: Схема событий (Поведенческий граф / Маршрут).
 * Направленный граф (цепочка) сплайсов (MP12), ведущий к достижению цели.
 * Наследует ValiBase.
 * Обладает маркером исхода (outcome marker) — дофаминовой оценкой успешности в [0.0, 1.0].
 */
class MP13Blom : public ValiBase {
public:
    std::vector<std::string> oprn_sequence;
    std::vector<std::shared_ptr<MP12Oprn>> oprn_items;
    bool is_complete{false};
    float outcome_marker{0.0f};

    explicit MP13Blom(std::string id = "")
        : ValiBase(std::move(id), 0),
          is_complete(false),
          outcome_marker(0.0f) {}

    /**
     * @brief Добавляет UUID сплайса в цепочку поведения и открывает/закрывает слот валентности.
     */
    void append_oprn(const std::string& oprn_uuid) {
        oprn_sequence.push_back(oprn_uuid);
        size_t slot = add_valence_slot();
        close_valence(slot, oprn_uuid);
    }

    /**
     * @brief Добавляет полноценный объект сплайса MP12Oprn в цепочку поведения.
     */
    void append_oprn(std::shared_ptr<MP12Oprn> oprn) {
        if (!oprn) return;
        oprn_items.push_back(oprn);
        append_oprn(oprn->uuid());
    }

    /**
     * @brief Устанавливает маркер исхода (оценка успеха в диапазоне [0.0, 1.0]).
     */
    void set_outcome_marker(float score) {
        outcome_marker = std::clamp(score, 0.0f, 1.0f);
    }

    float get_outcome_marker() const {
        return outcome_marker;
    }

    /**
     * @brief Проверяет, достигнута ли цель (расстояние <= epsilon).
     */
    static bool is_goal_achieved(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& target_pos, float epsilon = 0.25f) {
        return (current_pos - target_pos).norm() <= epsilon;
    }

    /**
     * @brief Фиксирует успешное закрытие цели: выставляет is_complete,
     * outcome_marker и переводит незавершенные OPRN в EXECUTED.
     */
    void close_as_successful(float score = 1.0f) {
        is_complete = true;
        set_outcome_marker(score);
        for (auto& oprn : oprn_items) {
            if (oprn && oprn->status != OprnStatus::EXECUTED) {
                if (oprn->can_transition_to(OprnStatus::EXECUTED)) {
                    oprn->transition_to(OprnStatus::EXECUTED);
                }
            }
        }
    }

    /**
     * @brief Пошаговый FSM-трекинг: обновляет статусы OPRN на EXECUTED при прохождении путевых точек.
     * @param current_pos Текущая мировая позиция робота.
     * @param start_pos Исходная стартовая позиция начала маршрута.
     * @param tolerance Допустимая окрестность путевой точки (по умолчанию 0.30м).
     * @return Индекс следующей активной (незавершенной) OPRN (0-based) или -1 если маршрут завершен.
     */
    int update_oprn_progress(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& start_pos, float tolerance = 0.30f) {
        if (oprn_items.empty()) return -1;

        Eigen::Vector3f wp_accum = start_pos;
        int next_active_index = -1;

        for (size_t i = 0; i < oprn_items.size(); ++i) {
            auto& oprn = oprn_items[i];
            if (!oprn) continue;

            wp_accum += oprn->b_vector;
            float dist_to_wp = (current_pos - wp_accum).norm();

            if (oprn->status == OprnStatus::VALIDATED) {
                if (dist_to_wp <= tolerance) {
                    oprn->transition_to(OprnStatus::EXECUTED);
                } else if (next_active_index == -1) {
                    next_active_index = static_cast<int>(i);
                }
            } else if (oprn->status == OprnStatus::PENDING && next_active_index == -1) {
                next_active_index = static_cast<int>(i);
            }
        }

        return next_active_index;
    }

    /**
     * @brief Возвращает список всех уже выполненных OPRN (со статусом EXECUTED).
     */
    std::vector<std::shared_ptr<MP12Oprn>> get_executed_oprns() const {
        std::vector<std::shared_ptr<MP12Oprn>> executed;
        for (const auto& item : oprn_items) {
            if (item && item->status == OprnStatus::EXECUTED) {
                executed.push_back(item);
            }
        }
        return executed;
    }

    using ValiBase::to_proto;
    using ValiBase::from_proto;

    /**
     * @brief Сериализация в Protobuf DTO MP13Blom.
     */
    void to_proto(gtr1::models::MP13Blom& proto) const {
        ValiBase::to_proto(*proto.mutable_base());

        proto.clear_oprn_sequence();
        for (const auto& uuid_str : oprn_sequence) {
            proto.add_oprn_sequence(uuid_str);
        }

        proto.set_is_complete(is_complete);
        proto.set_outcome_marker(outcome_marker);
    }

    /**
     * @brief Фабричный метод десериализации из Protobuf DTO.
     */
    static MP13Blom from_proto(const gtr1::models::MP13Blom& proto) {
        std::string id = proto.base().uuid().empty() ? generate_uuid() : proto.base().uuid();
        MP13Blom blom(id);

        if (!proto.base().uuid().empty()) {
            blom.ValiBase::from_proto(proto.base());
        }

        blom.oprn_sequence.clear();
        for (int i = 0; i < proto.oprn_sequence_size(); ++i) {
            blom.oprn_sequence.push_back(proto.oprn_sequence(i));
        }

        blom.is_complete = proto.is_complete();
        blom.outcome_marker = proto.outcome_marker();
        return blom;
    }
};

} // namespace core
} // namespace gtr1