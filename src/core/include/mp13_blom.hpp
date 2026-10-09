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