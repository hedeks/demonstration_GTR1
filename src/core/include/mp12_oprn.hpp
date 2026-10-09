#pragma once

#include "vali.hpp"
#include "mp11_object.hpp"
#include <Eigen/Dense>
#include <stdexcept>

#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace core {

enum class OprnStatus {
    PENDING = 0,
    VALIDATED = 1,
    COLLISION = 2,
    EXECUTED = 3
};

/**
 * @brief MP12 OPRN (Сплайс).
 * Базовая единица поведения.
 * Триплет: Субъект -> Движение (b-вектор) -> Объект-цель.
 * Наследует ValiBase с двумя валентностями (слот 0: Субъект, слот 1: Объект).
 * Управляется строгим FSM-автоматом состояний.
 */
class MP12Oprn : public ValiBase {
public:
    std::string subject_ref;
    std::string target_ref;
    Eigen::Vector3f b_vector{0.0f, 0.0f, 0.0f};
    OprnStatus status{OprnStatus::PENDING};

    explicit MP12Oprn(std::string id = "", std::string subject = "", std::string target = "")
        : ValiBase(std::move(id), 2),
          subject_ref(std::move(subject)),
          target_ref(std::move(target)),
          status(OprnStatus::PENDING) {
        if (!subject_ref.empty()) {
            close_valence(0, subject_ref);
        }
        if (!target_ref.empty()) {
            close_valence(1, target_ref);
        }
    }

    /**
     * @brief Вычисляет пространственное смещение (b-вектор) и закрывает валентности.
     */
    void calculate_b_vector(const MP11Object& subject, const MP11Object& target) {
        subject_ref = subject.uuid().empty() ? generate_uuid() : subject.uuid();
        target_ref = target.uuid().empty() ? generate_uuid() : target.uuid();

        close_valence(0, subject_ref);
        close_valence(1, target_ref);

        b_vector = target.bounding_box.center - subject.bounding_box.center;
    }

    /**
     * @brief Проверяет валидность FSM перехода в новое состояние.
     * Жизненный цикл:
     * - PENDING -> VALIDATED, COLLISION
     * - VALIDATED -> EXECUTED, COLLISION
     * - COLLISION -> PENDING (повторная валидация), COLLISION
     * - EXECUTED -> терминальное состояние
     */
    bool can_transition_to(OprnStatus new_status) const {
        if (status == new_status) return true; // Идемпотентность

        switch (status) {
            case OprnStatus::PENDING:
                return (new_status == OprnStatus::VALIDATED || new_status == OprnStatus::COLLISION);
            case OprnStatus::VALIDATED:
                return (new_status == OprnStatus::EXECUTED || new_status == OprnStatus::COLLISION);
            case OprnStatus::COLLISION:
                return (new_status == OprnStatus::PENDING);
            case OprnStatus::EXECUTED:
                return false; // Из выполненного состояния нет выходов
            default:
                return false;
        }
    }

    /**
     * @brief Выполняет FSM переход в новое состояние.
     * @throws std::logic_error при невалидном переходе.
     */
    void transition_to(OprnStatus new_status) {
        if (!can_transition_to(new_status)) {
            throw std::logic_error("Invalid FSM transition in MP12Oprn (" + uuid() +
                                   "): state " + std::to_string(static_cast<int>(status)) +
                                   " -> " + std::to_string(static_cast<int>(new_status)));
        }
        status = new_status;
    }

    using ValiBase::to_proto;
    using ValiBase::from_proto;

    /**
     * @brief Сериализация в Protobuf DTO MP12Oprn.
     */
    void to_proto(gtr1::models::MP12Oprn& proto) const {
        ValiBase::to_proto(*proto.mutable_base());

        proto.set_subject_ref(subject_ref);
        proto.set_target_ref(target_ref);

        auto* b_vec = proto.mutable_b_vector();
        b_vec->set_x(b_vector.x());
        b_vec->set_y(b_vector.y());
        b_vec->set_z(b_vector.z());

        proto.set_status(static_cast<gtr1::models::OprnStatus>(status));
    }

    /**
     * @brief Фабричный метод десериализации из Protobuf DTO.
     */
    static MP12Oprn from_proto(const gtr1::models::MP12Oprn& proto) {
        std::string id = proto.base().uuid().empty() ? generate_uuid() : proto.base().uuid();
        MP12Oprn oprn(id, proto.subject_ref(), proto.target_ref());

        if (!proto.base().uuid().empty()) {
            oprn.ValiBase::from_proto(proto.base());
        }

        if (proto.has_b_vector()) {
            oprn.b_vector = Eigen::Vector3f(
                proto.b_vector().x(),
                proto.b_vector().y(),
                proto.b_vector().z()
            );
        }

        oprn.status = static_cast<OprnStatus>(proto.status());
        return oprn;
    }
};

} // namespace core
} // namespace gtr1