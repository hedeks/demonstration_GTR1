#pragma once

#include "../core/include/mp10_space.hpp"
#include "../core/include/mp11_object.hpp"
#include "../core/include/mp12_oprn.hpp"
#include "gtr1_structures.pb.h"
#include <memory>

namespace gtr1 {
namespace operators {

/**
 * @brief SPL1: Синтаксический предиктор (Splicer).
 * Отвечает за валидацию физической возможности действия (Сплайса MP12).
 * Работает строго с доменными сущностями ядра (MP11Object, MP12Oprn),
 * исключая привязку к Protobuf DTO.
 */
class Spl1Splicer {
private:
    std::shared_ptr<core::MP10Space> mp10_space_;

public:
    explicit Spl1Splicer(std::shared_ptr<core::MP10Space> mp10) : mp10_space_(std::move(mp10)) {}

    /**
     * @brief Канонический метод: формирование и валидация доменного сплайса (MP12Oprn).
     */
    std::shared_ptr<core::MP12Oprn> form_splice(const core::MP11Object& subject, const core::MP11Object& target);

    /**
     * @brief Адаптер для Protobuf DTO (для совместимости на границе IPC / внешних тестов).
     */
    std::shared_ptr<gtr1::models::MP12Oprn> form_splice(const gtr1::models::MP11Object& subject, const gtr1::models::MP11Object& target);
};

} // namespace operators
} // namespace gtr1