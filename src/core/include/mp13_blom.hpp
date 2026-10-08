#pragma once

#include "vali.hpp"

namespace gtr1 {
namespace core {

/**
 * @brief MP13 BLOM: Схема событий (Поведенческий граф).
 * Описывает временную последовательность выполнения сплайсов (MP12).
 */
class MP13Blom : public ValiBase {
public:
    std::vector<std::string> oprn_sequence; // Список UUID сплайсов
    bool is_complete;
    float outcome_marker; // Дофаминовый вес (результативность)

    explicit MP13Blom(std::string id) 
        : ValiBase(std::move(id)), is_complete(false), outcome_marker(0.0f) {}

    /**
     * @brief Добавляет новый сплайс в цепочку поведения
     */
    void append_oprn(const std::string& oprn_uuid) {
        oprn_sequence.push_back(oprn_uuid);
    }
};

} // namespace core
} // namespace gtr1
