#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace gtr1 {
namespace core {

/**
 * @brief Базовый класс Vali (Valence-Instance). 
 * Лежит в основе всех структур памяти (MP11-MP13) в Гативус.
 */
class ValiBase {
public:
    std::string uuid;
    bool is_active;
    
    // Список UUID, закрывающих валентность. 
    // Открытая валентность представлена пустой строкой "".
    std::vector<std::string> valences;

    explicit ValiBase(std::string id, size_t num_valences = 0) 
        : uuid(std::move(id)), is_active(true) {
        if (num_valences > 0) {
            valences.resize(num_valences, "");
        }
    }
    
    virtual ~ValiBase() = default;

    /**
     * @brief Проверяет наличие открытых валентностей
     */
    bool has_open_valences() const {
        return std::find(valences.begin(), valences.end(), "") != valences.end();
    }

    /**
     * @brief Закрывает валентность определенным UUID объекта
     */
    void close_valence(size_t index, const std::string& target_uuid) {
        if (index >= valences.size()) {
            throw std::out_of_range("Valence index out of bounds");
        }
        valences[index] = target_uuid;
    }
};

} // namespace core
} // namespace gtr1
