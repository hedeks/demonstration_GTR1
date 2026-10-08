#pragma once

#include "vali.hpp"
#include "mp11_object.hpp"

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
 * Триплет: Субъект -> Движение (b-вектор) -> Объект.
 */
class MP12Oprn : public ValiBase {
public:
    std::string subject_ref; // UUID MP11 Субъекта
    std::string target_ref;  // UUID MP11 Объекта-цели
    mock_eigen::Vector3f b_vector;
    OprnStatus status;

    // Сплайс по умолчанию требует 2 валентности (субъект и объект)
    explicit MP12Oprn(std::string id) 
        : ValiBase(std::move(id), 2), status(OprnStatus::PENDING) {}

    /**
     * @brief Вычисляет пространственное смещение (b-вектор) между субъектом и объектом
     */
    void calculate_b_vector(const MP11Object& subject, const MP11Object& target) {
        b_vector.x = target.spatial_anchor.x - subject.spatial_anchor.x;
        b_vector.y = target.spatial_anchor.y - subject.spatial_anchor.y;
        b_vector.z = target.spatial_anchor.z - subject.spatial_anchor.z;
    }
};

} // namespace core
} // namespace gtr1
