#include "../core/include/mp10_space.hpp"
#include "gtr1_structures.pb.h"
#include <Eigen/Dense>
#include <memory>
#include <iostream>

namespace gtr1 {
namespace operators {

/**
 * @brief SPL1: Синтаксический предиктор (Splicer).
 * Отвечает за валидацию физической возможности действия (Сплайса MP12).
 * Связывает Субъекта и Объекта через геометрический луч в MP10.
 */
class Spl1Splicer {
private:
    std::shared_ptr<core::MP10Space> mp10_space_;

public:
    Spl1Splicer(std::shared_ptr<core::MP10Space> mp10) : mp10_space_(std::move(mp10)) {}

    /**
     * @brief Формирует и валидирует сплайс между субъектом и целью.
     */
    std::shared_ptr<gtr1::models::MP12Oprn> form_splice(const gtr1::models::MP11Object& subject, const gtr1::models::MP11Object& target) {
        auto splice = std::make_shared<gtr1::models::MP12Oprn>();
        splice->mutable_base()->set_uuid("oprn_" + subject.base().uuid() + "_" + target.base().uuid());
        
        // 1. Устанавливаем валентности
        splice->set_subject_ref(subject.base().uuid());
        splice->set_target_ref(target.base().uuid());

        // 2. Рассчитываем поведенческий b-вектор (дирекция)
        float bx = target.bounding_box().center().x() - subject.bounding_box().center().x();
        float by = target.bounding_box().center().y() - subject.bounding_box().center().y();
        float bz = target.bounding_box().center().z() - subject.bounding_box().center().z();
        
        splice->mutable_b_vector()->set_x(bx);
        splice->mutable_b_vector()->set_y(by);
        splice->mutable_b_vector()->set_z(bz);

        // 3. Синтаксическая валидация (Тест геометрического включения на MP10)
        Eigen::Vector3f origin(subject.bounding_box().center().x(), subject.bounding_box().center().y(), subject.bounding_box().center().z());
        Eigen::Vector3f t(target.bounding_box().center().x(), target.bounding_box().center().y(), target.bounding_box().center().z());
        
        Eigen::Vector3f hit_point;
        bool is_path_clear = mp10_space_->raycast(origin, t, hit_point);

        if (is_path_clear) {
            splice->set_status(gtr1::models::VALIDATED);
        } else {
            splice->set_status(gtr1::models::COLLISION);
        }

        return splice;
    }
};

} // namespace operators
} // namespace gtr1
