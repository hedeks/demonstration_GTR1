#include "spl1_splicer.hpp"
#include <iostream>

namespace gtr1 {
namespace operators {

std::shared_ptr<core::MP12Oprn> Spl1Splicer::form_splice(const core::MP11Object& subject, const core::MP11Object& target) {
    auto splice = std::make_shared<core::MP12Oprn>("oprn_" + subject.uuid() + "_" + target.uuid());

    // 1. Расчет b-вектора и заполнение валентностей
    splice->calculate_b_vector(subject, target);

    // 2. Синтаксическая валидация через raycast на MP10
    Eigen::Vector3f origin = subject.bounding_box.center;
    Eigen::Vector3f dest = target.bounding_box.center;

    Eigen::Vector3f hit_point;
    bool is_base_clear = mp10_space_->raycast(origin, dest, hit_point);

    // Проверка высотного профиля робота (защита от проезда под столами/нависаниями)
    bool is_body_clear = true;
    if (is_base_clear && robot_height_ > 0.30f) {
        // Проверяем лучи через каждые 15 см высоты корпуса робота
        for (float dh = 0.08f; dh <= robot_height_; dh += 0.08f) {
            Eigen::Vector3f layer_origin = origin + Eigen::Vector3f(0.0f, 0.0f, dh);
            Eigen::Vector3f layer_dest = dest;
            if (std::abs(dest.z() - origin.z()) < 0.25f) {
                layer_dest.z() += dh;
            }
            Eigen::Vector3f layer_hit;
            if (!mp10_space_->raycast(layer_origin, layer_dest, layer_hit)) {
                is_body_clear = false;
                break;
            }
        }
    }

    if (is_base_clear && is_body_clear) {
        splice->transition_to(core::OprnStatus::VALIDATED);
    } else {
        splice->transition_to(core::OprnStatus::COLLISION);
    }

    return splice;
}

std::shared_ptr<gtr1::models::MP12Oprn> Spl1Splicer::form_splice(const gtr1::models::MP11Object& subject_dto, const gtr1::models::MP11Object& target_dto) {
    // Конвертация DTO -> Доменная модель
    core::MP11Object subject = core::MP11Object::from_proto(subject_dto);
    core::MP11Object target = core::MP11Object::from_proto(target_dto);

    // Доменная логика
    auto domain_splice = form_splice(subject, target);

    // Конвертация Доменная модель -> DTO
    auto proto_splice = std::make_shared<gtr1::models::MP12Oprn>();
    domain_splice->to_proto(*proto_splice);

    return proto_splice;
}

} // namespace operators
} // namespace gtr1