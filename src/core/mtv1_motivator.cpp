#include "../include/mtv1_motivator.hpp"

namespace gtr1 {
namespace core {

Mtv1Motivator::Mtv1Motivator() {
    current_goal_.set_urgency(0.0f);
}

void Mtv1Motivator::inject_goal(const std::string& object_class, float urgency) {
    target_object_class_ = object_class;
    current_goal_.set_urgency(urgency);
}

const gtr1::models::MTV1Goal& Mtv1Motivator::get_active_goal() const {
    return current_goal_;
}

const std::string& Mtv1Motivator::get_target_class() const {
    return target_object_class_;
}

} // namespace core
} // namespace gtr1
