#include "include/mtv1_motivator.hpp"
#include <cmath>

namespace gtr1 {
namespace core {

Mtv1Motivator::Mtv1Motivator() {
    current_goal_.set_urgency(0.0f);
    current_goal_.set_target_class_label("");
    has_active_goal_ = false;
}

void Mtv1Motivator::inject_goal(const std::string& object_class, float urgency) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    target_object_class_ = object_class;
    current_goal_.set_target_class_label(object_class);
    current_goal_.set_urgency(urgency);
    has_active_goal_ = true;
}

void Mtv1Motivator::inject_goal(const std::string& object_class, const Eigen::Vector3f& target_pos, float urgency) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    target_object_class_ = object_class;
    target_position_ = target_pos;
    current_goal_.set_target_class_label(object_class);
    current_goal_.set_urgency(urgency);
    has_active_goal_ = true;
}

void Mtv1Motivator::set_target_position(const Eigen::Vector3f& target_pos) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    target_position_ = target_pos;
}

Eigen::Vector3f Mtv1Motivator::get_target_position() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return target_position_;
}

float Mtv1Motivator::calculate_distance_to_zero(const Eigen::Vector3f& current_agent_pos) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    Eigen::Vector3f diff = target_position_ - current_agent_pos;

    auto* vec = current_goal_.mutable_target_distance_vector();
    vec->set_x(diff.x());
    vec->set_y(diff.y());
    vec->set_z(diff.z());

    return diff.norm();
}

bool Mtv1Motivator::is_goal_achieved(const Eigen::Vector3f& current_agent_pos, float threshold) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    if (!has_active_goal_) return false;
    return (target_position_ - current_agent_pos).norm() <= threshold;
}

void Mtv1Motivator::zero_goal() {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    current_goal_.set_urgency(0.0f);
    auto* vec = current_goal_.mutable_target_distance_vector();
    vec->set_x(0.0f);
    vec->set_y(0.0f);
    vec->set_z(0.0f);
    has_active_goal_ = false;
}

bool Mtv1Motivator::has_active_goal() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return has_active_goal_;
}

float Mtv1Motivator::get_urgency() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return current_goal_.urgency();
}

std::string Mtv1Motivator::get_target_class() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return target_object_class_;
}

gtr1::models::MTV1Goal Mtv1Motivator::get_active_goal() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return current_goal_;
}

void Mtv1Motivator::to_proto(gtr1::models::MTV1Goal& proto) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    proto = current_goal_;
}

void Mtv1Motivator::from_proto(const gtr1::models::MTV1Goal& proto) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    current_goal_ = proto;
    target_object_class_ = proto.target_class_label();
    has_active_goal_ = (proto.urgency() > 0.0f);
    if (proto.has_target_distance_vector()) {
        target_position_ = Eigen::Vector3f(
            proto.target_distance_vector().x(),
            proto.target_distance_vector().y(),
            proto.target_distance_vector().z()
        );
    }
}

} // namespace core
} // namespace gtr1