#pragma once
#include "gtr1_structures.pb.h"
#include <string>

namespace gtr1 {
namespace core {

class Mtv1Motivator {
private:
    gtr1::models::MTV1Goal current_goal_;
    std::string target_object_class_;

public:
    Mtv1Motivator();

    void inject_goal(const std::string& object_class, float urgency = 1.0f);

    const gtr1::models::MTV1Goal& get_active_goal() const;
    const std::string& get_target_class() const;
};

} // namespace core
} // namespace gtr1
