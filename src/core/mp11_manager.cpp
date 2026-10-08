#include "../include/mp11_manager.hpp"
#include <cmath>
#include <iostream>

namespace gtr1 {
namespace core {

float MP11Manager::calculate_distance(const gtr1::models::Vector3& a, const gtr1::models::Vector3& b) const {
    float dx = a.x() - b.x();
    float dy = a.y() - b.y();
    float dz = a.z() - b.z();
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void MP11Manager::process_detections(const gtr1::ipc::DetectionResponse& response) {
    for (int i = 0; i < response.detected_objects_size(); ++i) {
        const auto& new_obj = response.detected_objects(i);
        bool merged = false;
        
        // REQ-01: Инварианты ближе 0.5м с одинаковым классом сливаются
        for (auto& [uuid, existing_obj] : active_invariants_) {
            if (existing_obj.class_label_id() == new_obj.class_label_id()) {
                float dist = calculate_distance(existing_obj.bounding_box().center(), new_obj.bounding_box().center());
                
                if (dist < MERGE_DISTANCE) {
                    // СЛИЯНИЕ (Update): Обновляем координаты центра
                    *existing_obj.mutable_bounding_box() = new_obj.bounding_box();
                    merged = true;
                    // std::cout << "[MP11] Merged invariant: " << existing_obj.class_label_id() << "\n";
                    break;
                }
            }
        }
        
        if (!merged) {
            // Новый уникальный инвариант
            active_invariants_[new_obj.base().uuid()] = new_obj;
            // std::cout << "[MP11] New Invariant Created: " << new_obj.class_label_id() << "\n";
        }
    }
}

std::vector<gtr1::models::MP11Object> MP11Manager::get_active_invariants() const {
    std::vector<gtr1::models::MP11Object> res;
    res.reserve(active_invariants_.size());
    for (const auto& kv : active_invariants_) {
        res.push_back(kv.second);
    }
    return res;
}

} // namespace core
} // namespace gtr1
