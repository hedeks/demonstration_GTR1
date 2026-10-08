#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <Eigen/Dense>

// Protobuf
#include "gtr1_structures.pb.h"
#include "ipc_messages.pb.h"

namespace gtr1 {
namespace core {

class MP11Manager {
private:
    std::unordered_map<std::string, gtr1::models::MP11Object> active_invariants_;
    const float MERGE_DISTANCE = 0.5f;

public:
    MP11Manager() = default;

    /**
     * @brief Обработка ответа от ML сервиса
     */
    void process_detections(const gtr1::ipc::DetectionResponse& response);

    std::vector<gtr1::models::MP11Object> get_active_invariants() const;

private:
    float calculate_distance(const gtr1::models::Vector3& a, const gtr1::models::Vector3& b) const;
};

} // namespace core
} // namespace gtr1
