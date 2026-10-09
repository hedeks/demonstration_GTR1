#include "include/mp11_manager.hpp"
#include <cmath>
#include <iostream>

namespace gtr1 {
namespace core {

std::string MP11Manager::add_or_merge_invariant(const MP11Object& new_obj) {
    std::unique_lock<std::shared_mutex> lock(mtx_);

    for (auto& pair : active_invariants_) {
        auto& existing_obj = pair.second;
        if (existing_obj.class_label_id == new_obj.class_label_id) {
            float dist = existing_obj.distance_to(new_obj);
            if (dist < distance_threshold_) {
                // Проверка семантического сходства по косинусной мере эмбеддингов
                float norm_existing = 0.0f;
                float norm_new = 0.0f;
                for (size_t i = 0; i < 512; ++i) {
                    norm_existing += existing_obj.feature_embedding[i] * existing_obj.feature_embedding[i];
                    norm_new += new_obj.feature_embedding[i] * new_obj.feature_embedding[i];
                }
                bool both_have_embeddings = (norm_existing > 1e-6f && norm_new > 1e-6f);
                float sim = both_have_embeddings ? existing_obj.compute_similarity(new_obj) : 1.0f;

                if (!both_have_embeddings || sim > similarity_threshold_) {
                    // Слияние: плавное обновление пространственных параметров (сглаживание шума сенсоров)
                    existing_obj.bounding_box.center = 0.6f * existing_obj.bounding_box.center + 0.4f * new_obj.bounding_box.center;
                    existing_obj.bounding_box.size = 0.6f * existing_obj.bounding_box.size + 0.4f * new_obj.bounding_box.size;
                    existing_obj.spatial_anchor = 0.6f * existing_obj.spatial_anchor + 0.4f * new_obj.spatial_anchor;

                    // Усреднение эмбеддингов с нормализацией к единичной сфере
                    float sum_sq = 0.0f;
                    for (size_t i = 0; i < 512; ++i) {
                        existing_obj.feature_embedding[i] = 0.5f * (existing_obj.feature_embedding[i] + new_obj.feature_embedding[i]);
                        sum_sq += existing_obj.feature_embedding[i] * existing_obj.feature_embedding[i];
                    }
                    if (sum_sq > 1e-8f) {
                        float inv_norm = 1.0f / std::sqrt(sum_sq);
                        for (size_t i = 0; i < 512; ++i) {
                            existing_obj.feature_embedding[i] *= inv_norm;
                        }
                    }

                    // Сохраняем исходный UUID инварианта без размножения (DoD: no UUID proliferation)
                    return existing_obj.uuid();
                }
            }
        }
    }

    // Объект не объединился: регистрируем новый уникальный инвариант
    std::string assigned_uuid = new_obj.uuid().empty() ? ValiBase::generate_uuid() : new_obj.uuid();
    MP11Object stored_obj = new_obj;
    if (stored_obj.uuid().empty()) {
        stored_obj = MP11Object(assigned_uuid);
        stored_obj.class_label_id = new_obj.class_label_id;
        stored_obj.bounding_box = new_obj.bounding_box;
        stored_obj.spatial_anchor = new_obj.spatial_anchor;
        stored_obj.feature_embedding = new_obj.feature_embedding;
    }
    active_invariants_[assigned_uuid] = std::move(stored_obj);
    return assigned_uuid;
}

void MP11Manager::process_detections(const gtr1::ipc::DetectionResponse& response) {
    for (int i = 0; i < response.detected_objects_size(); ++i) {
        const auto& proto_obj = response.detected_objects(i);
        MP11Object domain_obj = MP11Object::from_proto(proto_obj);
        add_or_merge_invariant(domain_obj);
    }
}

std::vector<MP11Object> MP11Manager::find_by_class(const std::string& class_label) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<MP11Object> result;
    for (const auto& pair : active_invariants_) {
        if (pair.second.class_label_id == class_label) {
            result.push_back(pair.second);
        }
    }
    return result;
}

std::vector<MP11Object> MP11Manager::get_domain_invariants() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<MP11Object> res;
    res.reserve(active_invariants_.size());
    for (const auto& pair : active_invariants_) {
        res.push_back(pair.second);
    }
    return res;
}

std::vector<gtr1::models::MP11Object> MP11Manager::get_active_invariants() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<gtr1::models::MP11Object> res;
    res.reserve(active_invariants_.size());
    for (const auto& pair : active_invariants_) {
        gtr1::models::MP11Object proto_obj;
        pair.second.to_proto(proto_obj);
        res.push_back(std::move(proto_obj));
    }
    return res;
}

std::optional<MP11Object> MP11Manager::find_invariant(const std::string& uuid) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    auto it = active_invariants_.find(uuid);
    if (it != active_invariants_.end()) {
        return it->second;
    }
    return std::nullopt;
}

} // namespace core
} // namespace gtr1