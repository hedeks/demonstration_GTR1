#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <shared_mutex>
#include <optional>
#include <Eigen/Dense>

#include "mp11_object.hpp"
#include "gtr1_structures.pb.h"
#include "ipc_messages.pb.h"

namespace gtr1 {
namespace core {

/**
 * @brief MP11Manager: Менеджер карты инвариантов (MP11).
 * Принимает детекции от ML-сервиса (IPC DTO), кристаллизует их в доменные объекты MP11Object,
 * и производит пространственно-семантическое слияние близких инвариантов
 * (CosineSimilarity > 0.85 AND Distance < 0.5m) одного класса.
 * Предотвращает размножение UUID повторно наблюдаемых объектов.
 * Потокобезопасен (std::shared_mutex).
 */
class MP11Manager {
private:
    std::unordered_map<std::string, MP11Object> active_invariants_;
    float distance_threshold_{0.5f};
    float similarity_threshold_{0.85f};
    mutable std::shared_mutex mtx_;

public:
    MP11Manager() = default;
    explicit MP11Manager(float distance_threshold, float similarity_threshold = 0.85f)
        : distance_threshold_(distance_threshold),
          similarity_threshold_(similarity_threshold) {}

    /**
     * @brief Настройка порогов пространственного и семантического слияния.
     */
    void set_merge_thresholds(float distance, float similarity) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        distance_threshold_ = distance;
        similarity_threshold_ = similarity;
    }

    float get_merge_distance_threshold() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return distance_threshold_;
    }

    float get_merge_similarity_threshold() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return similarity_threshold_;
    }

    /**
     * @brief Добавление доменного инварианта со слиянием (Distance < 0.5m && CosineSimilarity > 0.85).
     * @return UUID инварианта (существующий при слиянии, либо новый при вставке).
     */
    std::string add_or_merge_invariant(const MP11Object& new_obj);

    /**
     * @brief Обработка ответа детекций от ML сервиса (граница IPC).
     */
    void process_detections(const gtr1::ipc::DetectionResponse& response);

    /**
     * @brief Возвращает количество активных инвариантов.
     */
    size_t get_invariants_count() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return active_invariants_.size();
    }

    /**
     * @brief Очистить все инварианты.
     */
    void clear() {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        active_invariants_.clear();
    }

    /**
     * @brief Поиск инварианта по UUID.
     */
    std::optional<MP11Object> find_invariant(const std::string& uuid) const;

    /**
     * @brief Алиас find_invariant для единообразия.
     */
    std::optional<MP11Object> get_invariant(const std::string& uuid) const {
        return find_invariant(uuid);
    }

    /**
     * @brief Поиск всех инвариантов заданного класса (например, "cup", "keyboard").
     */
    std::vector<MP11Object> find_by_class(const std::string& class_label) const;

    /**
     * @brief Возвращает все доменные инварианты MP11Object.
     */
    std::vector<MP11Object> get_domain_invariants() const;

    /**
     * @brief Возвращает Protobuf DTO инварианты (для обратной совместимости).
     */
    std::vector<gtr1::models::MP11Object> get_active_invariants() const;
};

} // namespace core
} // namespace gtr1