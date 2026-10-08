#pragma once

#include "vali.hpp"
#include "mp10_space.hpp" // For mock_eigen
#include <array>

namespace gtr1 {
namespace core {

struct Box3D {
    mock_eigen::Vector3f center;
    mock_eigen::Vector3f size;
};

/**
 * @brief MP11 Object: Карта инвариантов.
 * Каждый объект - это кристаллизованный в памяти результат узнавания среды (CNN1).
 */
class MP11Object : public ValiBase {
public:
    std::array<float, 512> feature_embedding;
    mock_eigen::Vector3f spatial_anchor;
    Box3D bounding_box;
    std::string class_label_id;

    explicit MP11Object(std::string id) : ValiBase(std::move(id)) {
        feature_embedding.fill(0.0f);
    }

    /**
     * @brief Вычисляет сходство между этим и другим объектом на базе эмбеддингов
     */
    float compute_similarity(const MP11Object& other) const {
        // TODO: Вычисление косинусного расстояния
        return 1.0f; 
    }
};

} // namespace core
} // namespace gtr1
