#pragma once

#include "vali.hpp"
#include <array>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>

#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace core {

/**
 * @brief Ограничивающий параллелепипед 3D (Bounding Box).
 */
struct Box3D {
    Eigen::Vector3f center{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f size{0.0f, 0.0f, 0.0f};

    Box3D() = default;
    Box3D(const Eigen::Vector3f& c, const Eigen::Vector3f& s) : center(c), size(s) {}
};

/**
 * @brief MP11 Object: Карта инвариантов.
 * Каждый объект — это кристаллизованный в памяти результат узнавания среды (CNN1).
 * Наследует ValiBase (обладает UUID и валентностями).
 */
class MP11Object : public ValiBase {
public:
    std::array<float, 512> feature_embedding{};
    Eigen::Vector3f spatial_anchor{0.0f, 0.0f, 0.0f};
    Box3D bounding_box{};
    std::string class_label_id;

    explicit MP11Object(std::string id = "", size_t num_valences = 0)
        : ValiBase(std::move(id), num_valences) {
        feature_embedding.fill(0.0f);
    }

    /**
     * @brief Вычисляет косинусное сходство между эмбеддингами двух инвариантов.
     * @return Значение в диапазоне [-1.0, 1.0], либо 0.0 при нулевой норме.
     */
    float compute_similarity(const MP11Object& other) const {
        float dot_product = 0.0f;
        float norm_a = 0.0f;
        float norm_b = 0.0f;

        for (size_t i = 0; i < 512; ++i) {
            dot_product += feature_embedding[i] * other.feature_embedding[i];
            norm_a += feature_embedding[i] * feature_embedding[i];
            norm_b += other.feature_embedding[i] * other.feature_embedding[i];
        }

        if (norm_a < 1e-12f || norm_b < 1e-12f) {
            return 0.0f;
        }

        return dot_product / (std::sqrt(norm_a) * std::sqrt(norm_b));
    }

    /**
     * @brief Вычисляет евклидово расстояние между центрами объектов.
     */
    float distance_to(const MP11Object& other) const {
        return (bounding_box.center - other.bounding_box.center).norm();
    }

    using ValiBase::to_proto;
    using ValiBase::from_proto;

    /**
     * @brief Сериализация в Protobuf DTO MP11Object.
     */
    void to_proto(gtr1::models::MP11Object& proto) const {
        ValiBase::to_proto(*proto.mutable_base());

        proto.clear_feature_embedding();
        for (float f : feature_embedding) {
            proto.add_feature_embedding(f);
        }

        auto* anchor = proto.mutable_spatial_anchor();
        anchor->set_x(spatial_anchor.x());
        anchor->set_y(spatial_anchor.y());
        anchor->set_z(spatial_anchor.z());

        auto* bbox = proto.mutable_bounding_box();
        bbox->mutable_center()->set_x(bounding_box.center.x());
        bbox->mutable_center()->set_y(bounding_box.center.y());
        bbox->mutable_center()->set_z(bounding_box.center.z());
        bbox->mutable_size()->set_x(bounding_box.size.x());
        bbox->mutable_size()->set_y(bounding_box.size.y());
        bbox->mutable_size()->set_z(bounding_box.size.z());

        proto.set_class_label_id(class_label_id);
    }

    /**
     * @brief Фабричный метод десериализации из Protobuf DTO.
     */
    static MP11Object from_proto(const gtr1::models::MP11Object& proto) {
        std::string id = proto.base().uuid().empty() ? generate_uuid() : proto.base().uuid();
        MP11Object obj(id);

        if (!proto.base().uuid().empty()) {
            obj.ValiBase::from_proto(proto.base());
        }

        if (proto.feature_embedding_size() == 512) {
            for (int i = 0; i < 512; ++i) {
                obj.feature_embedding[i] = proto.feature_embedding(i);
            }
        }

        if (proto.has_spatial_anchor()) {
            obj.spatial_anchor = Eigen::Vector3f(
                proto.spatial_anchor().x(),
                proto.spatial_anchor().y(),
                proto.spatial_anchor().z()
            );
        }

        if (proto.has_bounding_box()) {
            obj.bounding_box.center = Eigen::Vector3f(
                proto.bounding_box().center().x(),
                proto.bounding_box().center().y(),
                proto.bounding_box().center().z()
            );
            obj.bounding_box.size = Eigen::Vector3f(
                proto.bounding_box().size().x(),
                proto.bounding_box().size().y(),
                proto.bounding_box().size().z()
            );
        }

        obj.class_label_id = proto.class_label_id();
        return obj;
    }
};

} // namespace core
} // namespace gtr1