#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <thread>
#include <cmath>
#include <numeric>
#include <random>

#include "mp11_manager.hpp"
#include "mp11_object.hpp"
#include "gtr1_structures.pb.h"
#include "ipc_messages.pb.h"

using namespace gtr1::core;

namespace {

/**
 * @brief Вспомогательная функция для создания единичного 512-мерного эмбеддинга.
 */
std::array<float, 512> make_embedding(float seed_phase) {
    std::array<float, 512> emb;
    float sum_sq = 0.0f;
    for (size_t i = 0; i < 512; ++i) {
        emb[i] = std::sin(static_cast<float>(i) * 0.05f + seed_phase);
        sum_sq += emb[i] * emb[i];
    }
    float inv_norm = 1.0f / std::sqrt(sum_sq);
    for (size_t i = 0; i < 512; ++i) {
        emb[i] *= inv_norm;
    }
    return emb;
}

/**
 * @brief Создает объект MP11Object с заданными параметрами.
 */
MP11Object make_test_obj(const std::string& label,
                         const Eigen::Vector3f& center,
                         float seed_phase) {
    MP11Object obj(ValiBase::generate_uuid());
    obj.class_label_id = label;
    obj.bounding_box.center = center;
    obj.bounding_box.size = Eigen::Vector3f(0.15f, 0.15f, 0.15f);
    obj.spatial_anchor = center;
    obj.feature_embedding = make_embedding(seed_phase);
    return obj;
}

} // namespace

// 1. Проверка успешного слияния (Distance < 0.5m И CosineSimilarity > 0.85)
TEST(MP11MergeTest, SpatialSemanticDeduplicationSuccess) {
    MP11Manager manager(0.5f, 0.85f);

    auto obj1 = make_test_obj("cup", Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.0f);
    auto obj2 = make_test_obj("cup", Eigen::Vector3f(0.1f, 0.05f, 1.02f), 0.05f); // близко (0.11м), схожесть ~0.98

    float sim = obj1.compute_similarity(obj2);
    float dist = obj1.distance_to(obj2);
    ASSERT_GT(sim, 0.85f);
    ASSERT_LT(dist, 0.5f);

    std::string uuid1 = manager.add_or_merge_invariant(obj1);
    EXPECT_EQ(manager.get_invariants_count(), 1);

    std::string uuid2 = manager.add_or_merge_invariant(obj2);
    EXPECT_EQ(manager.get_invariants_count(), 1); // Не должно плодить новый объект
    EXPECT_EQ(uuid1, uuid2);                      // UUID обязан сохраниться

    auto stored = manager.get_invariant(uuid1);
    ASSERT_TRUE(stored.has_value());
    EXPECT_EQ(stored->class_label_id, "cup");
}

// 2. Расстояние > 0.5м предотвращает слияние
TEST(MP11MergeTest, DistanceSeparationPreventsMerge) {
    MP11Manager manager(0.5f, 0.85f);

    auto obj1 = make_test_obj("cup", Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.0f);
    auto obj2 = make_test_obj("cup", Eigen::Vector3f(0.8f, 0.0f, 1.0f), 0.0f); // 0.8м > 0.5м при идентичном эмбеддинге

    ASSERT_GT(obj1.distance_to(obj2), 0.5f);

    std::string uuid1 = manager.add_or_merge_invariant(obj1);
    std::string uuid2 = manager.add_or_merge_invariant(obj2);

    EXPECT_NE(uuid1, uuid2);
    EXPECT_EQ(manager.get_invariants_count(), 2);
}

// 3. Семантическое сходство < 0.85 предотвращает слияние при близком расположении
TEST(MP11MergeTest, SemanticDissimilarityPreventsMerge) {
    MP11Manager manager(0.5f, 0.85f);

    auto obj1 = make_test_obj("cup", Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.0f);
    auto obj2 = make_test_obj("cup", Eigen::Vector3f(0.05f, 0.0f, 1.0f), 2.5f); // близко (0.05м), но разный эмбеддинг

    float sim = obj1.compute_similarity(obj2);
    ASSERT_LT(sim, 0.85f);
    ASSERT_LT(obj1.distance_to(obj2), 0.5f);

    std::string uuid1 = manager.add_or_merge_invariant(obj1);
    std::string uuid2 = manager.add_or_merge_invariant(obj2);

    EXPECT_NE(uuid1, uuid2);
    EXPECT_EQ(manager.get_invariants_count(), 2);
}

// 4. Разные классы объектов никогда не объединяются
TEST(MP11MergeTest, DifferentClassLabelsNeverMerge) {
    MP11Manager manager(0.5f, 0.85f);

    auto obj1 = make_test_obj("cup", Eigen::Vector3f(0.5f, 0.5f, 1.0f), 0.0f);
    auto obj2 = make_test_obj("keyboard", Eigen::Vector3f(0.5f, 0.5f, 1.0f), 0.0f); // Тот же центр и вектор, но другой класс

    std::string uuid1 = manager.add_or_merge_invariant(obj1);
    std::string uuid2 = manager.add_or_merge_invariant(obj2);

    EXPECT_NE(uuid1, uuid2);
    EXPECT_EQ(manager.get_invariants_count(), 2);

    auto cups = manager.find_by_class("cup");
    auto keyboards = manager.find_by_class("keyboard");
    EXPECT_EQ(cups.size(), 1);
    EXPECT_EQ(keyboards.size(), 1);
}

// 5. Поток наблюдений с сенсорным шумом (20 кадров) не плодит UUID
TEST(MP11MergeTest, ObservationStreamJitterNoUUIDProliferation) {
    MP11Manager manager(0.5f, 0.85f);

    Eigen::Vector3f base_center(1.0f, 0.5f, 2.0f);
    auto initial_obj = make_test_obj("chair", base_center, 0.0f);
    std::string initial_uuid = manager.add_or_merge_invariant(initial_obj);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> noise_dist(-0.04f, 0.04f);
    std::uniform_real_distribution<float> phase_dist(-0.02f, 0.02f);

    // 20 повторных наблюдений со случайным дрожанием сенсора
    for (int frame = 0; frame < 20; ++frame) {
        Eigen::Vector3f noisy_center = base_center + Eigen::Vector3f(noise_dist(rng), noise_dist(rng), noise_dist(rng));
        auto noisy_obs = make_test_obj("chair", noisy_center, phase_dist(rng));

        std::string obs_uuid = manager.add_or_merge_invariant(noisy_obs);

        EXPECT_EQ(obs_uuid, initial_uuid) << "UUID changed on frame " << frame;
        EXPECT_EQ(manager.get_invariants_count(), 1) << "Proliferated UUID on frame " << frame;
    }

    auto final_obj = manager.get_invariant(initial_uuid);
    ASSERT_TRUE(final_obj.has_value());
    // Сглаженный центр должен остаться около исходного
    EXPECT_LT((final_obj->bounding_box.center - base_center).norm(), 0.05f);
}

// 6. Потокобезопасность при конкурентных вставках и запросах
TEST(MP11MergeTest, ConcurrentMultiThreadedIngestion) {
    MP11Manager manager(0.5f, 0.85f);
    const int NUM_THREADS = 4;
    const int OPS_PER_THREAD = 50;

    std::vector<std::thread> workers;
    workers.reserve(NUM_THREADS);

    for (int t = 0; t < NUM_THREADS; ++t) {
        workers.emplace_back([&manager, t]() {
            for (int i = 0; i < OPS_PER_THREAD; ++i) {
                // Каждый поток добавляет объекты в свою пространственную зону
                Eigen::Vector3f center(static_cast<float>(t) * 2.0f, 0.0f, static_cast<float>(i) * 0.01f);
                auto obj = make_test_obj("thread_obj_" + std::to_string(t), center, 0.0f);
                manager.add_or_merge_invariant(obj);

                // Периодическое чтение
                auto list = manager.get_domain_invariants();
                (void)list.size();
            }
        });
    }

    for (auto& w : workers) {
        w.join();
    }

    // Каждый из 4 потоков должен был создать ровно 1 слившийся инвариант своей зоны (т.к. 50 шагов * 0.01м = 0.49м < 0.5м)
    EXPECT_EQ(manager.get_invariants_count(), static_cast<size_t>(NUM_THREADS));
}

// 7. Обработка Protobuf IPC DetectionResponse
TEST(MP11MergeTest, DetectionResponseProtobufIngestion) {
    MP11Manager manager(0.5f, 0.85f);

    gtr1::ipc::DetectionResponse response;
    // 1-я чашка
    auto* d1 = response.add_detected_objects();
    d1->mutable_base()->set_uuid(ValiBase::generate_uuid());
    d1->set_class_label_id("bottle");
    d1->mutable_bounding_box()->mutable_center()->set_x(0.2f);
    d1->mutable_bounding_box()->mutable_center()->set_y(0.0f);
    d1->mutable_bounding_box()->mutable_center()->set_z(1.5f);

    // 2-я чашка того же класса в пределах 0.1м
    auto* d2 = response.add_detected_objects();
    d2->mutable_base()->set_uuid(ValiBase::generate_uuid());
    d2->set_class_label_id("bottle");
    d2->mutable_bounding_box()->mutable_center()->set_x(0.25f);
    d2->mutable_bounding_box()->mutable_center()->set_y(0.02f);
    d2->mutable_bounding_box()->mutable_center()->set_z(1.51f);

    // Сторонний объект другого класса
    auto* d3 = response.add_detected_objects();
    d3->mutable_base()->set_uuid(ValiBase::generate_uuid());
    d3->set_class_label_id("laptop");
    d3->mutable_bounding_box()->mutable_center()->set_x(0.25f);
    d3->mutable_bounding_box()->mutable_center()->set_y(0.02f);
    d3->mutable_bounding_box()->mutable_center()->set_z(1.51f);

    manager.process_detections(response);

    // Ожидаем 2 объекта: слитая "bottle" и отдельный "laptop"
    EXPECT_EQ(manager.get_invariants_count(), 2);
    EXPECT_EQ(manager.find_by_class("bottle").size(), 1);
    EXPECT_EQ(manager.find_by_class("laptop").size(), 1);

    // Проверка очистки
    manager.clear();
    EXPECT_EQ(manager.get_invariants_count(), 0);
}