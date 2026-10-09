#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include <cmath>

#include "../src/core/include/vali.hpp"
#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/mp11_object.hpp"
#include "../src/core/include/mp12_oprn.hpp"
#include "../src/core/include/mp13_blom.hpp"
#include "../src/core/include/trl1_log.hpp"
#include "../src/operators/spl1_splicer.hpp"
#include "../src/operators/prd1_predictor.hpp"
#include "gtr1_structures.pb.h"

using namespace gtr1::core;
using namespace gtr1::operators;

// ============================================================================
// 1. ТЕСТЫ БАЗОВОГО КЛАССА VALI (ValiBase): 100% покрытие методологии валентностей
// ============================================================================

TEST(ValiBaseTest, UUIDGenerationAndValidation) {
    // Автоматическая генерация валидного UUIDv4
    ValiBase vali1;
    EXPECT_FALSE(vali1.uuid().empty());
    EXPECT_TRUE(ValiBase::is_valid_uuid(vali1.uuid()));

    // Явный UUID
    std::string explicit_id = "123e4567-e89b-42d3-a456-426614174000";
    ValiBase vali2(explicit_id);
    EXPECT_EQ(vali2.uuid(), explicit_id);
    EXPECT_TRUE(ValiBase::is_valid_uuid(vali2.uuid()));

    // Префиксные UUID (OBJ-, OPRN-, BLOM-)
    EXPECT_TRUE(ValiBase::is_valid_uuid("OBJ-123e4567-e89b-42d3-a456-426614174000"));
    EXPECT_TRUE(ValiBase::is_valid_uuid("OPRN-123e4567-e89b-42d3-a456-426614174000"));
    EXPECT_TRUE(ValiBase::is_valid_uuid("BLOM-123e4567-e89b-42d3-a456-426614174000"));

    // Невалидные UUID
    EXPECT_FALSE(ValiBase::is_valid_uuid("not-a-uuid"));
    EXPECT_FALSE(ValiBase::is_valid_uuid(""));
    EXPECT_FALSE(ValiBase::is_valid_uuid("12345"));
}

TEST(ValiBaseTest, ValenceLifecycleAndStateTransitions) {
    // Инициализация объекта с 3 валентностями
    ValiBase vali("test_obj", 3);
    EXPECT_EQ(vali.total_valences_count(), 3);
    EXPECT_EQ(vali.open_valences_count(), 3);
    EXPECT_TRUE(vali.has_open_valences());

    // Все слоты изначально открыты ("")
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_TRUE(vali.is_valence_open(i));
        EXPECT_EQ(vali.get_valence(i), "");
    }

    // Закрываем слот 0
    vali.close_valence(0, "TARGET-UUID-0");
    EXPECT_FALSE(vali.is_valence_open(0));
    EXPECT_EQ(vali.get_valence(0), "TARGET-UUID-0");
    EXPECT_EQ(vali.open_valences_count(), 2);
    EXPECT_TRUE(vali.has_open_valences());

    // Закрываем оставшиеся слоты 1 и 2
    vali.close_valence(1, "TARGET-UUID-1");
    vali.close_valence(2, "TARGET-UUID-2");
    EXPECT_EQ(vali.open_valences_count(), 0);
    EXPECT_FALSE(vali.has_open_valences()); // Все валентности насыщены

    // Открываем слот 1 (сброс)
    vali.open_valence(1);
    EXPECT_TRUE(vali.is_valence_open(1));
    EXPECT_EQ(vali.get_valence(1), "");
    EXPECT_EQ(vali.open_valences_count(), 1);
    EXPECT_TRUE(vali.has_open_valences());

    // Динамическое добавление нового слота
    size_t new_idx = vali.add_valence_slot();
    EXPECT_EQ(new_idx, 3);
    EXPECT_EQ(vali.total_valences_count(), 4);
    EXPECT_TRUE(vali.is_valence_open(3));

    // Изменение размера валентностей
    vali.resize_valences(2);
    EXPECT_EQ(vali.total_valences_count(), 2);
}

TEST(ValiBaseTest, ValenceBoundaryExceptions) {
    ValiBase vali("test_obj", 2);

    // Закрытие пустой строкой должно выбрасывать invalid_argument
    EXPECT_THROW(vali.close_valence(0, ""), std::invalid_argument);

    // Выход за границы индекса
    EXPECT_THROW(vali.close_valence(5, "TARGET"), std::out_of_range);
    EXPECT_THROW(vali.open_valence(5), std::out_of_range);
    EXPECT_THROW(vali.get_valence(5), std::out_of_range);
    EXPECT_THROW(vali.is_valence_open(5), std::out_of_range);
}

TEST(ValiBaseTest, ThreadSafetyConcurrentValences) {
    ValiBase vali("concurrent_obj", 4);

    // Проверяем устойчивость к одновременным операциям чтения и записи
    std::vector<std::thread> threads;
    const int num_iterations = 200;

    // Потоки-писатели
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&vali, t, num_iterations]() {
            for (int i = 0; i < num_iterations; ++i) {
                vali.close_valence(t, "THREAD-" + std::to_string(t) + "-ITER-" + std::to_string(i));
                vali.open_valence(t);
            }
        });
    }

    // Потоки-читатели
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&vali, num_iterations]() {
            for (int i = 0; i < num_iterations; ++i) {
                bool has_open = vali.has_open_valences();
                size_t count = vali.open_valences_count();
                (void)has_open;
                (void)count;
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    EXPECT_EQ(vali.total_valences_count(), 4);
}

TEST(ValiBaseTest, ProtobufRoundTrip) {
    ValiBase orig("test-uuid-42", 2);
    orig.close_valence(0, "SUBJECT-UUID");
    orig.close_valence(1, "TARGET-UUID");
    orig.set_active(true);

    gtr1::models::ValiBase proto;
    orig.to_proto(proto);

    EXPECT_EQ(proto.uuid(), "test-uuid-42");
    EXPECT_TRUE(proto.is_active());
    ASSERT_EQ(proto.valences_size(), 2);
    EXPECT_EQ(proto.valences(0), "SUBJECT-UUID");
    EXPECT_EQ(proto.valences(1), "TARGET-UUID");

    ValiBase restored;
    restored.from_proto(proto);

    EXPECT_EQ(restored.uuid(), orig.uuid());
    EXPECT_EQ(restored.is_active(), orig.is_active());
    EXPECT_EQ(restored.get_valences(), orig.get_valences());
    EXPECT_FALSE(restored.has_open_valences());
}

// ============================================================================
// 2. ТЕСТЫ ВОКСЕЛЬНОГО ПРОСТРАНСТВА MP10 (MP10Space)
// ============================================================================

TEST(MP10SpaceTest, InitializationAndPose) {
    MP10Space space("room_map", 0.05);
    EXPECT_EQ(space.map_id, "room_map");

    Eigen::Matrix4f pose = space.get_pose();
    EXPECT_TRUE(pose.isIdentity());

    Eigen::Matrix4f new_pose = Eigen::Matrix4f::Identity();
    new_pose(0, 3) = 1.5f; // X = 1.5m
    new_pose(1, 3) = -0.5f; // Y = -0.5m
    new_pose(2, 3) = 0.8f; // Z = 0.8m
    space.update_pose(new_pose);

    EXPECT_FLOAT_EQ(space.get_pose()(0, 3), 1.5f);
    EXPECT_FLOAT_EQ(space.get_pose()(1, 3), -0.5f);
    EXPECT_FLOAT_EQ(space.get_pose()(2, 3), 0.8f);
}

TEST(MP10SpaceTest, RaycastFreeAndObstacle) {
    MP10Space space("raycast_map", 0.05);

    Eigen::Vector3f start(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f end(2.0f, 0.0f, 0.0f);
    Eigen::Vector3f hit;

    // В пустом пространстве луч свободно проходит
    EXPECT_TRUE(space.raycast(start, end, hit));
    EXPECT_TRUE(space.is_node_free(Eigen::Vector3f(1.0f, 0.0f, 0.0f)));

    // Добавляем препятствие (занятый воксель) на X = 1.0м
    space.occupancy_grid->updateNode(octomap::point3d(1.0, 0.0, 0.0), true);
    space.occupancy_grid->updateInnerOccupancy();

    // Теперь луч должен натолкнуться на препятствие
    bool clear = space.raycast(start, end, hit);
    EXPECT_FALSE(clear);
    EXPECT_NEAR(hit.x(), 1.0f, 0.1f);
    EXPECT_FALSE(space.is_node_free(Eigen::Vector3f(1.0f, 0.0f, 0.0f)));
}

TEST(MP10SpaceTest, ProtobufRoundTrip) {
    MP10Space orig("serialized_map", 0.1);
    orig.occupancy_grid->updateNode(octomap::point3d(0.5, 0.5, 0.5), true);
    orig.occupancy_grid->updateInnerOccupancy();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose(0, 3) = 2.0f;
    orig.update_pose(pose);

    gtr1::models::MP10Space proto;
    orig.to_proto(proto);

    EXPECT_EQ(proto.map_id(), "serialized_map");
    EXPECT_FALSE(proto.occupancy_grid_data().empty());
    ASSERT_EQ(proto.current_pose().data_size(), 16);
    EXPECT_FLOAT_EQ(proto.current_pose().data(3), 2.0f);

    MP10Space restored("empty");
    restored.from_proto(proto);

    EXPECT_EQ(restored.map_id, "serialized_map");
    EXPECT_FLOAT_EQ(restored.get_pose()(0, 3), 2.0f);
    EXPECT_FALSE(restored.is_node_free(Eigen::Vector3f(0.5f, 0.5f, 0.5f)));
}

// ============================================================================
// 3. ТЕСТЫ ИНВАРИАНТА MP11 (MP11Object)
// ============================================================================

TEST(MP11ObjectTest, CosineSimilarity) {
    MP11Object obj1("cup_1");
    MP11Object obj2("cup_2");

    // Нулевые эмбеддинги
    EXPECT_FLOAT_EQ(obj1.compute_similarity(obj2), 0.0f);

    // Идентичные единичные эмбеддинги -> сходство 1.0
    obj1.feature_embedding.fill(0.1f);
    obj2.feature_embedding.fill(0.1f);
    EXPECT_NEAR(obj1.compute_similarity(obj2), 1.0f, 1e-5f);

    // Ортогональные эмбеддинги -> сходство 0.0
    obj1.feature_embedding.fill(0.0f);
    obj2.feature_embedding.fill(0.0f);
    for (size_t i = 0; i < 256; ++i) obj1.feature_embedding[i] = 1.0f;
    for (size_t i = 256; i < 512; ++i) obj2.feature_embedding[i] = 1.0f;
    EXPECT_NEAR(obj1.compute_similarity(obj2), 0.0f, 1e-5f);

    // Противоположные эмбеддинги -> сходство -1.0
    for (size_t i = 0; i < 512; ++i) {
        obj1.feature_embedding[i] = 0.5f;
        obj2.feature_embedding[i] = -0.5f;
    }
    EXPECT_NEAR(obj1.compute_similarity(obj2), -1.0f, 1e-5f);
}

TEST(MP11ObjectTest, DistanceAndBoundingBox) {
    MP11Object obj1("obj_1");
    obj1.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object obj2("obj_2");
    obj2.bounding_box.center = Eigen::Vector3f(3.0f, 4.0f, 0.0f);

    EXPECT_FLOAT_EQ(obj1.distance_to(obj2), 5.0f);
}

TEST(MP11ObjectTest, ProtobufRoundTrip) {
    MP11Object orig("bottle_uuid");
    orig.class_label_id = "bottle";
    orig.spatial_anchor = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    orig.bounding_box.center = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    orig.bounding_box.size = Eigen::Vector3f(0.1f, 0.1f, 0.3f);
    orig.feature_embedding[0] = 0.42f;
    orig.feature_embedding[511] = 0.99f;

    gtr1::models::MP11Object proto;
    orig.to_proto(proto);

    EXPECT_EQ(proto.base().uuid(), "bottle_uuid");
    EXPECT_EQ(proto.class_label_id(), "bottle");
    EXPECT_FLOAT_EQ(proto.spatial_anchor().x(), 1.0f);
    EXPECT_FLOAT_EQ(proto.bounding_box().size().z(), 0.3f);
    EXPECT_FLOAT_EQ(proto.feature_embedding(0), 0.42f);
    EXPECT_FLOAT_EQ(proto.feature_embedding(511), 0.99f);

    MP11Object restored = MP11Object::from_proto(proto);
    EXPECT_EQ(restored.uuid(), orig.uuid());
    EXPECT_EQ(restored.class_label_id, "bottle");
    EXPECT_FLOAT_EQ(restored.spatial_anchor.y(), 2.0f);
    EXPECT_FLOAT_EQ(restored.bounding_box.center.z(), 3.0f);
    EXPECT_FLOAT_EQ(restored.feature_embedding[0], 0.42f);
    EXPECT_FLOAT_EQ(restored.feature_embedding[511], 0.99f);
}

// ============================================================================
// 4. ТЕСТЫ СПЛАЙСА MP12 (MP12Oprn): FSM и триплет поведения
// ============================================================================

TEST(MP12OprnTest, TripletAndBVector) {
    MP11Object subject("agent_bot");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("goal_cup");
    target.bounding_box.center = Eigen::Vector3f(1.5f, 2.0f, 0.5f);

    MP12Oprn oprn("oprn_1");
    EXPECT_EQ(oprn.status, OprnStatus::PENDING);
    EXPECT_TRUE(oprn.has_open_valences()); // 2 валентности открыты

    // Расчет b-вектора связывает субъекта и цель, закрывая валентности
    oprn.calculate_b_vector(subject, target);

    EXPECT_FLOAT_EQ(oprn.b_vector.x(), 1.5f);
    EXPECT_FLOAT_EQ(oprn.b_vector.y(), 2.0f);
    EXPECT_FLOAT_EQ(oprn.b_vector.z(), 0.5f);

    EXPECT_EQ(oprn.subject_ref, "agent_bot");
    EXPECT_EQ(oprn.target_ref, "goal_cup");
    EXPECT_FALSE(oprn.has_open_valences()); // Обе валентности закрыты!
    EXPECT_EQ(oprn.get_valence(0), "agent_bot");
    EXPECT_EQ(oprn.get_valence(1), "goal_cup");
}

TEST(MP12OprnTest, FSMTransitions) {
    MP12Oprn oprn("oprn_fsm");
    EXPECT_EQ(oprn.status, OprnStatus::PENDING);

    // Валидный переход: PENDING -> VALIDATED
    EXPECT_TRUE(oprn.can_transition_to(OprnStatus::VALIDATED));
    EXPECT_NO_THROW(oprn.transition_to(OprnStatus::VALIDATED));
    EXPECT_EQ(oprn.status, OprnStatus::VALIDATED);

    // Валидный переход: VALIDATED -> EXECUTED
    EXPECT_TRUE(oprn.can_transition_to(OprnStatus::EXECUTED));
    EXPECT_NO_THROW(oprn.transition_to(OprnStatus::EXECUTED));
    EXPECT_EQ(oprn.status, OprnStatus::EXECUTED);

    // Невалидный переход: из EXECUTED нельзя никуда перейти
    EXPECT_FALSE(oprn.can_transition_to(OprnStatus::PENDING));
    EXPECT_THROW(oprn.transition_to(OprnStatus::PENDING), std::logic_error);

    // Проверка ветки коллизии: PENDING -> COLLISION -> PENDING
    MP12Oprn oprn_col("oprn_col");
    EXPECT_NO_THROW(oprn_col.transition_to(OprnStatus::COLLISION));
    EXPECT_EQ(oprn_col.status, OprnStatus::COLLISION);

    // Из COLLISION нельзя сразу в EXECUTED
    EXPECT_FALSE(oprn_col.can_transition_to(OprnStatus::EXECUTED));
    EXPECT_THROW(oprn_col.transition_to(OprnStatus::EXECUTED), std::logic_error);

    // Из COLLISION можно вернуться в PENDING для повторной валидации
    EXPECT_TRUE(oprn_col.can_transition_to(OprnStatus::PENDING));
    EXPECT_NO_THROW(oprn_col.transition_to(OprnStatus::PENDING));
    EXPECT_EQ(oprn_col.status, OprnStatus::PENDING);
}

TEST(MP12OprnTest, ProtobufRoundTrip) {
    MP12Oprn orig("oprn_proto", "SUBJ-1", "TARG-2");
    orig.b_vector = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    orig.transition_to(OprnStatus::VALIDATED);

    gtr1::models::MP12Oprn proto;
    orig.to_proto(proto);

    EXPECT_EQ(proto.base().uuid(), "oprn_proto");
    EXPECT_EQ(proto.subject_ref(), "SUBJ-1");
    EXPECT_EQ(proto.target_ref(), "TARG-2");
    EXPECT_FLOAT_EQ(proto.b_vector().x(), 1.0f);
    EXPECT_EQ(proto.status(), gtr1::models::VALIDATED);

    MP12Oprn restored = MP12Oprn::from_proto(proto);
    EXPECT_EQ(restored.uuid(), orig.uuid());
    EXPECT_EQ(restored.subject_ref, "SUBJ-1");
    EXPECT_EQ(restored.target_ref, "TARG-2");
    EXPECT_FLOAT_EQ(restored.b_vector.z(), 3.0f);
    EXPECT_EQ(restored.status, OprnStatus::VALIDATED);
}

// ============================================================================
// 5. ТЕСТЫ СХЕМЫ СОБЫТИЙ MP13 (MP13Blom)
// ============================================================================

TEST(MP13BlomTest, SequenceAndOutcomeMarker) {
    MP13Blom blom("blom_route_1");
    EXPECT_FALSE(blom.is_complete);
    EXPECT_FLOAT_EQ(blom.get_outcome_marker(), 0.0f);

    blom.append_oprn("OPRN-STEP-1");
    blom.append_oprn("OPRN-STEP-2");
    blom.append_oprn("OPRN-STEP-3");

    ASSERT_EQ(blom.oprn_sequence.size(), 3);
    EXPECT_EQ(blom.oprn_sequence[0], "OPRN-STEP-1");
    EXPECT_EQ(blom.oprn_sequence[2], "OPRN-STEP-3");
    EXPECT_EQ(blom.total_valences_count(), 3);
    EXPECT_FALSE(blom.has_open_valences());

    // Клапминг маркера исхода
    blom.set_outcome_marker(1.5f);
    EXPECT_FLOAT_EQ(blom.get_outcome_marker(), 1.0f);

    blom.set_outcome_marker(-0.5f);
    EXPECT_FLOAT_EQ(blom.get_outcome_marker(), 0.0f);

    blom.set_outcome_marker(0.92f);
    EXPECT_FLOAT_EQ(blom.get_outcome_marker(), 0.92f);

    blom.is_complete = true;
    EXPECT_TRUE(blom.is_complete);
}

TEST(MP13BlomTest, ProtobufRoundTrip) {
    MP13Blom orig("blom_test");
    orig.append_oprn("OPRN-A");
    orig.append_oprn("OPRN-B");
    orig.is_complete = true;
    orig.set_outcome_marker(0.85f);

    gtr1::models::MP13Blom proto;
    orig.to_proto(proto);

    EXPECT_EQ(proto.base().uuid(), "blom_test");
    EXPECT_TRUE(proto.is_complete());
    EXPECT_FLOAT_EQ(proto.outcome_marker(), 0.85f);
    ASSERT_EQ(proto.oprn_sequence_size(), 2);
    EXPECT_EQ(proto.oprn_sequence(0), "OPRN-A");
    EXPECT_EQ(proto.oprn_sequence(1), "OPRN-B");

    MP13Blom restored = MP13Blom::from_proto(proto);
    EXPECT_EQ(restored.uuid(), "blom_test");
    EXPECT_TRUE(restored.is_complete);
    EXPECT_FLOAT_EQ(restored.get_outcome_marker(), 0.85f);
    ASSERT_EQ(restored.oprn_sequence.size(), 2);
}

// ============================================================================
// 6. ТЕСТЫ ХРАНИЛИЩА ТРАЕКТОРИЙ (TRL1Log)
// ============================================================================

TEST(TRL1LogTest, RecordQueryAndUpdate) {
    TRL1Log log;
    EXPECT_EQ(log.size(), 0);

    MP13Blom traj1("traj_cup_1");
    traj1.append_oprn("OPRN-1");
    traj1.set_outcome_marker(0.6f);
    log.record_trajectory(traj1, "cup");

    MP13Blom traj2("traj_cup_2");
    traj2.append_oprn("OPRN-2");
    traj2.set_outcome_marker(0.95f);
    log.record_trajectory(traj2, "cup");

    MP13Blom traj3("traj_bottle");
    traj3.append_oprn("OPRN-3");
    traj3.set_outcome_marker(0.85f);
    log.record_trajectory(traj3, "bottle");

    EXPECT_EQ(log.size(), 3);

    // Поиск лучшей траектории с порогом 0.8
    auto best = log.find_best_trajectory(0.8f);
    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->uuid(), "traj_cup_2");
    EXPECT_FLOAT_EQ(best->get_outcome_marker(), 0.95f);

    // Поиск по цели "cup"
    auto cup_trajs = log.find_trajectories_by_goal("cup", 0.7f);
    ASSERT_EQ(cup_trajs.size(), 1);
    EXPECT_EQ(cup_trajs[0].uuid(), "traj_cup_2");

    // Обновление маркера
    EXPECT_TRUE(log.update_outcome("traj_cup_1", 0.88f));
    auto updated = log.get_trajectory("traj_cup_1");
    ASSERT_TRUE(updated.has_value());
    EXPECT_FLOAT_EQ(updated->get_outcome_marker(), 0.88f);
}

// ============================================================================
// 7. ТЕСТЫ ОТВЯЗКИ ОПЕРАТОРОВ ОТ PROTOBUF DTO (SPL1 и PRD1 на чистых типах)
// ============================================================================

TEST(OperatorDecouplingTest, SPL1OperatesOnDomainTypes) {
    auto space = std::make_shared<MP10Space>("test_space");
    Spl1Splicer splicer(space);

    MP11Object subject("agent_bot");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("target_object");
    target.bounding_box.center = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    // Вызываем чисто доменный метод
    std::shared_ptr<MP12Oprn> domain_splice = splicer.form_splice(subject, target);
    ASSERT_NE(domain_splice, nullptr);
    EXPECT_EQ(domain_splice->status, OprnStatus::VALIDATED);
    EXPECT_FLOAT_EQ(domain_splice->b_vector.x(), 1.0f);
    EXPECT_EQ(domain_splice->subject_ref, "agent_bot");
    EXPECT_EQ(domain_splice->target_ref, "target_object");
}

TEST(OperatorDecouplingTest, PRD1OperatesOnDomainTypesAndTRL1) {
    auto space = std::make_shared<MP10Space>("test_space");
    auto log = std::make_shared<TRL1Log>();
    Prd1Predictor predictor(space, log);

    std::vector<std::shared_ptr<MP12Oprn>> splices;
    Eigen::Vector3f start(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f goal(1.0f, 0.0f, 0.0f);

    // Вызываем чисто доменный метод A* поиска
    std::shared_ptr<MP13Blom> domain_blom = predictor.build_event_chain("cup_goal", start, goal, splices);
    ASSERT_NE(domain_blom, nullptr);
    EXPECT_TRUE(domain_blom->is_complete);
    EXPECT_GT(domain_blom->oprn_sequence.size(), 1);

    // Сохраняем в TRL1Log
    predictor.save_to_log(domain_blom);
    EXPECT_EQ(log->size(), 1);

    auto best = log->find_best_trajectory(0.8f);
    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(best->uuid(), domain_blom->uuid());
}