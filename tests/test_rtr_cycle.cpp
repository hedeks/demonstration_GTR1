#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "mp10_space.hpp"
#include "mp11_object.hpp"
#include "mp12_oprn.hpp"
#include "mp13_blom.hpp"
#include "mtv1_motivator.hpp"
#include "trl1_log.hpp"
#include "spl1_splicer.hpp"
#include "prd1_predictor.hpp"

using namespace gtr1::core;
using namespace gtr1::operators;

namespace {

/**
 * @brief Создает стену из занятых вокселей в MP10Space на плоскости X = wall_x.
 */
void build_test_wall(std::shared_ptr<MP10Space> space, float wall_x, float y_span, float z_span) {
    for (float y = -y_span; y <= y_span; y += 0.05f) {
        for (float z = -z_span; z <= z_span; z += 0.05f) {
            space->set_occupied(Eigen::Vector3f(wall_x, y, z), true);
        }
    }
}

} // namespace

// 1. DoD 1: При наличии стены между субъектом и целью сплайсер выставляет status = COLLISION
TEST(RTRCycleTest, SplicerDetectsCollisionWhenWallPresent) {
    auto mp10 = std::make_shared<MP10Space>("collision_scene");
    // Строим стену на X = 1.0м
    build_test_wall(mp10, 1.0f, 0.6f, 0.6f);

    Spl1Splicer splicer(mp10);

    MP11Object subject("robot_base");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("target_cup");
    target.class_label_id = "cup";
    target.bounding_box.center = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    auto splice = splicer.form_splice(subject, target);

    ASSERT_NE(splice, nullptr);
    EXPECT_EQ(splice->status, OprnStatus::COLLISION);
    EXPECT_EQ(splice->subject_ref, subject.uuid());
    EXPECT_EQ(splice->target_ref, target.uuid());
    EXPECT_FALSE(splice->is_valence_open(0)); // Валентность субъекта закрыта
    EXPECT_FALSE(splice->is_valence_open(1)); // Валентность объекта закрыта
    EXPECT_EQ(splice->get_valence(0), subject.uuid());
    EXPECT_EQ(splice->get_valence(1), target.uuid());
    EXPECT_NEAR(splice->b_vector.x(), 2.0f, 1e-4);
}

// 2. Сплайсер переходит в VALIDATED при отсутствии преград
TEST(RTRCycleTest, SplicerValidatesWhenPathIsFree) {
    auto mp10 = std::make_shared<MP10Space>("free_scene");
    Spl1Splicer splicer(mp10);

    MP11Object subject("robot_base");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("target_cup");
    target.class_label_id = "cup";
    target.bounding_box.center = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    auto splice = splicer.form_splice(subject, target);

    ASSERT_NE(splice, nullptr);
    EXPECT_EQ(splice->status, OprnStatus::VALIDATED);
    EXPECT_NEAR(splice->b_vector.norm(), 2.0f, 1e-4);
}

// 3. DoD 2: Предиктор A* строит валидный многозвенный граф в обход препятствия
TEST(RTRCycleTest, AStarBypassesObstacleWithSafetyMargin) {
    auto mp10 = std::make_shared<MP10Space>("bypass_scene");
    // Стена на X = 1.0м, Y от -0.35 до +0.35
    build_test_wall(mp10, 1.0f, 0.35f, 0.35f);

    Spl1Splicer splicer(mp10);
    Prd1Predictor prd1(mp10);

    MP11Object subject("robot_base");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("target_cup");
    target.class_label_id = "cup";
    target.bounding_box.center = Eigen::Vector3f(2.0f, 0.0f, 0.0f);

    // Прямой сплайс наталкивается на стену
    auto direct_splice = splicer.form_splice(subject, target);
    ASSERT_EQ(direct_splice->status, OprnStatus::COLLISION);

    // RTR обратный ход: планирование обходного пути
    std::vector<std::shared_ptr<MP12Oprn>> splices = { direct_splice };
    auto blom = prd1.build_event_chain(target.uuid(),
                                       subject.bounding_box.center,
                                       target.bounding_box.center,
                                       splices,
                                       "cup");

    ASSERT_NE(blom, nullptr);
    EXPECT_TRUE(blom->is_complete);
    EXPECT_GT(blom->oprn_sequence.size(), 1); // Многозвенный граф
    EXPECT_GE(blom->get_outcome_marker(), 0.8f);
}

// 4. DoD 3: Успешно пройденный маршрут сохраняется в TRL1Log и повторно используется
TEST(RTRCycleTest, TRL1LogCachingAndReuse) {
    auto mp10 = std::make_shared<MP10Space>("caching_scene");
    auto trl1 = std::make_shared<TRL1Log>();
    Prd1Predictor prd1(mp10, trl1);

    // 1. Создаем успешный граф маршрута
    auto successful_blom = std::make_shared<MP13Blom>("blom_expert_route");
    successful_blom->append_oprn("step_1");
    successful_blom->append_oprn("step_2");
    successful_blom->append_oprn("step_3");
    successful_blom->is_complete = true;
    successful_blom->set_outcome_marker(0.95f); // > 0.8

    // Сохраняем в лог
    prd1.save_to_log(successful_blom, "cup");
    EXPECT_EQ(trl1->size(), 1);

    // 2. Запрашиваем планирование цели того же класса "cup"
    Eigen::Vector3f start(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f goal(3.0f, 0.0f, 0.0f);
    std::vector<std::shared_ptr<MP12Oprn>> empty_splices;
    auto retrieved_blom = prd1.build_event_chain("new_cup_uuid", start, goal, empty_splices, "cup");

    ASSERT_NE(retrieved_blom, nullptr);
    // Должен мгновенно вернуть кэшированный граф из лога опыта
    EXPECT_EQ(retrieved_blom->uuid(), "blom_expert_route");
    EXPECT_EQ(retrieved_blom->oprn_sequence.size(), 3);
    EXPECT_NEAR(retrieved_blom->get_outcome_marker(), 0.95f, 1e-4);

    // 3. Маршрут с низким маркером (< 0.8) игнорируется
    auto bad_blom = std::make_shared<MP13Blom>("blom_bad_route");
    bad_blom->append_oprn("bad_step");
    bad_blom->is_complete = true;
    bad_blom->set_outcome_marker(0.35f); // < 0.8
    trl1->record_trajectory(*bad_blom, "book");

    auto result_for_book = prd1.build_event_chain("book_uuid", start, goal, empty_splices, "book");
    // Не должен использовать bad_route (outcome < 0.8), строит собственный путь
    EXPECT_NE(result_for_book->uuid(), "blom_bad_route");
}

// 5. Мотиватор: инжекция цели и расчет дистанции обнуления
TEST(RTRCycleTest, MotivatorGoalInjectionAndZeroingDistance) {
    Mtv1Motivator motivator;

    Eigen::Vector3f target_pos(2.0f, 0.0f, 0.0f);
    motivator.inject_goal("cup", target_pos, 1.0f);

    EXPECT_TRUE(motivator.has_active_goal());
    EXPECT_EQ(motivator.get_target_class(), "cup");
    EXPECT_EQ(motivator.get_urgency(), 1.0f);

    // Начальная позиция агента (0,0,0) -> дистанция 2.0м
    Eigen::Vector3f pos0(0.0f, 0.0f, 0.0f);
    float d0 = motivator.calculate_distance_to_zero(pos0);
    EXPECT_NEAR(d0, 2.0f, 1e-4);
    EXPECT_FALSE(motivator.is_goal_achieved(pos0));

    // Агент подошел к (1.5, 0, 0) -> дистанция 0.5м
    Eigen::Vector3f pos1(1.5f, 0.0f, 0.0f);
    float d1 = motivator.calculate_distance_to_zero(pos1);
    EXPECT_NEAR(d1, 0.5f, 1e-4);
    EXPECT_FALSE(motivator.is_goal_achieved(pos1));

    // Агент достиг касания цели (1.95, 0, 0) -> дистанция 0.05м <= 0.2м
    Eigen::Vector3f pos_final(1.95f, 0.0f, 0.0f);
    float d_final = motivator.calculate_distance_to_zero(pos_final);
    EXPECT_NEAR(d_final, 0.05f, 1e-4);
    EXPECT_TRUE(motivator.is_goal_achieved(pos_final, 0.2f));

    // Обнуление цели
    motivator.zero_goal();
    EXPECT_FALSE(motivator.has_active_goal());
    EXPECT_EQ(motivator.get_urgency(), 0.0f);
}

// 6. Сквозной замкнутый цикл RTR: Потребность -> Коллизия -> A* -> Движение -> Касание -> Лог
TEST(RTRCycleTest, FullClosedRTRCycle) {
    auto mp10 = std::make_shared<MP10Space>("rtr_full_scene");
    auto trl1 = std::make_shared<TRL1Log>();
    Spl1Splicer splicer(mp10);
    Prd1Predictor prd1(mp10, trl1);
    Mtv1Motivator motivator;

    // Шаг 1: Инжекция цели в MTV1
    Eigen::Vector3f target_pos(2.0f, 0.0f, 0.0f);
    motivator.inject_goal("cup", target_pos, 1.0f);

    // Шаг 2: Установка преграды между стартом и целью
    build_test_wall(mp10, 1.0f, 0.35f, 0.35f);

    MP11Object subject("robot_base");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    MP11Object target("target_cup");
    target.class_label_id = "cup";
    target.bounding_box.center = target_pos;

    // Шаг 3: Проверка прямого сплайса -> COLLISION
    auto direct_splice = splicer.form_splice(subject, target);
    EXPECT_EQ(direct_splice->status, OprnStatus::COLLISION);

    // Шаг 4: Построение обходного графа A*
    std::vector<std::shared_ptr<MP12Oprn>> direct_splices = { direct_splice };
    auto planned_blom = prd1.build_event_chain(target.uuid(),
                                              subject.bounding_box.center,
                                              target_pos,
                                              direct_splices,
                                              motivator.get_target_class());
    ASSERT_TRUE(planned_blom->is_complete);
    EXPECT_GT(planned_blom->oprn_sequence.size(), 1);

    // Шаг 5: Эмуляция физического движения агента к цели и расчет дистанции обнуления
    Eigen::Vector3f current_pos = subject.bounding_box.center;
    float dist = motivator.calculate_distance_to_zero(current_pos);
    EXPECT_NEAR(dist, 2.0f, 1e-4);

    // Агент достигает окрестности цели
    current_pos = target_pos - Eigen::Vector3f(0.08f, 0.0f, 0.0f);
    dist = motivator.calculate_distance_to_zero(current_pos);
    EXPECT_LT(dist, 0.1f);
    EXPECT_TRUE(motivator.is_goal_achieved(current_pos, 0.15f));

    // Шаг 6: Фиксация успеха в TRL1Log
    planned_blom->set_outcome_marker(0.92f);
    prd1.save_to_log(planned_blom, motivator.get_target_class());
    motivator.zero_goal();
    EXPECT_FALSE(motivator.has_active_goal());

    // Шаг 7: Повторный запрос той же цели мгновенно извлекает опыт из TRL1Log
    std::vector<std::shared_ptr<MP12Oprn>> empty_splices;
    auto cached_blom = prd1.build_event_chain("cup_uuid_2",
                                              Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                              target_pos,
                                              empty_splices,
                                              "cup");
    EXPECT_EQ(cached_blom->uuid(), planned_blom->uuid());
    EXPECT_GT(cached_blom->get_outcome_marker(), 0.8f);
}