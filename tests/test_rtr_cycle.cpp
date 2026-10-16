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

// 7. Проверка смещения якорной точки на высоту пола (Z = 0.20м) для цели на столе (Z = 0.65м)
TEST(RTRCycleTest, FloorAnchorPointPlanningForElevatedTarget) {
    auto mp10 = std::make_shared<MP10Space>("floor_anchor_scene");
    auto trl1 = std::make_shared<TRL1Log>();
    Prd1Predictor prd1(mp10, trl1);

    // Робот на полу: Z = 0.20м
    Eigen::Vector3f robot_floor_pos(0.0f, 0.0f, 0.20f);
    // Предмет (кружка) на столе: Z = 0.65м
    Eigen::Vector3f table_cup_pos(1.6f, 0.0f, 0.65f);

    std::vector<std::shared_ptr<MP12Oprn>> empty_splices;
    auto blom = prd1.build_event_chain("elevated_cup_01", robot_floor_pos, table_cup_pos, empty_splices, "cup");

    ASSERT_NE(blom, nullptr);
    EXPECT_TRUE(blom->is_complete);
    EXPECT_FALSE(blom->oprn_items.empty());

    // Все шаги перемещения шасси робота должны оставаться на высоте пола (Z <= 0.35м)
    Eigen::Vector3f wp = robot_floor_pos;
    for (const auto& step : blom->oprn_items) {
        ASSERT_NE(step, nullptr);
        wp += step->b_vector;
        EXPECT_LE(wp.z(), 0.35f);
    }
}

// 8. Критерий закрытия цели BLOM и автосохранение в TRL1Log
TEST(RTRCycleTest, BlomGoalClosureAndTRL1Persistence) {
    auto trl1 = std::make_shared<TRL1Log>();

    Eigen::Vector3f target_pos(2.0f, 1.0f, 0.20f);
    Eigen::Vector3f far_pos(0.0f, 0.0f, 0.20f);
    Eigen::Vector3f near_pos(1.95f, 0.95f, 0.20f);

    // Проверка критерия расстояния
    EXPECT_FALSE(MP13Blom::is_goal_achieved(far_pos, target_pos, 0.25f));
    EXPECT_TRUE(MP13Blom::is_goal_achieved(near_pos, target_pos, 0.25f));

    auto blom = std::make_shared<MP13Blom>("blom_nav_closure");
    auto step1 = std::make_shared<MP12Oprn>("step1", "agent_base", "wp1");
    step1->b_vector = Eigen::Vector3f(1.0f, 0.5f, 0.0f);
    step1->transition_to(OprnStatus::VALIDATED);
    blom->append_oprn(step1);

    auto step2 = std::make_shared<MP12Oprn>("step2", "wp1", "manual_waypoint");
    step2->b_vector = Eigen::Vector3f(1.0f, 0.5f, 0.0f);
    step2->transition_to(OprnStatus::VALIDATED);
    blom->append_oprn(step2);

    // Закрытие цели как успешной
    blom->close_as_successful(1.0f);

    EXPECT_TRUE(blom->is_complete);
    EXPECT_FLOAT_EQ(blom->get_outcome_marker(), 1.0f);
    EXPECT_EQ(step1->status, OprnStatus::EXECUTED);
    EXPECT_EQ(step2->status, OprnStatus::EXECUTED);

    // Сохранение в долгосрочный лог TRL1
    trl1->record_trajectory(*blom, "manual_waypoint");
    auto found = trl1->find_trajectories_by_goal("manual_waypoint", 0.95f);
    ASSERT_EQ(found.size(), 1);
    EXPECT_EQ(found[0].uuid(), blom->uuid());
    EXPECT_FLOAT_EQ(found[0].get_outcome_marker(), 1.0f);
}

// 9. Пошаговый FSM-трекинг OPRN (VALIDATED -> EXECUTED)
TEST(RTRCycleTest, StepByStepOprnFsmTracking) {
    auto blom = std::make_shared<MP13Blom>("blom_tracking_test");
    Eigen::Vector3f start_pos(0.0f, 0.0f, 0.20f);

    auto op1 = std::make_shared<MP12Oprn>("step_1", "agent", "wp1");
    op1->b_vector = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
    op1->transition_to(OprnStatus::VALIDATED);
    blom->append_oprn(op1);

    auto op2 = std::make_shared<MP12Oprn>("step_2", "wp1", "wp2");
    op2->b_vector = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
    op2->transition_to(OprnStatus::VALIDATED);
    blom->append_oprn(op2);

    auto op3 = std::make_shared<MP12Oprn>("step_3", "wp2", "wp3");
    op3->b_vector = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
    op3->transition_to(OprnStatus::VALIDATED);
    blom->append_oprn(op3);

    // 1. Робот достиг WP1 (0.5, 0, 0.2)
    Eigen::Vector3f pos1(0.48f, 0.02f, 0.20f);
    int next_idx = blom->update_oprn_progress(pos1, start_pos, 0.25f);
    EXPECT_EQ(op1->status, OprnStatus::EXECUTED);
    EXPECT_EQ(op2->status, OprnStatus::VALIDATED);
    EXPECT_EQ(op3->status, OprnStatus::VALIDATED);
    EXPECT_EQ(next_idx, 1); // Следующий активный шаг - индекс 1 (op2)

    // 2. Робот достиг WP2 (1.0, 0, 0.2)
    Eigen::Vector3f pos2(0.99f, 0.0f, 0.20f);
    next_idx = blom->update_oprn_progress(pos2, start_pos, 0.25f);
    EXPECT_EQ(op1->status, OprnStatus::EXECUTED);
    EXPECT_EQ(op2->status, OprnStatus::EXECUTED);
    EXPECT_EQ(op3->status, OprnStatus::VALIDATED);
    EXPECT_EQ(next_idx, 2); // Следующий шаг - индекс 2 (op3)

    auto executed = blom->get_executed_oprns();
    EXPECT_EQ(executed.size(), 2);
}

// 10. Динамическое локальное перепланирование с сохранением преемственности маршрута
TEST(RTRCycleTest, DynamicLocalReplanningPreservesExecutedHistory) {
    auto mp10 = std::make_shared<MP10Space>("replan_scene");
    auto trl1 = std::make_shared<TRL1Log>();
    Prd1Predictor prd1(mp10, trl1);

    // Исходный BLOM с уже выполненным первым шагом
    auto initial_blom = std::make_shared<MP13Blom>("blom_initial");
    auto executed_step = std::make_shared<MP12Oprn>("step_1", "agent_base", "waypoint_1");
    executed_step->b_vector = Eigen::Vector3f(0.4f, 0.0f, 0.0f);
    executed_step->transition_to(OprnStatus::VALIDATED);
    executed_step->transition_to(OprnStatus::EXECUTED);
    initial_blom->append_oprn(executed_step);

    // Ставим стену между текущей точкой и целью
    build_test_wall(mp10, 1.0f, 0.4f, 0.4f);

    Eigen::Vector3f current_pos(0.4f, 0.0f, 0.0f);
    Eigen::Vector3f target_pos(2.0f, 0.0f, 0.0f);

    auto replanned_blom = prd1.replan_from_current(initial_blom, current_pos, target_pos, "goal_cup", "cup");

    ASSERT_NE(replanned_blom, nullptr);
    EXPECT_TRUE(replanned_blom->is_complete);
    EXPECT_GT(replanned_blom->oprn_items.size(), 1);

    // Первый шаг должен быть сохранен как EXECUTED из истории
    EXPECT_EQ(replanned_blom->oprn_items[0]->status, OprnStatus::EXECUTED);
    EXPECT_FLOAT_EQ(replanned_blom->oprn_items[0]->b_vector.x(), 0.4f);

    // Последующие шаги должны быть валидированы и обходить препятствие
    for (size_t i = 1; i < replanned_blom->oprn_items.size(); ++i) {
        EXPECT_EQ(replanned_blom->oprn_items[i]->status, OprnStatus::VALIDATED);
    }
}

// 11. Проверка предотвращения проезда под столом благодаря учету габаритов робота (Clearance Envelope)
TEST(RTRCycleTest, TableClearancePreventsPlanningUnderTable) {
    auto mp10 = std::make_shared<MP10Space>("table_clearance_scene");
    auto trl1 = std::make_shared<TRL1Log>();
    Prd1Predictor prd1(mp10, trl1, 0.22f, 0.75f);
    Spl1Splicer splicer(mp10, 0.75f);

    // Строим нависающую столешницу на X in [0.8, 1.2], Y in [-0.45, 0.45], высота Z in [0.55, 0.65]
    // Пространство под столом (Z in [0.0, 0.40]) полностью свободно!
    for (float x = 0.8f; x <= 1.2f; x += 0.05f) {
        for (float y = -0.45f; y <= 0.45f; y += 0.05f) {
            for (float z = 0.55f; z <= 0.65f; z += 0.05f) {
                mp10->set_occupied(Eigen::Vector3f(x, y, z), true);
            }
        }
    }

    MP11Object subject("robot_base");
    subject.bounding_box.center = Eigen::Vector3f(0.0f, 0.0f, 0.20f);

    MP11Object target("goal_behind_table");
    target.bounding_box.center = Eigen::Vector3f(2.0f, 0.0f, 0.20f);

    // 1. Сплайсер проверяет верхний габаритный луч: под столом проехать нельзя -> COLLISION
    auto direct_splice = splicer.form_splice(subject, target);
    EXPECT_EQ(direct_splice->status, OprnStatus::COLLISION);

    // 2. Предиктор строит обходной маршрут A* с учетом высоты 0.75м
    std::vector<std::shared_ptr<MP12Oprn>> splices = { direct_splice };
    auto blom = prd1.build_event_chain(target.uuid(),
                                       subject.bounding_box.center,
                                       target.bounding_box.center,
                                       splices);

    ASSERT_NE(blom, nullptr);
    EXPECT_TRUE(blom->is_complete);

    // Маршрут должен обогнуть стол сбоку, а не проезжать под столешницей через центр Y = 0.0
    Eigen::Vector3f wp = subject.bounding_box.center;
    bool went_around_y = false;
    for (const auto& step : blom->oprn_items) {
        ASSERT_NE(step, nullptr);
        wp += step->b_vector;
        if (wp.x() >= 0.8f && wp.x() <= 1.2f) {
            // В зоне X стола робот должен сместиться по Y за пределы столешницы
            if (std::abs(wp.y()) > 0.45f) {
                went_around_y = true;
            }
        }
    }
    EXPECT_TRUE(went_around_y);
}
