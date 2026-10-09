#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>

#include "mp10_space.hpp"
#include "mp11_manager.hpp"
#include "mtv1_motivator.hpp"
#include "trl1_log.hpp"
#include "spl1_splicer.hpp"
#include "prd1_predictor.hpp"
#include "slgw_node.hpp"
#include "dataset_player.hpp"
#include "trace_logger.hpp"

using namespace gtr1::core;
using namespace gtr1::operators;
using namespace gtr1::infra;

TEST(GoldenDatasetTest, RegressionRunFr1Desk) {
    std::string dataset_path = "../07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
    std::ifstream check_file(dataset_path + "/associations_gt.txt");
    if (!check_file.is_open()) {
        dataset_path = "07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
    }

    // 1. Инициализация всего ядра
    auto mp10 = std::make_shared<MP10Space>("golden_scene", 0.05);
    auto mp11 = std::make_shared<MP11Manager>();
    auto trl1 = std::make_shared<TRL1Log>();
    auto slgw = std::make_shared<SlgwNode>(mp10, mp11, "tcp://127.0.0.1:5555", false);
    auto splicer = std::make_shared<Spl1Splicer>(mp10);
    auto prd1 = std::make_shared<Prd1Predictor>(mp10, trl1);
    auto motivator = std::make_shared<Mtv1Motivator>();

    // 2. Инжекция целевой потребности
    motivator->inject_goal("cup", 0.95f);
    EXPECT_TRUE(motivator->has_active_goal());
    EXPECT_EQ(motivator->get_target_class(), "cup");

    // 3. Воспроизведение датасета TUM fr1_desk (60 кадров для детерминированного регрессионного прогона)
    DatasetPlayer player(dataset_path, slgw, 1000.0);
    bool loaded = player.load_associations("associations_gt.txt");
    ASSERT_TRUE(loaded);

    size_t frames_played = 0;
    player.play(60, false, [&](size_t idx, double) {
        frames_played++;
        std::string trace_id = TraceLogger::generate_trace_id(idx);
        if (idx == 0) {
            GTR_TRACE(trace_id, "GOLDEN", "Regression stream started.");
        }
    });

    EXPECT_EQ(frames_played, 60);

    // 4. Верификация вокселизации MP10
    auto voxels = mp10->get_occupied_voxels();
    EXPECT_GT(voxels.size(), 1000) << "MP10 OctoMap must contain reconstructed voxels";

    // 5. Проверка локализации агента
    Eigen::Matrix4f pose = mp10->get_pose();
    EXPECT_FALSE(pose.isIdentity());

    // 6. Формирование цели и сплайса
    Eigen::Vector3f target_pos = pose.block<3,1>(0,3) + Eigen::Vector3f(0.8f, 0.0f, 0.0f);
    motivator->set_target_position(target_pos);

    MP11Object subject("agent_base");
    subject.bounding_box.center = pose.block<3,1>(0,3);

    MP11Object target("golden_cup");
    target.class_label_id = "cup";
    target.bounding_box.center = target_pos;
    mp11->add_or_merge_invariant(target);

    // 7. Сплайсер и предиктор A*
    auto splice = splicer->form_splice(subject, target);
    ASSERT_NE(splice, nullptr);

    std::vector<std::shared_ptr<MP12Oprn>> splices = { splice };
    auto blom = prd1->build_event_chain(target.uuid(),
                                       subject.bounding_box.center,
                                       target_pos,
                                       splices,
                                       "cup");

    ASSERT_NE(blom, nullptr);
    EXPECT_TRUE(blom->is_complete);
    EXPECT_GE(blom->get_outcome_marker(), 0.8f);

    // 8. Дистанция обнуления и завершение цели
    float d_zero = motivator->calculate_distance_to_zero(subject.bounding_box.center);
    EXPECT_NEAR(d_zero, 0.8f, 1e-3);

    // Эмуляция перемещения к цели
    Eigen::Vector3f final_agent_pos = target_pos - Eigen::Vector3f(0.05f, 0.0f, 0.0f);
    float d_contact = motivator->calculate_distance_to_zero(final_agent_pos);
    EXPECT_LE(d_contact, 0.1f);
    EXPECT_TRUE(motivator->is_goal_achieved(final_agent_pos, 0.2f));

    // 9. Сохранение эталонного графа в TRL1Log
    blom->set_outcome_marker(0.95f);
    prd1->save_to_log(blom, "cup");
    EXPECT_EQ(trl1->size(), 1);

    // 10. Экспорт эталонного артефакта golden_fr1_desk.json
    std::ofstream golden_out("golden_fr1_desk.json");
    if (golden_out.is_open()) {
        golden_out << "{\n";
        golden_out << "  \"scene\": \"fr1_desk\",\n";
        golden_out << "  \"frames_processed\": " << frames_played << ",\n";
        golden_out << "  \"occupied_voxels\": " << voxels.size() << ",\n";
        golden_out << "  \"blom_uuid\": \"" << blom->uuid() << "\",\n";
        golden_out << "  \"oprn_count\": " << blom->oprn_sequence.size() << ",\n";
        golden_out << "  \"outcome_marker\": " << blom->get_outcome_marker() << ",\n";
        golden_out << "  \"goal_class\": \"cup\"\n";
        golden_out << "}\n";
        golden_out.close();
    }

    std::ifstream golden_in("golden_fr1_desk.json");
    ASSERT_TRUE(golden_in.is_open());
}