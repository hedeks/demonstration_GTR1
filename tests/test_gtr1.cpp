#include <gtest/gtest.h>
#include <memory>
#include <Eigen/Dense>

#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/mp11_manager.hpp"
#include "../src/operators/slgw_node.hpp"
#include "../src/operators/spl1_splicer.hpp"
#include "../src/operators/prd1_predictor.hpp"
#include "gtr1_structures.pb.h"

// REQ-01: Слияние инвариантов (MP11)
TEST(GTR1_Tests, MP11_MergeInvariants) {
    auto mp11 = std::make_shared<gtr1::core::MP11Manager>();
    
    gtr1::ipc::DetectionResponse resp;
    // Объект 1: чашка на (0,0,0)
    auto* obj1 = resp.add_detected_objects();
    obj1->set_class_label_id("cup");
    obj1->mutable_bounding_box()->mutable_center()->set_x(0);
    obj1->mutable_bounding_box()->mutable_center()->set_y(0);
    obj1->mutable_bounding_box()->mutable_center()->set_z(0);

    // Объект 2: чашка на (0.1, 0, 0) - должна слиться (ближе 0.5м)
    auto* obj2 = resp.add_detected_objects();
    obj2->set_class_label_id("cup");
    obj2->mutable_bounding_box()->mutable_center()->set_x(0.1);
    obj2->mutable_bounding_box()->mutable_center()->set_y(0);
    obj2->mutable_bounding_box()->mutable_center()->set_z(0);

    mp11->process_detections(resp);

    // Должен остаться только 1 объект после слияния
    const auto& invariants = mp11->get_active_invariants();
    ASSERT_EQ(invariants.size(), 1);
}

// REQ-02: SPL1 Raycast Валидация
TEST(GTR1_Tests, SPL1_RaycastValidation) {
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("test_map");
    gtr1::operators::Spl1Splicer splicer(mp10);

    gtr1::models::MP11Object subject, target;
    subject.mutable_bounding_box()->mutable_center()->set_x(0);
    target.mutable_bounding_box()->mutable_center()->set_x(1.0);

    // В пустом пространстве луч должен пройти насквозь
    auto splice = splicer.form_splice(subject, target);
    ASSERT_EQ(splice->status(), gtr1::models::VALIDATED);
}

// REQ-03: PRD1 A* Поиск маршрута
TEST(GTR1_Tests, PRD1_AStarRouting) {
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("test_map");
    gtr1::operators::Prd1Predictor prd1(mp10);

    // Эмуляция отсутствия прямых сплайсов
    std::vector<std::shared_ptr<gtr1::models::MP12Oprn>> empty_splices;
    
    Eigen::Vector3f start(0,0,0);
    Eigen::Vector3f goal(1.0, 0, 0);

    auto blom = prd1.build_event_chain("target_cup", start, goal, empty_splices);
    
    // Граф должен сгенерироваться (построиться путь из нескольких шагов/waypoint)
    ASSERT_TRUE(blom->is_complete());
    ASSERT_GT(blom->oprn_sequence_size(), 1); 
}

// REQ-04: ZMQ Timeout (Recovery)
TEST(GTR1_Tests, SLGW_ZmqTimeout) {
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("test_map");
    auto mp11 = std::make_shared<gtr1::core::MP11Manager>();
    gtr1::operators::SlgwNode slgw(mp10, mp11, "tcp://127.0.0.1:5591", false);
    
    cv::Mat fake_rgb(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
    
    auto start_time = std::chrono::steady_clock::now();
    slgw.send_to_cnn_sync(0.0, fake_rgb);
    auto end_time = std::chrono::steady_clock::now();
    
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    // ZMQ таймаут настроен на 200мс. Время должно быть около 200мс.
    ASSERT_GE(elapsed_ms, 180.0);
    ASSERT_LT(elapsed_ms, 500.0);
}