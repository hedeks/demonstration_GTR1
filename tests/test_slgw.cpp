#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <numeric>
#include <cmath>
#include <fstream>

#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/mp11_manager.hpp"
#include "../src/core/include/octomap_manager.hpp"
#include "../src/operators/slgw_node.hpp"
#include "../07_infrastructure/dataset_player.hpp"

using namespace gtr1::core;
using namespace gtr1::operators;
using namespace gtr1::infra;

// ============================================================================
// REQ-04 / NFR-2.4: Падение ML сервиса не блокирует C++ ядро (ZMQ Timeout 200ms)
// ============================================================================
TEST(SlgwTest, ZmqTimeoutRecovery) {
    auto mp10 = std::make_shared<MP10Space>("test_map", 0.05);
    auto mp11 = std::make_shared<MP11Manager>();
    // Отключаем фоновый воркер, чтобы тестировать синхронный вызов таймаута
    SlgwNode slgw(mp10, mp11, "tcp://127.0.0.1:5591", false);

    cv::Mat fake_rgb(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

    auto start_time = std::chrono::steady_clock::now();
    bool success = slgw.send_to_cnn_sync(0.0, fake_rgb);
    auto end_time = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    EXPECT_FALSE(success);
    // ZMQ таймаут настроен на 200 мс (допустимый интервал 180 - 450 мс)
    EXPECT_GE(elapsed_ms, 180.0);
    EXPECT_LT(elapsed_ms, 500.0);
}

// ============================================================================
// NFR-9.1: Задержка обновления воксельной карты не превышает 33 мс (< 33ms)
// ============================================================================
TEST(SlgwTest, OctoMapVoxelizationPerformance) {
    auto mp10 = std::make_shared<MP10Space>("bench_map", 0.05);
    OctoMapManager octo_mgr(mp10, 517.3f, 516.5f, 318.6f, 255.3f, 5000.0f, 3);

    // Синтетический реалистичный кадр глубины 640x480 (плоскость на расстоянии 1.5м + шум)
    cv::Mat realistic_depth(480, 640, CV_16UC1);
    for (int r = 0; r < 480; ++r) {
        for (int c = 0; c < 640; ++c) {
            uint16_t depth_val = static_cast<uint16_t>(1500 + (r % 20) * 10 + (c % 20) * 5);
            realistic_depth.at<uint16_t>(r, c) = depth_val;
        }
    }

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    // Прогрев (1 итерация)
    octo_mgr.update_from_depth(realistic_depth, pose);

    // Замер задержки на 30 кадрах
    std::vector<double> latencies;
    for (int i = 0; i < 30; ++i) {
        pose(0, 3) = i * 0.02f; // Смещение камеры по X
        auto t0 = std::chrono::steady_clock::now();
        octo_mgr.update_from_depth(realistic_depth, pose);
        auto t1 = std::chrono::steady_clock::now();
        latencies.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

    double avg_latency = std::accumulate(latencies.begin(), latencies.end(), 0.0) / latencies.size();
    std::cout << "[PERF] Average OctoMap update latency: " << avg_latency << " ms per frame.\n";

    // Строгое требование NFR-9.1: задержка вокселизации должна быть < 33 мс
    EXPECT_LT(avg_latency, 33.0);
}

// ============================================================================
// REQ-02 / Sprint 2 DoD: Raycast безошибочно фиксирует преграду и 3D координату
// ============================================================================
TEST(SlgwTest, RaycastObstacleDetection) {
    auto mp10 = std::make_shared<MP10Space>("raycast_scene", 0.05);

    // Создаем препятствие (стену) в воксельной сетке на X = 1.2м
    octomap::Pointcloud wall_cloud;
    for (float y = -0.5f; y <= 0.5f; y += 0.05f) {
        for (float z = -0.5f; z <= 0.5f; z += 0.05f) {
            wall_cloud.push_back(1.2f, y, z);
        }
    }
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);
    mp10->occupancy_grid->insertPointCloud(wall_cloud, sensor_origin);

    Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f target_blocked(2.0f, 0.0f, 0.0f);
    Eigen::Vector3f hit_point;

    // Луч через стену должен зафиксировать коллизию
    bool clear = mp10->raycast(origin, target_blocked, hit_point);
    EXPECT_FALSE(clear);
    EXPECT_NEAR(hit_point.x(), 1.2f, 0.06f); // Точность в пределах разрешения 0.05м
    EXPECT_NEAR(hit_point.y(), 0.0f, 0.06f);
    EXPECT_NEAR(hit_point.z(), 0.0f, 0.06f);

    // Луч в свободном направлении (вверх) должен пройти свободно
    Eigen::Vector3f target_free(0.0f, 0.0f, 2.0f);
    EXPECT_TRUE(mp10->raycast(origin, target_free, hit_point));
}

// ============================================================================
// NFR-9.1: Асинхронный буфер "Drop Latest" гарантирует 30 FPS без задержек на ML
// ============================================================================
TEST(SlgwTest, DropLatestBufferConcurrency) {
    auto mp10 = std::make_shared<MP10Space>("async_map", 0.05);
    auto mp11 = std::make_shared<MP11Manager>();
    // Запускаем SLGW с активным фоновым воркером на несуществующий ZMQ адрес (эмуляция медленного ML)
    SlgwNode slgw(mp10, mp11, "tcp://127.0.0.1:5599", true);

    cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(100, 100, 100));
    cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1200));

    // Подаем 30 кадров со скоростью 30 FPS
    auto start_time = std::chrono::steady_clock::now();
    for (int i = 0; i < 30; ++i) {
        slgw.process_frame(i * 0.033, rgb, depth);
    }
    auto end_time = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Все 30 кадров должны быть приняты шлюзом
    EXPECT_EQ(slgw.get_frames_processed(), 30);

    // Время обработки всех 30 кадров не должно быть заблокировано 200мс таймаутами ML!
    std::cout << "[ASYNC] 30 frames pushed in: " << total_ms << " ms.\n";
    EXPECT_LT(total_ms, 1200.0);
}

// ============================================================================
// Sprint 2 DoD: Непрерывное воспроизведение 500+ кадров fr1_desk без утечек
// ============================================================================
TEST(SlgwTest, ContinuousIngestion500Frames) {
    // Поддержка относительных путей как из корня проекта, так и из каталога build
    std::string dataset_path = "../07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
    std::ifstream check_file(dataset_path + "/associations_gt.txt");
    if (!check_file.is_open()) {
        dataset_path = "07_infrastructure/datasets/rgbd_dataset_freiburg1_desk";
    }

    auto mp10 = std::make_shared<MP10Space>("desk_scene", 0.05);
    auto mp11 = std::make_shared<MP11Manager>();
    auto slgw = std::make_shared<SlgwNode>(mp10, mp11, "tcp://127.0.0.1:5555", false);

    DatasetPlayer player(dataset_path, slgw, 1000.0); // Высокий FPS для скоростного воспроизведения 500 кадров
    bool loaded = player.load_associations("associations_gt.txt");
    ASSERT_TRUE(loaded);
    ASSERT_GE(player.get_frame_count(), 500);

    size_t processed_counter = 0;
    // Проигрываем 500 кадров датасета fr1_desk
    player.play(500, false, [&processed_counter](size_t, double) {
        processed_counter++;
    });

    EXPECT_EQ(processed_counter, 500);
    EXPECT_EQ(slgw->get_frames_processed(), 500);

    // Проверяем, что в MP10 построена воксельная карта
    auto occupied = mp10->get_occupied_voxels();
    std::cout << "[DATASET] Continuous playback finished: " << occupied.size()
              << " occupied voxels reconstructed in MP10.\n";
    EXPECT_GT(occupied.size(), 500);

    // Проверяем, что поза агента обновилась
    Eigen::Matrix4f final_pose = mp10->get_pose();
    EXPECT_FALSE(final_pose.isIdentity());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}