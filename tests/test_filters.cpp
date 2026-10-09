#include <gtest/gtest.h>
#include <random>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "../src/core/include/kalman_filter_se3.hpp"
#include "../src/core/include/depth_filter.hpp"
#include "../src/operators/slgw_node.hpp"
#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/mp11_manager.hpp"

using namespace gtr1::core;
using namespace gtr1::operators;

// ============================================================================
// 1. Тест фильтра Калмана: подавление шума измерений (RMSE reduction > 50%)
// ============================================================================
TEST(KalmanFilterSE3Test, NoiseReductionVerification) {
    KalmanFilterSE3 kf(0.05f, 0.2f); // Погрешность измерения 5 см

    std::mt19937 gen(42);
    std::normal_distribution<float> noise(0.0f, 0.05f); // 5 см гауссовский шум

    double raw_sq_err = 0.0;
    double kf_sq_err = 0.0;
    int samples = 100;

    for (int i = 0; i < samples; ++i) {
        double t = i * 0.033; // 30 FPS
        Eigen::Vector3f true_pos(0.2f * static_cast<float>(t), 0.1f * static_cast<float>(t), 0.0f);

        // Измерение с шумом
        Eigen::Matrix4f measured_pose = Eigen::Matrix4f::Identity();
        measured_pose.block<3, 1>(0, 3) = true_pos + Eigen::Vector3f(noise(gen), noise(gen), noise(gen));

        Eigen::Matrix4f filtered_pose = kf.update(t, measured_pose);
        Eigen::Vector3f est_pos = filtered_pose.block<3, 1>(0, 3);

        if (i > 15) { // Пропускаем период сходимости
            raw_sq_err += (measured_pose.block<3, 1>(0, 3) - true_pos).squaredNorm();
            kf_sq_err += (est_pos - true_pos).squaredNorm();
        }
    }

    double raw_rmse = std::sqrt(raw_sq_err / (samples - 15));
    double kf_rmse = std::sqrt(kf_sq_err / (samples - 15));

    // Фильтр Калмана должен снизить среднеквадратичную ошибку более чем в 1.5 раза
    EXPECT_LT(kf_rmse, raw_rmse * 0.65);
    EXPECT_LT(kf_rmse, 0.04); // Итоговая ошибка < 4 см
}

// ============================================================================
// 2. Тест фильтра Калмана: оценка линейной скорости агента
// ============================================================================
TEST(KalmanFilterSE3Test, VelocityEstimationConvergence) {
    KalmanFilterSE3 kf(0.02f, 0.5f);

    float true_vx = 0.5f; // Движение со скоростью 0.5 м/с по X
    float true_vy = -0.2f;

    for (int i = 0; i < 60; ++i) {
        double t = i * 0.033;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(0, 3) = true_vx * static_cast<float>(t);
        pose(1, 3) = true_vy * static_cast<float>(t);
        pose(2, 3) = 0.5f;

        kf.update(t, pose);
    }

    Eigen::Vector3f est_vel = kf.get_velocity();
    EXPECT_NEAR(est_vel.x(), true_vx, 0.08f);
    EXPECT_NEAR(est_vel.y(), true_vy, 0.08f);
    EXPECT_NEAR(est_vel.z(), 0.0f, 0.05f);
}

// ============================================================================
// 3. Тест двустороннего фильтра глубины: подавление шума на плоской поверхности
// ============================================================================
TEST(DepthFilterTest, PlanarSurfaceNoiseReduction) {
    DepthFilter depth_filter(7, 0.08, 6.0, 0.20f, 5.00f, 1000.0f);

    // Создаем синтетическую поверхность (стол на 1500 мм) с шумом +-35 мм
    int w = 200, h = 200;
    cv::Mat noisy_depth(h, w, CV_16UC1);
    std::mt19937 gen(123);
    std::normal_distribution<float> noise(0.0f, 35.0f);

    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            float val = 1500.0f + noise(gen);
            noisy_depth.at<uint16_t>(r, c) = static_cast<uint16_t>(std::clamp(val, 500.0f, 3000.0f));
        }
    }

    cv::Scalar mean_raw, stddev_raw;
    cv::meanStdDev(noisy_depth, mean_raw, stddev_raw);

    // Применяем фильтр
    cv::Mat filtered = depth_filter.filter(noisy_depth);

    // Исключаем 5 пикселей по границе изображения
    cv::Rect roi(5, 5, w - 10, h - 10);
    cv::Scalar mean_filt, stddev_filt;
    cv::meanStdDev(filtered(roi), mean_filt, stddev_filt);

    EXPECT_NEAR(mean_filt[0], 1500.0, 10.0);
    // Дисперсия шума должна упасть более чем в 2 раза
    EXPECT_LT(stddev_filt[0], stddev_raw[0] * 0.55);
}

// ============================================================================
// 4. Тест двустороннего фильтра глубины: сохранение резких границ (Edge Preservation)
// ============================================================================
TEST(DepthFilterTest, StepEdgePreservation) {
    DepthFilter depth_filter(7, 0.08, 6.0, 0.20f, 5.00f, 1000.0f);

    // Создаем изображение со ступенькой:
    // Левая половина: объект на 1000 мм (1.0 м)
    // Правая половина: фон на 2500 мм (2.5 м)
    int w = 100, h = 100;
    cv::Mat step_depth(h, w, CV_16UC1);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            step_depth.at<uint16_t>(r, c) = (c < 50) ? 1000 : 2500;
        }
    }

    cv::Mat filtered = depth_filter.filter(step_depth);

    // Проверяем пиксели в непосредственной близости от границы (столбец 48 и 52)
    for (int r = 10; r < 90; ++r) {
        uint16_t left_val = filtered.at<uint16_t>(r, 47);
        uint16_t right_val = filtered.at<uint16_t>(r, 53);

        // Граница не должна размываться в промежуточные значения!
        EXPECT_EQ(left_val, 1000);
        EXPECT_EQ(right_val, 2500);
    }
}

// ============================================================================
// 5. Тест подавления летающих пикселей (Flying Pixels)
// ============================================================================
TEST(DepthFilterTest, FlyingPixelRejection) {
    DepthFilter depth_filter(7, 0.08, 6.0, 0.20f, 5.00f, 1000.0f);

    // Фон на 3000 мм с единичным изолированным артефактом на 1200 мм
    int w = 40, h = 40;
    cv::Mat depth(h, w, CV_16UC1, cv::Scalar(3000));
    depth.at<uint16_t>(20, 20) = 1200; // Летающий пиксель в воздухе

    cv::Mat filtered = depth_filter.filter(depth);

    // Летающий пиксель должен быть аннулирован (0) или приведен к окружению
    uint16_t artifact_val = filtered.at<uint16_t>(20, 20);
    EXPECT_TRUE(artifact_val == 0 || artifact_val >= 2900);
}

// ============================================================================
// 6. Тест подавления летающих граней на контурах (Flying Edges Suppression)
// ============================================================================
TEST(DepthFilterTest, FlyingEdgesSuppressionAtOcclusions) {
    DepthFilter depth_filter(5, 0.04, 4.0, 0.20f, 5.00f, 1000.0f);
    EXPECT_TRUE(depth_filter.is_flying_edges_enabled());

    // Кадр с резким перепадом (передний план 800 мм, фон 3200 мм)
    int w = 100, h = 100;
    cv::Mat depth(h, w, CV_16UC1);
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            depth.at<uint16_t>(r, c) = (c < 50) ? 800 : 3200;
        }
    }

    cv::Mat filtered = depth_filter.filter(depth);

    // Точки на линии разрыва контура (столбец 49-50) должны быть вырезаны (0),
    // чтобы предотвратить образование "паутины" между объектом и фоном
    EXPECT_EQ(filtered.at<uint16_t>(50, 49), 0);
    EXPECT_EQ(filtered.at<uint16_t>(50, 50), 0);
}

// ============================================================================
// 7. Тест заделывания микро-проколов сенсора (Hole Inpainting)
// ============================================================================
TEST(DepthFilterTest, MicroHoleFillingOnContinuousSurfaces) {
    DepthFilter depth_filter(5, 0.04, 4.0, 0.20f, 5.00f, 1000.0f);
    EXPECT_TRUE(depth_filter.is_fill_small_holes_enabled());

    // Поверхность 1800 мм с несколькими проколами сенсора (значение 0)
    int w = 50, h = 50;
    cv::Mat depth(h, w, CV_16UC1, cv::Scalar(1800));
    depth.at<uint16_t>(25, 25) = 0; // изолированный прокол

    cv::Mat filtered = depth_filter.filter(depth);

    // Прокол должен быть восстановлен значением близким к 1800 мм
    uint16_t filled_val = filtered.at<uint16_t>(25, 25);
    EXPECT_NEAR(filled_val, 1800, 50);
}

// ============================================================================
// 8. Тест интеграции в SlgwNode
// ============================================================================
TEST(SlgwFilterIntegrationTest, SlgwAppliesKalmanAndDepthFilters) {
    auto mp10 = std::make_shared<MP10Space>("test_filtered_space", 0.05);
    auto mp11 = std::make_shared<MP11Manager>();
    SlgwNode slgw(mp10, mp11, "tcp://127.0.0.1:5555", false);

    EXPECT_TRUE(slgw.is_kalman_enabled());
    EXPECT_TRUE(slgw.is_depth_filter_enabled());

    cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(1500));

    Eigen::Matrix4f noisy_pose = Eigen::Matrix4f::Identity();
    noisy_pose(0, 3) = 1.05f; // Шумное смещение

    auto t0 = std::chrono::steady_clock::now();
    slgw.process_frame(0.033, rgb, depth, noisy_pose);
    auto t1 = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    // Обработка с фильтрами должна укладываться в бюджет реального времени 30 FPS (< 33 мс)
    EXPECT_LT(elapsed_ms, 33.0);
    EXPECT_EQ(slgw.get_frames_processed(), 1);
}
