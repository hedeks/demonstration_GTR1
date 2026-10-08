#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>
#include "dataset_player.hpp"
#include "../src/operators/slgw_node.hpp"

namespace gtr1 {
namespace infra {
DatasetPlayer::DatasetPlayer(std::string dataset_root, std::shared_ptr<operators::SlgwNode> slgw_node, double target_fps)
    : dataset_root_(std::move(dataset_root)), slgw_node_(std::move(slgw_node)), target_fps_(target_fps) {}

bool DatasetPlayer::load_associations(const std::string& assoc_filename) {
    std::ifstream file(dataset_root_ + "/" + assoc_filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open " << assoc_filename << ". Falling back to MOCK mode!\n";
        for (int i = 0; i < 30; ++i) { // 30 фейковых кадров
            frames_.push_back({i * 0.033, "mock_rgb", "mock_depth"});
        }
        return true;
    }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            FrameData frame;
            
            // Формат associations_gt.txt: ts rgb depth tx ty tz qx qy qz qw
            float tx, ty, tz, qx, qy, qz, qw;
            if (iss >> frame.timestamp >> frame.rgb_path >> frame.depth_path >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
                // Преобразование кватерниона в матрицу поворота
                Eigen::Quaternionf q(qw, qx, qy, qz);
                frame.gt_pose.block<3,3>(0,0) = q.toRotationMatrix();
                frame.gt_pose(0,3) = tx;
                frame.gt_pose(1,3) = ty;
                frame.gt_pose(2,3) = tz;
                frames_.push_back(frame);
            }
        }
    std::cout << "Loaded " << frames_.size() << " frames from dataset.\n";
    return true;
}

void DatasetPlayer::play(std::function<void()> on_frame_end) {
    auto frame_duration = std::chrono::milliseconds(static_cast<int>(1000.0 / target_fps_));

    while (true) {
        for (const auto& frame : frames_) {
            auto start_time = std::chrono::steady_clock::now();

            // 1. Загрузка картинок с диска
            cv::Mat rgb = cv::imread(dataset_root_ + "/" + frame.rgb_path, cv::IMREAD_COLOR);
            cv::Mat depth = cv::imread(dataset_root_ + "/" + frame.depth_path, cv::IMREAD_UNCHANGED);

            // Если MOCK-режим или файлы битые, генерируем пустые картинки
            if (rgb.empty()) {
                rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 50, 50));
                depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar(1000));
            }

            // 2. Отправка кадров в SLGW (SLAM)
            if (slgw_node_) {
                slgw_node_->process_frame(frame.timestamp, rgb, depth, frame.gt_pose);
            }

            if (on_frame_end) {
                on_frame_end();
            }

        // 3. Удержание FPS
        auto processing_time = std::chrono::steady_clock::now() - start_time;
        if (processing_time < frame_duration) {
            std::this_thread::sleep_for(frame_duration - processing_time);
        }
    } // End of for loop
    } // End of while(true)
}

} // namespace infra
} // namespace gtr1
