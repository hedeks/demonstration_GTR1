#include "dataset_player.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

namespace gtr1 {
namespace infra {

DatasetPlayer::DatasetPlayer(std::string dataset_root,
                             std::shared_ptr<operators::SlgwNode> slgw_node,
                             double target_fps)
    : dataset_root_(std::move(dataset_root)),
      slgw_node_(std::move(slgw_node)),
      target_fps_(target_fps),
      is_running_(false) {}

bool DatasetPlayer::load_associations(const std::string& assoc_filename) {
    frames_.clear();
    std::string path = dataset_root_ + "/" + assoc_filename;
    std::ifstream file(path);

    if (!file.is_open()) {
        // Попытка альтернативного имени файла
        std::string alt_path = dataset_root_ + "/associations_gt.txt";
        if (assoc_filename != "associations_gt.txt") {
            file.open(alt_path);
        }
        if (!file.is_open()) {
            alt_path = dataset_root_ + "/associations.txt";
            file.open(alt_path);
        }
    }

    if (!file.is_open()) {
        std::cerr << "[DatasetPlayer] WARN: Could not open associations file in " << dataset_root_
                  << ". Generating synthetic MOCK frames for fallback.\n";
        for (int i = 0; i < 30; ++i) {
            FrameData frame;
            frame.timestamp = i * (1.0 / target_fps_);
            frame.rgb_path = "mock_rgb_" + std::to_string(i);
            frame.depth_path = "mock_depth_" + std::to_string(i);
            frame.gt_pose = Eigen::Matrix4f::Identity();
            frames_.push_back(frame);
        }
        return true;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 10) {
            // Формат associations_gt: ts rgb depth tx ty tz qx qy qz qw
            FrameData frame;
            frame.timestamp = std::stod(tokens[0]);
            frame.rgb_path = tokens[1];
            frame.depth_path = tokens[2];

            float tx = std::stof(tokens[3]);
            float ty = std::stof(tokens[4]);
            float tz = std::stof(tokens[5]);
            float qx = std::stof(tokens[6]);
            float qy = std::stof(tokens[7]);
            float qz = std::stof(tokens[8]);
            float qw = std::stof(tokens[9]);

            Eigen::Quaternionf q(qw, qx, qy, qz);
            frame.gt_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
            frame.gt_pose(0, 3) = tx;
            frame.gt_pose(1, 3) = ty;
            frame.gt_pose(2, 3) = tz;
            frames_.push_back(frame);
        } else if (tokens.size() >= 4) {
            // Формат associations: rgb_ts rgb_path depth_ts depth_path
            double rgb_ts = std::stod(tokens[0]);
            double depth_ts = std::stod(tokens[2]);

            // Синхронизация по времени (< 0.02 с)
            if (std::abs(rgb_ts - depth_ts) < 0.05) {
                FrameData frame;
                frame.timestamp = rgb_ts;
                frame.rgb_path = tokens[1];
                frame.depth_path = tokens[3];
                frame.gt_pose = Eigen::Matrix4f::Identity();
                frames_.push_back(frame);
            }
        }
    }

    std::cout << "[DatasetPlayer] Successfully loaded " << frames_.size()
              << " synchronized frames from " << dataset_root_ << ".\n";
    return !frames_.empty();
}

bool DatasetPlayer::step_frame(size_t index) {
    if (index >= frames_.size()) return false;

    const auto& frame = frames_[index];
    cv::Mat rgb = cv::imread(dataset_root_ + "/" + frame.rgb_path, cv::IMREAD_COLOR);
    cv::Mat depth = cv::imread(dataset_root_ + "/" + frame.depth_path, cv::IMREAD_UNCHANGED);

    if (rgb.empty() || depth.empty()) {
        // Fallback на синтетический кадр при повреждении или отсутствии файла
        rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(60, 60, 60));
        depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar(1500)); // 1.5м глубина
    }

    if (slgw_node_) {
        slgw_node_->process_frame(frame.timestamp, rgb, depth, frame.gt_pose);
    }
    return true;
}

void DatasetPlayer::play(size_t max_frames,
                         bool loop,
                         std::function<void(size_t frame_idx, double timestamp)> on_frame_end) {
    if (frames_.empty()) {
        std::cerr << "[DatasetPlayer] Cannot play: frame list is empty.\n";
        return;
    }

    is_running_.store(true);
    auto frame_duration = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / target_fps_));

    size_t total_played = 0;
    while (is_running_.load()) {
        for (size_t i = 0; i < frames_.size(); ++i) {
            if (!is_running_.load()) break;

            auto start_time = std::chrono::steady_clock::now();

            step_frame(i);
            total_played++;

            if (on_frame_end) {
                on_frame_end(i, frames_[i].timestamp);
            }

            if (max_frames > 0 && total_played >= max_frames) {
                is_running_.store(false);
                break;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - start_time);
            if (elapsed < frame_duration) {
                std::this_thread::sleep_for(frame_duration - elapsed);
            }
        }

        if (!loop) {
            break;
        }
    }
    is_running_.store(false);
}

void DatasetPlayer::stop() {
    is_running_.store(false);
}

} // namespace infra
} // namespace gtr1