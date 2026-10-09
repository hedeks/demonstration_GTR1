#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <atomic>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "../src/operators/slgw_node.hpp"

namespace gtr1 {
namespace infra {

struct FrameData {
    double timestamp{0.0};
    std::string rgb_path;
    std::string depth_path;
    Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
};

/**
 * @brief Плеер воспроизведения RGB-D датасета TUM.
 * Загружает ассоциации кадров и траекторию Ground Truth,
 * синхронизирует RGB и Depth по таймстемпу (< 0.02с) и подает кадры в SLGW с частотой 30 FPS.
 */
class DatasetPlayer {
private:
    std::string dataset_root_;
    std::shared_ptr<operators::SlgwNode> slgw_node_;
    double target_fps_;
    std::vector<FrameData> frames_;
    std::atomic<bool> is_running_{false};

public:
    DatasetPlayer(std::string dataset_root,
                  std::shared_ptr<operators::SlgwNode> slgw_node,
                  double target_fps = 30.0);

    /**
     * @brief Загрузка ассоциаций кадров и GT позы.
     * Поддерживает associations_gt.txt (с позой) и канонический associations.txt.
     */
    bool load_associations(const std::string& assoc_filename = "associations_gt.txt");

    /**
     * @brief Воспроизводит датасет.
     * @param max_frames Количество кадров для воспроизведения (0 = все доступные кадры).
     * @param loop Зацикливать ли воспроизведение.
     * @param on_frame_end Callback после каждого кадра (передает frame_idx, timestamp).
     */
    void play(size_t max_frames = 0,
              bool loop = false,
              std::function<void(size_t frame_idx, double timestamp)> on_frame_end = nullptr);

    /**
     * @brief Перегрузка play для обратной совместимости с legacy main.cpp.
     */
    void play(std::function<void()> on_frame_end) {
        play(0, true, [on_frame_end](size_t, double) {
            if (on_frame_end) on_frame_end();
        });
    }

    /**
     * @brief Обработка одиночного кадра по индексу (для детерминированных тестов).
     */
    bool step_frame(size_t index);

    void stop();
    bool is_playing() const { return is_running_.load(); }
    size_t get_frame_count() const { return frames_.size(); }
    const std::vector<FrameData>& get_frames() const { return frames_; }
    double get_target_fps() const { return target_fps_; }
    void set_target_fps(double fps) { target_fps_ = fps; }
};

} // namespace infra
} // namespace gtr1