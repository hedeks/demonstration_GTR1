#pragma once
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "../src/operators/slgw_node.hpp"

namespace gtr1 {
namespace infra {

struct FrameData {
    double timestamp;
    std::string rgb_path;
    std::string depth_path;
    Eigen::Matrix4f gt_pose = Eigen::Matrix4f::Identity();
};

class DatasetPlayer {
private:
    std::string dataset_root_;
    std::shared_ptr<operators::SlgwNode> slgw_node_;
    double target_fps_;
    std::vector<FrameData> frames_;
public:
    DatasetPlayer(std::string dataset_root, std::shared_ptr<operators::SlgwNode> slgw_node, double target_fps = 30.0);
    bool load_associations(const std::string& assoc_filename = "associations.txt");
    void play(std::function<void()> on_frame_end = nullptr);
};

} // namespace infra
} // namespace gtr1
