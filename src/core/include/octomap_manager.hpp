#pragma once
#include "mp10_space.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

namespace gtr1 {
namespace core {

class OctoMapManager {
private:
    std::shared_ptr<MP10Space> mp10_;
    // Параметры камеры (для репроекции Depth в 3D) - по умолчанию TUM RGB-D (Kinect)
    float fx_, fy_, cx_, cy_; 

public:
    OctoMapManager(std::shared_ptr<MP10Space> mp10, float fx = 525.0f, float fy = 525.0f, float cx = 319.5f, float cy = 239.5f);
    
    /**
     * @brief Обновляет воксельную сетку на основе карты глубины и позы камеры
     */
    void update_from_depth(const cv::Mat& depth, const Eigen::Matrix4f& pose);
};

} // namespace core
} // namespace gtr1
