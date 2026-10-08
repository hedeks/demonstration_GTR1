#pragma once
#include "../core/include/mp10_space.hpp"
#include "../core/include/octomap_manager.hpp"
#include "../core/include/mp11_manager.hpp"
#include <opencv2/opencv.hpp>
#include <memory>

namespace gtr1 {
namespace operators {

/**
 * @brief SLGW (Sensor-Level Gateway)
 * Шлюз базового каскада. Отвечает за Visual SLAM.
 * Принимает RGB-D кадры, вычисляет позу агента (SE3) и строит карту (MP10).
 */
class SlgwNode {
private:
    std::shared_ptr<core::MP10Space> mp10_;
    std::shared_ptr<core::MP11Manager> mp11_;
    std::unique_ptr<core::OctoMapManager> octo_mgr_;
    // ZMQ context and socket for communicating with CNN1
    void* zmq_ctx_;
    void* zmq_req_socket_;
    
public:
    SlgwNode(std::shared_ptr<core::MP10Space> mp10, std::shared_ptr<core::MP11Manager> mp11);
    ~SlgwNode();

    void process_frame(double timestamp, const cv::Mat& rgb, const cv::Mat& depth, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

private:
    void send_to_cnn(double timestamp, const cv::Mat& rgb);
};

} // namespace operators
} // namespace gtr1
