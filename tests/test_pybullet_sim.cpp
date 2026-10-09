#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "../07_infrastructure/sim_stream_receiver.hpp"
#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/octomap_manager.hpp"
#include "ipc_messages.pb.h"
#include <zmq.h>

using namespace gtr1::infra;
using namespace gtr1::core;

// 1. Тест корректности инициализации и таймаута SimStreamReceiver при отсутствии симулятора
TEST(PyBulletSimTest, ReceiverTimeoutHandling) {
    SimStreamReceiver receiver("tcp://127.0.0.1:5590", "tcp://127.0.0.1:5591");
    EXPECT_TRUE(receiver.is_connected());

    cv::Mat rgb, depth;
    Eigen::Matrix4f pose;
    double ts = 0.0;
    std::string trace_id;
    bool is_clean = true;

    auto t0 = std::chrono::steady_clock::now();
    bool got_frame = receiver.poll_frame(rgb, depth, pose, ts, trace_id, is_clean, 50);
    auto t1 = std::chrono::steady_clock::now();

    EXPECT_FALSE(got_frame);
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    EXPECT_GE(elapsed_ms, 35.0);
    EXPECT_LT(elapsed_ms, 300.0);
}

// 2. Тест сквозной передачи SensorFrame (ZMQ PUB/SUB) в чистом и эмулированном режимах
TEST(PyBulletSimTest, SensorFrameIPCVerification) {
    void* ctx = zmq_ctx_new();
    void* pub = zmq_socket(ctx, ZMQ_PUB);
    ASSERT_EQ(zmq_bind(pub, "tcp://127.0.0.1:5592"), 0);

    SimStreamReceiver receiver("tcp://127.0.0.1:5592", "tcp://127.0.0.1:5593");
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Создаем тестовый синтетический кадр 640x480
    cv::Mat test_rgb(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));
    cv::Mat test_depth(480, 640, CV_16UC1, cv::Scalar(1500)); // 1.5 метра

    std::vector<uchar> rgb_enc, depth_enc;
    cv::imencode(".jpg", test_rgb, rgb_enc);
    cv::imencode(".png", test_depth, depth_enc);

    // Формируем Protobuf
    gtr1::ipc::SensorFrame frame_proto;
    frame_proto.mutable_meta()->set_timestamp(1.234);
    frame_proto.mutable_meta()->set_frame_id(42);
    frame_proto.mutable_meta()->set_trace_id("TEST-00042");
    frame_proto.set_rgb_image_data(rgb_enc.data(), rgb_enc.size());
    frame_proto.set_depth_image_data(depth_enc.data(), depth_enc.size());
    frame_proto.set_is_clean_mode(true);

    Eigen::Matrix4f pose_in = Eigen::Matrix4f::Identity();
    pose_in(0, 3) = 1.0f;
    pose_in(1, 3) = 2.0f;
    pose_in(2, 3) = 0.5f;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            frame_proto.mutable_camera_pose()->add_data(pose_in(i, j));
        }
    }

    std::string payload;
    frame_proto.SerializeToString(&payload);

    // Отправляем кадр
    zmq_send(pub, payload.data(), payload.size(), 0);

    // Принимаем через наш Receiver
    cv::Mat rx_rgb, rx_depth;
    Eigen::Matrix4f rx_pose;
    double rx_ts = 0.0;
    std::string rx_trace_id;
    bool rx_is_clean = false;

    bool received = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
        if (receiver.poll_frame(rx_rgb, rx_depth, rx_pose, rx_ts, rx_trace_id, rx_is_clean, 200)) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EXPECT_TRUE(received);
    EXPECT_EQ(rx_trace_id, "TEST-00042");
    EXPECT_DOUBLE_EQ(rx_ts, 1.234);
    EXPECT_TRUE(rx_is_clean);
    EXPECT_EQ(rx_rgb.cols, 640);
    EXPECT_EQ(rx_rgb.rows, 480);
    EXPECT_EQ(rx_depth.at<uint16_t>(240, 320), 1500);
    EXPECT_FLOAT_EQ(rx_pose(0, 3), 1.0f);
    EXPECT_FLOAT_EQ(rx_pose(1, 3), 2.0f);
    EXPECT_FLOAT_EQ(rx_pose(2, 3), 0.5f);

    zmq_close(pub);
    zmq_ctx_term(ctx);
}

// 3. Тест проверки KinematicCommand
TEST(PyBulletSimTest, KinematicCommandVerification) {
    SimStreamReceiver receiver("tcp://127.0.0.1:5594", "tcp://127.0.0.1:5595");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    void* ctx = zmq_ctx_new();
    void* sub = zmq_socket(ctx, ZMQ_SUB);
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
    int timeout = 300;
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    ASSERT_EQ(zmq_connect(sub, "tcp://127.0.0.1:5595"), 0);

    // Даем подписчику завершить TCP рукопожатие (ZMQ slow joiner)
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Отправляем команду
    bool received = false;
    gtr1::ipc::KinematicCommand cmd;

    for (int i = 0; i < 5; ++i) {
        receiver.send_command(0.25f, -0.05f, 0.40f, "CMD-TEST");

        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int rc = zmq_msg_recv(&msg, sub, 0);
        if (rc > 0) {
            received = cmd.ParseFromArray(zmq_msg_data(&msg), rc);
            zmq_msg_close(&msg);
            if (received) break;
        } else {
            zmq_msg_close(&msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    ASSERT_TRUE(received);
    EXPECT_EQ(cmd.meta().trace_id(), "CMD-TEST");
    EXPECT_FLOAT_EQ(cmd.velocity().x(), 0.25f);
    EXPECT_FLOAT_EQ(cmd.velocity().y(), -0.05f);
    EXPECT_FLOAT_EQ(cmd.angular_velocity().z(), 0.40f);

    zmq_close(sub);
    zmq_ctx_term(ctx);
}
