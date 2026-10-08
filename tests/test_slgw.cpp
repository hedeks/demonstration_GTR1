#include <gtest/gtest.h>
#include "../src/core/include/mp10_space.hpp"
#include "../src/core/include/mp11_manager.hpp"
#include "../src/operators/slgw_node.hpp"

// REQ-04: Падение ML сервиса не вешает C++ ядро (test_zmq_timeout_recovery)
TEST(SlgwNodeTest, ZmqTimeoutRecovery) {
    // 1. Создаем пространство, менеджер MP11 и узел SLGW
    auto mp10 = std::make_shared<gtr1::core::MP10Space>("test_map");
    auto mp11 = std::make_shared<gtr1::core::MP11Manager>();
    gtr1::operators::SlgwNode slgw(mp10, mp11);

    // 2. Эмулируем фейковые данные RGB (1x1 пиксель) и Depth
    cv::Mat fake_rgb(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat fake_depth(1, 1, CV_16U, cv::Scalar(0));

    // 3. Вызываем process_frame. 
    // Поскольку Python CNN1 сервис не запущен, ZMQ должен отвалиться по таймауту 200 мс.
    // Тест пройдет успешно, если вызов завершится без крэша.
    
    auto start = std::chrono::steady_clock::now();
    
    EXPECT_NO_THROW({
        slgw.process_frame(0.0, fake_rgb, fake_depth);
    });
    
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // Проверка, что механизм таймаута отработал корректно (мы не зависли навсегда)
    EXPECT_LT(elapsed, 1000); 
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
