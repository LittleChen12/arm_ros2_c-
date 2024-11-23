#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "key_board.hpp"
#include "forward_kinematics.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    //多个node的话直接加相同代码即可
    auto node = std::make_shared<RobotController>();
    auto node_key = KeyboardSubscriber::getInstance();
    // 使用多线程执行器,多个node的话直接加相同代码即可
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node_key);

    // 在单独的线程中运行执行器
    std::thread system_running([&]() {
        while (rclcpp::ok()) {
            printf("System is running!\n");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    });
    // 启动执行器
    executor.spin();
    // 等待线程结束
    //system_running.join();
    rclcpp::shutdown();
    return 0;
}


