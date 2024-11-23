#ifndef KEY_BOARD_HPP
#define KEY_BOARD_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KeyboardSubscriber : public rclcpp::Node
{
public:
    // 获取单例实例的静态方法
    static std::shared_ptr<KeyboardSubscriber> getInstance();
    KeyboardSubscriber();
    char getKeyBoard() const;

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    char key_value;
};

#endif