#include "key_board.hpp"

// 实现获取单例实例的静态方法
std::shared_ptr<KeyboardSubscriber> KeyboardSubscriber::getInstance()
{
    // 使用局部静态变量保证实例只被创建一次
    static std::shared_ptr<KeyboardSubscriber> instance(new KeyboardSubscriber());
    return instance;
}

KeyboardSubscriber::KeyboardSubscriber() : Node("keyboard_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input",
        10,
        std::bind(&KeyboardSubscriber::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Keyboard subscriber node has been started.");
}

void KeyboardSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    key_value = msg->data.back();
}

char KeyboardSubscriber::getKeyBoard() const
{
    return key_value;
}