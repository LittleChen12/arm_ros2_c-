#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardListener : public rclcpp::Node {
public:
  KeyboardListener() : Node("keyboard_listener") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10);
    RCLCPP_INFO(this->get_logger(), "Keyboard listener node has been started.");
  }
 
  void listen_keyboard() {
    RCLCPP_INFO(this->get_logger(), "Press any key to send data...");
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (rclcpp::ok()) {
      char c;
      read(STDIN_FILENO, &c, 1);
      std_msgs::msg::String msg;
      msg.data = std::string("Key pressed: ") + c;
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardListener>();
  node->listen_keyboard();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}