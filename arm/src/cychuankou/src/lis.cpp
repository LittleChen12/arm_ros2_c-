#include <functional>
#include <memory>
#include <string>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>


using std::placeholders::_1;
 
serial::Serial ros_ser;
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_sync_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    if (!ros_ser.isOpen()) {
      RCLCPP_WARN(this->get_logger(), "Serial port not open");
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      uint8_t joint_id = static_cast<uint8_t>(i + 1);  // 关节ID从1开始
      float angle = static_cast<float>(msg->position[i]);  // 下位机需要浮点数

      try {
        ros_ser.write(&joint_id, 1);  // 写入关节ID
        ros_ser.write(reinterpret_cast<const uint8_t*>(&angle), sizeof(float));  // 写入浮点数据
        RCLCPP_INFO(this->get_logger(), "Sent joint %d angle: %f", joint_id, angle);
      } catch (serial::IOException &e)  {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data: %s", e.what());
      }
    }
  }                          
   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
 
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  ros_ser.setPort("/dev/ttyUSB0");            //设置端口号
  ros_ser.setBaudrate(115200);                //设置波特率
  serial::Timeout to =serial::Timeout::simpleTimeout(1000);
  ros_ser.setTimeout(to);
  try
  {
    ros_ser.open();
  }
  catch(serial::IOException &e)
  {
    std::cout<<"unable to open"<<std::endl;     //若打开串口失败打印"unable to open"到终端
    return -1;
  }
  if(ros_ser.isOpen())
  {
    std::cout<<"open"<<std::endl;              //若打开串口打印"open"到终端
  }
  else
  {
    return -1;
  }
 
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  ros_ser.close();
  return 0;
}