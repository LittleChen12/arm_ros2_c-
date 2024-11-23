#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

#include "key_board.hpp"

class RobotController : public rclcpp::Node 
{
public:
    RobotController();
private:
    void timer_callback();

    // DH参数转换矩阵
    Eigen::Matrix4d dh_matrix(double a, double d, double alpha, double theta) {
        Eigen::Matrix4d T;
        T << cos(theta),             -sin(theta),             0,            a,
             sin(theta)*cos(alpha),  cos(theta)*cos(alpha),   -sin(alpha),  -sin(alpha)*d,
             sin(theta)*sin(alpha),  cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d,
             0,                      0,                       0,            1;
        return T;
    }

    geometry_msgs::msg::Pose forward_kinematics(const std::vector<double>& joint_positions);
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr end_effector_publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> dh_params_; // DH参数表

    char KEY_board;
};

#endif