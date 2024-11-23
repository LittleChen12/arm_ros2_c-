#include "forward_kinematics.hpp"

RobotController::RobotController() : Node("robot_controller")
{
// 关节状态发布者
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        // 关节轨迹发布者
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // 添加末端执行器位姿发布者
        end_effector_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/end_effector_pose", 10);

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&RobotController::timer_callback, this)
        );

        // 关节名称（与你的URDF匹配）
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // 初始化DH参数（示例参数，需要根据实际机器人修改）
        dh_params_ = {
         //alphai-1   ,ai-1            ,di          ,theta
            {0.0,      0.0,            0.169,       0.0    },  // Joint 1
            {-M_PI_2,  0.064418,       0,           -M_PI_2},  // Joint 2
            {0.0,      0.305,          0,           0},  // Joint 3
            {-M_PI_2,  0.0,            0.22563,     0.0},  // Joint 4
            {M_PI_2,   0.0,            0,           M_PI_2},  // Joint 5
            {M_PI_2,   0.0,            -0.09385,    0.0}   // Joint 6
        };
}

void RobotController::timer_callback()
{
    // 发布关节状态
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = joint_names_;

    // 生成示例关节位置
    static std::vector<double> positions(joint_names_.size(), 0.0);
    static float tick;
    KEY_board = KeyboardSubscriber::getInstance()->getKeyBoard();
    if(KEY_board == 's'){
        for (size_t i = 0; i < positions.size(); ++i) {
            tick += 0.01;
            positions[i] = sin(tick); // 简单的正弦波运动
            printf("开始运动\n");
        }
    }

    joint_state_msg.position = positions;
    joint_state_publisher_->publish(joint_state_msg);

    // 发布关节轨迹
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.header.stamp = this->get_clock()->now();
    trajectory_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration(1, 0); // 1秒后到达

    trajectory_msg.points.push_back(point);
    joint_trajectory_publisher_->publish(trajectory_msg);

    // 计算并发布末端执行器位姿
    auto end_effector_pose = forward_kinematics(positions);
    end_effector_publisher_->publish(end_effector_pose);
}

geometry_msgs::msg::Pose RobotController::forward_kinematics(const std::vector<double>& joint_positions) 
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    // 假设末端执行器有一个特定的点位（joint6的原点）
    Eigen::Vector3d test_point(0.0, 0.0, 0.0); // 根据实际情况设置末端点位坐标

    for (size_t i = 0; i < joint_positions.size(); ++i) {
        double theta = joint_positions[i] + dh_params_[i][3]; // DH参数中的theta加上关节角度
        T *= dh_matrix(dh_params_[i][1], dh_params_[i][2], dh_params_[i][0], theta);
    }

    // 将末端点位坐标与总变换矩阵 T 相乘
    Eigen::Vector4d homogeneous_point(test_point.x(), test_point.y(), test_point.z(), 1.0);
    homogeneous_point = T * homogeneous_point;

    geometry_msgs::msg::Pose pose;
    pose.position.x = homogeneous_point.x();
    pose.position.y = homogeneous_point.y();
    pose.position.z = homogeneous_point.z();
    // 从旋转矩阵计算四元数
    Eigen::Matrix3d rotation = T.block<3,3>(0,0);
    Eigen::Quaterniond q(rotation);
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}