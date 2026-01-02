#ifndef THRUSTERS_8_KINEMATICS_HPP_
#define THRUSTERS_8_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <vector>

class Thrusters8Kinematics : public rclcpp::Node {
public:
    Thrusters8Kinematics();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_motors_;

    float current_roll_ = 0.0f, current_pitch_ = 0.0f, current_yaw_ = 0.0f;
    float K_X = 200.0f, K_Y = 200.0f, K_Z = 200.0f;
    float K_ROLL = 200.0f, K_PITCH = 200.0f, K_YAW = 200.0f;
    float IDLE_RPM_H = 1200.0f, IDLE_RPM_V = 1200.0f, CLIP_RPM_H = 200.0f, CLIP_RPM_V = 200.0f;
    float K_STAB_NUM = 200.0f, K_STAB_DEN = 0.785f; // ~45 deg in rad
    float K_STAB = K_STAB_NUM / K_STAB_DEN;
};

#endif