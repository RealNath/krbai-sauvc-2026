#ifndef THRUSTERS_8_KINEMATICS_HPP_
#define THRUSTERS_8_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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
    void euler_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_euler_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    float current_roll_ = 0.0f;
    float current_pitch_ = 0.0f;
    float current_yaw_ = 0.0f;

    const float K_X = 500.0f;
    const float K_Y = 500.0f;
    const float K_Z = 500.0f;
    const float K_ROLL = 500.0f;
    const float K_PITCH = 500.0f;
    const float K_YAW = 500.0f;

    // Balancing state variables
    float current_roll_ = 0.0f;
    float current_pitch_ = 0.0f;
    float last_roll_ = 0.0f;
    float last_pitch_ = 0.0f;
    float roll_correction_ = 0.0f;
    float pitch_correction_ = 0.0f;
};

#endif