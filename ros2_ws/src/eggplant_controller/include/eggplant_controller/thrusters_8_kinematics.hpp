#ifndef THRUSTERS_8_KINEMATICS_HPP_
#define THRUSTERS_8_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <algorithm>
#include <vector>

class Thrusters8Kinematics : public rclcpp::Node {
public:
    Thrusters8Kinematics();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    const float K_X = 500.0f;
    const float K_Y = 500.0f;
    const float K_Z = 500.0f;
    const float K_ROLL = 500.0f;
    const float K_PITCH = 500.0f;
    const float K_YAW = 500.0f;
};

#endif