#ifndef THRUSTERS_8_NODE_HPP_
#define THRUSTERS_8_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Thrusters8Node : public rclcpp::Node {
public:
    Thrusters8Node();
    ~Thrusters8Node();

private:
    void move_thrusters(const geometry_msgs::msg::Twist::SharedPtr msg);
    void twist_to_thrusters(const geometry_msgs::msg::Twist::SharedPtr msg);
    void send_to_serial();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    int serial_port_;
    int thruster_speeds_[8];

    const float K_X = 500.0f;
    const float K_Y = 500.0f;
    const float K_Z = 500.0f;
    const float K_ROLL = 500.0f;
    const float K_PITCH = 500.0f;
    const float K_YAW = 500.0f;
};

#endif