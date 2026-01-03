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

    float current_roll_, current_pitch_, current_yaw_;
    float K_X, K_Y, K_Z;
    float K_ROLL, K_PITCH, K_YAW;
    float IDLE_RPM_H, IDLE_RPM_V, CLIP_RPM_H, CLIP_RPM_V;
    float K_STAB_NUM, K_STAB_DEN;
    float K_STAB = K_STAB_NUM / K_STAB_DEN;
};

#endif