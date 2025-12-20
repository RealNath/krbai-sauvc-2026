#ifndef SERIAL_MANAGER_HPP_
#define SERIAL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>

class SerialManager : public rclcpp::Node {
public:
    SerialManager();
    ~SerialManager();

private:
    void motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void read_serial();
    
    int serial_port_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_motors_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif