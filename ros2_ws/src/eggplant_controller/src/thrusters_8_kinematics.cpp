#include "eggplant_controller/thrusters_8_kinematics.hpp"

Thrusters8Kinematics::Thrusters8Kinematics() : Node("thrusters_8_kinematics") {
    // Declare parameters for tuning
    this->declare_parameter("balancing", false);
    this->declare_parameter("kp_balancing", 250.0);
    this->declare_parameter("kd_balancing", 50.0);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Thrusters8Kinematics::twist_callback, this, std::placeholders::_1));
    sub_euler_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "imu/euler", 10, std::bind(&Thrusters8Kinematics::euler_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motors_vel", 10);
}

void Thrusters8Kinematics::euler_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    current_roll_ = msg->x;
    current_pitch_ = msg->y;
    current_yaw_ = msg->z;
}

void Thrusters8Kinematics::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x = msg->linear.x * K_X;
    float y = msg->linear.y * K_Y;
    float z = msg->linear.z * K_Z;
    
    // Stabilization gains (assuming ~45 degrees tilt = max correction)
    float K_STAB = 500.0f / 45.0f;

    // Apply stabilization to Roll and Pitch (Angle Mode)
    // Command sets target angle, IMU provides feedback
    float roll = (msg->angular.x * K_ROLL) - (current_roll_ * K_STAB);
    float pitch = (msg->angular.y * K_PITCH) - (current_pitch_ * K_STAB);
    
    float yaw = msg->angular.z * K_YAW;

    float roll, pitch;

    // Toggle logic: Use IMU PID output or manual Twist input
    if (this->get_parameter("balancing").as_bool()) {
        roll = roll_correction_;
        pitch = pitch_correction_;
    } else {
        roll = msg->angular.x * K_ROLL;
        pitch = msg->angular.y * K_PITCH;
    }

    auto out_msg = std_msgs::msg::Float32MultiArray();
    out_msg.data.resize(8);

    // Horizontal Omni Thrusters (Thrusters 1-4)
    float h[4];
    h[0] = x - y - yaw;
    h[1] = x + y + yaw;
    h[2] = x + y - yaw;
    h[3] = x - y + yaw;

    // Vertical Thrusters (Thrusters 5-8)
    float v[4];
    v[0] = -z + roll + pitch;
    v[1] = -z - roll + pitch;
    v[2] = -z + roll - pitch;
    v[3] = -z - roll - pitch;

    // Clamp and map to PWM range (1500 is neutral)
    for (int i = 0; i < 4; i++) {
        out_msg.data[i] = 1500.0f + std::max(-500.0f, std::min(500.0f, h[i]));
        out_msg.data[i + 4] = 1500.0f + std::max(-500.0f, std::min(500.0f, v[i]));
    }

    publisher_->publish(out_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thrusters8Kinematics>());
    rclcpp::shutdown();
    return 0;
}