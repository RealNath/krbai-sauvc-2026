#include "eggplant_controller/thrusters_8_kinematics.hpp"

Thrusters8Kinematics::Thrusters8Kinematics() : Node("thrusters_8_kinematics") {
    // Declare parameters for tuning
    this->declare_parameter("balancing", false);
    this->declare_parameter("kp_balancing", 250.0);
    this->declare_parameter("kd_balancing", 50.0);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Thrusters8Kinematics::twist_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&Thrusters8Kinematics::imu_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motors_vel", 10);
}

void Thrusters8Kinematics::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Only process PID if balancing is enabled
    if (!this->get_parameter("balancing").as_bool()) return;

    // Convert Quaternion to Euler RPY
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);

    current_roll_ = static_cast<float>(r);
    current_pitch_ = static_cast<float>(p);

    float d_roll = current_roll_ - last_roll_;
    float d_pitch = current_pitch_ - last_pitch_;

    float kp = this->get_parameter("kp_balancing").as_double();
    float kd = this->get_parameter("kd_balancing").as_double();

    // PD Control: Target is 0.0 for both roll and pitch
    roll_correction_ = -(current_roll_ * kp) - (d_roll * kd);
    pitch_correction_ = -(current_pitch_ * kp) - (d_pitch * kd);

    last_roll_ = current_roll_;
    last_pitch_ = current_pitch_;
}

void Thrusters8Kinematics::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x = msg->linear.x * K_X;
    float y = msg->linear.y * K_Y;
    float z = msg->linear.z * K_Z;
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