#include "eggplant_controller/thrusters_8_kinematics.hpp"

Thrusters8Kinematics::Thrusters8Kinematics() : Node("thrusters_8_kinematics") {
    this->declare_parameter("balancing", false);

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Thrusters8Kinematics::twist_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10, std::bind(&Thrusters8Kinematics::imu_callback, this, std::placeholders::_1));
    pub_motors_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motors_vel", 10);
}

void Thrusters8Kinematics::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    current_roll_ = static_cast<float>(r);
    current_pitch_ = static_cast<float>(p);
    current_yaw_ = static_cast<float>(y);
}

void Thrusters8Kinematics::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x = msg->linear.x * K_X;
    float y = msg->linear.y * K_Y;
    float z = msg->linear.z * K_Z;
    float yaw = msg->angular.z * K_YAW;

    float roll, pitch;
    if (this->get_parameter("balancing").as_bool()) {
        float K_STAB = 500.0f / 0.785f; // ~45 deg in rad
        roll = -current_roll_ * K_STAB;
        pitch = -current_pitch_ * K_STAB;
    } else {
        roll = msg->angular.x * K_ROLL;
        pitch = msg->angular.y * K_PITCH;
    }

    auto out_msg = std_msgs::msg::Float32MultiArray();
    out_msg.data.resize(8);

    float h[4] = {x-y-yaw, x+y+yaw, x+y-yaw, x-y+yaw};
    float v[4] = {-z+roll+pitch, -z-roll+pitch, -z+roll-pitch, -z-roll-pitch};

    for (int i = 0; i < 4; i++) {
        out_msg.data[i] = 1500.0f + std::max(-500.0f, std::min(500.0f, h[i]));
        out_msg.data[i+4] = 1500.0f + std::max(-500.0f, std::min(500.0f, v[i]));
    }
    pub_motors_->publish(out_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thrusters8Kinematics>());
    rclcpp::shutdown();
    return 0;
}