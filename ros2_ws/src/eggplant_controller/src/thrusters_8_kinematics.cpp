#include "eggplant_controller/thrusters_8_kinematics.hpp"

Thrusters8Kinematics::Thrusters8Kinematics() : Node("thrusters_8_kinematics") {
    this->declare_parameter("balancing", false);
    this->declare_parameter("K_X", 200.0f);
    this->declare_parameter("K_Y", 200.0f);
    this->declare_parameter("K_Z", 200.0f);
    this->declare_parameter("K_ROLL", 200.0f);
    this->declare_parameter("K_PITCH", 200.0f);
    this->declare_parameter("K_YAW", 200.0f);
    this->declare_parameter("IDLE_RPM_H", 1200.0f);
    this->declare_parameter("IDLE_RPM_V", 1200.0f);
    this->declare_parameter("CLIP_RPM_H", 200.0f);
    this->declare_parameter("CLIP_RPM_V", 200.0f);
    this->declare_parameter("K_STAB_NUM", 200.0f);
    this->declare_parameter("K_STAB_DEN", 0.785f); // ~45 deg in rad

    K_X = this->get_parameter("K_X").as_double();
    K_Y = this->get_parameter("K_Y").as_double();
    K_Z = this->get_parameter("K_Z").as_double();
    K_ROLL = this->get_parameter("K_ROLL").as_double();
    K_PITCH = this->get_parameter("K_PITCH").as_double();
    K_YAW = this->get_parameter("K_YAW").as_double();
    IDLE_RPM_H = this->get_parameter("IDLE_RPM_H").as_double();
    IDLE_RPM_V = this->get_parameter("IDLE_RPM_V").as_double();
    CLIP_RPM_H = this->get_parameter("CLIP_RPM_H").as_double();
    CLIP_RPM_V = this->get_parameter("CLIP_RPM_V").as_double();
    K_STAB_NUM = this->get_parameter("K_STAB_NUM").as_double();
    K_STAB_DEN = this->get_parameter("K_STAB_DEN").as_double();
    K_STAB = K_STAB_NUM / K_STAB_DEN;

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
    this->current_roll_ = static_cast<float>(r);
    this->current_pitch_ = static_cast<float>(p);
    this->current_yaw_ = static_cast<float>(y);
}

void Thrusters8Kinematics::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x = msg->linear.x * K_X;
    float y = msg->linear.y * K_Y;
    float z = msg->linear.z * K_Z;
    float yaw = msg->angular.z * K_YAW;

    float roll, pitch;
    if (this->get_parameter("balancing").as_bool()) {
        roll = -this->current_roll_ * K_STAB;
        pitch = -this->current_pitch_ * K_STAB;
    } else {
        roll = msg->angular.x * K_ROLL;
        pitch = msg->angular.y * K_PITCH;
    }

    auto out_msg = std_msgs::msg::Float32MultiArray();
    out_msg.data.resize(8);

    float h[4] = {x-y-yaw, x+y+yaw, x+y-yaw, x-y+yaw};
    float v[4] = {-z+roll+pitch, -z-roll+pitch, -z+roll-pitch, -z-roll-pitch};

    for (int i = 0; i < 4; i++) {
        out_msg.data[i] = IDLE_RPM_H + std::max(-CLIP_RPM_H, std::min(CLIP_RPM_H, h[i]));
        out_msg.data[i+4] = IDLE_RPM_V + std::max(-CLIP_RPM_V, std::min(CLIP_RPM_V, v[i]));
    }
    pub_motors_->publish(out_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thrusters8Kinematics>());
    rclcpp::shutdown();
    return 0;
}