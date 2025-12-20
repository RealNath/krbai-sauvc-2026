#include "thrusters_8/thrusters_8_node.hpp"

Thrusters8Node::Thrusters8Node() : Node("thrusters_8") {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("topic_name", "cmd_vel");

    std::string port = this->get_parameter("serial_port").as_string();
    std::string topic = this->get_parameter("topic_name").as_string();

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic, 10, std::bind(&Thrusters8Node::move_thrusters, this, std::placeholders::_1));

    serial_port_ = open(port.c_str(), O_RDWR);
    termios tty;
    tcgetattr(serial_port_, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tcsetattr(serial_port_, TCSANOW, &tty);

    for (int i = 0; i < 8; i++) thruster_speeds_[i] = 1500;
}

Thrusters8Node::~Thrusters8Node() {
    if (serial_port_ != -1) close(serial_port_);
}

void Thrusters8Node::move_thrusters(const geometry_msgs::msg::Twist::SharedPtr msg) {
    twist_to_thrusters(msg);
    send_to_serial();
}

void Thrusters8Node::twist_to_thrusters(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float x = msg->linear.x * K_X;
    float y = msg->linear.y * K_Y;
    float z = msg->linear.z * K_Z;
    float roll = msg->angular.x * K_ROLL;
    float pitch = msg->angular.y * K_PITCH;
    float yaw = msg->angular.z * K_YAW;

    float h[4];
    h[0] = x - y - yaw;
    h[1] = x + y + yaw;
    h[2] = x + y - yaw;
    h[3] = x - y + yaw;

    float v[4];
    v[0] = -z + roll + pitch;
    v[1] = -z - roll + pitch;
    v[2] = -z + roll - pitch;
    v[3] = -z - roll - pitch;

    for (int i = 0; i < 4; i++) {
        thruster_speeds_[i] = static_cast<int>(std::max(-500.0f, std::min(500.0f, h[i])));
        thruster_speeds_[i + 4] = static_cast<int>(std::max(-500.0f, std::min(500.0f, v[i])));
    }
}

void Thrusters8Node::send_to_serial() {
    std::string cmd = "CMD";
    for (int i = 0; i < 8; i++) {
        cmd += "," + std::to_string(thruster_speeds_[i]);
    }
    cmd += "\n";
    write(serial_port_, cmd.c_str(), cmd.length());
    RCLCPP_INFO(this->get_logger(), "Sent to thrusters: %s", cmd.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thrusters8Node>());
    rclcpp::shutdown();
    return 0;
}