#include "eggplant_controller/serial_manager.hpp"

SerialManager::SerialManager() : Node("serial_manager") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    std::string port = this->get_parameter("port").as_string();

    serial_port_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    termios tty;
    tcgetattr(serial_port_, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tcsetattr(serial_port_, TCSANOW, &tty);

    sub_motors_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "motors_vel", 10, std::bind(&SerialManager::motor_callback, this, std::placeholders::_1));

    pub_status_ = this->create_publisher<std_msgs::msg::Bool>("motors_status", 10);
    pub_depth_ = this->create_publisher<std_msgs::msg::Float32>("depth", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    pub_euler_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu/euler", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialManager::read_serial, this));
}

SerialManager::~SerialManager() {
    if (serial_port_ != -1) close(serial_port_);
}

void SerialManager::motor_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::string cmd = "CMD";
    for (size_t i = 0; i < 8; i++) {
        cmd += "," + std::to_string(static_cast<int>(msg->data[i]));
    }
    cmd += "\n";
    write(serial_port_, cmd.c_str(), cmd.length());
}

void SerialManager::read_serial() {
    char buf[256];
    int n = read(serial_port_, buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        std::string s(buf);
        if (s.find("FB") == 0) {
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream tokenStream(s);
            while (std::getline(tokenStream, token, ',')) tokens.push_back(token);

            if (tokens.size() >= 9) {
                auto status_msg = std_msgs::msg::Bool();
                status_msg.data = (tokens[1] == "1");
                pub_status_->publish(status_msg);

                auto depth_msg = std_msgs::msg::Float32();
                depth_msg.data = std::stof(tokens[2]);
                pub_depth_->publish(depth_msg);

                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = "imu_link";
                imu_msg.linear_acceleration.x = std::stof(tokens[3]);
                imu_msg.linear_acceleration.y = std::stof(tokens[4]);
                imu_msg.linear_acceleration.z = std::stof(tokens[5]);
                imu_msg.angular_velocity.x = std::stof(tokens[6]);
                imu_msg.angular_velocity.y = std::stof(tokens[7]);
                imu_msg.angular_velocity.z = std::stof(tokens[8]);
                pub_imu_->publish(imu_msg);

                if (tokens.size() >= 12) {
                    auto euler_msg = geometry_msgs::msg::Vector3();
                    euler_msg.x = std::stof(tokens[10]); // Roll
                    euler_msg.y = std::stof(tokens[9]);  // Pitch
                    euler_msg.z = std::stof(tokens[11]); // Yaw
                    pub_euler_->publish(euler_msg);
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialManager>());
    rclcpp::shutdown();
    return 0;
}