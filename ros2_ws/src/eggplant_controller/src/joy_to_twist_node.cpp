#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyToTwistNode : public rclcpp::Node {
public:
    JoyToTwistNode() : Node("joy_to_twist") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyToTwistNode::joy_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();

        // Joystick Kiri: Maju-Mundur (axes[1]), Kiri-Kanan (axes[0])
        twist.linear.x = msg->axes[1]; 
        twist.linear.y = msg->axes[0];

        // Joystick Kanan (Horizontal): Yaw (axes[3])
        twist.angular.z = msg->axes[3];

        // Jika LB ditahan (buttons[4]), Joystick Kanan (Vertical) untuk Naik-Turun
        // axes[4] biasanya adalah Vertical pada stick kanan
        if (msg->buttons[4] == 1) {
            twist.linear.z = msg->axes[4];
        } else {
            twist.linear.z = 0.0;
        }

        publisher_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwistNode>());
    rclcpp::shutdown();
    return 0;
}