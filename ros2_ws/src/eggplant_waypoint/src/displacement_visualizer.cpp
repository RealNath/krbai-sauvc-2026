#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DisplacementVisualizer : public rclcpp::Node {
public:
    DisplacementVisualizer() : Node("displacement_visualizer"), x_(0), y_(0), z_(0), yaw_(0) {
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DisplacementVisualizer::twist_callback, this, std::placeholders::_1));
        
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("visual/odom", 10);
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("visual/path", 10);
        
        last_time_ = this->now();
        path_msg_.header.frame_id = "odom";
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 1. Integrate position (Dead Reckoning)
        // Note: For underwater robots, yaw affects X/Y movement direction
        x_ += (msg->linear.x * cos(yaw_) - msg->linear.y * sin(yaw_)) * dt;
        y_ += (msg->linear.x * sin(yaw_) + msg->linear.y * cos(yaw_)) * dt;
        z_ += msg->linear.z * dt;
        yaw_ += msg->angular.z * dt;

        // 2. Create Odometry Message
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = z_;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        pub_odom_->publish(odom);

        // 3. Update and Publish Path (trail)
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = current_time;
        pub_path_->publish(path_msg_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    
    double x_, y_, z_, yaw_;
    rclcpp::Time last_time_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplacementVisualizer>());
    rclcpp::shutdown();
    return 0;
}