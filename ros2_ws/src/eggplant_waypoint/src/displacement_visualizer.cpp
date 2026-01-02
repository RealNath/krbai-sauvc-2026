#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class DisplacementVisualizer : public rclcpp::Node {
public:
    DisplacementVisualizer() : Node("displacement_visualizer") {
        // Subscriptions
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DisplacementVisualizer::twist_callback, this, std::placeholders::_1));
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&DisplacementVisualizer::imu_callback, this, std::placeholders::_1));
        
        // Publishers
        pub_odom_dr_ = this->create_publisher<nav_msgs::msg::Odometry>("visual/odom_dead_reckoning", 10);
        pub_odom_imu_ = this->create_publisher<nav_msgs::msg::Odometry>("visual/odom_imu", 10);
        pub_path_dr_ = this->create_publisher<nav_msgs::msg::Path>("visual/path_dead_reckoning", 10);
        pub_path_imu_ = this->create_publisher<nav_msgs::msg::Path>("visual/path_imu", 10);
        
        last_time_dr_ = this->now();
        last_time_imu_ = this->now();
        
        path_dr_msg_.header.frame_id = "odom";
        path_imu_msg_.header.frame_id = "odom";
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_imu_).seconds();
        last_time_imu_ = now;

        if (dt <= 0 || dt > 0.5) return;

        // 1. Update Orientation directly from IMU Quaternion
        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        // 2. Double integrate Linear Acceleration to get Position
        // Note: This is prone to significant drift over time
        imu_vx_ += msg->linear_acceleration.x < 0.5 && msg->linear_acceleration.x > -0.5 ? 0 : msg->linear_acceleration.x * dt;
        imu_vy_ += msg->linear_acceleration.y < 0.5 && msg->linear_acceleration.y > -0.5 ? 0 : msg->linear_acceleration.y * dt;
        imu_vz_ += msg->linear_acceleration.z < 0.5 && msg->linear_acceleration.z > -0.5 ? 0 : msg->linear_acceleration.z * dt;

        imu_x_ += imu_vx_ * dt;
        imu_y_ += imu_vy_ * dt;
        imu_z_ += imu_vz_ * dt;

        // 3. Publish IMU Odometry
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        // odom.pose.pose.position.x = imu_x_;
        // odom.pose.pose.position.y = imu_y_;
        // odom.pose.pose.position.z = imu_z_;
        odom.pose.pose.orientation = msg->orientation;
        pub_odom_imu_->publish(odom);

        // 4. Update IMU Path
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose = odom.pose.pose;
        path_imu_msg_.poses.push_back(ps);
        pub_path_imu_->publish(path_imu_msg_);
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_dr_).seconds();
        last_time_dr_ = now;

        if (dt <= 0 || dt > 0.5) return;

        // 1. Pure Integration of Twist
        dr_yaw_ += msg->angular.z * dt;
        dr_x_ += (msg->linear.x * std::cos(dr_yaw_) - msg->linear.y * std::sin(dr_yaw_)) * dt;
        dr_y_ += (msg->linear.x * std::sin(dr_yaw_) + msg->linear.y * std::cos(dr_yaw_)) * dt;
        dr_z_ += msg->linear.z * dt;

        // 2. Publish Dead Reckoning Odometry
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = dr_x_;
        odom.pose.pose.position.y = dr_y_;
        odom.pose.pose.position.z = dr_z_;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, dr_yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        pub_odom_dr_->publish(odom);

        // 3. Update DR Path
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose = odom.pose.pose;
        path_dr_msg_.poses.push_back(ps);
        pub_path_dr_->publish(path_dr_msg_);
    }

    // Subs/Pubs
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_dr_, pub_odom_imu_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_dr_, pub_path_imu_;

    // DR States (Twist Integration)
    double dr_x_ = 0, dr_y_ = 0, dr_z_ = 0, dr_yaw_ = 0;
    rclcpp::Time last_time_dr_;
    nav_msgs::msg::Path path_dr_msg_;

    // IMU States (Acceleration/Orientation Integration)
    double imu_x_ = 0, imu_y_ = 0, imu_z_ = 0;
    double imu_vx_ = 0, imu_vy_ = 0, imu_vz_ = 0;
    rclcpp::Time last_time_imu_;
    nav_msgs::msg::Path path_imu_msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplacementVisualizer>());
    rclcpp::shutdown();
    return 0;
}