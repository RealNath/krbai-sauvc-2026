#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class WaypointSequencer : public rclcpp::Node {
public:
    WaypointSequencer() : Node("waypoint_sequencer"), current_idx_(0), waiting_for_trigger_(false) {
        this->declare_parameter<std::string>("mission_file", "");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Corrected Service Type Syntax
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/next", std::bind(&WaypointSequencer::handle_trigger, this, std::placeholders::_1, std::placeholders::_2));

        std::string file_path = this->get_parameter("mission_file").as_string();
        if (!file_path.empty()) {
            load_mission(file_path);
        } else {
            RCLCPP_WARN(this->get_logger(), "No mission file provided via parameters.");
        }
        
        timer_ = this->create_wall_timer(50ms, std::bind(&WaypointSequencer::update, this));
    }

private:
    struct Waypoint {
        std::string name;
        std::string type = "move"; 
        geometry_msgs::msg::Twist twist;
        int value = 0; 
    };

    void load_mission(std::string file_path) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            for (const auto& node : config["waypoints"]) {
                Waypoint wp;
                wp.name = node["name"].as<std::string>();
                if (node["type"]) wp.type = node["type"].as<std::string>();

                if (wp.type == "move") {
                    wp.twist.linear.x = node["cmd"]["x"].as<double>();
                    wp.twist.linear.y = node["cmd"]["y"].as<double>();
                    wp.twist.linear.z = node["cmd"]["z"].as<double>();
                    wp.twist.angular.z = node["cmd"]["yaw"].as<double>();
                    wp.value = node["duration"].as<int>();
                } else if (wp.type == "delay") {
                    wp.value = node["value"].as<int>();
                }
                mission_.push_back(wp);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", mission_.size());
            start_time_ = this->now();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission file: %s", e.what());
        }
    }

    void handle_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (waiting_for_trigger_) {
            waiting_for_trigger_ = false;
            current_idx_++;
            start_time_ = this->now();
            res->success = true;
            res->message = "Trigger received. Proceeding.";
        } else {
            res->success = false;
            res->message = "Not waiting for a trigger.";
        }
    }

    void update() {
        if (current_idx_ >= mission_.size()) {
            publisher_->publish(geometry_msgs::msg::Twist()); 
            return;
        }

        Waypoint& current = mission_[current_idx_];
        auto elapsed = (this->now() - start_time_).seconds() * 1000.0;

        if (current.type == "wait_for_trigger") {
            if (!waiting_for_trigger_) {
                RCLCPP_INFO(this->get_logger(), "(%zu/%zu) Waiting for trigger at waypoint: %s", current_idx_+1, mission_.size(), current.name.c_str());
            }
            waiting_for_trigger_ = true;
            publisher_->publish(geometry_msgs::msg::Twist()); 
        } 
        else if (current.type == "delay") {
            static bool delay_started = false;
            if (!delay_started) {
                RCLCPP_INFO(this->get_logger(), "Delaying for %d ms at waypoint: %s", current.value, current.name.c_str());
                delay_started = true;
            }
            publisher_->publish(geometry_msgs::msg::Twist());
            if (elapsed >= current.value) {
                next_waypoint();
                delay_started = false;
            }
        } 
        else if (current.type == "move") {
            publisher_->publish(current.twist);
            if (elapsed >= current.value) {
                next_waypoint();
            }
        }
    }

    void next_waypoint() {
        RCLCPP_INFO(this->get_logger(), "(%zu/%zu) Completed waypoint: %s", current_idx_+1, mission_.size(), mission_[current_idx_].name.c_str());
        current_idx_++;
        start_time_ = this->now();
    }

    std::vector<Waypoint> mission_;
    size_t current_idx_;
    bool waiting_for_trigger_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointSequencer>());
    rclcpp::shutdown();
    return 0;
}