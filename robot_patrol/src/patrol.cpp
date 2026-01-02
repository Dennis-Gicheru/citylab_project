#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node") {
        // QoS for LaserScan (Best Effort)
        auto qos_sensor = rclcpp::SensorDataQoS();
        // QoS for Cmd_Vel (Reliable)
        auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_sensor, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos_cmd);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Patrol::control_loop, this));
        
        last_scan_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Remote Patrol Node started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_time_ = this->now(); 
        
        float max_dist = 0.0;
        float found_angle = 0.0; // Local temporary variable
        int total_rays = msg->ranges.size();
        
        // Front 180 degrees
        int start_idx = total_rays / 4;
        int end_idx = 3 * total_rays / 4;

        obstacle_detected_ = false;

        for (int i = start_idx; i < end_idx; i++) {
            float range = msg->ranges[i];
            int center_idx = total_rays / 2;
            
            // 1. Obstacle Detection (Center 80 rays)
            if (i > center_idx - 40 && i < center_idx + 40) {
                // Check if something is within the 'danger zone'
                if (range < 0.8 && range > 0.05) { 
                    obstacle_detected_ = true;
                }
            }

            // 2. Finding the widest opening (Max distance)
            if (!std::isinf(range) && !std::isnan(range) && range > 0.4) {
                if (range > max_dist) {
                    max_dist = range;
                    found_angle = msg->angle_min + (i * msg->angle_increment);
                }
            }
        }
        
        // Update member variables so control_loop can see the results
        direction_ = found_angle;
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();
        auto time_since_last_scan = this->now() - last_scan_time_;

        // 1. Watchdog Logic: Stop if we lose sensor data
        if (time_since_last_scan.seconds() > 0.6) {
            RCLCPP_WARN(this->get_logger(), "Watchdog: No scan data for %.2f s!", time_since_last_scan.seconds());
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            vel_pub_->publish(msg);
            return; 
        }

        // 2. Movement Logic
        msg.linear.x = 0.08; 
        if (obstacle_detected_) {
            // Turn toward the 'max_dist' direction found in callback
            float turn_speed = direction_; 
            msg.angular.z = std::clamp(turn_speed, -0.6f, 0.6f);
        } else {
            // Move straight if no obstacle in front
            msg.angular.z = 0.0;
        }

        // Debug Log
        RCLCPP_INFO(this->get_logger(), "Moving -> Lin: %.2f, Ang: %.2f, Obs: %s", 
                    msg.linear.x, msg.angular.z, obstacle_detected_ ? "YES" : "NO");
        
        vel_pub_->publish(msg);
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_scan_time_; 
    float direction_ = 0.0;
    bool obstacle_detected_ = false; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}