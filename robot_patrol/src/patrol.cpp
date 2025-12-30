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
        // Use SensorDataQoS for real hardware (more robust against lag)
        auto qos = rclcpp::SensorDataQoS();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Patrol::control_loop, this));
        
        // Initialize the last scan time to now
        last_scan_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Remote Patrol Node (Barcelona Edition) started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_time_ = this->now(); // Update watchdog timer
        
        float max_dist = 0.0;
        float target_angle = 0.0;

        int total_rays = msg->ranges.size();
        int start_idx = total_rays / 4;
        int end_idx = 3 * total_rays / 4;

        obstacle_detected_ = false;

        for (int i = start_idx; i < end_idx; i++) {
            float range = msg->ranges[i];
            
            // Check for obstacles in the center rays
            int center_idx = total_rays / 2;
            if (i > center_idx - 15 && i < center_idx + 15) {
                // INCREASED THRESHOLD: 0.7m to compensate for internet latency
                if (range < 0.7 && range > 0.01) { 
                    obstacle_detected_ = true;
                }
            }

            if (!std::isinf(range) && !std::isnan(range) && range > 0.01) {
                if (range > max_dist) {
                    max_dist = range;
                    target_angle = msg->angle_min + (i * msg->angle_increment);
                }
            }
        }
        direction_ = target_angle;
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();
        
        // 1. WATCHDOG CHECK: If data is older than 0.5s, STOP.
        auto time_since_last_scan = this->now() - last_scan_time_;
        if (time_since_last_scan.seconds() > 0.5) {
            RCLCPP_WARN(this->get_logger(), "Network Lag Detected (%.2f s). Stopping!", time_since_last_scan.seconds());
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            vel_pub_->publish(msg);
            return; 
        }

        // 2. MOVEMENT LOGIC
        msg.linear.x = 0.08; // Slightly slower for safer remote operation
        
        if (obstacle_detected_) {
            // Apply rotation, but CAP it to 0.6 rad/s to prevent wheel slip
            float turn_speed = direction_ / 2.0;
            msg.angular.z = std::clamp(turn_speed, -0.6f, 0.6f);
        } else {
            msg.angular.z = 0.0;
        }
        
        vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_scan_time_; // Watchdog variable
    float direction_ = 0.0;
    bool obstacle_detected_ = false; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}