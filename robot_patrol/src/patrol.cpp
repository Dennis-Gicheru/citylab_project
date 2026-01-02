#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Patrol Node has been started.");
    }

private:
    // --- Member Variables (Declared once) ---
    float target_angle_ = 0.0;
    float direction_ = 0.0;
    bool obstacle_detected_ = false;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_dist = 0.0;
        float current_target = 0.0; // This temporary variable holds the best angle found

        int total_rays = msg->ranges.size();
        int start_idx = total_rays / 4;
        int end_idx = 3 * total_rays / 4;

        obstacle_detected_ = false;

        for (int i = start_idx; i < end_idx; i++) {
            float range = msg->ranges[i];
            
            // Check for obstacles in the center rays
            int center_idx = total_rays / 2;
            if (i > center_idx - 10 && i < center_idx + 10) {
                if (range < 0.35) {
                    obstacle_detected_ = true;
                }
            }

            if (!std::isinf(range) && !std::isnan(range) && range > 0.4) {
                if (range > max_dist) {
                    max_dist = range;
                    current_target = msg->angle_min + (i * msg->angle_increment);
                }
            }
        }
        
        // Update the member variables so the control_loop can see the results
        direction_ = current_target;
        target_angle_ = current_target; 
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.1; 

        if (obstacle_detected_) {
            // Use the member variable direction_
            msg.angular.z = direction_ / 2.0;
        } else {
            
            msg.angular.z = std::clamp(target_angle_, -0.5f, 0.5f);
        }
        
        vel_pub_->publish(msg);
    }

    // ROS 2 objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}