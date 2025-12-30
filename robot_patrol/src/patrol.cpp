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
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_dist = 0.0;
        float target_angle = 0.0;

        int total_rays = msg->ranges.size();
        // front 180 degrees is roughly from index total/4 to 3*total/4
        int start_idx = total_rays / 4;
        int end_idx = 3 * total_rays / 4;

        // Reset obstacle flag for this scan
        obstacle_detected_ = false;

        for (int i = start_idx; i < end_idx; i++) {
            float range = msg->ranges[i];
            
            // Check for obstacles in the center rays (front of the robot)
            // We check a small window in the middle of our 180 degree arc
            int center_idx = total_rays / 2;
            if (i > center_idx - 10 && i < center_idx + 10) {
                if (range < 0.35) {
                    obstacle_detected_ = true;
                }
            }

            if (!std::isinf(range) && !std::isnan(range)) {
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
        msg.linear.x = 0.1; // Constant linear velocity as per requirement

        if (obstacle_detected_) {
            // Apply rotation logic only if obstacle is near
            msg.angular.z = direction_ / 2.0;
        } else {
            // Move straight if path is clear
            msg.angular.z = 0.0;
        }
        
        vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_ = 0.0;
    bool obstacle_detected_ = false; // Flag to track 35cm threshold
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}