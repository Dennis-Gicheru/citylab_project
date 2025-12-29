#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node") {
        // Subscriber to LaserScan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        // Publisher to cmd_vel
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Control loop at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::control_loop, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_dist = 0.0;
        float target_angle = 0.0;

        // Calculate index range for 180 degrees (front of robot)
        // Assuming index 0 is -pi/2 and mid index is 0 (front)
        int total_rays = msg->ranges.size();
        int start_idx = total_rays / 4;      // -90 degrees
        int end_idx = 3 * total_rays / 4;    // +90 degrees

        for (int i = start_idx; i < end_idx; i++) {
            float range = msg->ranges[i];
            
            // Filter out inf and find the largest distance
            if (!std::isinf(range) && !std::isnan(range)) {
                if (range > max_dist) {
                    max_dist = range;
                    // Calculate angle in radians relative to front (X-axis)
                    target_angle = msg->angle_min + (i * msg->angle_increment);
                }
            }
        }
        direction_ = target_angle;
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.1;               // Constant forward speed
        msg.angular.z = direction_ / 2.0; // Angular velocity calculation
        vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}