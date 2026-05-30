#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("patrol_node"), direction_(0.0), obstacle_detected_(false)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol node has started.");
  }

private:
  // Converts a real-world angle (radians) to its array index in the scan
  // Uses the scan's own angle_min and angle_increment — works on any lidar
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr & msg, float angle) const
  {
    const int total_rays = static_cast<int>(msg->ranges.size());
    int idx = static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
    return std::max(0, std::min(idx, total_rays - 1));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Derive indices from real angles — not ray count fractions
    const int start_idx  = angle_to_index(msg, -M_PI / 2.0f);  // -90°
    const int end_idx    = angle_to_index(msg,  M_PI / 2.0f);  // +90°
    const int front_start = angle_to_index(msg, -M_PI / 6.0f); // -30° cone
    const int front_end   = angle_to_index(msg,  M_PI / 6.0f); // +30° cone

    obstacle_detected_ = false;
    float max_dist = 0.0f;

    for (int i = start_idx; i <= end_idx; ++i) {
      const float range = msg->ranges[i];

      if (!std::isfinite(range)) continue;

      // Obstacle check — front ±30° cone (angle-derived, not ray-count-derived)
      if (i >= front_start && i <= front_end && range < OBSTACLE_THRESHOLD) {
        obstacle_detected_ = true;
      }

      // Track ray with greatest valid distance across full 180°
      if (range > max_dist) {
        max_dist   = range;
        direction_ = msg->angle_min + (i * msg->angle_increment);
      }
    }

    // Clear path — drive straight, discard any stale angle
    if (!obstacle_detected_) {
      direction_ = 0.0f;
    }
  }

  void control_loop()
  {
    auto msg      = geometry_msgs::msg::Twist();
    msg.linear.x  = LINEAR_VEL;
    msg.angular.z = obstacle_detected_ ? direction_ / 2.0f : 0.0f;
    vel_pub_->publish(msg);
  }

  static constexpr float LINEAR_VEL        = 0.1f;   // m/s
  static constexpr float OBSTACLE_THRESHOLD = 0.35f;  // metres

  float direction_;
  bool  obstacle_detected_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}