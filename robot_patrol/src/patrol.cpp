#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("patrol_node"), direction_(0.0f), obstacle_detected_(false)
  {
    // ── Callback 1: laser subscriber ────────────────────────────────────────
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    // ── Callback 2: control loop timer at 10 Hz ──────────────────────────────
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_   = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol node has started.");
  }

private:
  // ── Derives a scan array index from a real-world angle ──────────────────
  // Uses the scan's own metadata — correct on any lidar model or mounting.
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr & msg, float angle) const
  {
    const int total = static_cast<int>(msg->ranges.size());
    int idx = static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
    return std::max(0, std::min(idx, total - 1));
  }

  // ── Callback 1 — laser scan ──────────────────────────────────────────────
  // Scans the front 180° (-π/2 → +π/2).
  // Detects obstacles in the front ±45° cone.
  // Computes direction_ as the angle toward the most open space.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Front 180° bounds — angle-derived, hardware agnostic
    const int start_idx   = angle_to_index(msg, -M_PI / 2.0f);  // -90°
    const int end_idx     = angle_to_index(msg,  M_PI / 2.0f);  // +90°

    // Front ±45° danger cone — wide enough to catch physical obstacles
    const int front_start = angle_to_index(msg, -M_PI / 4.0f);  // -45°
    const int front_end   = angle_to_index(msg,  M_PI / 4.0f);  // +45°

    obstacle_detected_ = false;
    float max_dist     = 0.0f;
    float best_angle   = 0.0f;

    for (int i = start_idx; i <= end_idx; ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range)) continue;

      // Obstacle check inside the front cone
      if (i >= front_start && i <= front_end && range < OBSTACLE_THRESHOLD) {
        obstacle_detected_ = true;
      }

      // Track the ray with the greatest valid distance across all 180°
      // This is the direction of the most open space
      if (range > max_dist) {
        max_dist   = range;
        best_angle = msg->angle_min + (i * msg->angle_increment);
      }
    }

    // direction_ only updated when an obstacle is present.
    // When clear, it resets to 0 so the robot drives straight.
    direction_ = obstacle_detected_ ? best_angle : 0.0f;
  }

  // ── Callback 2 — control loop (10 Hz) ────────────────────────────────────
  // Robot moves continuously — linear.x is always 0.1 m/s.
  // Angular velocity is driven entirely by direction_:
  //   no obstacle → direction_ = 0   → drives straight
  //   obstacle    → direction_ = angle to most open space → steers away
  void control_loop()
  {
    auto cmd      = geometry_msgs::msg::Twist();
    cmd.linear.x  = LINEAR_VEL;                  // always moving forward
    cmd.angular.z = direction_ / 2.0f;           // proportional to most open angle
    vel_pub_->publish(cmd);
  }

  // ── Constants ─────────────────────────────────────────────────────────────
  static constexpr float LINEAR_VEL        = 0.1f;  // m/s
  static constexpr float OBSTACLE_THRESHOLD = 0.5f; // m — 0.5m gives adequate reaction time

  // ── Member variables ───────────────────────────────────────────────────────
  float direction_;         // angle (rad) toward most open space; range [-π/2, π/2]
  bool  obstacle_detected_; // true when an obstacle is inside the front cone

  // ── ROS 2 handles ─────────────────────────────────────────────────────────
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