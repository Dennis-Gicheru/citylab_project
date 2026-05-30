#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <cmath>


// Normalise any angle into [-π, π]
static float normalize_angle(float a)
{
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

// Extract yaw from odometry quaternion — no tf2 dependency needed
static float yaw_from_odom(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  const auto & q = msg->pose.pose.orientation;
  return std::atan2(
    2.0f * (q.w * q.z + q.x * q.y),
    1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

// ── State machine ─────────────────────────────────────────────────────────────
enum class State { PATROLLING, STOPPING, ROTATING };

// ── Node ─────────────────────────────────────────────────────────────────────
class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : Node("patrol_node"),
    direction_(0.0f),
    obstacle_detected_(false),
    state_(State::PATROLLING),
    current_yaw_(0.0f),
    target_yaw_(0.0f),
    stop_counter_(0)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 20,
      std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20,
      std::bind(&Patrol::odom_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol node started — tank mode with odom.");
  }

private:
  // ── Utility ────────────────────────────────────────────────────────────────
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr & msg, float angle) const
  {
    const int total_rays = static_cast<int>(msg->ranges.size());
    int idx = static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
    return std::max(0, std::min(idx, total_rays - 1));
  }

  // ── Odom callback — tracks real yaw continuously ───────────────────────────
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_yaw_ = yaw_from_odom(msg);
  }

  // ── Laser callback — only updates when PATROLLING ─────────────────────────
  // Frozen during ROTATING so a stale scan cannot cancel an ongoing turn.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (state_ == State::ROTATING) return;

    // Angle-derived bounds — hardware agnostic
    const int start_idx   = angle_to_index(msg, -M_PI / 2.0f);   // -90°
    const int end_idx     = angle_to_index(msg,  M_PI / 2.0f);   // +90°
    const int front_start = angle_to_index(msg, -M_PI / 4.0f);   // -45° widened cone
    const int front_end   = angle_to_index(msg,  M_PI / 4.0f);   // +45°

    obstacle_detected_ = false;
    float max_dist = 0.0f;

    for (int i = start_idx; i <= end_idx; ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range)) continue;

      // Obstacle check — wider ±45° cone catches physical obstacles early
      if (i >= front_start && i <= front_end && range < OBSTACLE_THRESHOLD) {
        obstacle_detected_ = true;
      }

      // Track the most open direction across the full 180°
      if (range > max_dist) {
        max_dist   = range;
        direction_ = msg->angle_min + (i * msg->angle_increment);
      }
    }

    if (!obstacle_detected_) direction_ = 0.0f;
  }

  // ── Control loop — 10 Hz state machine ────────────────────────────────────
  void control_loop()
  {
    auto cmd = geometry_msgs::msg::Twist();

    switch (state_) {

      // ── PATROLLING: move straight until an obstacle is seen ──────────────
      case State::PATROLLING:
        if (obstacle_detected_) {
          // Compute the absolute world yaw to rotate to
          target_yaw_  = normalize_angle(current_yaw_ + direction_);
          stop_counter_ = STOP_TICKS;
          state_        = State::STOPPING;
          RCLCPP_INFO(this->get_logger(),
            "Obstacle — stopping. Will rotate to yaw: %.2f rad", target_yaw_);
        } else {
          cmd.linear.x  = LINEAR_VEL;
          cmd.angular.z = 0.0f;
        }
        break;

      // ── STOPPING: full stop for 300 ms before rotating ───────────────────
      case State::STOPPING:
        cmd.linear.x  = 0.0f;
        cmd.angular.z = 0.0f;
        if (--stop_counter_ <= 0) {
          state_ = State::ROTATING;
          RCLCPP_INFO(this->get_logger(), "Rotating in place...");
        }
        break;

      // ── ROTATING: spin in place until odom confirms target yaw reached ───
      case State::ROTATING: {
        const float yaw_error = normalize_angle(target_yaw_ - current_yaw_);
        if (std::abs(yaw_error) < YAW_TOLERANCE) {
          obstacle_detected_ = false;
          state_ = State::PATROLLING;
          RCLCPP_INFO(this->get_logger(), "Rotation complete — resuming patrol.");
        } else {
          cmd.linear.x  = 0.0f;                                         // tank: no forward motion while turning
          cmd.angular.z = (yaw_error > 0.0f) ? ROTATION_SPEED : -ROTATION_SPEED;
        }
        break;
      }
    }

    vel_pub_->publish(cmd);
  }

  // ── Constants ──────────────────────────────────────────────────────────────
  static constexpr float LINEAR_VEL        = 0.1f;   // m/s   — forward speed
  static constexpr float OBSTACLE_THRESHOLD = 0.5f;   // m     — detection distance (increased)
  static constexpr float ROTATION_SPEED    = 0.3f;   // rad/s — in-place turn speed
  static constexpr float YAW_TOLERANCE     = 0.1f;  // rad   — rotation completion threshold
  static constexpr int   STOP_TICKS        = 3;      // × 100ms = 300 ms full stop

  // ── State ──────────────────────────────────────────────────────────────────
  float direction_;
  bool  obstacle_detected_;
  State state_;
  float current_yaw_;
  float target_yaw_;
  int   stop_counter_;

  // ── ROS 2 handles ──────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}