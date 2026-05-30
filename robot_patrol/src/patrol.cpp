#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

using Clock = std::chrono::steady_clock;

// ── Helpers ───────────────────────────────────────────────────────────────────

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

// ── Node ──────────────────────────────────────────────────────────────────────
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
    start_yaw_(0.0f)
  {
    // Queue depth of 20 — absorbs message bursts over a remote network connection
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

    RCLCPP_INFO(this->get_logger(), "Patrol node started — tank mode, odom-guided, latency-tolerant.");
  }

private:
  // ── Utility ──────────────────────────────────────────────────────────────────
  // Derives a scan array index from a real-world angle.
  // Uses the scan's own metadata — works on any lidar model or mounting.
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr & msg, float angle) const
  {
    const int total_rays = static_cast<int>(msg->ranges.size());
    int idx = static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
    return std::max(0, std::min(idx, total_rays - 1));
  }

  // Returns seconds elapsed since a recorded time point
  float elapsed_since(const Clock::time_point & t) const
  {
    return std::chrono::duration<float>(Clock::now() - t).count();
  }

  // ── Odom callback ─────────────────────────────────────────────────────────
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_yaw_ = yaw_from_odom(msg);
  }

  // ── Laser callback ────────────────────────────────────────────────────────
  // Frozen during STOPPING and ROTATING — a stale scan must never
  // interfere with a manoeuvre already in progress.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (state_ == State::STOPPING || state_ == State::ROTATING) return;

    // All bounds derived from real angles — hardware agnostic
    const int start_idx   = angle_to_index(msg, -M_PI / 2.0f);   // -90°
    const int end_idx     = angle_to_index(msg,  M_PI / 2.0f);   // +90°
    const int front_start = angle_to_index(msg, -M_PI / 4.0f);   // -45° cone
    const int front_end   = angle_to_index(msg,  M_PI / 4.0f);   // +45° cone

    obstacle_detected_ = false;
    float max_dist = 0.0f;

    for (int i = start_idx; i <= end_idx; ++i) {
      const float range = msg->ranges[i];
      if (!std::isfinite(range)) continue;

      // Obstacle check — ±45° front cone at 0.5m detection distance
      if (i >= front_start && i <= front_end && range < OBSTACLE_THRESHOLD) {
        obstacle_detected_ = true;
      }

      // Track the ray with the greatest valid distance across the full 180°
      if (range > max_dist) {
        max_dist   = range;
        direction_ = msg->angle_min + (i * msg->angle_increment);
      }
    }

    // Clear path — reset steering, discard any stale direction
    if (!obstacle_detected_) direction_ = 0.0f;
  }

  // ── Control loop — 10 Hz state machine ───────────────────────────────────
  // All timing uses wall-clock (steady_clock) — robust under network latency.
  // Tick-based counters are unreliable in remote robot environments where
  // messages arrive irregularly due to network jitter.
  void control_loop()
  {
    auto cmd = geometry_msgs::msg::Twist();

    switch (state_) {

      // ── PATROLLING ───────────────────────────────────────────────────────
      // Moves forward. Post-rotation cooldown suppresses obstacle detection
      // for COOLDOWN_DURATION_S seconds, ensuring the robot clears the
      // obstacle zone before the detector re-arms.
      case State::PATROLLING:

        if (elapsed_since(cooldown_start_) < COOLDOWN_DURATION_S) {
          cmd.linear.x  = LINEAR_VEL;
          cmd.angular.z = 0.0f;
          break;
        }

        if (obstacle_detected_) {
          target_yaw_   = normalize_angle(current_yaw_ + direction_);
          state_enter_time_ = Clock::now();
          state_        = State::STOPPING;
          RCLCPP_INFO(this->get_logger(),
            "Obstacle detected — stopping. Target yaw: %.2f rad", target_yaw_);
        } else {
          cmd.linear.x  = LINEAR_VEL;
          cmd.angular.z = 0.0f;
        }
        break;

      // ── STOPPING ─────────────────────────────────────────────────────────
      // Full stop for STOP_DURATION_S seconds before rotating.
      // Wall-clock based — a delayed network message cannot shorten this.
      case State::STOPPING:
        cmd.linear.x  = 0.0f;
        cmd.angular.z = 0.0f;
        if (elapsed_since(state_enter_time_) >= STOP_DURATION_S) {
          start_yaw_ = current_yaw_;   // record physical start of rotation
          state_     = State::ROTATING;
          RCLCPP_INFO(this->get_logger(),
            "Rotating — from yaw: %.2f rad → target: %.2f rad",
            start_yaw_, target_yaw_);
        }
        break;

      // ── ROTATING ─────────────────────────────────────────────────────────
      // Rotates in place using odom feedback.
      // MIN_ROTATION guard ensures the robot has physically turned before
      // completion is declared — prevents one-tick false completions caused
      // by odom drift or network latency.
      case State::ROTATING: {
        const float yaw_error      = normalize_angle(target_yaw_ - current_yaw_);
        const float rotated_so_far = std::abs(normalize_angle(current_yaw_ - start_yaw_));

        if (rotated_so_far >= MIN_ROTATION && std::abs(yaw_error) < YAW_TOLERANCE) {
          obstacle_detected_ = false;
          cooldown_start_    = Clock::now();   // start post-rotation immunity
          state_             = State::PATROLLING;
          RCLCPP_INFO(this->get_logger(),
            "Rotation complete (%.2f rad) — resuming patrol.", rotated_so_far);
        } else {
          cmd.linear.x  = 0.0f;               // tank: no forward motion while turning
          cmd.angular.z = (yaw_error > 0.0f) ? ROTATION_SPEED : -ROTATION_SPEED;
        }
        break;
      }
    }

    vel_pub_->publish(cmd);
  }

  // ── Constants ─────────────────────────────────────────────────────────────
  static constexpr float LINEAR_VEL         = 0.1f;   // m/s   — forward speed
  static constexpr float OBSTACLE_THRESHOLD  = 0.5f;   // m     — detection distance
  static constexpr float ROTATION_SPEED     = 0.3f;   // rad/s — in-place turn speed
  static constexpr float YAW_TOLERANCE      = 0.1f;   // rad   — rotation completion threshold (generous for network lag)
  static constexpr float MIN_ROTATION       = 0.5f;   // rad   — minimum physical turn before completion check
  static constexpr float STOP_DURATION_S    = 0.5f;   // s     — full stop duration before rotating
  static constexpr float COOLDOWN_DURATION_S = 1.5f;  // s     — post-rotation forward movement before re-arming

  // ── State ─────────────────────────────────────────────────────────────────
  float          direction_;
  bool           obstacle_detected_;
  State          state_;
  float          current_yaw_;
  float          target_yaw_;
  float          start_yaw_;           // yaw recorded when ROTATING begins
  Clock::time_point state_enter_time_; // wall-clock timestamp of last state transition
  Clock::time_point cooldown_start_;   // wall-clock timestamp of post-rotation cooldown

  // ── ROS 2 handles ─────────────────────────────────────────────────────────
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