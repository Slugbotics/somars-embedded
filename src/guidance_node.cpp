#include "somars_controls/guidance_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace somars_controls
{

GuidanceNode::GuidanceNode()
: Node("guidance_node"),
  last_target_time_(this->now())
{
  // ---- declare & load parameters ----
  this->declare_parameter("approach_speed", 2.0);
  this->declare_parameter("acceptance_radius", 1.0);
  this->declare_parameter("loiter_altitude", -10.0);
  this->declare_parameter("control_rate_hz", 20.0);

  approach_speed_    = this->get_parameter("approach_speed").as_double();
  acceptance_radius_ = this->get_parameter("acceptance_radius").as_double();
  loiter_altitude_   = this->get_parameter("loiter_altitude").as_double();
  double rate_hz     = this->get_parameter("control_rate_hz").as_double();

  RCLCPP_INFO(this->get_logger(),
    "Guidance: speed=%.1f m/s  radius=%.1f m  alt=%.1f m  rate=%.0f Hz",
    approach_speed_, acceptance_radius_, loiter_altitude_, rate_hz);

  // ---- QoS ----
  auto px4_qos = rclcpp::SensorDataQoS();

  // ---- subscriptions ----
  target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/targets/ned", 10,
    std::bind(&GuidanceNode::target_cb, this, std::placeholders::_1));

  local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", px4_qos,
    std::bind(&GuidanceNode::local_position_cb, this, std::placeholders::_1));

  // ---- publishers (to PX4 via micro-XRCE-DDS) ----
  setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);

  control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);

  // ---- control loop timer ----
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GuidanceNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "GuidanceNode started");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void GuidanceNode::target_cb(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  target_position_ned_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
  target_received_ = true;
  last_target_time_ = this->now();
}

void GuidanceNode::local_position_cb(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  vehicle_position_ned_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
  position_received_ = true;
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

void GuidanceNode::control_loop()
{
  // -- Always publish OffboardControlMode so PX4 stays in offboard ----------
  px4_msgs::msg::OffboardControlMode mode_msg{};
  mode_msg.position     = true;
  mode_msg.velocity     = false;
  mode_msg.acceleration = false;
  mode_msg.attitude     = false;
  mode_msg.body_rate    = false;
  mode_msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;  // µs
  control_mode_pub_->publish(mode_msg);

  // -- Compute setpoint -----------------------------------------------------
  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = mode_msg.timestamp;

  // NaN = "don't care" for PX4
  sp.velocity[0]     = std::numeric_limits<float>::quiet_NaN();
  sp.velocity[1]     = std::numeric_limits<float>::quiet_NaN();
  sp.velocity[2]     = std::numeric_limits<float>::quiet_NaN();
  sp.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
  sp.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
  sp.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
  sp.jerk[0]         = std::numeric_limits<float>::quiet_NaN();
  sp.jerk[1]         = std::numeric_limits<float>::quiet_NaN();
  sp.jerk[2]         = std::numeric_limits<float>::quiet_NaN();
  sp.yaw             = std::numeric_limits<float>::quiet_NaN();  // hold current
  sp.yawspeed        = std::numeric_limits<float>::quiet_NaN();

  if (!position_received_) {
    // Nothing we can do until we know where we are.
    setpoint_pub_->publish(sp);
    return;
  }

  // Expire targets older than 2 seconds
  bool target_stale = (this->now() - last_target_time_).seconds() > 2.0;

  if (target_received_ && !target_stale) {
    // ---- Navigate towards target ----
    Eigen::Vector3d to_target = target_position_ned_ - vehicle_position_ned_;

    // Keep altitude at loiter_altitude_, only move horizontally toward target
    to_target.z() = 0.0;
    double dist = to_target.norm();

    Eigen::Vector3d desired_pos;
    if (dist < acceptance_radius_) {
      // Close enough – hold above the target at loiter altitude
      desired_pos = Eigen::Vector3d(
        target_position_ned_.x(),
        target_position_ned_.y(),
        loiter_altitude_);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Within acceptance radius (%.1f m) – holding position", dist);
    } else {
      // Move towards target at approach_speed_, staying at loiter altitude
      Eigen::Vector3d direction = to_target.normalized();
      double step = std::min(approach_speed_ * 0.5, dist);  // half-second lookahead
      desired_pos = vehicle_position_ned_ + direction * step;
      desired_pos.z() = loiter_altitude_;

      RCLCPP_DEBUG(this->get_logger(),
        "Approaching target: dist=%.1f m  setpoint=[%.1f, %.1f, %.1f]",
        dist, desired_pos.x(), desired_pos.y(), desired_pos.z());
    }

    sp.position[0] = static_cast<float>(desired_pos.x());
    sp.position[1] = static_cast<float>(desired_pos.y());
    sp.position[2] = static_cast<float>(desired_pos.z());

  } else {
    // ---- No active target – hold current XY at loiter altitude ----
    sp.position[0] = static_cast<float>(vehicle_position_ned_.x());
    sp.position[1] = static_cast<float>(vehicle_position_ned_.y());
    sp.position[2] = static_cast<float>(loiter_altitude_);

    if (target_stale && target_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Target data stale – holding position");
    }
  }

  setpoint_pub_->publish(sp);
}

}  // namespace somars_controls

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<somars_controls::GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}
