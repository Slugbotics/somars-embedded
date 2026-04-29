#ifndef SOMARS_CONTROLS__GUIDANCE_NODE_HPP_
#define SOMARS_CONTROLS__GUIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <Eigen/Dense>

namespace somars_controls
{

/// Generates trajectory setpoints that steer the drone towards detected
/// targets.  When no target is active the node publishes a position-hold
/// at the current loiter altitude.
///
/// Subscriptions
///   /targets/ned                      – PointStamped (from TargetLocalizer)
///   /fmu/out/vehicle_local_position   – VehicleLocalPosition (PX4)
///
/// Publications
///   /fmu/in/trajectory_setpoint       – TrajectorySetpoint (to PX4)
///   /fmu/in/offboard_control_mode     – OffboardControlMode (to PX4)
class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode();

private:
  // ---- callbacks ----
  void target_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  /// Fixed-rate control loop called by the timer.
  void control_loop();

  // ---- subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  // ---- publishers ----
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;

  // ---- timer ----
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---- state ----
  Eigen::Vector3d vehicle_position_ned_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_ned_{0.0, 0.0, 0.0};
  bool target_received_   = false;
  bool position_received_ = false;

  /// Timestamp (steady clock) of the last target message.  Used to expire
  /// stale targets.
  rclcpp::Time last_target_time_;

  // ---- parameters ----
  double approach_speed_;
  double acceptance_radius_;
  double loiter_altitude_;   // NED z – negative means above ground
};

}  // namespace somars_controls

#endif  // SOMARS_CONTROLS__GUIDANCE_NODE_HPP_
