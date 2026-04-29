#ifndef SOMARS_CONTROLS__OFFBOARD_MANAGER_HPP_
#define SOMARS_CONTROLS__OFFBOARD_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace somars_controls
{

/// Manages the PX4 offboard-mode lifecycle.
///
/// Responsibilities:
///   1. Publish OffboardControlMode + a hold-position TrajectorySetpoint at a
///      steady heartbeat rate so PX4 accepts/maintains offboard mode.
///   2. Optionally send arm and mode-switch VehicleCommands after a
///      configurable delay (guarded by the `auto_arm` parameter).
///   3. Monitor VehicleStatus to track arming and nav state.
///
/// Subscriptions
///   /fmu/out/vehicle_status  – VehicleStatus (PX4)
///
/// Publications
///   /fmu/in/offboard_control_mode  – OffboardControlMode
///   /fmu/in/trajectory_setpoint    – TrajectorySetpoint (hold setpoint)
///   /fmu/in/vehicle_command        – VehicleCommand
class OffboardManager : public rclcpp::Node
{
public:
  OffboardManager();

private:
  // ---- callbacks ----
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  /// Timer-driven heartbeat that keeps offboard mode alive.
  void heartbeat_loop();

  // ---- command helpers ----
  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f);
  void arm();
  void disarm();
  void set_offboard_mode();

  // ---- subscribers ----
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

  // ---- publishers ----
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;

  // ---- timer ----
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // ---- state ----
  uint8_t nav_state_    = 0;
  uint8_t arming_state_ = 0;
  int     heartbeat_count_ = 0;

  // ---- parameters ----
  bool   auto_arm_;
  double arm_delay_s_;
  double heartbeat_rate_hz_;
};

}  // namespace somars_controls

#endif  // SOMARS_CONTROLS__OFFBOARD_MANAGER_HPP_
