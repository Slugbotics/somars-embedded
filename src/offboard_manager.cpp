#include "somars_controls/offboard_manager.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace somars_controls
{

OffboardManager::OffboardManager()
: Node("offboard_manager")
{
  // ---- declare & load parameters ----
  this->declare_parameter("heartbeat_rate_hz", 10.0);
  this->declare_parameter("auto_arm", false);
  this->declare_parameter("arm_delay_s", 2.0);

  heartbeat_rate_hz_ = this->get_parameter("heartbeat_rate_hz").as_double();
  auto_arm_          = this->get_parameter("auto_arm").as_bool();
  arm_delay_s_       = this->get_parameter("arm_delay_s").as_double();

  if (auto_arm_) {
    RCLCPP_WARN(this->get_logger(),
      "auto_arm is ENABLED – drone will arm automatically after %.1f s", arm_delay_s_);
  }

  // ---- QoS ----
  auto px4_qos = rclcpp::SensorDataQoS();

  // ---- subscriptions ----
  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", px4_qos,
    std::bind(&OffboardManager::status_cb, this, std::placeholders::_1));

  // ---- publishers ----
  control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);

  setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);

  command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  // ---- heartbeat timer ----
  auto period = std::chrono::duration<double>(1.0 / heartbeat_rate_hz_);
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&OffboardManager::heartbeat_loop, this));

  RCLCPP_INFO(this->get_logger(),
    "OffboardManager started – heartbeat at %.0f Hz", heartbeat_rate_hz_);
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void OffboardManager::status_cb(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  nav_state_    = msg->nav_state;
  arming_state_ = msg->arming_state;
}

// ---------------------------------------------------------------------------
// Heartbeat
// ---------------------------------------------------------------------------

void OffboardManager::heartbeat_loop()
{
  uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;  // µs

  // 1. OffboardControlMode – must be published continuously
  px4_msgs::msg::OffboardControlMode mode{};
  mode.position     = true;
  mode.velocity     = false;
  mode.acceleration = false;
  mode.attitude     = false;
  mode.body_rate    = false;
  mode.timestamp    = ts;
  control_mode_pub_->publish(mode);

  // 2. Hold-position setpoint (keeps PX4 happy while no guidance is active).
  //    The guidance_node will overwrite this once it starts publishing.
  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp    = ts;
  sp.position[0]  = 0.0f;
  sp.position[1]  = 0.0f;
  sp.position[2]  = -10.0f;   // 10 m AGL (NED)
  sp.yaw          = 0.0f;
  setpoint_pub_->publish(sp);

  heartbeat_count_++;

  // 3. Auto-arm sequence (only if enabled)
  if (!auto_arm_) {
    return;
  }

  int required_beats = static_cast<int>(arm_delay_s_ * heartbeat_rate_hz_);

  if (heartbeat_count_ == required_beats) {
    RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
    set_offboard_mode();
  }

  if (heartbeat_count_ == required_beats + 1) {
    RCLCPP_INFO(this->get_logger(), "Sending ARM command");
    arm();
  }
}

// ---------------------------------------------------------------------------
// Vehicle commands
// ---------------------------------------------------------------------------

void OffboardManager::publish_vehicle_command(
  uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp       = this->get_clock()->now().nanoseconds() / 1000;
  msg.param1          = param1;
  msg.param2          = param2;
  msg.command          = command;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  command_pub_->publish(msg);
}

void OffboardManager::arm()
{
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0f);
  RCLCPP_INFO(this->get_logger(), "ARM command sent");
}

void OffboardManager::disarm()
{
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.0f);
  RCLCPP_INFO(this->get_logger(), "DISARM command sent");
}

void OffboardManager::set_offboard_mode()
{
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    1.0f,
    6.0f);   // 6 = PX4 custom mode for offboard
  RCLCPP_INFO(this->get_logger(), "OFFBOARD mode command sent");
}

}  // namespace somars_controls

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<somars_controls::OffboardManager>());
  rclcpp::shutdown();
  return 0;
}
