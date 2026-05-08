#ifndef SOMARS_CONTROLS__GUIDANCE_NODE_HPP_
#define SOMARS_CONTROLS__GUIDANCE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <Eigen/Dense>
#include <vector>

namespace somars_controls
{

/// Unified guidance node — handles both waypoint navigation and
/// vision-target tracking with the following priority:
///
///   1. Vision target active  → track it (for payload delivery)
///   2. Waypoints remaining   → fly to next waypoint
///   3. Nothing to do         → hold position at loiter altitude
///
/// Waypoints are loaded from config/waypoints.yaml (GPS lat/lon/alt).
/// GPS coordinates are converted to NED once the PX4 reference origin
/// is received.
///
/// Subscriptions
///   /targets/ned                      – PointStamped (from TargetLocalizer)
///   /fmu/out/vehicle_local_position   – VehicleLocalPosition (PX4)
///
/// Publications
///   /fmu/in/trajectory_setpoint       – TrajectorySetpoint (to PX4)
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

  // ---- waypoint helpers ----
  void load_waypoints();
  void convert_waypoints_to_ned();
  Eigen::Vector3d gps_to_ned(double lat_deg, double lon_deg, double alt_m_msl) const;

  // ---- subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  // ---- publishers ----
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;

  // ---- timer ----
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---- vehicle state ----
  Eigen::Vector3d vehicle_position_ned_{0.0, 0.0, 0.0};
  Eigen::Vector3d target_position_ned_{0.0, 0.0, 0.0};
  bool target_received_   = false;
  bool position_received_ = false;

  /// Timestamp of the last vision target.  Used to expire stale targets.
  rclcpp::Time last_target_time_;

  // ---- waypoint state ----
  struct GpsWaypoint { double lat, lon, alt_m; };

  std::vector<GpsWaypoint>   gps_waypoints_;    // raw from config
  std::vector<Eigen::Vector3d> ned_waypoints_;   // computed after ref received
  size_t current_wp_index_ = 0;
  int    current_lap_      = 0;
  int    total_laps_       = 1;
  double waypoint_radius_  = 30.0;   // meters
  bool   waypoints_loaded_     = false;
  bool   ned_waypoints_ready_  = false;

  // PX4 NED reference origin (captured once from VehicleLocalPosition)
  double ref_lat_ = 0.0;
  double ref_lon_ = 0.0;
  double ref_alt_m_ = 0.0;   // MSL in meters
  bool   ref_received_ = false;

  // ---- parameters ----
  double approach_speed_;
  double acceptance_radius_;
  double loiter_altitude_;   // NED z – negative means above ground
};

}  // namespace somars_controls

#endif  // SOMARS_CONTROLS__GUIDANCE_NODE_HPP_
