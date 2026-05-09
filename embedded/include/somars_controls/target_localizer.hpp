#ifndef SOMARS_CONTROLS__TARGET_LOCALIZER_HPP_
#define SOMARS_CONTROLS__TARGET_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "messages/msg/detection.hpp"

#include <Eigen/Dense>

namespace somars_controls
{

/// Projects pixel-space detections from the vision pipeline into NED-frame
/// world coordinates using the drone's attitude, position and camera model.
///
/// Subscriptions
///   /fmu/out/vehicle_attitude         – VehicleAttitude   (PX4)
///   /fmu/out/vehicle_local_position   – VehicleLocalPosition (PX4)
///   /vision/detection                – Detection (from somars-vision)
///       Fields: x = pixel column (u)
///               y = pixel row (v)
///               class_id = class id (0=red, 1=black, 2=white)
///
/// Publications
///   /targets/ned  – PointStamped (target position in local NED frame)
class TargetLocalizer : public rclcpp::Node
{
public:
  TargetLocalizer();

private:
  // ---- callbacks ----
  void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void detection_cb(const messages::msg::Detection::SharedPtr msg);

  // ---- helpers ----
  /// Back-project a single pixel (u, v) through the camera model, rotate into
  /// NED, and intersect with the ground plane (z = 0).
  /// Returns false if the ray does not intersect the ground.
  bool pixel_to_ned(double u, double v, Eigen::Vector3d & target_ned) const;

  /// Convert quaternion to rotation matrix (Hamilton convention, wxyz).
  static Eigen::Matrix3d quat_to_rotation(double w, double x, double y, double z);

  // ---- subscribers ----
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<messages::msg::Detection>::SharedPtr detection_sub_;

  // ---- publishers ----
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_ned_pub_;

  // ---- state ----
  Eigen::Quaterniond vehicle_attitude_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d    vehicle_position_ned_{0.0, 0.0, 0.0};
  bool attitude_received_  = false;
  bool position_received_  = false;

  // ---- camera parameters (loaded from params.yaml) ----
  double fx_, fy_, cx_, cy_;

  /// Fixed rotation from camera optical frame to body FRD frame.
  Eigen::Matrix3d R_cam_to_body_;
};

}  // namespace somars_controls

#endif  // SOMARS_CONTROLS__TARGET_LOCALIZER_HPP_
