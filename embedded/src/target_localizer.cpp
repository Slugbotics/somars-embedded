#include "somars_controls/target_localizer.hpp"
#include "somars_controls/projection.hpp"
#include "messages/msg/detection.hpp"

#include <cmath>
#include <string>

namespace somars_controls
{

TargetLocalizer::TargetLocalizer()
: Node("target_localizer")
{
  // ---- declare & load parameters ----
  this->declare_parameter("camera_fx", 600.0);
  this->declare_parameter("camera_fy", 600.0);
  this->declare_parameter("camera_cx", 320.0);
  this->declare_parameter("camera_cy", 240.0);
  this->declare_parameter("camera_pitch_rad", -M_PI_2);

  fx_ = this->get_parameter("camera_fx").as_double();
  fy_ = this->get_parameter("camera_fy").as_double();
  cx_ = this->get_parameter("camera_cx").as_double();
  cy_ = this->get_parameter("camera_cy").as_double();
  double cam_pitch = this->get_parameter("camera_pitch_rad").as_double();

  // Build the fixed camera-to-body rotation.
  // Handles the base alignment (camera optical → body FRD) PLUS the
  // pitch offset for the physical mount angle.
  //   camera: X = right, Y = down, Z = into scene
  //   body:   X = forward, Y = right, Z = down  (FRD)
  R_cam_to_body_ = projection::build_cam_to_body_rotation(cam_pitch);

  RCLCPP_INFO(this->get_logger(),
    "Camera intrinsics  fx=%.1f fy=%.1f cx=%.1f cy=%.1f  pitch=%.2f rad",
    fx_, fy_, cx_, cy_, cam_pitch);

  // Warn if using default (uncalibrated) intrinsics
  if (fx_ == 600.0 && fy_ == 600.0 && cx_ == 320.0 && cy_ == 240.0) {
    RCLCPP_WARN(this->get_logger(),
      "Using DEFAULT camera intrinsics — run tools/calibrate_camera.py and "
      "update config/params.yaml for accurate target localisation!");
  }
  if (cam_pitch > -0.1 && cam_pitch < 0.1) {
    RCLCPP_WARN(this->get_logger(),
      "camera_pitch_rad=%.2f (near 0) — camera is forward-facing, "
      "most pixels will NOT hit the ground plane!", cam_pitch);
  }
  if (std::fabs(cam_pitch + M_PI_2) > 0.01 && std::fabs(cam_pitch) > 0.01) {
    RCLCPP_INFO(this->get_logger(),
      "Camera mounted at %.1f° from vertical — ensure mount angle is correct",
      (cam_pitch + M_PI_2) * 180.0 / M_PI);
  }

  // ---- QoS for PX4 topics (best-effort / volatile) ----
  auto px4_qos = rclcpp::SensorDataQoS();

  // ---- subscriptions ----
  attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", px4_qos,
    std::bind(&TargetLocalizer::attitude_cb, this, std::placeholders::_1));

  local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", px4_qos,
    std::bind(&TargetLocalizer::local_position_cb, this, std::placeholders::_1));

  // Vision detections – uses default reliable QoS (local ROS2 topic)
  detection_sub_ = this->create_subscription<messages::msg::Detection>(
    "/vision/detection", 10,
    std::bind(&TargetLocalizer::detection_cb, this, std::placeholders::_1));

  // ---- publishers ----
  target_ned_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/targets/ned", 10);
  
  best_target_ned_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/targets/best_ned", 10);

  RCLCPP_INFO(this->get_logger(), "TargetLocalizer node started");
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void TargetLocalizer::attitude_cb(
  const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  // PX4 quaternion order: [w, x, y, z] stored in q[0..3]
  vehicle_attitude_ = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  attitude_received_ = true;
}

void TargetLocalizer::local_position_cb(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  vehicle_position_ned_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
  position_received_ = true;

  // Warn if altitude is too low for reliable ground projection
  if (msg->z > -3.0 && msg->z < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Altitude %.1f m AGL — target projection unreliable below ~5m", -(double)msg->z);
  }
}

void TargetLocalizer::detection_cb(
  const messages::msg::Detection::SharedPtr msg)
{
  if (!attitude_received_ || !position_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for vehicle attitude & position before localising targets");
    return;
  }

  double u = static_cast<double>(msg->x);   // pixel column
  double v = static_cast<double>(msg->y);   // pixel row
  double timestamp = msg->timestamp;        // seconds since epoch
  double confidence = msg->confidence;      // 0.0 to 1.0
  int class_id = static_cast<int>(msg->class_id);
  if(class_id != 0 || timestamp <= 0.0 || confidence <= 0.0){
    // Not a valid target
    return;
  }

  Eigen::Vector3d target_ned;
  // TODO: account for latency by using the timestamp to get the correct vehicle state at time of capture
  if (!pixel_to_ned(u, v, target_ned)) {
    RCLCPP_DEBUG(this->get_logger(), "Ray did not intersect ground plane");
    return;
  }

  // Sanity check: reject targets projected unreasonably far from drone
  double horiz_dist = (target_ned - vehicle_position_ned_).head<2>().norm();
  if (horiz_dist > 200.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Target projected %.0f m away — likely bad projection, discarding",
       horiz_dist);
    return;
  }

  geometry_msgs::msg::PointStamped out;
  out.header.stamp    = this->now();
  out.header.frame_id = std::string("target");
  out.point.x = target_ned.x();
  out.point.y = target_ned.y();
  out.point.z = target_ned.z();
  target_ned_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "Target NED: [%.2f, %.2f, %.2f]  from pixel [%.0f, %.0f]",
    target_ned.x(), target_ned.y(), target_ned.z(), u, v);

  if(confidence > best_confidence_){
    best_confidence_ = confidence;
    best_target_ned_pub_->publish(out);
  }
}

// ---------------------------------------------------------------------------
// Projection math
// ---------------------------------------------------------------------------

bool TargetLocalizer::pixel_to_ned(
  double u, double v, Eigen::Vector3d & target_ned) const
{
  return projection::project_pixel_to_ned(
    u, v, fx_, fy_, cx_, cy_,
    R_cam_to_body_, vehicle_attitude_, vehicle_position_ned_,
    target_ned);
}

Eigen::Matrix3d TargetLocalizer::quat_to_rotation(
  double w, double x, double y, double z)
{
  return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
}

}  // namespace somars_controls

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<somars_controls::TargetLocalizer>());
  rclcpp::shutdown();
  return 0;
}
