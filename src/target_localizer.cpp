#include "somars_controls/target_localizer.hpp"

#include <cmath>

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
  // Assumes the camera optical axis is mounted in the body XZ-plane,
  // pitched by cam_pitch about the body Y-axis.
  //   body X = forward, Y = right, Z = down  (FRD)
  //   camera Z = optical axis (into scene)
  R_cam_to_body_ = Eigen::AngleAxisd(cam_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

  RCLCPP_INFO(this->get_logger(),
    "Camera intrinsics  fx=%.1f fy=%.1f cx=%.1f cy=%.1f  pitch=%.2f rad",
    fx_, fy_, cx_, cy_, cam_pitch);

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
  detection_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/vision/detections", 10,
    std::bind(&TargetLocalizer::detection_cb, this, std::placeholders::_1));

  // ---- publishers ----
  target_ned_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/targets/ned", 10);

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
}

void TargetLocalizer::detection_cb(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!attitude_received_ || !position_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for vehicle attitude & position before localising targets");
    return;
  }

  for (const auto & pose : msg->poses) {
    double u = pose.position.x;   // pixel column
    double v = pose.position.y;   // pixel row
    // int class_id = static_cast<int>(pose.position.z);  // TODO: use class

    Eigen::Vector3d target_ned;
    if (!pixel_to_ned(u, v, target_ned)) {
      RCLCPP_DEBUG(this->get_logger(), "Ray did not intersect ground plane");
      continue;
    }

    geometry_msgs::msg::PointStamped out;
    out.header.stamp    = this->now();
    out.header.frame_id = "map_ned";
    out.point.x = target_ned.x();
    out.point.y = target_ned.y();
    out.point.z = target_ned.z();
    target_ned_pub_->publish(out);

    RCLCPP_DEBUG(this->get_logger(),
      "Target NED: [%.2f, %.2f, %.2f]",
      target_ned.x(), target_ned.y(), target_ned.z());
  }
}

// ---------------------------------------------------------------------------
// Projection math
// ---------------------------------------------------------------------------

bool TargetLocalizer::pixel_to_ned(
  double u, double v, Eigen::Vector3d & target_ned) const
{
  // 1. Pixel → normalised camera-frame ray
  Eigen::Vector3d ray_cam((u - cx_) / fx_, (v - cy_) / fy_, 1.0);
  ray_cam.normalize();

  // 2. Camera frame → body frame → NED frame
  Eigen::Matrix3d R_body_to_ned = vehicle_attitude_.toRotationMatrix();
  Eigen::Vector3d ray_ned = R_body_to_ned * R_cam_to_body_ * ray_cam;

  // 3. Intersect with ground plane (NED z = 0).
  //    Drone is at vehicle_position_ned_.z() (negative when above ground).
  //    We need the ray to point downward (positive z in NED) to hit ground.
  if (ray_ned.z() <= 1e-6) {
    return false;  // ray is parallel to or pointing away from ground
  }

  double t = -vehicle_position_ned_.z() / ray_ned.z();
  target_ned = vehicle_position_ned_ + t * ray_ned;
  target_ned.z() = 0.0;  // on the ground plane
  return true;
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
