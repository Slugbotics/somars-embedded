#ifndef SOMARS_CONTROLS__PROJECTION_HPP_
#define SOMARS_CONTROLS__PROJECTION_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace somars_controls
{
namespace projection
{

/// Build the fixed rotation matrix from camera optical frame to body FRD frame.
///
/// Camera optical convention:  X = right,   Y = down,    Z = into scene
/// Body FRD convention:        X = forward, Y = right,   Z = down
///
/// When cam_pitch_rad = 0   → camera looks forward (along body X)
/// When cam_pitch_rad = -π/2 → camera looks straight down (along body Z)
inline Eigen::Matrix3d build_cam_to_body_rotation(double cam_pitch_rad)
{
  // Step 1: Base alignment when camera is forward-looking (pitch = 0).
  //   body X (fwd)   = cam Z (into scene)
  //   body Y (right)  = cam X (right)
  //   body Z (down)   = cam Y (down)
  Eigen::Matrix3d R_base;
  R_base << 0, 0, 1,
            1, 0, 0,
            0, 1, 0;

  // Step 2: Apply pitch rotation about the body Y-axis.
  Eigen::Matrix3d R_pitch =
    Eigen::AngleAxisd(cam_pitch_rad, Eigen::Vector3d::UnitY()).toRotationMatrix();

  return R_pitch * R_base;
}

/// Back-project a single pixel (u, v) through a pinhole camera model,
/// rotate into NED via the vehicle attitude, and intersect with the
/// ground plane (NED z = 0).
///
/// @param[in]  u, v                Pixel coordinates (col, row)
/// @param[in]  fx, fy, cx, cy      Camera intrinsics
/// @param[in]  R_cam_to_body       Camera-to-body rotation (from build_cam_to_body_rotation)
/// @param[in]  vehicle_attitude    Body-to-NED quaternion (from PX4 VehicleAttitude)
/// @param[in]  vehicle_position_ned  Drone position in NED (z < 0 when above ground)
/// @param[out] target_ned          Resulting ground-plane intersection in NED
/// @return true if the ray intersects the ground plane (z = 0)
inline bool project_pixel_to_ned(
  double u, double v,
  double fx, double fy, double cx, double cy,
  const Eigen::Matrix3d & R_cam_to_body,
  const Eigen::Quaterniond & vehicle_attitude,
  const Eigen::Vector3d & vehicle_position_ned,
  Eigen::Vector3d & target_ned)
{
  // 1. Pixel → normalised camera-frame ray
  Eigen::Vector3d ray_cam((u - cx) / fx, (v - cy) / fy, 1.0);
  ray_cam.normalize();

  // 2. Camera frame → body frame → NED frame
  Eigen::Matrix3d R_body_to_ned = vehicle_attitude.toRotationMatrix();
  Eigen::Vector3d ray_ned = R_body_to_ned * R_cam_to_body * ray_cam;

  // 3. Intersect with ground plane (NED z = 0).
  //    Drone z is negative when above ground.
  //    Ray must point downward (positive z component) to hit ground.
  if (ray_ned.z() <= 1e-6) {
    return false;
  }

  double t = -vehicle_position_ned.z() / ray_ned.z();
  target_ned = vehicle_position_ned + t * ray_ned;
  target_ned.z() = 0.0;
  return true;
}

}  // namespace projection
}  // namespace somars_controls

#endif  // SOMARS_CONTROLS__PROJECTION_HPP_
