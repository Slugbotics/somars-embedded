// ============================================================================
// Standalone projection math tests – no ROS2, only Eigen.
//
// Build & run:
//   cd test && mkdir build && cd build
//   cmake .. && make && ./test_projection_math
// ============================================================================

#include "somars_controls/projection.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace somars_controls::projection;

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char * label)
{
  if (cond) {
    std::printf("  [PASS] %s\n", label);
    g_pass++;
  } else {
    std::printf("  [FAIL] %s\n", label);
    g_fail++;
  }
}

static void check_near(double a, double b, double tol, const char * label)
{
  bool ok = std::fabs(a - b) < tol;
  if (!ok) {
    std::printf("  [FAIL] %s  (got %.6f, expected %.6f, tol %.6f)\n", label, a, b, tol);
    g_fail++;
  } else {
    std::printf("  [PASS] %s\n", label);
    g_pass++;
  }
}

// ---------------------------------------------------------------------------
// Test: camera-to-body rotation matrix
// ---------------------------------------------------------------------------
static void test_cam_to_body_rotation()
{
  std::printf("\n--- test_cam_to_body_rotation ---\n");

  // Forward-looking camera (pitch = 0)
  {
    Eigen::Matrix3d R = build_cam_to_body_rotation(0.0);
    // Camera Z (into scene) should map to body X (forward)
    Eigen::Vector3d cam_z(0, 0, 1);
    Eigen::Vector3d result = R * cam_z;
    check_near(result.x(), 1.0, 1e-9, "pitch=0: cam_Z → body_X");
    check_near(result.y(), 0.0, 1e-9, "pitch=0: cam_Z → body_X (y=0)");
    check_near(result.z(), 0.0, 1e-9, "pitch=0: cam_Z → body_X (z=0)");

    // Camera X (right) should map to body Y (right)
    Eigen::Vector3d cam_x(1, 0, 0);
    result = R * cam_x;
    check_near(result.x(), 0.0, 1e-9, "pitch=0: cam_X → body_Y (x=0)");
    check_near(result.y(), 1.0, 1e-9, "pitch=0: cam_X → body_Y");
    check_near(result.z(), 0.0, 1e-9, "pitch=0: cam_X → body_Y (z=0)");

    // Camera Y (down in image) should map to body Z (down)
    Eigen::Vector3d cam_y(0, 1, 0);
    result = R * cam_y;
    check_near(result.x(), 0.0, 1e-9, "pitch=0: cam_Y → body_Z (x=0)");
    check_near(result.y(), 0.0, 1e-9, "pitch=0: cam_Y → body_Z (y=0)");
    check_near(result.z(), 1.0, 1e-9, "pitch=0: cam_Y → body_Z");
  }

  // Straight-down camera (pitch = -π/2)
  {
    Eigen::Matrix3d R = build_cam_to_body_rotation(-M_PI_2);
    // Camera Z (into scene) should map to body Z (down)
    Eigen::Vector3d cam_z(0, 0, 1);
    Eigen::Vector3d result = R * cam_z;
    check_near(result.x(), 0.0, 1e-9, "pitch=-90°: cam_Z → body_Z (x=0)");
    check_near(result.y(), 0.0, 1e-9, "pitch=-90°: cam_Z → body_Z (y=0)");
    check_near(result.z(), 1.0, 1e-9, "pitch=-90°: cam_Z → body_Z");

    // Camera X (right) should still map to body Y (right)
    Eigen::Vector3d cam_x(1, 0, 0);
    result = R * cam_x;
    check_near(result.x(), 0.0, 1e-9, "pitch=-90°: cam_X → body_Y (x=0)");
    check_near(result.y(), 1.0, 1e-9, "pitch=-90°: cam_X → body_Y");
    check_near(result.z(), 0.0, 1e-9, "pitch=-90°: cam_X → body_Y (z=0)");
  }
}

// ---------------------------------------------------------------------------
// Test: pixel projection with downward camera at level hover
// ---------------------------------------------------------------------------
static void test_pixel_projection_straight_down()
{
  std::printf("\n--- test_pixel_projection_straight_down ---\n");

  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  Eigen::Matrix3d R_cam_to_body = build_cam_to_body_rotation(-M_PI_2);
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  // level, heading north
  Eigen::Vector3d pos_ned(100.0, 50.0, -20.0);  // 20 m above ground

  Eigen::Vector3d target;

  // Case 1: Center pixel → target directly below drone
  {
    bool hit = project_pixel_to_ned(
      cx, cy, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "center pixel hits ground");
    check_near(target.x(), 100.0, 0.1, "center pixel → target N = drone N");
    check_near(target.y(),  50.0, 0.1, "center pixel → target E = drone E");
    check_near(target.z(),   0.0, 1e-9, "center pixel → target D = 0 (ground)");
  }

  // Case 2: Pixel shifted right in image → target shifted East (body Y = NED East when heading North)
  {
    double u_right = cx + 300.0;  // 300 pixels right of center
    bool hit = project_pixel_to_ned(
      u_right, cy, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "right-of-center pixel hits ground");
    check(target.y() > 50.0, "right pixel → target East > drone East");
    // Expected offset: 20m * tan(atan(300/600)) = 20 * 0.5 = 10m east
    check_near(target.y(), 60.0, 0.5, "right pixel offset ≈ 10m east");
    check_near(target.x(), 100.0, 0.5, "right pixel → same North");
  }

  // Case 3: Pixel shifted DOWN in image (v > cy) → target shifted SOUTH.
  //   For a downward camera the top of the image faces forward (body +X).
  //   Image bottom → body -X → South when heading North.
  {
    double v_down = cy + 300.0;  // 300 pixels below center
    bool hit = project_pixel_to_ned(
      cx, v_down, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "below-center pixel hits ground");
    check(target.x() < 100.0, "below pixel → target South (behind drone)");
    check_near(target.x(), 90.0, 0.5, "below pixel offset ≈ 10m south");
    check_near(target.y(), 50.0, 0.5, "below pixel → same East");
  }

  // Case 4: Pixel shifted UP in image (v < cy) → target shifted NORTH (forward).
  {
    double v_up = cy - 300.0;  // 300 pixels above center
    bool hit = project_pixel_to_ned(
      cx, v_up, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "above-center pixel hits ground");
    check(target.x() > 100.0, "above pixel → target North (ahead of drone)");
    check_near(target.x(), 110.0, 0.5, "above pixel offset ≈ 10m north");
  }
}

// ---------------------------------------------------------------------------
// Test: pixel projection with yawed drone
// ---------------------------------------------------------------------------
static void test_pixel_projection_yawed()
{
  std::printf("\n--- test_pixel_projection_yawed ---\n");

  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  Eigen::Matrix3d R_cam_to_body = build_cam_to_body_rotation(-M_PI_2);
  Eigen::Vector3d pos_ned(0.0, 0.0, -20.0);

  // Drone yawed 90° clockwise (heading East): body X = NED East, body Y = NED South
  Eigen::Quaterniond attitude(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));  // yaw +90° in NED

  Eigen::Vector3d target;

  // Center pixel → still directly below
  {
    bool hit = project_pixel_to_ned(
      cx, cy, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "yawed: center pixel hits ground");
    check_near(target.x(), 0.0, 0.1, "yawed: center → N = 0");
    check_near(target.y(), 0.0, 0.1, "yawed: center → E = 0");
  }

  // Pixel shifted right → now body Y = NED South, so target goes South (negative N)
  {
    double u_right = cx + 300.0;
    bool hit = project_pixel_to_ned(
      u_right, cy, fx, fy, cx, cy,
      R_cam_to_body, attitude, pos_ned, target);
    check(hit, "yawed: right pixel hits ground");
    check(target.x() < -0.1, "yawed: right pixel → South (negative North)");
    check_near(target.x(), -10.0, 0.5, "yawed: right pixel ≈ 10m south");
  }
}

// ---------------------------------------------------------------------------
// Test: ray that misses ground
// ---------------------------------------------------------------------------
static void test_ray_miss()
{
  std::printf("\n--- test_ray_miss ---\n");

  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  // Forward-looking camera (pitch = 0)
  Eigen::Matrix3d R_cam_to_body = build_cam_to_body_rotation(0.0);
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d pos_ned(0.0, 0.0, -20.0);
  Eigen::Vector3d target;

  // Center pixel with forward-looking camera → ray goes forward (horizontal),
  // never hits ground (z component ≈ 0)
  bool hit = project_pixel_to_ned(
    cx, cy, fx, fy, cx, cy,
    R_cam_to_body, attitude, pos_ned, target);
  check(!hit, "forward-looking camera, center pixel: ray misses ground");
}

// ---------------------------------------------------------------------------
// Test: altitude scaling
// ---------------------------------------------------------------------------
static void test_altitude_scaling()
{
  std::printf("\n--- test_altitude_scaling ---\n");

  double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  Eigen::Matrix3d R_cam_to_body = build_cam_to_body_rotation(-M_PI_2);
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();

  double u_offset = cx + 300.0;  // same pixel
  Eigen::Vector3d target_10, target_40;

  // At 10m altitude
  Eigen::Vector3d pos_10(0, 0, -10);
  project_pixel_to_ned(u_offset, cy, fx, fy, cx, cy,
    R_cam_to_body, attitude, pos_10, target_10);

  // At 40m altitude
  Eigen::Vector3d pos_40(0, 0, -40);
  project_pixel_to_ned(u_offset, cy, fx, fy, cx, cy,
    R_cam_to_body, attitude, pos_40, target_40);

  // Offset should scale linearly with altitude
  double ratio = target_40.y() / target_10.y();
  check_near(ratio, 4.0, 0.01, "4x altitude → 4x ground offset");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
  std::printf("========================================\n");
  std::printf("  SOMARS Projection Math Tests\n");
  std::printf("========================================\n");

  test_cam_to_body_rotation();
  test_pixel_projection_straight_down();
  test_pixel_projection_yawed();
  test_ray_miss();
  test_altitude_scaling();

  std::printf("\n========================================\n");
  std::printf("  Results: %d passed, %d failed\n", g_pass, g_fail);
  std::printf("========================================\n");

  return g_fail > 0 ? 1 : 0;
}
