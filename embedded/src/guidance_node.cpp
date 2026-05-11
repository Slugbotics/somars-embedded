#include "somars_controls/guidance_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace somars_controls
{

static constexpr double R_EARTH = 6371000.0;      // meters
static constexpr double DEG_TO_RAD = M_PI / 180.0;

GuidanceNode::GuidanceNode()
: Node("guidance_node"),
  last_target_time_(this->now())
{
  // ---- declare & load parameters ----
  this->declare_parameter("approach_speed", 2.0);
  this->declare_parameter("acceptance_radius", 1.0);
  this->declare_parameter("loiter_altitude", -10.0);
  this->declare_parameter("control_rate_hz", 20.0);

  // Waypoint parameters (loaded from config/waypoints.yaml)
  this->declare_parameter("waypoint_laps", 1);
  this->declare_parameter("waypoint_radius_m", 30.0);
  this->declare_parameter("waypoints_lat", std::vector<double>{});
  this->declare_parameter("waypoints_lon", std::vector<double>{});
  this->declare_parameter("waypoints_alt_m", std::vector<double>{});

  approach_speed_    = this->get_parameter("approach_speed").as_double();
  acceptance_radius_ = this->get_parameter("acceptance_radius").as_double();
  loiter_altitude_   = this->get_parameter("loiter_altitude").as_double();
  double rate_hz     = this->get_parameter("control_rate_hz").as_double();
  total_laps_        = this->get_parameter("waypoint_laps").as_int();
  waypoint_radius_   = this->get_parameter("waypoint_radius_m").as_double();

  load_waypoints();

  // ---- parameter sanity checks ----
  if (loiter_altitude_ >= 0.0) {
    RCLCPP_WARN(this->get_logger(),
      "loiter_altitude=%.1f — should be NEGATIVE in NED (negative = above ground)!",
      loiter_altitude_);
  }
  if (approach_speed_ > 10.0) {
    RCLCPP_WARN(this->get_logger(),
      "approach_speed=%.1f m/s — unusually high, verify this is intentional",
      approach_speed_);
  }
  if (waypoint_radius_ < 5.0) {
    RCLCPP_WARN(this->get_logger(),
      "waypoint_radius=%.1f m — very tight, GPS accuracy is typically 2-5m",
      waypoint_radius_);
  }

  RCLCPP_INFO(this->get_logger(),
    "Guidance: speed=%.1f m/s  wp_radius=%.0f m  loiter_alt=%.1f m  rate=%.0f Hz",
    approach_speed_, waypoint_radius_, loiter_altitude_, rate_hz);

  // ---- QoS ----
  auto px4_qos = rclcpp::SensorDataQoS();

  // ---- subscriptions ----
  target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/targets/ned", 10,
    std::bind(&GuidanceNode::target_cb, this, std::placeholders::_1));
  
  best_target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/targets/best_ned", 10,
    std::bind(&GuidanceNode::best_target_cb, this, std::placeholders::_1));

  local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", px4_qos,
    std::bind(&GuidanceNode::local_position_cb, this, std::placeholders::_1));

  // ---- publisher (to PX4 via micro-XRCE-DDS) ----
  setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);

  // ---- control loop timer ----
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GuidanceNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "GuidanceNode started");
}

// ---------------------------------------------------------------------------
// Waypoint loading
// ---------------------------------------------------------------------------

void GuidanceNode::load_waypoints()
{
  auto lats = this->get_parameter("waypoints_lat").as_double_array();
  auto lons = this->get_parameter("waypoints_lon").as_double_array();
  auto alts = this->get_parameter("waypoints_alt_m").as_double_array();

  if (lats.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "No waypoints loaded — skipping waypoint phase");
    return;
  }

  if (lats.size() != lons.size() || lats.size() != alts.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "waypoints_lat/lon/alt arrays must be the same length! "
      "Got %zu / %zu / %zu", lats.size(), lons.size(), alts.size());
    return;
  }

  gps_waypoints_.clear();
  for (size_t i = 0; i < lats.size(); ++i) {
    gps_waypoints_.push_back({lats[i], lons[i], alts[i]});  // alt in meters MSL
  }

  waypoints_loaded_ = true;
  RCLCPP_INFO(this->get_logger(),
    "Loaded %zu waypoints, %d laps planned", gps_waypoints_.size(), total_laps_);

  for (size_t i = 0; i < gps_waypoints_.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  WP %zu: lat=%.6f  lon=%.6f  alt=%.1f m",
      i + 1, gps_waypoints_[i].lat, gps_waypoints_[i].lon, gps_waypoints_[i].alt_m);
  }
}

void GuidanceNode::convert_waypoints_to_ned()
{
  ned_waypoints_.clear();
  for (const auto & wp : gps_waypoints_) {
    ned_waypoints_.push_back(gps_to_ned(wp.lat, wp.lon, wp.alt_m));
  }
  ned_waypoints_ready_ = true;

  RCLCPP_INFO(this->get_logger(), "Waypoints converted to NED (ref: %.6f, %.6f, %.1f m MSL):",
    ref_lat_, ref_lon_, ref_alt_m_);
  for (size_t i = 0; i < ned_waypoints_.size(); ++i) {
    const auto & n = ned_waypoints_[i];
    RCLCPP_INFO(this->get_logger(), "  WP %zu NED: [%.1f, %.1f, %.1f]",
      i + 1, n.x(), n.y(), n.z());
  }
}

Eigen::Vector3d GuidanceNode::gps_to_ned(
  double lat_deg, double lon_deg, double alt_m_msl) const
{
  double dlat = (lat_deg - ref_lat_) * DEG_TO_RAD;
  double dlon = (lon_deg - ref_lon_) * DEG_TO_RAD;

  double north = dlat * R_EARTH;
  double east  = dlon * R_EARTH * std::cos(ref_lat_ * DEG_TO_RAD);
  double down  = -(alt_m_msl - ref_alt_m_);

  return Eigen::Vector3d(north, east, down);
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

void GuidanceNode::best_target_cb(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  best_target_position_ned_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
  target_position_ned_ = best_target_position_ned_;
  target_received_ = true;
  last_target_time_ = this->now();
}

void GuidanceNode::local_position_cb(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  vehicle_position_ned_ = Eigen::Vector3d(msg->x, msg->y, msg->z);
  position_received_ = true;

  // Capture the NED reference origin once (needs valid global frame)
  if (!ref_received_ && msg->xy_global && msg->z_global) {
    ref_lat_   = msg->ref_lat;
    ref_lon_   = msg->ref_lon;
    ref_alt_m_ = static_cast<double>(msg->ref_alt);
    ref_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "NED reference origin: lat=%.6f  lon=%.6f  alt=%.1f m MSL",
      ref_lat_, ref_lon_, ref_alt_m_);

    if (waypoints_loaded_ && !ned_waypoints_ready_) {
      convert_waypoints_to_ned();
    }
  }
}

// ---------------------------------------------------------------------------
// Control loop — priority: vision target > waypoint > hold
// ---------------------------------------------------------------------------

void GuidanceNode::control_loop()
{
  uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;  // µs
  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = ts;

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
  sp.yaw             = std::numeric_limits<float>::quiet_NaN();
  sp.yawspeed        = std::numeric_limits<float>::quiet_NaN();

  if (!position_received_) {
    // Hold at NED origin at loiter altitude — valid setpoint required by PX4
    sp.position[0] = 0.0f;
    sp.position[1] = 0.0f;
    sp.position[2] = static_cast<float>(loiter_altitude_);
    setpoint_pub_->publish(sp);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No vehicle position received yet — publishing hold-at-origin setpoint");
    return;
  }

  bool target_stale = (this->now() - last_target_time_).seconds() > 2.0;
  // TODO: maybe use best target instead in this case or fly to cpmpetition provided target location
  bool waypoints_active = ned_waypoints_ready_ && (current_lap_ < total_laps_);

  // Check if target is valid (fresh + within sane distance)
  bool target_valid = target_received_ && !target_stale;
  if (target_valid) {
    double target_dist = (target_position_ned_ - vehicle_position_ned_).norm();
    if (target_dist > 500.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Target %.0f m away — possibly bad localisation, ignoring", target_dist);
      target_valid = false;
    }
  }

  // ===================================================================
  //  PRIORITY 1: Vision target detected → track it for payload delivery
  // ===================================================================
  if (target_valid) {
    // TODO: only fly to target if we are trying to drop a payload
    //    If we are flying the waypoint loop or have already dropped the payload, do not track target
    // ---- Navigate towards target ----
    Eigen::Vector3d to_target = target_position_ned_ - vehicle_position_ned_;

    // Keep altitude at loiter_altitude_, only move horizontally toward target
    to_target.z() = 0.0;
    double dist = to_target.norm();

    Eigen::Vector3d desired_pos;
    if (dist < acceptance_radius_) {
      // Close enough – hold above the target at loiter altitude
      // TODO: trigger payload drop signal here
      desired_pos = Eigen::Vector3d(
        target_position_ned_.x(),
        target_position_ned_.y(),
        loiter_altitude_);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TARGET: within %.1f m – holding for delivery", dist);
    } else {
      // Move towards target at approach_speed_, staying at loiter altitude
      Eigen::Vector3d direction = to_target.normalized();
      // TODO: maybe fly directly to target instead of stepping
      double step = std::min(approach_speed_ * 0.5, dist);
      desired_pos = vehicle_position_ned_ + direction * step;
      desired_pos.z() = loiter_altitude_;

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TARGET: approaching, dist=%.1f m", dist);
    }

    sp.position[0] = static_cast<float>(desired_pos.x());
    sp.position[1] = static_cast<float>(desired_pos.y());
    sp.position[2] = static_cast<float>(desired_pos.z());

  // ===================================================================
  //  PRIORITY 2: Waypoints remaining → fly the mission lap
  // ===================================================================
  } else if (waypoints_active) {
    const auto & wp = ned_waypoints_[current_wp_index_];

    Eigen::Vector3d to_wp = wp - vehicle_position_ned_;
    double horiz_dist = Eigen::Vector2d(to_wp.x(), to_wp.y()).norm();

    // Check if current waypoint is reached
    if (horiz_dist < waypoint_radius_) {
      RCLCPP_INFO(this->get_logger(),
        "WAYPOINT %zu/%zu reached (%.0f m) — lap %d/%d",
        current_wp_index_ + 1, ned_waypoints_.size(), horiz_dist,
        current_lap_ + 1, total_laps_);

      current_wp_index_++;
      if (current_wp_index_ >= ned_waypoints_.size()) {
        current_wp_index_ = 0;
        current_lap_++;
        RCLCPP_INFO(this->get_logger(),
          "===== LAP %d/%d COMPLETE =====", current_lap_, total_laps_);

        if (current_lap_ >= total_laps_) {
          RCLCPP_INFO(this->get_logger(),
            "All %d laps done — entering search/detect phase", total_laps_);
        }
      }
    }

    // Navigate towards the current (or next) waypoint
    if (current_lap_ < total_laps_) {
      const auto & next_wp = ned_waypoints_[current_wp_index_];
      Eigen::Vector3d to_next = next_wp - vehicle_position_ned_;
      double dist = to_next.norm();
      Eigen::Vector3d direction = to_next.normalized();
      double step = std::min(approach_speed_ * 0.5, dist);

      Eigen::Vector3d desired = vehicle_position_ned_ + direction * step;
      desired.z() = next_wp.z();   // use waypoint altitude

      sp.position[0] = static_cast<float>(desired.x());
      sp.position[1] = static_cast<float>(desired.y());
      sp.position[2] = static_cast<float>(desired.z());

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "WAYPOINT %zu/%zu  dist=%.0f m  lap %d/%d",
        current_wp_index_ + 1, ned_waypoints_.size(), dist,
        current_lap_ + 1, total_laps_);
    } else {
      // All laps done — hold position
      sp.position[0] = static_cast<float>(vehicle_position_ned_.x());
      sp.position[1] = static_cast<float>(vehicle_position_ned_.y());
      sp.position[2] = static_cast<float>(loiter_altitude_);
    }

  // ===================================================================
  //  PRIORITY 3: Nothing to do → hold position
  // ===================================================================
  } else {
    // TODO: fly to competition provided target location
    // ---- No active target – hold current XY at loiter altitude ----
    sp.position[0] = static_cast<float>(vehicle_position_ned_.x());
    sp.position[1] = static_cast<float>(vehicle_position_ned_.y());
    sp.position[2] = static_cast<float>(loiter_altitude_);

    if (target_stale && target_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Vision target data stale (>2s old) — holding position");
    }
    if (!waypoints_loaded_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "No waypoints loaded — drone will hold position indefinitely");
    }
  }

  // Altitude safety warning
  if (vehicle_position_ned_.z() > -2.0 && vehicle_position_ned_.z() < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "LOW ALTITUDE: %.1f m AGL — check for obstacles", -vehicle_position_ned_.z());
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
