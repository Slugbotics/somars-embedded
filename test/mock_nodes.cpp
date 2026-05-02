// ============================================================================
// Mock PX4 + Vision publisher for integration testing without hardware.
// Requires ROS2 + px4_msgs (but NOT a real Pixhawk or micro-XRCE-DDS Agent).
//
// Publishes:
//   /fmu/out/vehicle_attitude         – identity quaternion (level, heading N)
//   /fmu/out/vehicle_local_position   – hovering at configurable NED position
//   /fmu/out/vehicle_status           – armed, in offboard mode
//   /vision/detections                – one detection at configurable pixel coords
//
// Usage (after building in a ROS2 workspace):
//   ros2 run somars_controls mock_nodes
//   ros2 run somars_controls mock_nodes --ros-args \
//       -p altitude_m:=25.0 -p detection_u:=400.0 -p detection_v:=300.0
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MockNodes : public rclcpp::Node
{
public:
  MockNodes() : Node("mock_nodes")
  {
    this->declare_parameter("altitude_m", 20.0);
    this->declare_parameter("north_m", 100.0);
    this->declare_parameter("east_m", 50.0);
    this->declare_parameter("yaw_deg", 0.0);
    this->declare_parameter("detection_u", 320.0);
    this->declare_parameter("detection_v", 240.0);
    this->declare_parameter("detection_class", 0);  // 0=red, 1=black, 2=white
    this->declare_parameter("publish_rate_hz", 10.0);
    this->declare_parameter("enable_detections", true);

    alt_   = this->get_parameter("altitude_m").as_double();
    north_ = this->get_parameter("north_m").as_double();
    east_  = this->get_parameter("east_m").as_double();
    double yaw_deg = this->get_parameter("yaw_deg").as_double();
    yaw_rad_ = yaw_deg * M_PI / 180.0;
    det_u_ = this->get_parameter("detection_u").as_double();
    det_v_ = this->get_parameter("detection_v").as_double();
    det_class_ = this->get_parameter("detection_class").as_int();
    enable_det_ = this->get_parameter("enable_detections").as_bool();
    double rate = this->get_parameter("publish_rate_hz").as_double();

    // Publishers
    att_pub_   = this->create_publisher<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS());
    pos_pub_   = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS());
    stat_pub_  = this->create_publisher<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS());
    det_pub_   = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/vision/detections", 10);

    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockNodes::publish_all, this));

    RCLCPP_INFO(this->get_logger(),
      "MockNodes: pos=[%.0f,%.0f,%.0f] yaw=%.0f° det=[%.0f,%.0f] class=%ld @ %.0fHz",
      north_, east_, -alt_, yaw_deg, det_u_, det_v_, det_class_, rate);
  }

private:
  void publish_all()
  {
    uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;

    // -- VehicleAttitude --
    px4_msgs::msg::VehicleAttitude att{};
    att.timestamp = ts;
    // Quaternion for yaw rotation about NED Z-axis
    double half_yaw = yaw_rad_ / 2.0;
    att.q[0] = static_cast<float>(std::cos(half_yaw));  // w
    att.q[1] = 0.0f;                                     // x
    att.q[2] = 0.0f;                                     // y
    att.q[3] = static_cast<float>(std::sin(half_yaw));  // z
    att_pub_->publish(att);

    // -- VehicleLocalPosition --
    px4_msgs::msg::VehicleLocalPosition pos{};
    pos.timestamp = ts;
    pos.x  = static_cast<float>(north_);
    pos.y  = static_cast<float>(east_);
    pos.z  = static_cast<float>(-alt_);   // NED: negative = above ground
    pos.vx = 0.0f;
    pos.vy = 0.0f;
    pos.vz = 0.0f;
    pos.xy_valid = true;
    pos.z_valid  = true;
    pos.heading  = static_cast<float>(yaw_rad_);
    pos_pub_->publish(pos);

    // -- VehicleStatus --
    px4_msgs::msg::VehicleStatus stat{};
    stat.timestamp    = ts;
    stat.arming_state = 2;   // armed
    stat.nav_state    = 14;  // offboard
    stat_pub_->publish(stat);

    // -- Vision detections --
    if (enable_det_) {
      geometry_msgs::msg::PoseArray det{};
      det.header.stamp    = this->now();
      det.header.frame_id = "camera";

      geometry_msgs::msg::Pose p;
      p.position.x = det_u_;
      p.position.y = det_v_;
      p.position.z = static_cast<double>(det_class_);
      det.poses.push_back(p);
      det_pub_->publish(det);
    }

    tick_++;
    if (tick_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(), "Published %d mock cycles", tick_);
    }
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr att_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr stat_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr det_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double alt_, north_, east_, yaw_rad_;
  double det_u_, det_v_;
  int64_t det_class_;
  bool enable_det_;
  int tick_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockNodes>());
  rclcpp::shutdown();
  return 0;
}
