# somars-embedded

ROS 2 C++ controls package for the SOMARS autonomous drone (C-UASC competition).

Micro-XRCE-DDS serves as the transparent bridge between PX4 (Pixhawk 6X) and
these ROS 2 nodes running on the Jetson Orin Nano (JetPack 6.2.2).  Start the
Agent via the CLI; this package contains only the control logic.

## Architecture

```
Camera ──► somars-vision (detect targets, publish pixel coords)
                │  /vision/detections [PoseArray]
                ▼
        target_localizer  (pixel → NED projection)
                │  /targets/ned [PointStamped]
                ▼
          guidance_node   (compute trajectory setpoints)
                │  /fmu/in/trajectory_setpoint
                ▼
         micro-XRCE-DDS Agent ──► PX4 (Pixhawk 6X)
                ▲
        offboard_manager  (arm / mode switch / heartbeat)
                │  /fmu/in/vehicle_command
                │  /fmu/in/offboard_control_mode
```

## Nodes

| Node | Description |
|------|-------------|
| `target_localizer` | Subscribes to vision detections + vehicle pose. Projects pixel detections through the camera model and intersects with the ground plane to produce NED target positions. |
| `guidance_node` | Subscribes to target NED positions. Generates `TrajectorySetpoint` messages (position mode) to fly toward the target. Holds position when no target is active. |
| `offboard_manager` | Maintains the PX4 offboard heartbeat (`OffboardControlMode` + hold setpoint). Optionally arms and switches to offboard mode. |

## Quick Start

```bash
# 1. Start the micro-XRCE-DDS Agent (serial example)
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

# 2. Build this package inside your colcon workspace
cd ~/somars_ws
colcon build --packages-select somars_controls
source install/setup.bash

# 3. Launch all controls nodes
ros2 launch somars_controls controls.launch.py
```

## Parameters

All tunables live in `config/params.yaml`.  Override on the CLI:

```bash
ros2 launch somars_controls controls.launch.py \
  --ros-args -p guidance_node:approach_speed:=3.0
```

## Workspace Integration

This package is intended to be used as a submodule inside a ROS 2 colcon
workspace alongside `somars-vision`:

```
somars-ws/
├── src/
│   ├── somars_controls/   ← this repo (git submodule)
│   ├── somars_vision/     ← slugbotics/somars-vision (git submodule)
│   └── px4_msgs/          ← PX4/px4_msgs (git submodule)
├── launch/                ← system-level launch files
└── docker/                ← Jetson container setup
```

## Dependencies

- ROS 2 Humble / Jazzy
- [px4_msgs](https://github.com/PX4/px4_msgs) – PX4 message definitions
- Eigen3
- micro-XRCE-DDS Agent (run separately)
