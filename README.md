# somars-embedded

ROS 2 C++ controls for the SOMARS drone. Runs on the Jetson Orin Nano,
talks to PX4 (Pixhawk 6X) through micro-XRCE-DDS.

---

## Flight Day Checklist

### 1. Plug in waypoints

Open **`config/waypoints.yaml`** and replace the example coordinates with the
ones given at check-in:

```yaml
waypoints_lat:    [38.315339, 38.315805, ...]
waypoints_lon:    [-76.548108, -76.550537, ...]
waypoints_alt_m: [61.0, 76.2, ...]
```

Set `waypoint_laps` to the number of full laps you want to fly before
entering the search/deliver phase.

### 2. Start the stack

```bash
# Terminal 1 — micro-XRCE-DDS Agent (bridge PX4 ↔ ROS 2)
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

# Terminal 2 — build & launch
cd ~/somars-main
colcon build --packages-select somars_controls
source install/setup.bash
ros2 launch somars_controls controls.launch.py
```

### 3. Verify topics

```bash
ros2 topic echo /fmu/out/vehicle_local_position   # PX4 position
ros2 topic echo /targets/ned                       # vision localisation
ros2 topic echo /fmu/in/trajectory_setpoint        # setpoints going to PX4
```

---

## Mission Flow

```
Takeoff
  │
  ├─ Phase 1: Waypoint laps (GPS coords from config/waypoints.yaml)
  │    guidance_node sequences through waypoints, counts laps
  │
  ├─ Phase 2: Search / detect / deliver
  │    somars-vision detects targets → target_localizer projects to NED
  │    → guidance_node steers drone to target → payload drop
  │
  └─ Land
```

**Priority in `guidance_node`:**

| Priority | Condition | Action |
|----------|-----------|--------|
| 1 | Vision target active (< 2 s old) | Track target for delivery |
| 2 | Waypoints remaining | Fly to next waypoint |
| 3 | Nothing to do | Hold position at loiter altitude |

---

## Config Files

| File | What to edit | When |
|------|-------------|------|
| **`config/waypoints.yaml`** | GPS waypoints, number of laps | **Competition day** |
| `config/params.yaml` | Camera intrinsics, speeds, altitudes | Tuning / calibration (see **Camera Calibration** below) |

---

## Nodes

| Node | Purpose |
|------|---------|
| `offboard_manager` | PX4 heartbeat (OffboardControlMode), arm/disarm, mode switching |
| `guidance_node` | Waypoint navigation + vision-target tracking → publishes `TrajectorySetpoint` |
| `target_localizer` | Pixel detections → NED world coordinates (camera ray–ground intersection) |

---

## Camera Calibration

The `target_localizer` needs accurate camera intrinsics (fx, fy, cx, cy).
Run the calibration tool **before flight** with a printed checkerboard:

```bash
# Live capture (SPACE = capture, Q = done, need ~20 frames):
python3 tools/calibrate_camera.py --camera 0 --cols 9 --rows 6 --square-size 0.025

# Or from saved images:
python3 tools/calibrate_camera.py --images ./cal_images --cols 9 --rows 6 --square-size 0.025
```

Paste the output values into `config/params.yaml` under `target_localizer.ros__parameters`.

---

## Testing Without Hardware

**Math tests** (runs on any machine with Eigen — no ROS 2 needed):

```bash
cd test && mkdir -p build && cd build
cmake .. && make && ./test_projection_math
```

---

## Workspace Setup

This repo lives inside [`somars-main`](https://github.com/Slugbotics/somars-main)
as a git submodule:

```
somars-main/               ← colcon workspace root
├── src/
│   ├── somars-embedded/   ← this repo (submodule)
│   ├── somars-vision/     ← slugbotics/somars-vision (submodule)
│   └── px4_msgs/          ← PX4/px4_msgs (submodule)
├── .gitmodules
└── README.md
```

```bash
git clone --recurse-submodules https://github.com/Slugbotics/somars-main.git
cd somars-main
colcon build
source install/setup.bash
```

## Dependencies

- **ROS 2** Humble or Jazzy
- **[px4_msgs](https://github.com/PX4/px4_msgs)**
- **Eigen3**
- **micro-XRCE-DDS Agent** (run separately on Jetson)
