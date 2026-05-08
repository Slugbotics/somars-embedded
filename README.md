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
waypoints_alt_ft: [200.0, 250.0, ...]
```

Set `waypoint_laps` to the number of full laps you want to fly before
entering the search/deliver phase.

### 2. Start the stack

```bash
# Terminal 1 — micro-XRCE-DDS Agent (bridge PX4 ↔ ROS 2)
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600

# Terminal 2 — build & launch
cd ~/somars_ws
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
| `config/params.yaml` | Camera intrinsics, speeds, altitudes | Tuning / calibration |

---

## Nodes

| Node | Purpose |
|------|---------|
| `offboard_manager` | PX4 heartbeat, arm/disarm, mode switching |
| `guidance_node` | Waypoint navigation + vision-target tracking → publishes `TrajectorySetpoint` |
| `target_localizer` | Pixel detections → NED world coordinates (camera ray–ground intersection) |

---

## Testing Without Hardware

**Math tests** (runs on any machine with Eigen — no ROS 2 needed):

```bash
cd test && mkdir -p build && cd build
cmake .. && make && ./test_projection_math
```

**Integration test** (needs ROS 2 but no Pixhawk — uses mock publishers):

```bash
ros2 launch somars_controls integration.launch.py
```

---

## Workspace Setup

```
somars-ws/
├── src/
│   ├── somars-embedded/   ← this repo
│   ├── somars-vision/     ← slugbotics/somars-vision
│   └── px4_msgs/          ← PX4/px4_msgs
└── ...
```

```bash
cd ~/somars_ws
colcon build
source install/setup.bash
```

## Dependencies

- **ROS 2** Humble or Jazzy
- **[px4_msgs](https://github.com/PX4/px4_msgs)**
- **Eigen3**
- **micro-XRCE-DDS Agent** (run separately on Jetson)
