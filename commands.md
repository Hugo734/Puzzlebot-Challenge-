# Puzzlebot AMR — Command Reference

## Before Every Session

Kill stale processes from any previous run, then refresh the daemon:

```bash
pkill -f "ign gazebo|ign_gazebo|slam_node|map_saver|nav_node|rviz2" 2>/dev/null; sleep 2
ros2 daemon stop && ros2 daemon start
source install/setup.bash
```

---

## 1. Build

```bash
colcon build
source install/setup.bash
```

> If `slam` fails with `--uninstall not recognized`: `rm build/slam/setup.py` then rebuild.

---

## 2. Mapping — build a new map

### Simulation (all-in-one)

One command: starts Gazebo, spawns the robot, waits 22 s for controllers, then opens SLAM + RViz + teleop.

```bash
ros2 launch bringup slam_sim.launch.py world_name:=new_warehouse map_path:=~/ros2_maps/new_warehouse
```

| Arg | Default | Description |
|---|---|---|
| `world_name` | `warehouse` | World file name (no `.world` extension) |
| `map_path` | `~/ros2_maps/warehouse` | Output base path — creates `<path>.pgm` + `<path>.yaml` |

### Simulation (two-terminal, faster iteration)

Terminal 1 — Gazebo:
```bash
ros2 launch description gazebo.launch.py world_name:=new_warehouse
```

Terminal 2 — SLAM stack (after Gazebo physics is running):
```bash
ros2 launch slam mapping.launch.py map_path:=~/ros2_maps/new_warehouse
```

| Arg | Default | Description |
|---|---|---|
| `map_path` | `~/ros2_maps/warehouse` | Output base path |

### Real robot (Jetson Nano)

Two terminals, in order:

**Terminal 1 — LiDAR driver** (RPLidar A1):
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB0
```
> The installed package is `rplidar_ros` (not `sllidar_ros2`).
> Do NOT pass `frame_id` — leave the driver at its default (`laser`). The mapping launch handles the frame aliasing automatically.
> Check `ls /dev/ttyUSB*` to confirm the port.

**Terminal 2 — SLAM + RViz + teleop**:
```bash
source install/setup.bash
ros2 launch slam mapping_real.launch.py \
    map_path:=~/ros2_maps/new_warehouse
```
This starts: RSP, `joint_state_publisher`, `velocity_bridge` (encoder → `/wl` `/wr`), `real_odom` (→ `/puzzlebot_controller/odom` + `odom→base_footprint` TF), `lidar_frame_bridge` (`lidar_link→laser`), `slam_node`, `map_saver`, RViz, teleop xterm.

| Arg | Default | Description |
|---|---|---|
| `map_path` | `~/ros2_maps/warehouse` | Output base path (no extension) |
| `laser_frame_id` | `laser` | Frame published by driver. Change to `lidar_link` only if driver is already configured to use that name |
| `wheel_radius` | `0.05` | Wheel radius in metres |
| `wheel_separation` | `0.19` | Wheel centre-to-centre in metres |

**Terminal 3 (optional) — Monitor**:
```bash
ros2 topic hz /scan                          # LiDAR OK → ~10 Hz
ros2 topic hz /puzzlebot_controller/odom     # real_odom OK → ~50 Hz
ros2 run tf2_ros tf2_echo map base_footprint # SLAM TF working
```

#### Real robot TF chain

```
/scan (laser) ──► lidar_frame_bridge ──► lidar_link
                                              │
                              slam_node ──► map → odom
                                                   │
/VelocityEncl,R ──► velocity_bridge (wl, wr)       │
                              │                    │
                         real_odom ──────────► odom → base_footprint
                                                   │
              robot_state_publisher ──► base_footprint → base_link → lidar_link
```

### Saving the map

Map saves automatically on `Ctrl+C`. To save mid-session without stopping:

```bash
ros2 service call /map_saver/save_map std_srvs/srv/Trigger {}
```

---

## 3. Navigation — drive with a saved map

### Simulation (all-in-one)

```bash
ros2 launch bringup nav_sim.launch.py \
    world_name:=new_warehouse \
    map_yaml:=~/ros2_maps/new_warehouse.yaml
```

| Arg | Default | Description |
|---|---|---|
| `world_name` | `warehouse` | Gazebo world to load |
| `map_yaml` | `~/ros2_maps/warehouse.yaml` | Absolute path to the saved map YAML |

### Simulation (two-terminal)

Terminal 1 — Gazebo:
```bash
ros2 launch description gazebo.launch.py world_name:=new_warehouse
```

Terminal 2 — SLAM localizer + nav node + RViz (after Gazebo is up):
```bash
ros2 launch navigation navigation.launch.py \
    map_yaml:=~/ros2_maps/new_warehouse.yaml
```

### Real robot (Jetson Nano) — full stack

```bash
ros2 launch bringup real.launch.py
```

Starts everything: controller, SLAM, EKF, navigation, perception, lifter, mission control, dashboard, voice control.

---

## 4. Sending a Mission

```bash
ros2 topic pub --once /mission std_msgs/msg/String \
    '{"data": "{\"source\": \"rack_1\", \"dest\": \"truck_1\", \"pallet_id\": 0}"}'
```

Valid zone names (defined in `navigation/config/waypoints.yaml`): `rack_1`–`rack_N`, `truck_1`–`truck_3`, `roller_1`–`roller_N`.

---

## 5. Useful Diagnostics

```bash
# Live TF tree
ros2 run tf2_tools view_frames

# Check the full TF chain (should show map→odom→base_footprint→base_link→lidar_link)
ros2 run tf2_ros tf2_echo map base_footprint

# Topic list / rates
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /puzzlebot_controller/odom

# Current robot pose from SLAM
ros2 topic echo /robot_pose --once

# Current occupancy map info
ros2 topic echo /map --once --field info

# State machine state
ros2 topic echo /robot_state

# Lifter level
ros2 topic echo /lifter_level
```

---

## 6. Map Files Location

```
~/ros2_maps/
  new_warehouse.pgm   ← grayscale pixel map
  new_warehouse.yaml  ← resolution, origin, thresholds
```

The `.yaml` points to the `.pgm` by relative path — keep both files together.

---

## 7. Key Topic / Frame Reference

| Topic | Publisher | Notes |
|---|---|---|
| `/scan` | Gazebo LiDAR / real RPLidar A1 | `frame_id: lidar_link` |
| `/puzzlebot_controller/odom` | `diff_drive_controller` (sim) | `odom → base_footprint` TF |
| `/odom` | `velocity_bridge` (real) | encoder-based |
| `/map` | `slam_node` | OccupancyGrid, 0.05 m/cell |
| `/robot_pose` | EKF / slam_node | PoseStamped in `map` frame |
| `/cmd_vel` | nav_node / teleop | Twist |
| `/mission` | web / voice | JSON string |
| `/robot_state` | mission_control | current FSM state |
| `/lifter_level` | mission_control | UInt8, 0–7 |

| TF edge | Publisher |
|---|---|
| `map → odom` | `slam_node` |
| `odom → base_footprint` | `diff_drive_controller` (sim) / `velocity_bridge` (real) |
| `base_footprint → base_link` | `robot_state_publisher` |
| `base_link → lidar_link` | `robot_state_publisher` (fixed joint) |
| `base_link → camera_link` | `robot_state_publisher` (fixed joint) |
