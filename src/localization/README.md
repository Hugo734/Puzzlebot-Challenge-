# puzzlebot_localization_cpp

EKF-based localization for the Puzzlebot differential-drive robot, implemented in C++ (ROS2 Humble). 

---

## How It Works

The robot's pose `[x, y, θ]` is estimated using an **Extended Kalman Filter (EKF)** fed exclusively by wheel encoder data. No LiDAR or map is required.

### Prediction step (every 50 ms)

Wheel angular velocities `wr` and `wl` (rad/s) are converted to linear and angular velocity:

```
v     = r * (wr + wl) / 2
ω     = r * (wr - wl) / L
```

The state is propagated forward using the unicycle motion model:

```
x     ← x + v·cos(θ)·dt
y     ← y + v·sin(θ)·dt
θ     ← θ + ω·dt
```

The covariance matrix `P` is updated via the motion Jacobian `F` and process noise `Q`:

```
P ← F·P·Fᵀ + Q
```

As the robot moves without corrections, `P` grows — reflecting accumulated uncertainty.

### Update step (on demand)

When a pose measurement arrives on `/pose_measurement` (e.g. from an ArUco marker), the EKF applies a correction:

```
K = P·(P + R)⁻¹        (Kalman gain)
x ← x + K·(z − x̂)     (state correction)
P ← (I − K)·P          (covariance shrinks)
```

This step is optional — the node runs fine with prediction alone.

---

## Nodes

| Executable | Role |
|---|---|
| `ekf_localization` | Runs the EKF. Subscribes to `/wr`, `/wl`. Publishes `/odom` and broadcasts `odom → base_footprint` TF. |
| `kinematic_simulator` | **Simulation only.** Converts `/cmd_vel` (Twist) to `/wr`, `/wl` (Float32) using inverse kinematics. |
| `velocity_bridge` | **Real hardware only.** Relays `/VelocityEncl` → `/wl` and `/VelocityEncnR` → `/wr`. |

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/wr` | `std_msgs/Float32` | Input — right wheel angular velocity (rad/s) |
| `/wl` | `std_msgs/Float32` | Input — left wheel angular velocity (rad/s) |
| `/cmd_vel` | `geometry_msgs/Twist` | Input — velocity commands (sim mode only) |
| `/VelocityEncl` | `std_msgs/Float32` | Input — left encoder (real mode only) |
| `/VelocityEncnR` | `std_msgs/Float32` | Input — right encoder (real mode only) |
| `/pose_measurement` | `geometry_msgs/PoseWithCovarianceStamped` | Input — optional pose correction (ArUco, etc.) |
| `/odom` | `nav_msgs/Odometry` | Output — EKF pose estimate with full covariance |
| `/tf` | TF broadcast | Output — `odom → base_footprint` |

---

## Parameters (`config/ekf_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `wheel_radius` | `0.05` m | Wheel radius `r` |
| `wheel_separation` | `0.19` m | Wheel-to-wheel distance `L` |
| `update_rate` | `20.0` Hz | EKF prediction frequency |
| `q_x`, `q_y`, `q_theta` | `0.01`, `0.01`, `0.005` | Process noise — increase if motion is erratic |
| `r_x`, `r_y`, `r_theta` | `0.05`, `0.05`, `0.02` | Measurement noise — tune to sensor accuracy |

---

## Usage

**Simulation** (keyboard teleoperation):
```bash
ros2 launch puzzlebot_localization_cpp ekf_localization.launch.py mode:=sim
```

**Real hardware** (Jetson encoder topics):
```bash
ros2 launch puzzlebot_localization_cpp ekf_localization.launch.py mode:=real
```

---

## Adding a Pose Sensor

Publish a `geometry_msgs/PoseWithCovarianceStamped` message on `/pose_measurement` from any sensor (ArUco marker, UWB, etc.) and the EKF will automatically fuse it as a correction. The covariance in the message is ignored — use `r_x`, `r_y`, `r_theta` in the params file to tune the trust level.
