# Puzzlebot Control UI

Web interface for teleoperating and monitoring a Puzzlebot robot using ROS 2 Humble.

---

## Architecture

```
ROS 2 (robot)
    │  /odom  /image_result  /cmd_vel
    ▼
ros2-grpc-wrapper.py   ← Terminal 1
    │  HTTP REST  :7043   (GET /odom, GET /image, POST /cmd_vel)
    │  gRPC       :7042   (GetMultCoords, GetImageResult)
    ▼
app.py  (Flask)        ← Terminal 2
    │  :8002
    ▼
Browser  →  http://localhost:8002
```

| Component | File | Port |
|---|---|---|
| ROS 2 wrapper | `PY-RPC-Wrapper-Server-Linux/ros2-grpc-wrapper.py` | HTTP 7043 · gRPC 7042 |
| Flask web server | `FLASK-REST-Call-Linux/app.py` | 8002 |
| React UI | `FLASK-REST-Call-Linux/templates/result.html` | — |

---

## Prerequisites

- ROS 2 Humble installed with `source /opt/ros/humble/setup.bash`
- Python 3.10+
- Python packages: `flask`, `requests`, `grpcio`, `grpcio-tools`, `opencv-python`, `cv_bridge`, `rclpy`

---

## How to Launch

### Terminal 1 — ROS 2 Wrapper (must run in ROS 2 environment)

> **Important:** if Miniconda/Anaconda is active, deactivate it first.
> ROS 2 Humble requires Python 3.10 from the system; conda uses Python 3.12 and breaks `rclpy`.

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd /home/rosendorios/Desktop/Modulo3/UI/PY-RPC-Wrapper-Server-Linux
python3 ros2-grpc-wrapper.py
```

This process:
- Creates a ROS 2 node `object_position_wrapper`
- Subscribes to `/odom` and `/image_result`
- Publishes to `/cmd_vel` at 20 Hz
- Exposes an HTTP server on port **7043** and a gRPC server on port **7042**

### Terminal 2 — Flask Server (web UI)

```bash
cd /home/rosendorios/Desktop/Modulo3/UI/FLASK-REST-Call-Linux
python3 app.py
```

This process:
- Serves the web interface at `http://localhost:8002`
- Acts as a proxy between the browser and the wrapper:
  - `GET /api/odom` → retrieves robot position
  - `POST /api/cmd_vel` → sends velocity commands
  - `GET /api/camera` → MJPEG stream from camera

### Open the UI

```
http://localhost:8002
```

---

## Interface Tabs

| Tab | Description |
|---|---|
| **MAP** | Top-down view with synthetic LIDAR, actual robot pose, and waypoints. Click on the map to add waypoints. |
| **CAMERA** | Live MJPEG stream from `/image_result`. Shows "No signal" if wrapper is not running. |
| **TELEOP** | Keyboard control, velocity bars, battery, and E-Stop. |

---

## Keyboard Controls (TELEOP tab)

| Key | Action |
|---|---|
| `W` / `↑` | Move forward |
| `S` / `↓` | Move backward |
| `A` / `←` | Turn left |
| `D` / `→` | Turn right |
| `Shift` | Boost (×1.6) |
| `Space` | E-Stop (toggles armed/stopped) |

---

## ROS 2 Topics Used

| Topic | Type | Direction |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Subscribe |
| `/image_result` | `sensor_msgs/Image` | Subscribe |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish |

---

## Regenerate Protobuf (only if modifying `.proto`)

```bash
cd /home/rosendorios/Desktop/Modulo3/UI/PY-RPC-Wrapper-Server-Linux
python3 -m grpc_tools.protoc -I./protos \
    --python_out=../generated_protos \
    --grpc_python_out=../generated_protos \
    ./protos/rpc-demo.proto
```
