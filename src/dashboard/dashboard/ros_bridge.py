"""
RosBridge — thread-safe bridge between ROS2 subscribers and the Flask web app.

Subscriptions run in the ROS2 executor thread.
Flask reads data via thread-safe locks.
"""

import io
import json
import os
import struct
import threading
import zlib
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


@dataclass
class RobotState:
    pose: dict = field(default_factory=lambda: {"x": 0.0, "y": 0.0, "theta": 0.0})
    state: str = "UNKNOWN"
    mission: Optional[dict] = None
    velocity: dict = field(default_factory=lambda: {"linear": 0.0, "angular": 0.0})
    lifter_level: int = 0
    scan: Optional[dict] = None
    nav_plan: Optional[list] = None


class RosBridge:
    """
    Thread-safe bridge between ROS2 and Flask.

    All ROS2 callbacks write to shared state under a lock.
    Flask threads read that state via get_state() / get_latest_frame().
    """

    def __init__(self, node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._state = RobotState()
        self._latest_frame: Optional[bytes] = None  # JPEG bytes
        self._latest_map: Optional[bytes] = None    # PNG bytes
        self._waypoints: dict = {}

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Lazy imports — ROS2 message types only available at runtime
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import String, UInt8

        self._node.create_subscription(
            PoseStamped, "/slam_pose", self._cb_pose, best_effort_qos
        )
        self._node.create_subscription(
            String, "/nav_status", self._cb_state, reliable_qos
        )
        self._node.create_subscription(
            String, "/mission", self._cb_mission, reliable_qos
        )
        self._node.create_subscription(
            Twist, "/cmd_vel", self._cb_vel, best_effort_qos
        )
        self._node.create_subscription(
            UInt8, "/lifter_status", self._cb_lifter, best_effort_qos
        )
        self._node.create_subscription(
            Image, "/cam_img", self._cb_image, best_effort_qos
        )
        self._node.create_subscription(
            OccupancyGrid, "/map", self._cb_map, best_effort_qos
        )
        self._node.create_subscription(
            LaserScan, "/scan", self._cb_scan, best_effort_qos
        )
        self._node.create_subscription(
            Path, "/plan", self._cb_plan, reliable_qos
        )

        self._mission_pub = self._node.create_publisher(String, "/mission", reliable_qos)
        self._goal_pub = self._node.create_publisher(String, "/goal_waypoint", reliable_qos)
        self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", reliable_qos)

        # Load waypoints from file at startup
        self._load_waypoints()

    # ------------------------------------------------------------------
    # Waypoints loading
    # ------------------------------------------------------------------

    def _load_waypoints(self) -> None:
        """Load waypoints.yaml from ~/ros2_maps/ at startup."""
        import yaml

        waypoints_path = os.path.expanduser("~/ros2_maps/waypoints.yaml")
        try:
            with open(waypoints_path, "r") as f:
                data = yaml.safe_load(f)
            if isinstance(data, dict):
                self._waypoints = data.get("waypoints", data)
                self._node.get_logger().info(
                    f"Loaded {len(self._waypoints)} waypoints from {waypoints_path}"
                )
            else:
                self._node.get_logger().warning(
                    f"Unexpected waypoints format in {waypoints_path}"
                )
        except FileNotFoundError:
            self._node.get_logger().warning(
                f"Waypoints file not found: {waypoints_path} — using empty waypoints"
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"Could not load waypoints: {exc}")

    # ------------------------------------------------------------------
    # Public API (called from Flask threads)
    # ------------------------------------------------------------------

    def get_state(self) -> RobotState:
        with self._lock:
            import copy
            return copy.deepcopy(self._state)

    def get_latest_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_frame

    def get_latest_map(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_map

    def publish_mission(self, mission_json: str) -> None:
        from std_msgs.msg import String
        msg = String()
        msg.data = mission_json
        self._mission_pub.publish(msg)

    def publish_goal(self, name: str) -> None:
        """Publish a goal waypoint name to /goal_waypoint."""
        from std_msgs.msg import String
        msg = String()
        msg.data = name
        self._goal_pub.publish(msg)

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        """Publish a Twist command to /cmd_vel."""
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def get_waypoints(self) -> dict:
        """Return the waypoints dict {name: {x, y, theta}}."""
        return dict(self._waypoints)

    # ------------------------------------------------------------------
    # ROS2 callbacks (executor thread)
    # ------------------------------------------------------------------

    def _cb_pose(self, msg) -> None:
        import math
        q = msg.pose.orientation
        # Quaternion → yaw (theta)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._state.pose = {
                "x": round(msg.pose.position.x, 3),
                "y": round(msg.pose.position.y, 3),
                "theta": round(theta, 4),
            }

    def _cb_state(self, msg) -> None:
        with self._lock:
            self._state.state = msg.data

    def _cb_mission(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            data = None
        with self._lock:
            self._state.mission = data

    def _cb_vel(self, msg) -> None:
        with self._lock:
            self._state.velocity = {
                "linear": round(msg.linear.x, 3),
                "angular": round(msg.angular.z, 3),
            }

    def _cb_lifter(self, msg) -> None:
        with self._lock:
            self._state.lifter_level = int(msg.data)

    def _cb_image(self, msg) -> None:
        try:
            jpeg = self._frame_to_jpeg(msg)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"Image conversion failed: {exc}")
            return
        with self._lock:
            self._latest_frame = jpeg

    def _cb_map(self, msg) -> None:
        try:
            png = self._map_to_png(msg)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f"Map conversion failed: {exc}")
            return
        with self._lock:
            self._latest_map = png

    def _cb_scan(self, msg) -> None:
        """Downsample every 4th ray and store scan data."""
        ranges_raw = list(msg.ranges)
        # Downsample: keep every 4th ray
        ranges_ds = ranges_raw[::4]
        with self._lock:
            self._state.scan = {
                "ranges": ranges_ds,
                "angle_min": msg.angle_min,
                "angle_increment": msg.angle_increment * 4,
                "range_max": msg.range_max,
            }

    def _cb_plan(self, msg) -> None:
        """Store nav plan as list of {x, y} dicts."""
        points = [
            {"x": round(pose.pose.position.x, 3), "y": round(pose.pose.position.y, 3)}
            for pose in msg.poses
        ]
        with self._lock:
            self._state.nav_plan = points

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------

    def _frame_to_jpeg(self, msg) -> bytes:
        """Convert sensor_msgs/Image to JPEG bytes (no cv_bridge dependency)."""
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        h, w = msg.height, msg.width

        if encoding in ("rgb8", "bgr8"):
            img = data.reshape((h, w, 3))
            if encoding == "bgr8":
                img = img[:, :, ::-1]  # BGR → RGB
        elif encoding in ("mono8", "8uc1"):
            img = np.stack([data.reshape((h, w))] * 3, axis=-1)
        elif encoding == "rgba8":
            img = data.reshape((h, w, 4))[:, :, :3]
        elif encoding == "bgra8":
            img = data.reshape((h, w, 4))[:, :, 2::-1]
        else:
            # Fallback: treat raw data as grayscale
            img = np.zeros((h, w, 3), dtype=np.uint8)

        return self._numpy_rgb_to_jpeg(img)

    def _numpy_rgb_to_jpeg(self, rgb: np.ndarray, quality: int = 75) -> bytes:
        """Encode an H×W×3 uint8 NumPy array to JPEG using only stdlib + numpy."""
        try:
            # Prefer OpenCV when available (fast, always present on robot)
            import cv2
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            _, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
            return bytes(buf)
        except ImportError:
            pass
        # PIL fallback
        try:
            from PIL import Image as PILImage
            buf = io.BytesIO()
            PILImage.fromarray(rgb, "RGB").save(buf, format="JPEG", quality=quality)
            return buf.getvalue()
        except ImportError:
            pass
        # Last resort: return a minimal valid JPEG (1×1 black pixel)
        return self._minimal_jpeg()

    @staticmethod
    def _minimal_jpeg() -> bytes:
        """Return a 1×1 black JPEG for when no encoder is available."""
        return bytes(
            b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
            b"\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t"
            b"\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a"
            b"\x1f\x1e\x1d\x1a\x1c\x1c $.' \",#\x1c\x1c(7),01444\x1f'9=82<.342\x1e"
            b"\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1f"
            b"\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00"
            b"\x00\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08"
            b"\x01\x01\x00\x00?\x00\xf5\x0a\xff\xd9"
        )

    def _map_to_png(self, msg) -> bytes:
        """Convert nav_msgs/OccupancyGrid to a PNG bytes image."""
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        # OccupancyGrid semantics:
        #  -1  → unknown  → gray (128)
        #   0  → free     → white (255)
        # 100  → occupied → black (0)
        img = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0] = 255
        img[data == 100] = 0
        # Partial occupancy (1-99) → scale
        mask = (data > 0) & (data < 100)
        img[mask] = (255 - data[mask].astype(np.int16) * 255 // 100).astype(np.uint8)

        # Flip vertically so Y-up ROS → Y-down image
        img = np.flipud(img)

        return self._numpy_gray_to_png(img)

    @staticmethod
    def _numpy_gray_to_png(gray: np.ndarray) -> bytes:
        """Encode a grayscale H×W uint8 array to PNG bytes using only stdlib."""
        try:
            import cv2
            _, buf = cv2.imencode(".png", gray)
            return bytes(buf)
        except ImportError:
            pass
        try:
            from PIL import Image as PILImage
            buf = io.BytesIO()
            PILImage.fromarray(gray, "L").save(buf, format="PNG")
            return buf.getvalue()
        except ImportError:
            pass
        # Pure-Python PNG encoder (grayscale, no filtering)
        return _encode_png_gray(gray)


# ---------------------------------------------------------------------------
# Pure-Python minimal PNG encoder (grayscale 8-bit, no compression filters)
# Only used when neither OpenCV nor Pillow is available.
# ---------------------------------------------------------------------------

def _encode_png_gray(gray: np.ndarray) -> bytes:
    """Encode a H×W uint8 grayscale array to a valid PNG byte string."""

    def _chunk(tag: bytes, data: bytes) -> bytes:
        length = struct.pack(">I", len(data))
        crc = struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        return length + tag + data + crc

    h, w = gray.shape
    # Build raw image data (filter byte 0 = None before each row)
    raw = b"".join(b"\x00" + bytes(gray[y]) for y in range(h))
    compressed = zlib.compress(raw, 6)

    png = b"\x89PNG\r\n\x1a\n"
    png += _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += _chunk(b"IDAT", compressed)
    png += _chunk(b"IEND", b"")
    return png
