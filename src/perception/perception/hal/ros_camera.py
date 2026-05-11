"""ROS2 camera source — subscribes to a sensor_msgs/Image topic via cv_bridge."""

import threading
from typing import Optional

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:
    CvBridge = None  # type: ignore[assignment,misc]
    CvBridgeError = Exception  # type: ignore[assignment,misc]

from .base import CameraSource


class RosCameraSource(CameraSource):
    """Thread-safe camera source backed by a ROS2 Image subscription."""

    def __init__(self, node: Node, topic: str = '/cam_img') -> None:
        self._node = node
        self._bridge = CvBridge() if CvBridge is not None else None
        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._ready = False

        self._sub = node.create_subscription(
            Image,
            topic,
            self._image_callback,
            10,
        )
        node.get_logger().info(f'RosCameraSource: subscribed to {topic}')

    def _image_callback(self, msg: Image) -> None:
        if self._bridge is None:
            self._node.get_logger().warn(
                'cv_bridge not available; cannot convert Image message.', throttle_duration_sec=5.0
            )
            return
        try:
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self._node.get_logger().warn(f'CvBridgeError: {exc}', throttle_duration_sec=2.0)
            return

        with self._lock:
            self._latest_frame = frame
            self._ready = True

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def is_ready(self) -> bool:
        with self._lock:
            return self._ready
