"""Main ROS2 node — vision-based pallet alignment with ArUco detection.

Responsibilities
----------------
* Drives a PID controller that centres the robot on a detected pallet by
  publishing velocity commands on ``/cmd_vel``.
* Publishes the pixel alignment error on ``/alignment_error``.
* Continuously publishes ArUco marker poses on ``/aruco_poses``.
* Continuously publishes the pallet centroid on ``/pallet_detection``.
* Only issues ``/cmd_vel`` when the state machine is in an ALIGNING state
  (topic ``/robot_state``).
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseArray, Twist
from std_msgs.msg import String

from .aruco_detector import ArucoDetector
from .pallet_detector import PalletDetector
from .trailer_detector import TrailerDetector
from .hal.ros_camera import RosCameraSource
from .hal.mock_camera import MockCameraSource
from .hal.base import CameraSource


def _clamp(value: float, limit: float) -> float:
    return float(np.clip(value, -limit, limit))


class AlignmentNode(Node):
    """ROS2 node for vision-based pallet alignment and perception."""

    def __init__(self) -> None:
        super().__init__('alignment_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('use_mock_camera',        False)
        self.declare_parameter('camera_topic',           '/cam_img')
        self.declare_parameter('pallet_hsv_lower',       [0,   100, 100])
        self.declare_parameter('pallet_hsv_upper',       [15,  255, 255])
        self.declare_parameter('pallet_min_area',        500.0)
        self.declare_parameter('kp_angular',             0.003)
        self.declare_parameter('kp_linear',              0.002)
        self.declare_parameter('max_angular',            0.8)
        self.declare_parameter('max_linear',             0.15)
        self.declare_parameter('alignment_threshold_px', 20.0)
        self.declare_parameter('control_frequency',      15.0)
        self.declare_parameter('aruco_marker_length',    0.05)
        # Camera intrinsics (9 elements, row-major, for ArUco pose)
        self.declare_parameter(
            'camera_matrix',
            [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])

        # ── Read parameters ────────────────────────────────────────────
        use_mock: bool = self.get_parameter('use_mock_camera').get_parameter_value().bool_value
        camera_topic:  str   = self.get_parameter('camera_topic').get_parameter_value().string_value
        hsv_lower:     list  = self.get_parameter('pallet_hsv_lower').get_parameter_value().integer_array_value
        hsv_upper:     list  = self.get_parameter('pallet_hsv_upper').get_parameter_value().integer_array_value
        pallet_min_area: float = self.get_parameter('pallet_min_area').get_parameter_value().double_value

        self._kp_angular:  float = self.get_parameter('kp_angular').get_parameter_value().double_value
        self._kp_linear:   float = self.get_parameter('kp_linear').get_parameter_value().double_value
        self._max_angular: float = self.get_parameter('max_angular').get_parameter_value().double_value
        self._max_linear:  float = self.get_parameter('max_linear').get_parameter_value().double_value
        self._threshold:   float = self.get_parameter('alignment_threshold_px').get_parameter_value().double_value
        freq:              float = self.get_parameter('control_frequency').get_parameter_value().double_value
        marker_length:     float = self.get_parameter('aruco_marker_length').get_parameter_value().double_value
        cam_matrix_flat:   list  = self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        dist_flat:         list  = self.get_parameter('dist_coeffs').get_parameter_value().double_array_value

        camera_matrix = np.array(cam_matrix_flat, dtype=np.float64).reshape(3, 3)
        dist_coeffs   = np.array(dist_flat,        dtype=np.float64)

        # ── Camera HAL ────────────────────────────────────────────────
        self._camera: CameraSource
        if use_mock:
            self._camera = MockCameraSource()
            self.get_logger().info('Using MockCameraSource (simulation mode).')
        else:
            self._camera = RosCameraSource(self, camera_topic)
            self.get_logger().info(f'Using RosCameraSource on topic "{camera_topic}".')

        # ── Vision modules ────────────────────────────────────────────
        self._pallet_detector = PalletDetector(
            hsv_lower=list(hsv_lower),
            hsv_upper=list(hsv_upper),
            min_area=pallet_min_area,
        )
        self._aruco_detector = ArucoDetector(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            marker_length=marker_length,
        )
        self._trailer_detector = TrailerDetector()

        # ── State ─────────────────────────────────────────────────────
        self._robot_state: str = ''
        self._img_width:   int = 640
        self._img_height:  int = 480

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            String,
            '/robot_state',
            self._state_callback,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────
        self._pub_alignment_error = self.create_publisher(
            Point, '/alignment_error', 10
        )
        self._pub_cmd_vel = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self._pub_aruco_poses = self.create_publisher(
            PoseArray, '/aruco_poses', 10
        )
        self._pub_pallet_detection = self.create_publisher(
            Point, '/pallet_detection', 10
        )

        # ── Control timer ─────────────────────────────────────────────
        period = 1.0 / max(freq, 1.0)
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'AlignmentNode started — freq={freq:.1f} Hz, '
            f'threshold={self._threshold:.0f} px'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _state_callback(self, msg: String) -> None:
        self._robot_state = msg.data

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self._camera.is_ready():
            return

        frame = self._camera.get_frame()
        if frame is None:
            return

        h, w = frame.shape[:2]
        self._img_height = h
        self._img_width  = w

        is_aligning = 'ALIGNING' in self._robot_state.upper()

        # 1. Pallet detection (always run — publish centroid regardless of state)
        pallet = self._pallet_detector.detect(frame)
        self._publish_pallet_detection(pallet)

        # 2. Alignment PID — only when robot is in an ALIGNING state
        if is_aligning:
            self._run_alignment_pid(pallet, w, h)

        # 3. ArUco detection (always run)
        aruco_detections = self._aruco_detector.detect(frame)
        self._publish_aruco_poses(aruco_detections)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_pallet_detection(self, pallet: Optional[dict]) -> None:
        msg = Point()
        if pallet is not None:
            cx, cy = pallet['centroid']
            msg.x = float(cx)
            msg.y = float(cy)
            msg.z = 1.0  # detected
        else:
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0  # not detected
        self._pub_pallet_detection.publish(msg)

    def _run_alignment_pid(
        self,
        pallet: Optional[dict],
        width:  int,
        height: int,
    ) -> None:
        err_msg = Point()

        if pallet is None:
            # No target found — stop and publish not-aligned
            err_msg.x = 0.0
            err_msg.y = 0.0
            err_msg.z = 0.0
            self._pub_alignment_error.publish(err_msg)
            self._publish_zero_cmd_vel()
            return

        cx, cy = pallet['centroid']
        error_x = float(cx - width  // 2)   # + = pallet right of centre
        error_y = float(cy - height // 2)   # + = pallet below centre

        # PID (proportional-only; integral/derivative can be added later)
        # Negative sign: if pallet is to the right, turn right (CCW negative)
        angular = _clamp(-self._kp_angular * error_x, self._max_angular)
        # Drive forward when pallet is in lower half (cy > height/2), back off
        # if too close (pallet fills upper half)
        linear  = _clamp( self._kp_linear * (height // 2 - cy), self._max_linear)

        # Check alignment
        aligned = (
            math.fabs(error_x) < self._threshold
            and math.fabs(error_y) < self._threshold
        )

        # Publish alignment error
        err_msg.x = error_x
        err_msg.y = error_y
        err_msg.z = 1.0 if aligned else 0.0
        self._pub_alignment_error.publish(err_msg)

        # Publish velocity command
        twist = Twist()
        if not aligned:
            twist.linear.x  = linear
            twist.angular.z = angular
        self._pub_cmd_vel.publish(twist)

        if aligned:
            self.get_logger().info(
                'Pallet aligned: error_x=%.1f px  error_y=%.1f px' % (error_x, error_y),
                throttle_duration_sec=2.0,
            )

    def _publish_zero_cmd_vel(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _publish_aruco_poses(self, detections: list) -> None:
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.poses = [det['pose'] for det in detections]
        self._pub_aruco_poses.publish(msg)

        if detections:
            ids = [det['id'] for det in detections]
            self.get_logger().debug(
                f'ArUco markers detected: {ids}',
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
