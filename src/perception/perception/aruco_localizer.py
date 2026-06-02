"""aruco_localizer.py — Absolute-pose EKF correction from ArUco markers.

Subscribes to /cam_img, detects ArUco markers, and publishes
/pose_measurement (PoseWithCovarianceStamped) so the EKF can fuse an
absolute-pose correction on top of its wheel-odometry prediction.

Frame convention
────────────────
  camera_link (ROS robot frame): X=forward, Y=left, Z=up
  OpenCV optical frame:           X=right,   Y=down,  Z=forward
  ArUco result: pose of the marker IN the OpenCV camera frame.

Transform chain for each detected marker
─────────────────────────────────────────
  p_map = T_m2map · inv(T_cv2m) · T_cl2cv · T_cl2bf · p_bf

  T_cl2bf  : camera_link → base_footprint  (TF2, cached on first use)
  T_cl2cv  : camera_link → OpenCV frame    (fixed rotation _R_CL_TO_CV)
  T_cv2m   : OpenCV frame → marker frame   (ArUco estimatePoseSingleMarkers)
  T_m2map  : marker frame → map frame      (known from aruco_map.yaml)

  The translation of the result gives the robot position in the map;
  the yaw is extracted from the 2D rotation part.
"""

from __future__ import annotations

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import tf2_ros


# Rotation: camera_link (X=fwd, Y=left, Z=up) → OpenCV (X=right, Y=down, Z=fwd)
# v_opencv = _R_CL_TO_CV @ v_camera_link
_R_CL_TO_CV: np.ndarray = np.array([
    [0., -1.,  0.],
    [0.,  0., -1.],
    [1.,  0.,  0.],
], dtype=np.float64)

_T_CL_TO_CV = np.eye(4)
_T_CL_TO_CV[:3, :3] = _R_CL_TO_CV


def _tf_to_matrix(tf_stamped) -> np.ndarray:
    """Convert a TransformStamped to a 4×4 homogeneous matrix."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def _build_T_marker_to_map(mx: float, my: float, mtheta: float, mz: float) -> np.ndarray:
    """4×4 transform: ArUco marker frame → map frame.

    ArUco marker axes (when marker is mounted vertically on a wall):
      X = right when viewed          [sin θ, -cos θ, 0]  in map
      Y = down                       [0,      0,     -1] in map
      Z = face normal (toward robot) [cos θ,  sin θ,  0] in map

    mtheta: direction the marker face points in the horizontal plane (rad).
    mz:     height of marker centre above the floor (m).
    """
    ct, st = math.cos(mtheta), math.sin(mtheta)
    R = np.array([
        [st,   0.0,  ct ],
        [-ct,  0.0,  st ],
        [0.0, -1.0,  0.0],
    ], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [mx, my, mz]
    return T


class ArucoLocalizer(Node):
    def __init__(self) -> None:
        super().__init__('aruco_localizer')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('aruco_dictionary',          'DICT_4X4_50')
        self.declare_parameter('marker_length',             0.05)
        self.declare_parameter('marker_height',             0.25)
        self.declare_parameter('max_detection_dist',        3.0)
        self.declare_parameter('jump_threshold',            1.0)
        self.declare_parameter('base_cov_xy',               0.010)
        self.declare_parameter('base_cov_theta',            0.005)
        self.declare_parameter('cov_dist_k',                0.04)
        self.declare_parameter('camera_frame',              'camera_link')
        self.declare_parameter('base_frame',                'base_footprint')
        self.declare_parameter('marker_ids',                rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('marker_x',                  rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('marker_y',                  rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('marker_theta',              rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('camera_matrix',             rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('distortion_coefficients',   rclpy.Parameter.Type.DOUBLE_ARRAY)

        p = self.get_parameter
        self._marker_len  = p('marker_length').value
        self._marker_z    = p('marker_height').value
        self._max_dist    = p('max_detection_dist').value
        self._jump_thr    = p('jump_threshold').value
        self._base_cov_xy = p('base_cov_xy').value
        self._base_cov_th = p('base_cov_theta').value
        self._cov_k       = p('cov_dist_k').value
        self._cam_frame   = p('camera_frame').value
        self._base_frame  = p('base_frame').value

        # Camera intrinsics
        K_flat = p('camera_matrix').value
        D_flat = p('distortion_coefficients').value
        self._K = np.array(K_flat, dtype=np.float64).reshape(3, 3)
        self._D = np.array(D_flat, dtype=np.float64)

        # Build marker map: id → (T_marker_to_map 4×4)
        ids    = list(p('marker_ids').value)
        xs     = list(p('marker_x').value)
        ys     = list(p('marker_y').value)
        thetas = list(p('marker_theta').value)
        self._marker_map: dict[int, np.ndarray] = {
            int(i): _build_T_marker_to_map(float(x), float(y), float(th), self._marker_z)
            for i, x, y, th in zip(ids, xs, ys, thetas)
        }
        self.get_logger().info(
            f'ArUco localizer ready — {len(self._marker_map)} markers loaded, '
            f'dict={p("aruco_dictionary").value}')

        # ── ArUco detector (OpenCV 4.x and 4.7+ compatible) ───────────────
        dict_name  = p('aruco_dictionary').value
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        try:
            params = cv2.aruco.DetectorParameters()
            self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self._detect = lambda g: self._aruco_detector.detectMarkers(g)
        except AttributeError:
            params = cv2.aruco.DetectorParameters_create()
            self._aruco_dict   = aruco_dict
            self._aruco_params = params
            self._detect = (
                lambda g: cv2.aruco.detectMarkers(g, self._aruco_dict,
                                                  parameters=self._aruco_params))

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._T_cl2bf: Optional[np.ndarray] = None   # cached after first successful lookup

        # ── State ─────────────────────────────────────────────────────────
        self._ekf_x    = 0.0
        self._ekf_y    = 0.0
        self._has_ekf  = False

        # ── I/O ───────────────────────────────────────────────────────────
        self._bridge    = CvBridge()
        self._sub_img   = self.create_subscription(Image, '/cam_img', self._img_cb, 10)
        self._sub_odom  = self.create_subscription(Odometry, '/odom',  self._odom_cb, 10)
        self._pub_meas  = self.create_publisher(PoseWithCovarianceStamped, '/pose_measurement', 10)
        self._pub_poses = self.create_publisher(PoseArray, '/aruco_poses', 10)
        self._pub_debug = self.create_publisher(Image, '/aruco_debug_image', 5)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._ekf_x   = msg.pose.pose.position.x
        self._ekf_y   = msg.pose.pose.position.y
        self._has_ekf = True

    def _img_cb(self, msg: Image) -> None:
        # Lazy TF cache — camera is static, so one lookup suffices
        if self._T_cl2bf is None:
            try:
                tf_s = self._tf_buffer.lookup_transform(
                    self._cam_frame, self._base_frame, Time())
                self._T_cl2bf = _tf_to_matrix(tf_s)
                self.get_logger().info(
                    f'TF {self._cam_frame} → {self._base_frame} cached')
            except Exception:
                self.get_logger().warn(
                    'TF not ready — skipping frame',
                    throttle_duration_sec=2.0)
                return

        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, raw_ids, _ = self._detect(gray)
        if raw_ids is None or len(raw_ids) == 0:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_len, self._K, self._D)

        cv2.aruco.drawDetectedMarkers(frame, corners, raw_ids)

        measurements: list[tuple[float, float, float, float, float]] = []
        # (x, y, yaw, cov_xy, cov_th)
        pose_array = PoseArray()
        pose_array.header.stamp    = msg.header.stamp
        pose_array.header.frame_id = 'map'

        for i, mid in enumerate(raw_ids.flatten()):
            mid = int(mid)
            if mid not in self._marker_map:
                continue

            dist = float(np.linalg.norm(tvecs[i]))
            if dist > self._max_dist:
                self.get_logger().debug(
                    f'id={mid} rejected: dist {dist:.2f} m > max {self._max_dist:.2f} m')
                continue

            try:
                x, y, yaw = self._compute_pose(rvecs[i], tvecs[i], mid)
            except Exception as exc:
                self.get_logger().warn(f'Pose computation failed id={mid}: {exc}')
                continue

            # Anti-jump: only apply once EKF has initialised
            if self._has_ekf:
                delta = math.hypot(x - self._ekf_x, y - self._ekf_y)
                if delta > self._jump_thr:
                    self.get_logger().warn(
                        f'id={mid} rejected: jump {delta:.2f} m '
                        f'(threshold {self._jump_thr:.2f} m)')
                    continue

            scale   = 1.0 + self._cov_k * dist ** 2
            cov_xy  = self._base_cov_xy * scale
            cov_th  = self._base_cov_th * scale
            measurements.append((x, y, yaw, cov_xy, cov_th))

            p = Pose()
            p.position.x    = x
            p.position.y    = y
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)
            pose_array.poses.append(p)

            self.get_logger().debug(
                f'id={mid}  x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°  '
                f'dist={dist:.2f} m  cov_xy={cov_xy:.4f}')

        if not measurements:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))
            return

        self._pub_poses.publish(pose_array)

        # Weighted average — weight = 1 / cov_xy (closer = tighter = more weight)
        weights  = [1.0 / m[3] for m in measurements]
        total_w  = sum(weights)
        x_avg    = sum(m[0] * w for m, w in zip(measurements, weights)) / total_w
        y_avg    = sum(m[1] * w for m, w in zip(measurements, weights)) / total_w

        # Circular mean for yaw
        sin_s = sum(math.sin(m[2]) * w for m, w in zip(measurements, weights))
        cos_s = sum(math.cos(m[2]) * w for m, w in zip(measurements, weights))
        yaw_avg = math.atan2(sin_s, cos_s)

        # Best (minimum) covariance drives the fused uncertainty
        cov_xy_f = min(m[3] for m in measurements)
        cov_th_f = min(m[4] for m in measurements)

        self._publish_measurement(x_avg, y_avg, yaw_avg, cov_xy_f, cov_th_f, msg.header.stamp)

        debug_msg = self._bridge.cv2_to_imgmsg(frame, 'bgr8')
        debug_msg.header = msg.header
        self._pub_debug.publish(debug_msg)

    # ── Core geometry ──────────────────────────────────────────────────────

    def _compute_pose(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        marker_id: int,
    ) -> tuple[float, float, float]:
        """Compute robot (x, y, yaw) in map frame from a single ArUco detection.

        Chain: M_map_bf = T_m2map · inv(T_cv2m) · T_cl2cv · T_cl2bf
          where p_map = M_map_bf · p_base_footprint
        """
        T_m2map = self._marker_map[marker_id]

        # ArUco: p_opencv = R · p_marker + tvec  →  T_cv2m
        R_aruco, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        T_cv2m = np.eye(4)
        T_cv2m[:3, :3] = R_aruco
        T_cv2m[:3, 3]  = tvec.flatten()

        # Full chain
        M = T_m2map @ np.linalg.inv(T_cv2m) @ _T_CL_TO_CV @ self._T_cl2bf

        x   = float(M[0, 3])
        y   = float(M[1, 3])
        yaw = math.atan2(float(M[1, 0]), float(M[0, 0]))
        return x, y, yaw

    def _publish_measurement(
        self,
        x: float, y: float, yaw: float,
        cov_xy: float, cov_th: float,
        stamp,
    ) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        cov[0]  = cov_xy   # x–x
        cov[7]  = cov_xy   # y–y
        cov[35] = cov_th   # yaw–yaw
        msg.pose.covariance = cov

        self._pub_meas.publish(msg)
        self.get_logger().info(
            f'ArUco → EKF: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
