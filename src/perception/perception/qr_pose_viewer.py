#!/usr/bin/env python3
"""Nodo diagnostico para verificar la deteccion y pose de QR codes del palet.

Se suscribe al topic de la camara, corre el QRPoseDetector con la
calibracion cargada desde YAML, muestra una ventana OpenCV con la
deteccion (esquinas, ejes 3D, distancia, yaw, id decodificado) y publica
las poses en `/qr_poses` para que el resto del sistema pueda consumirlas.

Uso:
  ros2 run perception qr_pose_viewer --ros-args \\
    -p image_topic:=/video_source/raw \\
    -p marker_length:=0.035 \\
    -p camera_params:=src/perception/config/camera_params.yaml

Controles ventana:
  q / ESC  salir
"""
from __future__ import annotations

import math
import os
import time
from typing import List

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .qr_pose_detector import QRPoseDetector, euler_from_rvec


def _load_camera_yaml(path: str) -> tuple[np.ndarray, np.ndarray, int, int]:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    cam = data['camera']
    K = np.array(cam['camera_matrix'], dtype=np.float64).reshape(3, 3)
    dist = np.array(cam['distortion_coefficients'], dtype=np.float64).flatten()
    w = int(cam.get('image_width', 0))
    h = int(cam.get('image_height', 0))
    return K, dist, w, h


class QRPoseViewer(Node):
    def __init__(self) -> None:
        super().__init__('qr_pose_viewer')

        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('qos', 'sensor_data')
        self.declare_parameter(
            'camera_params', 'src/perception/config/camera_params.yaml'
        )
        self.declare_parameter('marker_length', 0.035)
        self.declare_parameter('show_window', True)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('backend', 'auto')  # auto|aruco|classic

        self._image_topic = str(self.get_parameter('image_topic').value)
        qos_name = str(self.get_parameter('qos').value).lower()
        cam_path = str(self.get_parameter('camera_params').value)
        self._marker_length = float(self.get_parameter('marker_length').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        if not os.path.isabs(cam_path):
            cam_path = os.path.abspath(cam_path)
        if not os.path.exists(cam_path):
            self.get_logger().error(f'No existe camera_params: {cam_path}')
            raise SystemExit(1)

        K, dist, w, h = _load_camera_yaml(cam_path)
        self.get_logger().info(
            f'Calibracion cargada: {w}x{h}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} '
            f'cx={K[0,2]:.1f} cy={K[1,2]:.1f}'
        )

        backend = str(self.get_parameter('backend').value)
        self._detector = QRPoseDetector(
            camera_matrix=K,
            dist_coeffs=dist,
            marker_length=self._marker_length,
            refine=True,
            backend=backend,
        )
        self.get_logger().info(f'Detector QR backend = {self._detector.backend!r}')
        self._K = K
        self._dist = dist
        self._bridge = CvBridge()

        if qos_name in ('reliable', 'default'):
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                durability=DurabilityPolicy.VOLATILE,
            )

        self._sub = self.create_subscription(
            Image, self._image_topic, self._image_cb, qos
        )
        self._pub_poses = self.create_publisher(PoseArray, '/qr_poses', 10)
        self._pub_detected = self.create_publisher(Bool, '/qr_detected', 10)

        if self._show_window:
            cv2.namedWindow('qr_pose', cv2.WINDOW_NORMAL)

        self._last_frame_time = time.monotonic()
        self._fps = 0.0
        self._frame_count = 0
        self._first_frame_logged = False

        self.get_logger().info(
            f'qr_pose_viewer listo. topic={self._image_topic} '
            f'qos={qos_name} marker={self._marker_length*1000:.1f}mm'
        )

        # Timer de UI (procesa el frame mas reciente sin bloquear el callback)
        self._latest_frame: np.ndarray | None = None
        if self._show_window:
            self.create_timer(0.04, self._ui_tick)  # ~25 Hz

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge: {exc}', throttle_duration_sec=2.0)
            return

        if not self._first_frame_logged:
            self.get_logger().info(
                f'Primer frame {frame.shape[1]}x{frame.shape[0]} encoding={msg.encoding!r}'
            )
            self._first_frame_logged = True

        # FPS
        now = time.monotonic()
        dt = now - self._last_frame_time
        self._last_frame_time = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        self._frame_count += 1

        # Deteccion + publicacion (siempre). Aislada para no matar el nodo.
        try:
            detections = self._detector.detect(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'detect() lanzo {type(exc).__name__}: {exc}',
                throttle_duration_sec=2.0,
            )
            detections = []
        self._publish(detections, msg.header.stamp)

        # Log periodico de pose para verificar (cada ~1s)
        if detections and (self._frame_count % 15 == 0):
            d = detections[0]
            tx, ty, tz = d['tvec']
            _, pitch, yaw = euler_from_rvec(d['rvec'])
            self.get_logger().info(
                f"QR id={d['id']!r}  X={tx*1000:+.0f}mm Y={ty*1000:+.0f}mm "
                f"Z={tz*1000:+.0f}mm  yaw={math.degrees(yaw):+.1f}° "
                f"pitch={math.degrees(pitch):+.1f}°"
            )

        if self._show_window:
            self._latest_frame = (frame, detections)

    # ------------------------------------------------------------------
    def _publish(self, detections: List[dict], stamp) -> None:
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self._frame_id
        pa.poses = [d['pose'] for d in detections]
        self._pub_poses.publish(pa)

        b = Bool()
        b.data = len(detections) > 0
        self._pub_detected.publish(b)

    # ------------------------------------------------------------------
    def _ui_tick(self) -> None:
        if self._latest_frame is None:
            return
        frame, detections = self._latest_frame
        out = self._detector.draw_detections(frame, detections)

        # HUD
        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        cv2.line(out, (0, h // 2), (w, h // 2), (80, 80, 80), 1)

        if detections:
            for i, d in enumerate(detections):
                tx, ty, tz = d['tvec']
                roll, pitch, yaw = euler_from_rvec(d['rvec'])
                lines = [
                    f"id: {d['id'] or '(no decode)'}",
                    f"X:{tx*1000:+.0f}  Y:{ty*1000:+.0f}  Z:{tz*1000:+.0f}  mm",
                    f"yaw:{math.degrees(yaw):+.1f}  pitch:{math.degrees(pitch):+.1f}  roll:{math.degrees(roll):+.1f}",
                ]
                y0 = 20 + i * 60
                for j, txt in enumerate(lines):
                    cv2.putText(
                        out, txt, (8, y0 + j * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
                    )
            status = f'{len(detections)} QR'
            color = (0, 255, 0)
        else:
            status = 'sin deteccion'
            color = (0, 0, 255)

        cv2.putText(out, f'{status}  {self._fps:5.1f} fps',
                    (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        cv2.imshow('qr_pose', out)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('Cerrando...')
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = QRPoseViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
