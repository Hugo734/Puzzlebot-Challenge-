#!/usr/bin/env python3
"""Alineamiento al QR via path planning (cubic Bezier) + pure pursuit.

Estrategia:
    A cada tick (10 Hz), si tenemos deteccion fresca:
      1. Recomputamos la pose objetivo en frame robot (punto a D_dock
         frente al QR, sobre su normal, mirando al QR).
      2. Planeamos una Bezier cubica desde (0,0,heading=0) hasta el
         goal con tangentes que respetan el heading inicial y final.
      3. Pure pursuit: buscamos el look-ahead point sobre la curva y
         comandamos v y w para apuntar a el.

    Esto reemplaza el ALIGN scripted del nodo anterior.

Estados:
    IDLE     -> espera /alignment_start
    SEARCH   -> spin lento buscando QR
    FOLLOW   -> pure pursuit con replanning continuo
    DOCK     -> avance recto + bearing correction (ultimo tramo)
    DONE     -> detenido
    LOST     -> detenido por deteccion stale
"""
from __future__ import annotations

import math
import os
import time
from enum import Enum
from typing import List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseArray, Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from .qr_pose_detector import QRPoseDetector
from .path_planner import (
    BezierCurve,
    plan_bezier_in_robot_frame,
    pure_pursuit_cmd,
    compute_open_loop_trajectory,
)


class State(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    FOLLOW = 'FOLLOW'
    DOCK = 'DOCK'
    DONE = 'DONE'
    LOST = 'LOST'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _clip(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _load_camera_yaml(path: str) -> tuple[np.ndarray, np.ndarray]:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    cam = data['camera']
    K = np.array(cam['camera_matrix'], dtype=np.float64).reshape(3, 3)
    dist = np.array(cam['distortion_coefficients'], dtype=np.float64).flatten()
    return K, dist


def compute_geometry(
    rvec: np.ndarray,
    tvec: np.ndarray,
    cam_offset_x: float,
    cam_offset_y: float,
    D_dock: float,
) -> Optional[dict]:
    """Pose del QR y pose objetivo (en frame robot)."""
    X_cam, _Y_cam, Z_cam = float(tvec[0]), float(tvec[1]), float(tvec[2])
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))

    qr_x = cam_offset_x + Z_cam
    qr_y = cam_offset_y - X_cam

    nx_cam = float(R[0, 2])
    nz_cam = float(R[2, 2])
    n_x_r = nz_cam
    n_y_r = -nx_cam
    n_norm = math.hypot(n_x_r, n_y_r)
    if n_norm < 1e-6:
        return None
    n_x_r /= n_norm
    n_y_r /= n_norm

    # Goal: a D_dock del QR sobre su normal, mirando al QR
    gx = qr_x + D_dock * n_x_r
    gy = qr_y + D_dock * n_y_r
    # Heading deseado al llegar al goal: hacia el QR (direccion opuesta a la normal)
    g_th = math.atan2(qr_y - gy, qr_x - gx)

    bearing_qr = math.atan2(qr_y, qr_x)
    dist_qr = math.hypot(qr_x, qr_y)
    dist_goal = math.hypot(gx, gy)

    return {
        'qr_x': qr_x, 'qr_y': qr_y,
        'n_x_r': n_x_r, 'n_y_r': n_y_r,
        'gx': gx, 'gy': gy, 'g_th': g_th,
        'bearing_qr': bearing_qr,
        'dist_qr': dist_qr,
        'dist_goal': dist_goal,
    }


def _ema_blend(prev: dict, new: dict, a: float) -> dict:
    out: dict = {}
    for k in ('qr_x', 'qr_y', 'n_x_r', 'n_y_r', 'gx', 'gy',
              'dist_qr', 'dist_goal'):
        out[k] = (1 - a) * prev[k] + a * new[k]
    for k in ('bearing_qr', 'g_th'):
        s = (1 - a) * math.sin(prev[k]) + a * math.sin(new[k])
        c = (1 - a) * math.cos(prev[k]) + a * math.cos(new[k])
        out[k] = math.atan2(s, c)
    return out


# ---------------------------------------------------------------------------
# Nodo
# ---------------------------------------------------------------------------
class QRPathAlignmentNode(Node):
    def __init__(self) -> None:
        super().__init__('qr_path_alignment')

        # ---- Params I/O ----
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('qos', 'sensor_data')
        self.declare_parameter(
            'camera_params', 'src/perception/config/camera_params.yaml'
        )
        self.declare_parameter('marker_length', 0.030)
        self.declare_parameter('backend', 'auto')

        # ---- Geometria ----
        self.declare_parameter('dock_distance', 0.200)  # camara->QR final
        self.declare_parameter('cam_offset_x', 0.100)
        self.declare_parameter('cam_offset_y', 0.0)
        self.declare_parameter('dock_switch_dist', 0.10)  # m, cuando |robot-goal|<X -> DOCK

        # ---- Velocidades ----
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.19)
        self.declare_parameter('min_linear', 0.025)
        self.declare_parameter('dock_max_linear', 0.05)
        self.declare_parameter('search_angular', 0.15)

        # ---- Pure pursuit gains (no usado en open-loop) ----
        self.declare_parameter('lookahead_dist', 0.08)
        self.declare_parameter('kp_w', 1.5)
        self.declare_parameter('kp_v', 0.4)
        # Para el DOCK (ultimo tramo recto + bearing correction)
        self.declare_parameter('kp_bearing_dock', 0.6)
        self.declare_parameter('kp_v_dock', 0.5)
        # Modo: 'open_loop' (trayectoria precomputada) o 'pure_pursuit'
        self.declare_parameter('follow_mode', 'open_loop')
        # Para open-loop: velocidad constante durante FOLLOW
        self.declare_parameter('open_loop_v', 0.04)

        # ---- Curva ----
        self.declare_parameter('bezier_control_scale', 0.4)
        self.declare_parameter('bezier_min_control_dist', 0.05)

        # ---- Tolerancias ----
        self.declare_parameter('dock_tol_m', 0.015)

        # ---- Filtro y safety ----
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('detection_max_age_s', 1.5)
        self.declare_parameter('control_freq', 10.0)
        self.declare_parameter('plan_period_ticks', 2)   # replan cada N ticks (10/2=5Hz)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('show_window', True)
        self.declare_parameter('frame_id', 'camera_link')

        # ---- Leer params ----
        self._image_topic   = str(self.get_parameter('image_topic').value)
        qos_name            = str(self.get_parameter('qos').value).lower()
        cam_path            = str(self.get_parameter('camera_params').value)
        self._marker_length = float(self.get_parameter('marker_length').value)
        backend             = str(self.get_parameter('backend').value)

        self._D_dock        = float(self.get_parameter('dock_distance').value)
        self._cam_off_x     = float(self.get_parameter('cam_offset_x').value)
        self._cam_off_y     = float(self.get_parameter('cam_offset_y').value)
        self._dock_switch_dist = float(self.get_parameter('dock_switch_dist').value)

        self._v_max         = float(self.get_parameter('max_linear').value)
        self._w_max         = float(self.get_parameter('max_angular').value)
        self._v_min         = float(self.get_parameter('min_linear').value)
        self._v_dock_max    = float(self.get_parameter('dock_max_linear').value)
        self._w_search      = float(self.get_parameter('search_angular').value)

        self._d_lookahead   = float(self.get_parameter('lookahead_dist').value)
        self._kp_w          = float(self.get_parameter('kp_w').value)
        self._kp_v          = float(self.get_parameter('kp_v').value)
        self._kp_bearing_dock = float(self.get_parameter('kp_bearing_dock').value)
        self._kp_v_dock     = float(self.get_parameter('kp_v_dock').value)
        self._follow_mode   = str(self.get_parameter('follow_mode').value).lower()
        self._open_loop_v   = float(self.get_parameter('open_loop_v').value)

        self._bezier_scale  = float(self.get_parameter('bezier_control_scale').value)
        self._bezier_min_d  = float(self.get_parameter('bezier_min_control_dist').value)

        self._tol_dock      = float(self.get_parameter('dock_tol_m').value)

        self._ema_alpha     = float(self.get_parameter('ema_alpha').value)
        self._max_det_age   = float(self.get_parameter('detection_max_age_s').value)
        self._control_dt    = 1.0 / max(1e-3, float(self.get_parameter('control_freq').value))
        self._plan_period   = int(self.get_parameter('plan_period_ticks').value)
        self._dry_run       = bool(self.get_parameter('dry_run').value)
        self._auto_start    = bool(self.get_parameter('auto_start').value)
        self._show_window   = bool(self.get_parameter('show_window').value)
        self._frame_id      = str(self.get_parameter('frame_id').value)

        # ---- Calibracion ----
        if not os.path.isabs(cam_path):
            cam_path = os.path.abspath(cam_path)
        if not os.path.exists(cam_path):
            self.get_logger().error(f'No existe camera_params: {cam_path}')
            raise SystemExit(1)
        K, dist = _load_camera_yaml(cam_path)
        self._K, self._dist = K, dist

        # ---- Detector ----
        self._detector = QRPoseDetector(
            camera_matrix=K, dist_coeffs=dist,
            marker_length=self._marker_length, refine=True, backend=backend,
        )
        self.get_logger().info(
            f'Detector backend={self._detector.backend!r} marker={self._marker_length*1000:.1f}mm'
        )

        # ---- QoS ----
        if qos_name in ('reliable', 'default'):
            img_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
        else:
            img_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5,
                durability=DurabilityPolicy.VOLATILE,
            )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ---- ROS I/O ----
        self._bridge = CvBridge()
        self.create_subscription(Image, self._image_topic, self._image_cb, img_qos)
        self.create_subscription(Bool, '/alignment_start', self._start_cb, 10)
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', cmd_qos)
        self._pub_state = self.create_publisher(String, '/alignment_state', 10)
        self._pub_qr = self.create_publisher(PoseArray, '/qr_poses', 10)
        self._pub_goal = self.create_publisher(Point, '/path_debug/goal', 10)

        # ---- Estado interno ----
        self._state: State = State.IDLE if not self._auto_start else State.SEARCH
        self._geom_filt: Optional[dict] = None
        self._last_rvec_tvec: Optional[tuple[np.ndarray, np.ndarray]] = None
        self._last_det_time: Optional[float] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_dets: List[dict] = []
        self._curve: Optional[BezierCurve] = None
        self._lookahead_point: Optional[np.ndarray] = None
        self._tick_count: int = 0
        # Open-loop trajectory
        self._ol_trajectory: list[tuple[float, float]] = []
        self._ol_index: int = 0

        if self._show_window:
            cv2.namedWindow('qr_path_alignment', cv2.WINDOW_NORMAL)

        self.create_timer(self._control_dt, self._control_tick)

        self.get_logger().info(
            f'qr_path_alignment listo  dock={self._D_dock*1000:.0f}mm  '
            f'lookahead={self._d_lookahead*1000:.0f}mm  '
            f'replan cada {self._plan_period} ticks  '
            f'dry_run={self._dry_run}'
        )

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge: {exc}', throttle_duration_sec=2.0)
            return

        try:
            dets = self._detector.detect(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'detect: {exc}', throttle_duration_sec=2.0)
            dets = []

        self._latest_frame = frame
        self._latest_dets = dets

        pa = PoseArray()
        pa.header.stamp = msg.header.stamp
        pa.header.frame_id = self._frame_id
        pa.poses = [d['pose'] for d in dets]
        self._pub_qr.publish(pa)

        if dets:
            d = dets[0]
            new_geom = compute_geometry(
                d['rvec'], d['tvec'], self._cam_off_x, self._cam_off_y, self._D_dock,
            )
            if new_geom is not None:
                if self._geom_filt is None:
                    self._geom_filt = new_geom
                else:
                    self._geom_filt = _ema_blend(self._geom_filt, new_geom, self._ema_alpha)
                self._last_rvec_tvec = (d['rvec'], d['tvec'])
                self._last_det_time = time.monotonic()
                p = Point(); p.x = new_geom['gx']; p.y = new_geom['gy']; p.z = 0.0
                self._pub_goal.publish(p)

    def _start_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._is_detection_fresh() and self._geom_filt is not None:
                self._set_state(State.FOLLOW, reason='start request')
            else:
                self._set_state(State.SEARCH, reason='start, sin det fresca')
        else:
            self._set_state(State.IDLE, reason='stop request')

    def _is_detection_fresh(self) -> bool:
        if self._last_det_time is None:
            return False
        return (time.monotonic() - self._last_det_time) <= self._max_det_age

    # ------------------------------------------------------------------
    def _control_tick(self) -> None:
        self._tick_count += 1

        s = String(); s.data = self._state.value
        self._pub_state.publish(s)

        if self._state in (State.IDLE, State.DONE, State.LOST):
            self._publish_zero()
            self._render_ui()
            return

        if self._state == State.SEARCH:
            self._run_search()
            self._render_ui()
            return

        if not self._is_detection_fresh() or self._geom_filt is None:
            self._publish_zero()
            self._set_state(State.LOST, reason='deteccion stale')
            self._render_ui()
            return

        geom = self._geom_filt
        if self._state == State.FOLLOW:
            self._run_follow(geom)
        elif self._state == State.DOCK:
            self._run_dock(geom)
        self._render_ui()

    def _run_search(self) -> None:
        if self._is_detection_fresh() and self._geom_filt is not None:
            self._set_state(State.FOLLOW, reason='QR encontrado')
            return
        self._publish_cmd(0.0, self._w_search)

    def _run_follow(self, geom: dict) -> None:
        """FOLLOW: open-loop trajectory o pure pursuit segun follow_mode."""
        dist_goal = geom['dist_goal']

        # Transicion a DOCK si llegamos cerca del goal
        if dist_goal < self._dock_switch_dist:
            self._set_state(State.DOCK, reason=f'dist_goal={dist_goal*1000:.0f}mm < switch')
            return

        if self._follow_mode == 'open_loop':
            # Open-loop: ejecutar trayectoria pre-computada sin feedback
            if self._ol_index >= len(self._ol_trajectory):
                # Termino la trayectoria. Re-evaluar: re-plan y ejecutar de nuevo,
                # o pasar a DOCK si estamos cerca.
                if dist_goal < self._dock_switch_dist * 1.5:
                    self._set_state(State.DOCK,
                                    reason=f'open-loop done, dist_goal={dist_goal*1000:.0f}mm')
                    return
                # Re-plan para otra pasada open-loop
                self._curve = plan_bezier_in_robot_frame(
                    goal_xy=(geom['gx'], geom['gy']),
                    goal_heading=geom['g_th'],
                    control_scale=self._bezier_scale,
                    min_control_dist=self._bezier_min_d,
                )
                self._ol_trajectory = compute_open_loop_trajectory(
                    self._curve, self._open_loop_v, self._control_dt,
                )
                self._ol_index = 0
                self.get_logger().info(
                    f'OPEN_LOOP re-plan: {len(self._ol_trajectory)} ticks'
                )
            v, w = self._ol_trajectory[self._ol_index]
            self._ol_index += 1
            self._publish_cmd(v, w)
            return

        # Pure pursuit mode (closed-loop, replanning continuo)
        if self._curve is None or (self._tick_count % self._plan_period) == 0:
            self._curve = plan_bezier_in_robot_frame(
                goal_xy=(geom['gx'], geom['gy']),
                goal_heading=geom['g_th'],
                control_scale=self._bezier_scale,
                min_control_dist=self._bezier_min_d,
            )
        v, w, look = pure_pursuit_cmd(
            self._curve,
            d_lookahead=self._d_lookahead,
            kp_w=self._kp_w,
            kp_v=self._kp_v,
            v_max=self._v_max,
            v_min=self._v_min,
            w_max=self._w_max,
            dist_to_goal_for_v_scaling=dist_goal,
        )
        self._lookahead_point = look
        self._publish_cmd(v, w)

    def _run_dock(self, geom: dict) -> None:
        """Ultimo tramo: avance directo al QR con bearing correction."""
        err = geom['dist_qr'] - self._D_dock
        if abs(err) < self._tol_dock:
            self._set_state(State.DONE, reason='dock distance alcanzada')
            self._publish_zero()
            return
        v = _clip(self._kp_v_dock * err, -self._v_dock_max, self._v_dock_max)
        w = _clip(self._kp_bearing_dock * geom['bearing_qr'],
                  -self._w_max, self._w_max)
        self._publish_cmd(v, w)

    # ------------------------------------------------------------------
    # Hard safety caps (no se pueden superar via parametros)
    _HARD_V_MAX = 0.10
    _HARD_W_MAX = 0.19
    # Deadband: si v y w son muy chicos, mejor mandar cero (evita zumbido motor)
    _V_DEADBAND = 0.015
    _W_DEADBAND = 0.03
    # Low-pass filter sobre las cmds publicadas (suaviza vibracion)
    # cmd_out = alpha * cmd_new + (1-alpha) * cmd_prev
    _CMD_LPF_ALPHA = 0.30

    def _publish_cmd(self, v: float, w: float) -> None:
        if self._dry_run:
            return
        # 1) Clamp duro de seguridad
        v_safe = max(-self._HARD_V_MAX, min(self._HARD_V_MAX, float(v)))
        w_safe = max(-self._HARD_W_MAX, min(self._HARD_W_MAX, float(w)))
        # 2) Low-pass filter para suavizar transiciones (evita zigzag del motor)
        if not hasattr(self, '_cmd_v_prev'):
            self._cmd_v_prev = 0.0
            self._cmd_w_prev = 0.0
        a = self._CMD_LPF_ALPHA
        v_smooth = a * v_safe + (1.0 - a) * self._cmd_v_prev
        w_smooth = a * w_safe + (1.0 - a) * self._cmd_w_prev
        self._cmd_v_prev = v_smooth
        self._cmd_w_prev = w_smooth
        # 3) Deadband: cmd chiquito -> zero motor (evita PWM jitter)
        v_out = 0.0 if abs(v_smooth) < self._V_DEADBAND else v_smooth
        w_out = 0.0 if abs(w_smooth) < self._W_DEADBAND else w_smooth
        t = Twist()
        t.linear.x = v_out
        t.angular.z = w_out
        self._pub_cmd.publish(t)
        self.get_logger().info(
            f'cmd_vel v={v_out:+.3f} w={w_out:+.3f}  [{self._state.value}]',
            throttle_duration_sec=0.5,
        )

    def _publish_zero(self) -> None:
        if self._dry_run:
            return
        # Reset LPF para que un IDLE/STOP no quede residual del comando previo
        self._cmd_v_prev = 0.0
        self._cmd_w_prev = 0.0
        # Publicar zero VARIAS veces para asegurarse que el puzzlebot lo recibe
        zero = Twist()
        for _ in range(3):
            self._pub_cmd.publish(zero)

    def _set_state(self, new: State, reason: str = '') -> None:
        if new == self._state:
            return
        g = self._geom_filt
        gs = ''
        if g is not None:
            gs = (f'  [bear={math.degrees(g["bearing_qr"]):+.1f} '
                  f'dist_qr={g["dist_qr"]*1000:.0f}mm '
                  f'dist_goal={g["dist_goal"]*1000:.0f}mm]')
        self.get_logger().info(f'{self._state.value} -> {new.value} ({reason}){gs}')
        prev_state = self._state
        self._state = new
        # Reset curva al cambiar de estado
        self._curve = None
        # Si entramos a FOLLOW en modo open-loop, precomputar trayectoria
        if new == State.FOLLOW and self._follow_mode == 'open_loop' \
                and self._geom_filt is not None:
            self._curve = plan_bezier_in_robot_frame(
                goal_xy=(g['gx'], g['gy']),
                goal_heading=g['g_th'],
                control_scale=self._bezier_scale,
                min_control_dist=self._bezier_min_d,
            )
            self._ol_trajectory = compute_open_loop_trajectory(
                self._curve, self._open_loop_v, self._control_dt,
            )
            self._ol_index = 0
            self.get_logger().info(
                f'OPEN_LOOP trayectoria: {len(self._ol_trajectory)} ticks, '
                f'~{len(self._ol_trajectory)*self._control_dt:.1f}s'
            )
        self._publish_zero()

    # ------------------------------------------------------------------
    def _project_robot_xy_to_image(self, x_r: float, y_r: float,
                                    z_world: float = 0.0) -> Optional[tuple[int, int]]:
        """Convierte (x, y) en frame robot a pixeles en la imagen.

        Asume cam_offset_z=0 y robot+camera con misma orientacion (cam
        Z = robot +x, cam X = -robot y).
        """
        # robot frame -> cam frame
        # robot.x = +cam.Z + offset_x  ->  cam.Z = robot.x - offset_x
        # robot.y = -cam.X + offset_y  ->  cam.X = offset_y - robot.y
        Z_cam = x_r - self._cam_off_x
        X_cam = self._cam_off_y - y_r
        Y_cam = z_world  # plano del piso al nivel de la camara
        if Z_cam <= 0.01:
            return None
        try:
            pts = np.array([[X_cam, Y_cam, Z_cam]], dtype=np.float64).reshape(1, 1, 3)
            proj, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), self._K, self._dist)
            u, v = proj[0, 0, 0], proj[0, 0, 1]
            return int(u), int(v)
        except cv2.error:
            return None

    def _render_ui(self) -> None:
        if not self._show_window or self._latest_frame is None:
            return
        out = self._detector.draw_detections(self._latest_frame, self._latest_dets)

        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        cv2.line(out, (0, h // 2), (w, h // 2), (80, 80, 80), 1)

        # Dibujar la curva Bezier proyectada
        if self._curve is not None:
            pts_img = []
            for sample in self._curve.samples():
                pix = self._project_robot_xy_to_image(sample[0], sample[1])
                if pix is not None:
                    pts_img.append(pix)
            if len(pts_img) >= 2:
                cv2.polylines(out, [np.array(pts_img, dtype=np.int32)],
                              False, (0, 200, 255), 2)
            # Marcar P0..P3
            for i, p in enumerate([self._curve.p0, self._curve.p1,
                                    self._curve.p2, self._curve.p3]):
                pix = self._project_robot_xy_to_image(p[0], p[1])
                if pix is not None:
                    color = [(255, 255, 0), (200, 200, 0),
                             (200, 0, 200), (0, 255, 0)][i]
                    cv2.circle(out, pix, 4, color, -1)

        # Marcar look-ahead point
        if self._lookahead_point is not None:
            pix = self._project_robot_xy_to_image(
                self._lookahead_point[0], self._lookahead_point[1]
            )
            if pix is not None:
                cv2.drawMarker(out, pix, (0, 255, 255), cv2.MARKER_CROSS, 14, 2)

        # Banner de estado
        state_color = {
            State.IDLE:    (200, 200, 200),
            State.SEARCH:  (255, 200,   0),
            State.FOLLOW:  (  0, 200, 255),
            State.DOCK:    (  0, 200,   0),
            State.DONE:    (  0, 255,   0),
            State.LOST:    (  0,   0, 255),
        }[self._state]
        cv2.rectangle(out, (0, 0), (w, 22), state_color, -1)
        label = self._state.value + ('  [DRY]' if self._dry_run else '')
        cv2.putText(out, label, (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

        g = self._geom_filt
        fresh = self._is_detection_fresh()
        age_ms = ((time.monotonic() - self._last_det_time) * 1000.0
                  if self._last_det_time else 0.0)
        if g is not None:
            txts = [
                f"QR  x={g['qr_x']*1000:+.0f} y={g['qr_y']*1000:+.0f}  d={g['dist_qr']*1000:.0f}mm",
                f"GOAL x={g['gx']*1000:+.0f} y={g['gy']*1000:+.0f}  d={g['dist_goal']*1000:.0f}mm  th={math.degrees(g['g_th']):+.0f}",
                f"bearing_qr={math.degrees(g['bearing_qr']):+.1f}  det_age={age_ms:.0f}ms  fresh={fresh}",
            ]
        else:
            txts = ['sin deteccion', f'det_age={age_ms:.0f}ms']
        for i, t in enumerate(txts):
            cv2.putText(out, t, (6, 40 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('qr_path_alignment', out)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('Cierre solicitado')
            self._publish_zero()
            rclpy.shutdown()
        elif key == ord(' '):
            if self._state == State.IDLE:
                if self._is_detection_fresh() and self._geom_filt is not None:
                    self._set_state(State.FOLLOW, reason='SPACE')
                else:
                    self._set_state(State.SEARCH, reason='SPACE')
            else:
                self._set_state(State.IDLE, reason='SPACE')


def main() -> None:
    rclpy.init()
    node = QRPathAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()  # noqa: SLF001
        except Exception:
            pass
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
