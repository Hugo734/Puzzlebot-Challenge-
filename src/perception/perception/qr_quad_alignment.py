#!/usr/bin/env python3
"""Alineamiento por parabola y=ax^2 con odometria de ruedas.

Pipeline:
    IDLE -> SEARCH (no det) | PLAN (det fresca)
    SEARCH -> PLAN (al detectar)
    PLAN  -> FOLLOW
        Snapshot pose QR. Calcula goal (D_dock frente al QR sobre la
        normal). Plan parabola en frame robot. Reset odometria.
    FOLLOW
        Pure pursuit en frame de planeo (fijo al inicio), usando pose
        odometrica como (x_r, y_r, th_r). NO depende del QR aqui.
        Cuando x_r >= gx (o cerca) -> DOCK.
    DOCK
        Feedback con QR (si visible): avance recto + small bearing corr
        hasta D_dock. Si no ve QR, avance corto open-loop.
    DONE / LOST

Activacion: SPACE en ventana o `/alignment_start true`.
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
from .quadratic_path import (
    QuadraticPath,
    plan_quadratic_in_robot_frame,
    compute_quadratic_trajectory,
)
from .odometry_tracker import OdometryTracker


class State(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    PLAN = 'PLAN'
    FOLLOW = 'FOLLOW'
    DOCK = 'DOCK'
    DONE = 'DONE'
    LOST = 'LOST'


def _clip(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _with_floor(cmd: float, floor: float, ceiling: float) -> float:
    """Si cmd!=0 pero |cmd|<floor, lo eleva al floor manteniendo signo.
    Luego clipea al rango [-ceiling, ceiling]."""
    if cmd == 0:
        return 0.0
    s = math.copysign(1.0, cmd)
    mag = max(floor, min(ceiling, abs(cmd)))
    return s * mag


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
    gx = qr_x + D_dock * n_x_r
    gy = qr_y + D_dock * n_y_r
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


class QRQuadAlignmentNode(Node):
    def __init__(self) -> None:
        super().__init__('qr_quad_alignment')

        # ---- I/O ----
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('qos', 'sensor_data')
        self.declare_parameter(
            'camera_params', 'src/perception/config/camera_params.yaml'
        )
        self.declare_parameter('marker_length', 0.030)
        self.declare_parameter('backend', 'auto')

        # ---- Geometria ----
        self.declare_parameter('dock_distance', 0.200)
        self.declare_parameter('cam_offset_x', 0.100)
        self.declare_parameter('cam_offset_y', 0.0)
        # Aggressiveness de la curva: multiplica gy al planear.
        self.declare_parameter('path_lateral_scale', 1.0)
        # Escala progresiva: extra agresividad PROPORCIONAL a |qr_y|.
        # gy_efectivo = gy * (path_lateral_scale + path_progressive_gain * |gy|/path_progressive_ref)
        # Ej: con gain=1.0 y ref=0.05, un qr_y=0.10 anade +2.0 al scale.
        self.declare_parameter('path_progressive_gain', 1.0)
        self.declare_parameter('path_progressive_ref', 0.05)
        # Si |bearing_qr| < center_threshold_rad -> ir directo a DOCK.
        self.declare_parameter('center_threshold_rad', 0.10)
        # EMA sobre geometria para que el snapshot en PLAN sea estable.
        self.declare_parameter('plan_ema_alpha', 0.30)

        # ---- Odometria ----
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.19)

        # ---- Velocidades / pure pursuit ----
        self.declare_parameter('follow_v', 0.05)         # m/s constante en FOLLOW
        self.declare_parameter('lookahead_dist', 0.10)
        self.declare_parameter('kp_w', 1.5)
        self.declare_parameter('w_max', 0.19)
        self.declare_parameter('v_max', 0.10)
        # Fraccion de la trayectoria a ejecutar (resto lo hace DOCK).
        # 0.7 = ejecuta solo 70% del path, ultimo 30% lo cierra DOCK con QR.
        self.declare_parameter('follow_fraction', 0.50)

        # ---- DOCK ----
        self.declare_parameter('dock_max_linear', 0.04)
        self.declare_parameter('kp_v_dock', 0.5)
        self.declare_parameter('kp_bearing_dock', 0.6)

        # ---- Tolerancias ----
        self.declare_parameter('follow_done_dist', 0.04)  # m, dist (en plan-frame) al goal
        self.declare_parameter('dock_tol_m', 0.015)
        self.declare_parameter('dock_bearing_tol_rad', 0.040)
        # Ticks consecutivos centrados para confirmar DONE (estabilidad)
        self.declare_parameter('dock_done_stable_ticks', 5)

        # ---- Vision / safety ----
        self.declare_parameter('detection_max_age_s', 3.0)
        self.declare_parameter('control_freq', 10.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('show_window', True)
        self.declare_parameter('frame_id', 'camera_link')

        # ---- Leer ----
        self._image_topic   = str(self.get_parameter('image_topic').value)
        qos_name            = str(self.get_parameter('qos').value).lower()
        cam_path            = str(self.get_parameter('camera_params').value)
        self._marker_length = float(self.get_parameter('marker_length').value)
        backend             = str(self.get_parameter('backend').value)

        self._D_dock        = float(self.get_parameter('dock_distance').value)
        self._cam_off_x     = float(self.get_parameter('cam_offset_x').value)
        self._cam_off_y     = float(self.get_parameter('cam_offset_y').value)
        self._path_y_scale  = float(self.get_parameter('path_lateral_scale').value)
        self._path_prog_gain = float(self.get_parameter('path_progressive_gain').value)
        self._path_prog_ref  = float(self.get_parameter('path_progressive_ref').value)
        self._center_thr    = float(self.get_parameter('center_threshold_rad').value)
        self._plan_ema_a    = float(self.get_parameter('plan_ema_alpha').value)

        self._wheel_radius     = float(self.get_parameter('wheel_radius').value)
        self._wheel_separation = float(self.get_parameter('wheel_separation').value)

        self._follow_v      = float(self.get_parameter('follow_v').value)
        self._d_lookahead   = float(self.get_parameter('lookahead_dist').value)
        self._kp_w          = float(self.get_parameter('kp_w').value)
        self._w_max         = float(self.get_parameter('w_max').value)
        self._v_max         = float(self.get_parameter('v_max').value)
        self._follow_frac   = float(self.get_parameter('follow_fraction').value)

        self._v_dock_max    = float(self.get_parameter('dock_max_linear').value)
        self._kp_v_dock     = float(self.get_parameter('kp_v_dock').value)
        self._kp_bearing_dock = float(self.get_parameter('kp_bearing_dock').value)

        self._tol_follow    = float(self.get_parameter('follow_done_dist').value)
        self._tol_dock      = float(self.get_parameter('dock_tol_m').value)
        self._tol_dock_bear = float(self.get_parameter('dock_bearing_tol_rad').value)
        self._dock_stable_n = int(self.get_parameter('dock_done_stable_ticks').value)
        self._dock_stable_count: int = 0

        self._max_det_age   = float(self.get_parameter('detection_max_age_s').value)
        self._control_dt    = 1.0 / max(1e-3, float(self.get_parameter('control_freq').value))
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

        # ---- Odometria ----
        self._odom = OdometryTracker(
            self,
            wheel_radius=self._wheel_radius,
            wheel_separation=self._wheel_separation,
        )

        # ---- Estado interno ----
        self._state: State = State.IDLE if not self._auto_start else State.SEARCH
        self._geom_latest: Optional[dict] = None
        self._last_det_time: Optional[float] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_dets: List[dict] = []
        self._path: Optional[QuadraticPath] = None
        self._plan_goal: Optional[tuple[float, float]] = None
        # Trayectoria open-loop precomputada en PLAN
        self._trajectory: list = []
        self._traj_index: int = 0

        if self._show_window:
            cv2.namedWindow('qr_quad_alignment', cv2.WINDOW_NORMAL)

        self.create_timer(self._control_dt, self._control_tick)

        self.get_logger().info(
            f'qr_quad_alignment listo  dock={self._D_dock*1000:.0f}mm  '
            f'follow_v={self._follow_v}  lookahead={self._d_lookahead*1000:.0f}mm  '
            f'dry_run={self._dry_run}'
        )

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            # Fallback: passthrough y convertir manualmente segun encoding
            try:
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                enc = (msg.encoding or '').lower()
                if enc in ('rgb8', 'rgba8'):
                    frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
                elif enc in ('mono8', '8uc1'):
                    frame = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
                elif enc.startswith('yuv'):
                    frame = cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUYV)
                elif enc in ('bgra8',):
                    frame = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
                else:
                    frame = raw if raw.ndim == 3 else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            except Exception as exc2:  # noqa: BLE001
                self.get_logger().warn(
                    f'cv_bridge: encoding={msg.encoding!r} {exc2}',
                    throttle_duration_sec=2.0,
                )
                return
        if not hasattr(self, '_first_frame_logged'):
            self.get_logger().info(
                f'Primer frame {frame.shape[1]}x{frame.shape[0]} encoding={msg.encoding!r}'
            )
            self._first_frame_logged = True
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
                # EMA sobre geometria para evitar que el signo de qr_y
                # cambie de un frame a otro cuando el QR esta cerca del
                # centro (ruido del PnP).
                a = self._plan_ema_a
                if self._geom_latest is None:
                    self._geom_latest = new_geom
                else:
                    blended: dict = {}
                    for k in ('qr_x', 'qr_y', 'n_x_r', 'n_y_r',
                              'gx', 'gy', 'dist_qr', 'dist_goal'):
                        blended[k] = (1 - a) * self._geom_latest[k] + a * new_geom[k]
                    for k in ('bearing_qr', 'g_th'):
                        s = (1 - a) * math.sin(self._geom_latest[k]) + a * math.sin(new_geom[k])
                        c = (1 - a) * math.cos(self._geom_latest[k]) + a * math.cos(new_geom[k])
                        blended[k] = math.atan2(s, c)
                    self._geom_latest = blended
                self._last_det_time = time.monotonic()
                g = self._geom_latest
                p = Point(); p.x = g['gx']; p.y = g['gy']; p.z = 0.0
                self._pub_goal.publish(p)

    def _start_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._is_detection_fresh() and self._geom_latest is not None:
                self._set_state(State.PLAN, reason='start request')
            else:
                self._set_state(State.SEARCH, reason='start, sin det')
        else:
            self._set_state(State.IDLE, reason='stop request')

    def _is_detection_fresh(self) -> bool:
        if self._last_det_time is None:
            return False
        return (time.monotonic() - self._last_det_time) <= self._max_det_age

    # ------------------------------------------------------------------
    def _control_tick(self) -> None:
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
        if self._state == State.PLAN:
            self._run_plan()
            self._render_ui()
            return
        if self._state == State.FOLLOW:
            self._run_follow()
            self._render_ui()
            return
        if self._state == State.DOCK:
            self._run_dock()
            self._render_ui()
            return

    def _run_search(self) -> None:
        if self._is_detection_fresh() and self._geom_latest is not None:
            self._set_state(State.PLAN, reason='QR encontrado')
            return
        self._publish_cmd(0.0, 0.15)

    def _run_plan(self) -> None:
        if self._geom_latest is None:
            self.get_logger().warn('PLAN sin geom')
            self._set_state(State.LOST, reason='no geom en PLAN')
            return
        g = self._geom_latest
        bearing = g['bearing_qr']
        gx, gy = g['gx'], g['gy']

        # ATAJO: si el QR ya esta CENTRADO en la camara, NO necesita curva.
        # Saltar PLAN/FOLLOW y entrar directo a DOCK (que solo avanza recto
        # con correccion fina de bearing).
        if abs(bearing) < self._center_thr:
            self.get_logger().info(
                f'QR ya centrado (bearing={math.degrees(bearing):+.1f} < '
                f'{math.degrees(self._center_thr):.1f} deg) -> DOCK directo'
            )
            # Reset odom de todas formas para limpiar estado
            self._odom.reset()
            self._path = None
            self._plan_goal = None
            self._set_state(State.DOCK, reason='ya centrado, skip curva')
            return

        if gx <= 0.05:
            self.get_logger().warn(
                f'goal demasiado cerca/atras (gx={gx*1000:.0f}mm), abortar'
            )
            self._set_state(State.IDLE, reason='goal no valido')
            return

        # Decidir direccion del giro inicial basado en signo de qr_y
        # (estable porque viene de EMA filtered geom)
        side = '+y (izquierda)' if gy >= 0 else '-y (derecha)'

        # Escala progresiva: curva mas drastica cuando QR mas lejos del centro
        progressive = self._path_prog_gain * abs(gy) / max(1e-6, self._path_prog_ref)
        eff_scale = self._path_y_scale + progressive
        gy_plan = gy * eff_scale
        try:
            self._path = plan_quadratic_in_robot_frame((gx, gy_plan))
            self._plan_goal = (gx, gy)
        except ValueError as exc:
            self.get_logger().warn(f'no se pudo planear: {exc}')
            self._set_state(State.IDLE, reason='plan failed')
            return

        # Precomputar trayectoria OPEN-LOOP y truncarla. El ultimo
        # follow_fraction lo cierra DOCK con feedback de QR.
        full_traj = compute_quadratic_trajectory(
            self._path, self._follow_v, self._control_dt,
        )
        n_keep = max(1, int(len(full_traj) * self._follow_frac))
        self._trajectory = full_traj[:n_keep]
        self._traj_index = 0
        self._odom.reset()
        self.get_logger().info(
            f'PLAN: lado={side} eff_scale={eff_scale:.2f} a={self._path.a:.4f} '
            f'goal=({gx*1000:.0f},{gy*1000:.0f})mm '
            f'traj {len(self._trajectory)}/{len(full_traj)} ticks '
            f'({self._follow_frac*100:.0f}%) ~{len(self._trajectory)*self._control_dt:.1f}s'
        )
        self._set_state(State.FOLLOW, reason='plan + trayectoria lista')

    def _run_follow(self) -> None:
        """OPEN-LOOP: ejecuta la trayectoria pre-computada tick por tick.

        Sin feedback durante FOLLOW para evitar oscilaciones por ruido
        del PnP. Cualquier deriva acumulada se corrige en DOCK.
        """
        if not self._trajectory:
            self._set_state(State.LOST, reason='trayectoria vacia')
            return
        if self._traj_index >= len(self._trajectory):
            self._set_state(State.DOCK, reason='trayectoria completa')
            return
        v, w = self._trajectory[self._traj_index]
        self._traj_index += 1
        self._publish_cmd(v, w)

    def _run_dock(self) -> None:
        """DOCK en 3 fases:
          (a) SIN DETECCION  -> gira buscando QR.
          (b) DENTRO de tol_dock_m en distancia -> CENTERING:
              robot detenido, solo gira hasta tener bearing en tol.
              DONE cuando bearing estable N ticks.
          (c) FUERA de tol_dock en distancia -> APPROACH:
              - Si bearing muy off (>2*tol): solo gira.
              - Si bearing OK: avanza + correccion leve.
        """
        # (a) sin deteccion -> solo gira
        if not (self._is_detection_fresh() and self._geom_latest is not None):
            if not hasattr(self, '_last_search_dir'):
                self._last_search_dir = +1.0
            self._dock_stable_count = 0
            self._publish_cmd(0.0, 0.12 * self._last_search_dir)
            return

        g = self._geom_latest
        err_d = g['dist_qr'] - self._D_dock
        err_b = g['bearing_qr']
        self._last_search_dir = +1.0 if err_b >= 0 else -1.0

        # ---- (b) Ya estamos a la distancia objetivo -> centrar fino ----
        if abs(err_d) < self._tol_dock:
            if abs(err_b) < self._tol_dock_bear:
                self._dock_stable_count += 1
                if self._dock_stable_count >= self._dock_stable_n:
                    self._set_state(
                        State.DONE,
                        reason=f'centrado x{self._dock_stable_count} ticks: '
                               f'd_err={err_d*1000:+.1f}mm bear={math.degrees(err_b):+.2f}'
                    )
                    return
                # Estable, espera mas ticks
                self._publish_cmd(0.0, 0.0)
                return
            # Aun no centrado: solo gira
            self._dock_stable_count = 0
            w = _with_floor(self._kp_bearing_dock * err_b,
                            self._W_DEADBAND + 0.005, self._w_max)
            self._publish_cmd(0.0, w)
            return

        # ---- (c) APPROACH: aun lejos del dock distance ----
        self._dock_stable_count = 0

        # Si bearing muy fuera (>2x tol) -> solo gira para alinearse antes de avanzar
        if abs(err_b) > 2.0 * self._tol_dock_bear:
            w = _with_floor(self._kp_bearing_dock * err_b,
                            self._W_DEADBAND + 0.005, self._w_max)
            self._publish_cmd(0.0, w)
            return

        # OK avanzar: v con piso, w leve correccion
        v = _with_floor(self._kp_v_dock * err_d,
                        self._V_DEADBAND + 0.005, self._v_dock_max)
        w = _clip(0.5 * self._kp_bearing_dock * err_b, -self._w_max, self._w_max)
        self._publish_cmd(v, w)

    # ------------------------------------------------------------------
    # Hard safety caps
    _HARD_V_MAX = 0.10
    _HARD_W_MAX = 0.19
    _V_DEADBAND = 0.015
    _W_DEADBAND = 0.03
    _CMD_LPF_ALPHA = 0.40

    def _publish_cmd(self, v: float, w: float) -> None:
        if self._dry_run:
            return
        v_safe = max(-self._HARD_V_MAX, min(self._HARD_V_MAX, float(v)))
        w_safe = max(-self._HARD_W_MAX, min(self._HARD_W_MAX, float(w)))
        if not hasattr(self, '_cmd_w_prev'):
            self._cmd_w_prev = 0.0
        # LPF SOLO en w (suaviza oscilaciones), v pasa directo
        a = self._CMD_LPF_ALPHA
        w_smooth = a * w_safe + (1.0 - a) * self._cmd_w_prev
        self._cmd_w_prev = w_smooth
        # Deadbands
        v_out = 0.0 if abs(v_safe) < self._V_DEADBAND else v_safe
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
        self._cmd_w_prev = 0.0
        zero = Twist()
        for _ in range(3):
            self._pub_cmd.publish(zero)

    def _set_state(self, new: State, reason: str = '') -> None:
        if new == self._state:
            return
        x_r, y_r, th_r = self._odom.pose()
        gs = (f' [odom=({x_r*1000:.0f},{y_r*1000:.0f},{math.degrees(th_r):+.0f})')
        if self._geom_latest is not None:
            g = self._geom_latest
            gs += (f' qr=({g["qr_x"]*1000:.0f},{g["qr_y"]*1000:.0f})'
                   f' goal=({g["gx"]*1000:.0f},{g["gy"]*1000:.0f})]')
        else:
            gs += ' no-qr]'
        self.get_logger().info(f'{self._state.value} -> {new.value} ({reason}){gs}')
        self._state = new
        if new == State.PLAN:
            pass  # _run_plan se encarga
        self._publish_zero()

    # ------------------------------------------------------------------
    def _project_robot_xy_to_image(self, x_r: float, y_r: float,
                                    z_world: float = 0.0) -> Optional[tuple[int, int]]:
        """Proyecta un punto en frame robot ACTUAL a pixeles."""
        Z_cam = x_r - self._cam_off_x
        X_cam = self._cam_off_y - y_r
        Y_cam = z_world
        if Z_cam <= 0.01:
            return None
        try:
            pts = np.array([[X_cam, Y_cam, Z_cam]], dtype=np.float64).reshape(1, 1, 3)
            proj, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), self._K, self._dist)
            return int(proj[0, 0, 0]), int(proj[0, 0, 1])
        except cv2.error:
            return None

    def _render_ui(self) -> None:
        if not self._show_window or self._latest_frame is None:
            return
        out = self._detector.draw_detections(self._latest_frame, self._latest_dets)

        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        cv2.line(out, (0, h // 2), (w, h // 2), (80, 80, 80), 1)

        # Dibujar la parabola RELATIVA A LA CAMARA (transformando samples
        # del plan-frame al frame robot actual via odometria).
        if self._path is not None:
            x_r, y_r, th_r = self._odom.pose()
            ct, st = math.cos(-th_r), math.sin(-th_r)
            pts_img = []
            for sample in self._path.samples():
                # punto en plan-frame -> en robot frame actual:
                # restar pose robot, luego rotar por -th
                dx, dy = sample[0] - x_r, sample[1] - y_r
                xr = ct * dx - st * dy
                yr = st * dx + ct * dy
                pix = self._project_robot_xy_to_image(xr, yr)
                if pix is not None:
                    pts_img.append(pix)
            if len(pts_img) >= 2:
                cv2.polylines(out, [np.array(pts_img, dtype=np.int32)],
                              False, (0, 200, 255), 2)

        state_color = {
            State.IDLE:    (200, 200, 200),
            State.SEARCH:  (255, 200,   0),
            State.PLAN:    (255, 128,   0),
            State.FOLLOW:  (  0, 200, 255),
            State.DOCK:    (  0, 200,   0),
            State.DONE:    (  0, 255,   0),
            State.LOST:    (  0,   0, 255),
        }[self._state]
        cv2.rectangle(out, (0, 0), (w, 22), state_color, -1)
        label = self._state.value + ('  [DRY]' if self._dry_run else '')
        cv2.putText(out, label, (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

        x_r, y_r, th_r = self._odom.pose()
        txts = [
            f"odom x={x_r*1000:+.0f} y={y_r*1000:+.0f} th={math.degrees(th_r):+.1f}",
        ]
        if self._path is not None:
            txts.append(f"path a={self._path.a:.4f} gx={self._path.gx*1000:.0f} gy={self._path.gy*1000:.0f}")
        if self._geom_latest is not None:
            g = self._geom_latest
            fresh = self._is_detection_fresh()
            txts.append(f"QR x={g['qr_x']*1000:+.0f} y={g['qr_y']*1000:+.0f} d={g['dist_qr']*1000:.0f}mm fresh={fresh}")
        for i, t in enumerate(txts):
            cv2.putText(out, t, (6, 40 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('qr_quad_alignment', out)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self._publish_zero()
            rclpy.shutdown()
        elif key == ord(' '):
            if self._state == State.IDLE:
                if self._is_detection_fresh() and self._geom_latest is not None:
                    self._set_state(State.PLAN, reason='SPACE')
                else:
                    self._set_state(State.SEARCH, reason='SPACE')
            else:
                self._set_state(State.IDLE, reason='SPACE')


def main() -> None:
    rclpy.init()
    node = QRQuadAlignmentNode()
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
