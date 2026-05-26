#!/usr/bin/env python3
"""Alineamiento horizontal a un palet marcado con QR para diff-drive.

Estrategia "primero alinear, luego acercarse":

    IDLE       -- detenido, esperando /alignment_start
    SEARCH     -- gira en sitio buscando QR
    CENTER_QR  -- en sitio: gira hasta que el QR queda centrado en camara
    ALIGN      -- mantiene la distancia al QR (no se acerca!) mientras
                  corrige face_angle y bearing_qr para quedar perpendicular
                  al QR. Para evitar acercarse, si dist_qr cae por debajo
                  de D_dock + safety_margin, se fuerza v=0 (solo gira).
    DOCK       -- avance recto al QR hasta llegar a D_dock, con pequena
                  correccion de bearing.
    DONE       -- pose alcanzada
    LOST       -- detection no fresca

Filtro EMA sobre la geometria del QR para reducir el jitter del PnP que
con un QR de 30mm a 320x240 satura el control.
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


class State(str, Enum):
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    CENTER_QR = 'CENTER_QR'
    ALIGN = 'ALIGN'
    RE_CENTER = 'RE_CENTER'   # gira a bearing=0 despues de ALIGN, antes de DOCK
    DOCK = 'DOCK'
    DONE = 'DONE'
    LOST = 'LOST'


class AlignPhase(str, Enum):
    """Maniobra 5-pasos (paralelo-parking-like) dentro de ALIGN.

    Para QR a la izquierda (primary_side=+1):
        P1: girar +15 deg (hacia QR)
        P2: avanzar
        P3: girar -30 deg (opuesto, overshoot)
        P4: retroceder lo avanzado
        P5: girar +15 deg (vuelve al heading original)

    Net: robot se mueve lateralmente ~0.5d hacia el lado de QR.
    Para QR a la derecha (primary_side=-1) los giros van invertidos.
    """
    P1_TURN_TOWARD = 'P1_TURN_TOWARD'
    P2_FORWARD     = 'P2_FORWARD'
    P3_TURN_OVER   = 'P3_TURN_OVER'
    P4_BACKWARD    = 'P4_BACKWARD'
    P5_TURN_HOME   = 'P5_TURN_HOME'
    DONE_SCRIPT    = 'DONE_SCRIPT'


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
    corners: np.ndarray,
    cam_offset_x: float,
    cam_offset_y: float,
    marker_length: float = 0.030,
) -> Optional[dict]:
    """Pose del QR y senales utiles en frame robot (REP-103).

    Args:
        rvec, tvec: salida de solvePnP.
        corners: np.ndarray (4,2) con esquinas del QR en pixeles en orden
                 TL, TR, BR, BL (lo que devuelve cv2.QRCodeDetector).
        cam_offset_x, cam_offset_y: offset camara->centro_eje_ruedas en m.

    Returns:
        dict con:
            qr_x, qr_y    : posicion del QR en frame robot (m)
            bearing_qr    : angulo robot->QR (rad)
            dist_qr       : distancia plana al QR (m)
            face_angle    : angulo PnP-based (ruidoso con QR pequenos)
            face_skew     : senal alterna basada en proporciones de
                            esquinas en pixeles. Mucho mas robusto que
                            face_angle para QR pequenos a baja
                            resolucion. Rango [-1, 1]:
                            skew = 0   -> perpendicular
                            skew > 0   -> camara a la izquierda de la normal
                                          (lado derecho del QR se ve mas grande)
                            skew < 0   -> camara a la derecha de la normal
            n_x_r, n_y_r  : normal del marker en frame robot (unit)
    """
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

    bearing_qr = math.atan2(qr_y, qr_x)
    dist_qr = math.hypot(qr_x, qr_y)
    face_angle = math.atan2(-nx_cam, -nz_cam)

    # lateral_offset: cuanto se aleja el robot del eje perpendicular del QR.
    # 0 cuando el robot esta sobre el eje. Signo indica de que lado.
    # Vector QR->robot = (-qr_x, -qr_y). Normal_horiz = (n_x_r, n_y_r).
    # Componente perpendicular a la normal: cross product 2D.
    lateral_offset = qr_y * n_x_r - qr_x * n_y_r

    # face_skew: relacion entre lado derecho y lado izquierdo del cuadrado
    # proyectado. Robusto a sub-pixel y no depende del rvec del PnP.
    corners_arr = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    TL, TR, BR, BL = corners_arr[0], corners_arr[1], corners_arr[2], corners_arr[3]
    left_edge = float(np.linalg.norm(BL - TL))
    right_edge = float(np.linalg.norm(BR - TR))
    denom = right_edge + left_edge
    face_skew = (right_edge - left_edge) / denom if denom > 1e-6 else 0.0

    # face_est: angulo estimado desde el skew, en radianes, independiente de
    # la distancia. Modelo perspectiva pinhole: skew = L*sin(phi)/(2*Z).
    # Entonces sin(phi) = 2*Z*skew/L.  Z ~= max(dist_qr, marker_length).
    # Saturamos antes de asin para evitar dominio invalido si hay outlier.
    Z = max(dist_qr, marker_length * 0.5)
    sin_phi = (2.0 * Z * face_skew) / max(marker_length, 1e-6)
    sin_phi = max(-0.999, min(0.999, sin_phi))
    face_est = math.asin(sin_phi)

    return {
        'qr_x': qr_x, 'qr_y': qr_y,
        'bearing_qr': bearing_qr,
        'dist_qr': dist_qr,
        'face_angle': face_angle,
        'face_skew': face_skew,
        'face_est': face_est,
        'lateral_offset': lateral_offset,
        'n_x_r': n_x_r, 'n_y_r': n_y_r,
    }


def _ema_blend(prev: dict, new: dict, a: float) -> dict:
    """EMA lineal sobre escalares; angulos se promedian via sin/cos."""
    out: dict = {}
    for k in ('qr_x', 'qr_y', 'dist_qr', 'n_x_r', 'n_y_r', 'face_skew',
              'lateral_offset'):
        out[k] = (1 - a) * prev[k] + a * new[k]
    for k in ('bearing_qr', 'face_angle', 'face_est'):
        s = (1 - a) * math.sin(prev[k]) + a * math.sin(new[k])
        c = (1 - a) * math.cos(prev[k]) + a * math.cos(new[k])
        out[k] = math.atan2(s, c)
    return out


class QRAlignmentNode(Node):
    def __init__(self) -> None:
        super().__init__('qr_alignment')

        # ---- I/O ----
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('qos', 'sensor_data')
        self.declare_parameter(
            'camera_params', 'src/perception/config/camera_params.yaml'
        )
        self.declare_parameter('marker_length', 0.030)
        self.declare_parameter('backend', 'auto')

        # ---- Geometria ----
        self.declare_parameter('dock_distance', 0.200)     # m, camara->QR final
        self.declare_parameter('safety_margin', 0.05)      # m, no acercarse a <D_dock+margen mientras alineo
        self.declare_parameter('cam_offset_x', 0.100)
        self.declare_parameter('cam_offset_y', 0.0)

        # ---- Velocidades ----
        self.declare_parameter('max_linear', 0.10)
        self.declare_parameter('max_angular', 0.19)
        self.declare_parameter('align_linear', 0.04)       # m/s pequeno durante ALIGN
        self.declare_parameter('dock_max_linear', 0.05)
        self.declare_parameter('search_angular', 0.15)

        # ---- Ganancias ----
        self.declare_parameter('kp_center', 1.2)           # CENTER_QR
        self.declare_parameter('kp_skew', 1.5)             # ALIGN: correccion de face_skew (signal robusto)
        self.declare_parameter('kp_bearing_align', 0.6)    # ALIGN: re-centrado QR
        self.declare_parameter('kp_bearing_dock', 1.0)     # DOCK: correccion de heading
        self.declare_parameter('kp_v_dock', 0.5)           # DOCK: avance prop a distancia

        # ---- Tolerancias / scripted ALIGN ----
        self.declare_parameter('center_tol_rad', 0.06)            # ~3.4 deg
        self.declare_parameter('align_bearing_tol_rad', 0.04)     # ~2.3 deg, para R_TURN_BACK / L_TURN_BACK
        self.declare_parameter('dock_tol_m', 0.015)
        # Magnitud de cada paso lateral
        self.declare_parameter('sidestep_angle_rad', 0.26)        # ~15 deg
        self.declare_parameter('sidestep_forward_ticks', 12)      # 12*0.1s = 1.2s @ v_align
        self.declare_parameter('sidestep_backward_ticks', 0)      # 0 = usar mismo que forward
        self.declare_parameter('sidestep_w', 0.12)                # rad/s (suave)
        # Timeout para fases TURN: si no alcanzo el bearing target en N
        # ticks, proceder de todas formas para no colgar la maniobra.
        self.declare_parameter('turn_phase_max_ticks', 60)        # 6s @ 10Hz
        # Convergencia iterativa del ALIGN: cuanto bearing y lateral_offset
        # toleramos al final de cada ciclo R+L. Si excede, repetir.
        self.declare_parameter('align_max_cycles', 3)
        self.declare_parameter('align_converged_bearing_rad', 0.06)  # ~3.4 deg
        self.declare_parameter('align_converged_lat_offset_m', 0.03) # 3 cm

        # ---- Filtro EMA ----
        self.declare_parameter('ema_alpha', 0.35)          # 0 = no actualiza, 1 = sin filtro

        # ---- Safety / debug ----
        self.declare_parameter('detection_max_age_s', 0.30)
        self.declare_parameter('control_freq', 10.0)
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
        self._safety_margin = float(self.get_parameter('safety_margin').value)
        self._cam_off_x     = float(self.get_parameter('cam_offset_x').value)
        self._cam_off_y     = float(self.get_parameter('cam_offset_y').value)

        self._v_max         = float(self.get_parameter('max_linear').value)
        self._w_max         = float(self.get_parameter('max_angular').value)
        self._v_align       = float(self.get_parameter('align_linear').value)
        self._v_dock_max    = float(self.get_parameter('dock_max_linear').value)
        self._w_search      = float(self.get_parameter('search_angular').value)

        self._kp_center        = float(self.get_parameter('kp_center').value)
        self._kp_skew          = float(self.get_parameter('kp_skew').value)
        self._kp_bearing_align = float(self.get_parameter('kp_bearing_align').value)
        self._kp_bearing_dock  = float(self.get_parameter('kp_bearing_dock').value)
        self._kp_v_dock        = float(self.get_parameter('kp_v_dock').value)

        self._tol_center            = float(self.get_parameter('center_tol_rad').value)
        self._tol_bearing           = float(self.get_parameter('align_bearing_tol_rad').value)
        self._tol_dock              = float(self.get_parameter('dock_tol_m').value)
        self._sidestep_angle        = float(self.get_parameter('sidestep_angle_rad').value)
        self._sidestep_fwd_ticks    = int(self.get_parameter('sidestep_forward_ticks').value)
        bwd_ticks_param             = int(self.get_parameter('sidestep_backward_ticks').value)
        self._sidestep_bwd_ticks    = bwd_ticks_param if bwd_ticks_param > 0 else self._sidestep_fwd_ticks
        self._sidestep_w            = float(self.get_parameter('sidestep_w').value)
        self._turn_max_ticks        = int(self.get_parameter('turn_phase_max_ticks').value)
        self._align_max_cycles      = int(self.get_parameter('align_max_cycles').value)
        self._align_conv_bearing    = float(self.get_parameter('align_converged_bearing_rad').value)
        self._align_conv_lat        = float(self.get_parameter('align_converged_lat_offset_m').value)

        self._ema_alpha   = float(self.get_parameter('ema_alpha').value)

        self._max_det_age_s = float(self.get_parameter('detection_max_age_s').value)
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

        # ---- Estado interno ----
        self._state: State = State.IDLE if not self._auto_start else State.SEARCH
        self._geom_filt: Optional[dict] = None
        self._last_rvec_tvec: Optional[tuple[np.ndarray, np.ndarray]] = None
        self._last_det_time: Optional[float] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_dets: List[dict] = []
        # Sub-SM dentro de ALIGN
        self._align_phase: AlignPhase = AlignPhase.P1_TURN_TOWARD
        self._align_phase_ticks: int = 0
        self._align_cycle: int = 0
        # primary_side: +1 si QR a la izquierda (qr_y>0), -1 si a la derecha.
        self._align_primary_side: int = +1
        # Bearing inicial al comienzo del ciclo (usado para targets relativos).
        self._align_b0: float = 0.0
        # Bearing al inicio del ALIGN completo (para medir mejora total).
        self._align_initial_bearing: float = 0.0

        if self._show_window:
            cv2.namedWindow('qr_alignment', cv2.WINDOW_NORMAL)

        self.create_timer(self._control_dt, self._control_tick)

        self.get_logger().info(
            f'qr_alignment listo  dock={self._D_dock*1000:.0f}mm  '
            f'margin={self._safety_margin*1000:.0f}mm  ema_a={self._ema_alpha}  '
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
                d['rvec'], d['tvec'], d['corners'],
                self._cam_off_x, self._cam_off_y,
                marker_length=self._marker_length,
            )
            if new_geom is not None:
                if self._geom_filt is None:
                    self._geom_filt = new_geom
                else:
                    self._geom_filt = _ema_blend(self._geom_filt, new_geom, self._ema_alpha)
                self._last_rvec_tvec = (d['rvec'], d['tvec'])
                self._last_det_time = time.monotonic()

    def _start_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._is_detection_fresh() and self._geom_filt is not None:
                # Saltar CENTER_QR: ir directo a ALIGN, la maniobra centra
                self._set_state(State.ALIGN, reason='start request')
            else:
                self._set_state(State.SEARCH, reason='start request, sin det fresca')
        else:
            self._set_state(State.IDLE, reason='stop request')

    def _is_detection_fresh(self) -> bool:
        if self._last_det_time is None:
            return False
        return (time.monotonic() - self._last_det_time) <= self._max_det_age_s

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

        if not self._is_detection_fresh() or self._geom_filt is None:
            self._publish_zero()
            self._set_state(State.LOST, reason='deteccion no fresca')
            self._render_ui()
            return

        geom = self._geom_filt
        if self._state == State.CENTER_QR:
            self._run_center_qr(geom)
        elif self._state == State.ALIGN:
            self._run_align(geom)
        elif self._state == State.RE_CENTER:
            self._run_re_center(geom)
        elif self._state == State.DOCK:
            self._run_dock(geom)
        self._render_ui()

    def _run_search(self) -> None:
        if self._is_detection_fresh() and self._geom_filt is not None:
            # Saltar CENTER_QR: la maniobra ALIGN se encarga de centrar
            self._set_state(State.ALIGN, reason='QR encontrado')
            return
        self._publish_cmd(0.0, self._w_search)

    def _run_center_qr(self, geom: dict) -> None:
        err = geom['bearing_qr']
        if abs(err) < self._tol_center:
            self._set_state(State.ALIGN, reason='QR centrado')
            self._publish_zero()
            return
        w = _clip(self._kp_center * err, -self._w_max, self._w_max)
        self._publish_cmd(0.0, w)

    def _run_align(self, geom: dict) -> None:
        """Maniobra scripted: paso derecho + paso izquierdo.

        Sub-SM:
            R_TURN_OUT  : girar derecha hasta bearing = +sidestep_angle
            R_FORWARD   : avanzar N ticks open-loop
            R_TURN_BACK : girar izquierda hasta bearing = 0
            L_TURN_OUT  : girar izquierda hasta bearing = -sidestep_angle
            L_FORWARD   : avanzar N ticks open-loop
            L_TURN_BACK : girar derecha hasta bearing = 0
            DONE_SCRIPT : -> DOCK
        """
        bearing = geom['bearing_qr']
        phase = self._align_phase
        target = self._sidestep_angle   # 15 deg por default
        tol = self._tol_bearing
        side = self._align_primary_side  # +1 = QR a izq, -1 = QR a der
        b0 = self._align_b0              # bearing al inicio del ciclo

        # Logica (para QR a la izquierda, side=+1):
        #   P1: girar IZQ +sidestep_angle.
        #       heading rota +sidestep -> bearing baja a (b0 - sidestep)
        #   P2: avanzar fwd_ticks
        #   P3: girar DER -2*sidestep.
        #       heading rota -2*sidestep -> bearing sube a (b0 - sidestep + 2*sidestep) = b0 + sidestep
        #   P4: retroceder fwd_ticks
        #   P5: girar IZQ +sidestep -> bearing vuelve a b0
        # Para QR a la derecha (side=-1) todos los signos se invierten.

        # Tolerancia mas relajada para targets de bearing (compensar motor lento)
        bear_tol = max(tol, math.radians(5.0))

        # ---- P1: girar hacia QR ----
        if phase == AlignPhase.P1_TURN_TOWARD:
            target_bearing = b0 - side * target
            reached = (bearing <= target_bearing + bear_tol) if side > 0 else (bearing >= target_bearing - bear_tol)
            timeout = self._align_phase_ticks >= self._turn_max_ticks
            if reached or timeout:
                reason = (f'bearing={math.degrees(bearing):+.1f} P1 done'
                          if reached else
                          f'P1 timeout ({self._align_phase_ticks} ticks)')
                self._advance_align_phase(AlignPhase.P2_FORWARD, reason=reason)
                self._publish_zero()
                return
            self._align_phase_ticks += 1
            self._publish_cmd(0.0, side * self._sidestep_w)
            return

        # ---- P2: avanzar ----
        if phase == AlignPhase.P2_FORWARD:
            if self._align_phase_ticks >= self._sidestep_fwd_ticks:
                self._advance_align_phase(AlignPhase.P3_TURN_OVER,
                                          reason=f'P2 forward {self._sidestep_fwd_ticks} ticks done')
                self._publish_zero()
                return
            self._align_phase_ticks += 1
            # Mantener bearing en (b0 - side*target) durante el avance
            target_bearing = b0 - side * target
            bearing_err = bearing - target_bearing
            w_corr = _clip(-0.6 * bearing_err, -self._sidestep_w * 0.5, self._sidestep_w * 0.5)
            self._publish_cmd(self._v_align, w_corr)
            return

        # ---- P3: girar al lado opuesto (2*sidestep) ----
        if phase == AlignPhase.P3_TURN_OVER:
            target_bearing = b0 + side * target
            reached = (bearing >= target_bearing - bear_tol) if side > 0 else (bearing <= target_bearing + bear_tol)
            # P3 hace 2x el giro, mas tiempo de timeout
            timeout = self._align_phase_ticks >= 2 * self._turn_max_ticks
            if reached or timeout:
                reason = (f'bearing={math.degrees(bearing):+.1f} P3 done'
                          if reached else
                          f'P3 timeout ({self._align_phase_ticks} ticks)')
                self._advance_align_phase(AlignPhase.P4_BACKWARD, reason=reason)
                self._publish_zero()
                return
            self._align_phase_ticks += 1
            self._publish_cmd(0.0, -side * self._sidestep_w)
            return

        # ---- P4: retroceder ----
        if phase == AlignPhase.P4_BACKWARD:
            if self._align_phase_ticks >= self._sidestep_bwd_ticks:
                self._advance_align_phase(AlignPhase.P5_TURN_HOME,
                                          reason=f'P4 backward {self._sidestep_bwd_ticks} ticks done')
                self._publish_zero()
                return
            self._align_phase_ticks += 1
            target_bearing = b0 + side * target
            bearing_err = bearing - target_bearing
            w_corr = _clip(-0.6 * bearing_err, -self._sidestep_w * 0.5, self._sidestep_w * 0.5)
            self._publish_cmd(-self._v_align, w_corr)
            return

        # ---- P5: girar de vuelta al heading inicial ----
        if phase == AlignPhase.P5_TURN_HOME:
            target_bearing = b0
            reached = abs(bearing - target_bearing) < bear_tol
            timeout = self._align_phase_ticks >= self._turn_max_ticks
            if reached or timeout:
                reason = (f'bearing={math.degrees(bearing):+.1f} P5 done'
                          if reached else
                          f'P5 timeout ({self._align_phase_ticks} ticks)')
                self._advance_align_phase(AlignPhase.DONE_SCRIPT, reason=reason)
                self._publish_zero()
                return
            self._align_phase_ticks += 1
            self._publish_cmd(0.0, side * self._sidestep_w)
            return

        # ---- Fase final: chequear convergencia. Re-iterar si no convergio ----
        if phase == AlignPhase.DONE_SCRIPT:
            self._align_cycle += 1
            prev_b0 = b0
            improvement = abs(prev_b0) - abs(bearing)  # cuanto bajo |bearing|
            self.get_logger().info(
                f'ALIGN cycle {self._align_cycle}: '
                f'bear: {math.degrees(prev_b0):+.1f} -> {math.degrees(bearing):+.1f}deg '
                f'(mejora {math.degrees(improvement):+.1f})  '
                f'lat_off={geom["lateral_offset"]*1000:+.0f}mm'
            )
            # Convergencia: |bearing| pequeno o sin mejora significativa
            centered = abs(bearing) < self._align_conv_bearing
            no_progress = improvement < math.radians(0.5) and self._align_cycle >= 1
            if centered:
                self._set_state(State.RE_CENTER, reason='QR centrado en imagen')
                self._publish_zero()
                return
            if self._align_cycle >= self._align_max_cycles:
                self._set_state(State.RE_CENTER,
                                reason=f'max ciclos {self._align_max_cycles}')
                self._publish_zero()
                return
            if no_progress:
                self._set_state(State.RE_CENTER,
                                reason='sin mejora, salir de loop')
                self._publish_zero()
                return
            # Re-iterar: actualizar b0 al bearing actual, re-detectar side,
            # y reiniciar en P1
            self._align_b0 = bearing
            self._align_primary_side = +1 if geom['qr_y'] >= 0 else -1
            self._advance_align_phase(AlignPhase.P1_TURN_TOWARD,
                                      reason=f'ciclo {self._align_cycle}, repetir')
            self._publish_zero()
            return

    def _advance_align_phase(self, new_phase: AlignPhase, reason: str = '') -> None:
        self.get_logger().info(
            f'ALIGN phase {self._align_phase.value} -> {new_phase.value} ({reason})'
        )
        self._align_phase = new_phase
        self._align_phase_ticks = 0

    def _run_re_center(self, geom: dict) -> None:
        """Gira en sitio a bearing=0 despues del sidestep ALIGN.

        Esto orienta al robot perpendicular al QR antes del DOCK.
        """
        err = geom['bearing_qr']
        if abs(err) < self._tol_center:
            self._set_state(State.DOCK, reason='re-centrado, iniciar DOCK recto')
            self._publish_zero()
            return
        w = _clip(self._kp_center * err, -self._w_max, self._w_max)
        self._publish_cmd(0.0, w)

    def _run_dock(self, geom: dict) -> None:
        """DOCK con correccion gentil de bearing para mantener QR centrado.

        Avanza al QR mientras va corrigiendo bearing -> 0. La correccion
        angular es suave para no afectar la perpendicularidad lograda.
        """
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

    def _publish_cmd(self, v: float, w: float) -> None:
        if self._dry_run:
            return
        v_safe = max(-self._HARD_V_MAX, min(self._HARD_V_MAX, float(v)))
        w_safe = max(-self._HARD_W_MAX, min(self._HARD_W_MAX, float(w)))
        t = Twist()
        t.linear.x = v_safe
        t.angular.z = w_safe
        self._pub_cmd.publish(t)
        self.get_logger().info(
            f'cmd_vel v={v_safe:+.3f} w={w_safe:+.3f}  [{self._state.value}]',
            throttle_duration_sec=0.5,
        )

    def _publish_zero(self) -> None:
        if self._dry_run:
            return
        self._pub_cmd.publish(Twist())

    def _set_state(self, new: State, reason: str = '') -> None:
        if new == self._state:
            return
        # Log geometria al momento de la transicion para diagnostico
        g = self._geom_filt
        geom_str = ''
        if g is not None:
            geom_str = (
                f"  [bear={math.degrees(g['bearing_qr']):+.1f} "
                f"face_est={math.degrees(g['face_est']):+.1f} "
                f"skew={g['face_skew']:+.3f} "
                f"d={g['dist_qr']*1000:.0f}mm]"
            )
        self.get_logger().info(f'{self._state.value} -> {new.value} ({reason}){geom_str}')
        self._state = new
        # Resetear sub-fase de ALIGN al entrar
        if new == State.ALIGN:
            self._align_phase = AlignPhase.P1_TURN_TOWARD
            self._align_phase_ticks = 0
            self._align_cycle = 0
            if g is not None:
                self._align_primary_side = +1 if g['qr_y'] >= 0 else -1
                self._align_b0 = g['bearing_qr']
                self._align_initial_bearing = g['bearing_qr']
                self.get_logger().info(
                    f'ALIGN start  side={self._align_primary_side:+d} '
                    f'b0={math.degrees(g["bearing_qr"]):+.1f}deg '
                    f'qr_y={g["qr_y"]*1000:+.0f}mm'
                )
        self._publish_zero()

    # ------------------------------------------------------------------
    def _render_ui(self) -> None:
        if not self._show_window or self._latest_frame is None:
            return
        out = self._detector.draw_detections(self._latest_frame, self._latest_dets)

        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        cv2.line(out, (0, h // 2), (w, h // 2), (80, 80, 80), 1)

        state_color = {
            State.IDLE:      (200, 200, 200),
            State.SEARCH:    (255, 200,   0),
            State.CENTER_QR: (255, 128,   0),
            State.ALIGN:     (  0, 200, 255),
            State.RE_CENTER: (255,   0, 200),
            State.DOCK:      (  0, 200,   0),
            State.DONE:      (  0, 255,   0),
            State.LOST:      (  0,   0, 255),
        }[self._state]
        cv2.rectangle(out, (0, 0), (w, 22), state_color, -1)
        label = self._state.value
        if self._state == State.ALIGN:
            label += f' [{self._align_phase.value}]'
        if self._dry_run:
            label += '  [DRY]'
        cv2.putText(out, label, (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2, cv2.LINE_AA)

        g = self._geom_filt
        fresh = self._is_detection_fresh()
        age_ms = (time.monotonic() - self._last_det_time) * 1000.0 if self._last_det_time else 0.0
        if g is not None:
            txts = [
                f"QR  x={g['qr_x']*1000:+.0f}mm y={g['qr_y']*1000:+.0f}mm  d={g['dist_qr']*1000:.0f}mm",
                f"bearing={math.degrees(g['bearing_qr']):+.1f}  face_est={math.degrees(g['face_est']):+.1f}  lat_off={g['lateral_offset']*1000:+.0f}mm",
                f"sidestep ang={math.degrees(self._sidestep_angle):.0f}  fwd={self._sidestep_fwd_ticks}  cycle={self._align_cycle}/{self._align_max_cycles}",
                f"det_age={age_ms:.0f}ms  fresh={fresh}",
            ]
        else:
            txts = ['sin deteccion', f'det_age={age_ms:.0f}ms']
        for i, t in enumerate(txts):
            cv2.putText(out, t, (6, 40 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('qr_alignment', out)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('Cierre solicitado')
            self._publish_zero()
            rclpy.shutdown()
        elif key == ord(' '):
            if self._state == State.IDLE:
                if self._is_detection_fresh() and self._geom_filt is not None:
                    # Saltar CENTER_QR: ir directo a ALIGN
                    self._set_state(State.ALIGN, reason='SPACE')
                else:
                    self._set_state(State.SEARCH, reason='SPACE')
            else:
                self._set_state(State.IDLE, reason='SPACE')


def main() -> None:
    rclpy.init()
    node = QRAlignmentNode()
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
