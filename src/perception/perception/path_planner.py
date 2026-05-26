"""Cubic Bezier path planning + pure pursuit follower.

Todo se trabaja en el frame del robot (origen en robot, +x adelante, +y
izquierda, heading 0 = mirando +x). Eso simplifica la matematica porque
el robot siempre esta en (0, 0, 0) y la pose objetivo se expresa
relativa.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Cubic Bezier curve
# ---------------------------------------------------------------------------
class BezierCurve:
    """Cubic Bezier definida por 4 puntos de control P0..P3.

    B(t)  = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
    B'(t) = 3(1-t)^2 (P1-P0) + 6(1-t) t (P2-P1) + 3 t^2 (P3-P2)
    """

    def __init__(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
        n_samples: int = 60,
    ) -> None:
        self.p0 = np.asarray(p0, dtype=np.float64)
        self.p1 = np.asarray(p1, dtype=np.float64)
        self.p2 = np.asarray(p2, dtype=np.float64)
        self.p3 = np.asarray(p3, dtype=np.float64)
        self._n = int(n_samples)
        # Pre-sampleo para busqueda rapida de closest_point / lookahead
        ts = np.linspace(0.0, 1.0, self._n)
        self._samples = np.array([self._point(t) for t in ts])

    # ------------------------------------------------------------------
    def _point(self, t: float) -> np.ndarray:
        u = 1.0 - t
        return (u * u * u) * self.p0 + (3 * u * u * t) * self.p1 + \
               (3 * u * t * t) * self.p2 + (t * t * t) * self.p3

    def point_at(self, t: float) -> np.ndarray:
        t = max(0.0, min(1.0, t))
        return self._point(t)

    def tangent_at(self, t: float) -> np.ndarray:
        t = max(0.0, min(1.0, t))
        u = 1.0 - t
        return (3 * u * u) * (self.p1 - self.p0) + \
               (6 * u * t) * (self.p2 - self.p1) + \
               (3 * t * t) * (self.p3 - self.p2)

    # ------------------------------------------------------------------
    def closest_index(self, p: np.ndarray) -> int:
        """Indice del sample mas cercano al punto p."""
        p = np.asarray(p, dtype=np.float64)
        dists = np.linalg.norm(self._samples - p, axis=1)
        return int(np.argmin(dists))

    def lookahead_from(self, p: np.ndarray, d: float) -> Tuple[np.ndarray, int]:
        """Busca un punto sobre la curva, ADELANTE del closest, a distancia
        aproximada d desde p.

        Returns:
            (look_point, sample_index)
        """
        p = np.asarray(p, dtype=np.float64)
        i0 = self.closest_index(p)
        # Buscar hacia adelante (i creciente -> t creciente -> mas cerca de p3)
        for i in range(i0, self._n):
            sample = self._samples[i]
            if np.linalg.norm(sample - p) >= d:
                return sample, i
        # Si no encontramos uno a esa distancia, devolver el ultimo (goal)
        return self._samples[-1], self._n - 1

    def samples(self) -> np.ndarray:
        """Devuelve los samples pre-calculados (N, 2). Para visualizacion."""
        return self._samples


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------
def plan_bezier_in_robot_frame(
    goal_xy: Tuple[float, float],
    goal_heading: float,
    control_scale: float = 0.4,
    min_control_dist: float = 0.05,
) -> BezierCurve:
    """Crea una Bezier desde el robot (en origen, heading 0) hasta el goal.

    Args:
        goal_xy: posicion del goal en frame robot (m).
        goal_heading: heading deseado al llegar al goal (rad, en frame robot).
        control_scale: largo de los handles como fraccion de la distancia
            robot->goal. Tipicamente 0.3-0.5.
        min_control_dist: largo minimo de los handles (m). Evita curva
            degenerada cuando el goal esta muy cerca.

    Returns:
        BezierCurve con
            P0 = (0,0)
            P1 = (d, 0)               -> tangente al heading del robot (+x)
            P2 = goal - d*goal_heading_vec
            P3 = goal
    """
    p0 = np.array([0.0, 0.0], dtype=np.float64)
    p3 = np.array([float(goal_xy[0]), float(goal_xy[1])], dtype=np.float64)
    dist = float(np.linalg.norm(p3 - p0))
    d = max(min_control_dist, control_scale * dist)

    # Tangente inicial: robot mira a +x
    p1 = p0 + d * np.array([1.0, 0.0])
    # Tangente final: goal_heading
    p2 = p3 - d * np.array([math.cos(goal_heading), math.sin(goal_heading)])

    return BezierCurve(p0, p1, p2, p3)


# ---------------------------------------------------------------------------
# Pure pursuit
# ---------------------------------------------------------------------------
def compute_open_loop_trajectory(
    curve: BezierCurve,
    v_const: float,
    control_dt: float,
) -> list[tuple[float, float]]:
    """Convierte una Bezier a una lista de comandos (v, w) por tick.

    Estrategia:
      1. Tomar los samples pre-calculados de la curva (en arc length aprox).
      2. Calcular tangente (heading deseado) en cada sample.
      3. Distancia entre samples consecutivos = ds.
      4. Tiempo para ese ds = ds / v_const.
      5. Cuantos ticks toma = round(time / control_dt).
      6. w para ese tramo = dheading / time.

    Returns:
        Lista de (v, w). Total de ticks ~= arc_length(curve)/(v_const*control_dt).
    """
    if v_const <= 0 or control_dt <= 0:
        return []
    samples = curve.samples()  # (N, 2)
    if len(samples) < 2:
        return []

    # Headings entre samples consecutivos (atan2 del delta)
    deltas = samples[1:] - samples[:-1]                       # (N-1, 2)
    seg_lengths = np.linalg.norm(deltas, axis=1)              # (N-1,)
    seg_headings = np.arctan2(deltas[:, 1], deltas[:, 0])     # heading del segmento

    # Heading changes (dheading) entre segmentos consecutivos
    # Lo aplicamos durante el viaje por el segmento siguiente
    trajectory: list[tuple[float, float]] = []
    current_heading = seg_headings[0]
    for i, seg_len in enumerate(seg_lengths):
        seg_time = seg_len / v_const
        n_ticks = max(1, int(round(seg_time / control_dt)))

        target_heading = seg_headings[i]
        dh = target_heading - current_heading
        # Wrap a (-pi, pi]
        dh = math.atan2(math.sin(dh), math.cos(dh))
        # ω durante este segmento para llegar al heading objetivo
        w = dh / (n_ticks * control_dt)

        for _ in range(n_ticks):
            trajectory.append((v_const, w))
        current_heading = target_heading

    return trajectory


def pure_pursuit_cmd(
    curve: BezierCurve,
    d_lookahead: float,
    kp_w: float,
    kp_v: float,
    v_max: float,
    v_min: float,
    w_max: float,
    dist_to_goal_for_v_scaling: float,
) -> Tuple[float, float, np.ndarray]:
    """Comando (v, w) en frame robot (en origen) siguiendo la curva.

    El robot esta en (0, 0) mirando +x. Encontramos el look-ahead point
    sobre la curva y lo apuntamos.

    Returns:
        (v, w, look_point)
    """
    p_robot = np.zeros(2)
    look, _idx = curve.lookahead_from(p_robot, d_lookahead)

    # Bearing desde robot (mirando +x) hasta look point
    bearing = math.atan2(look[1], look[0])

    # v escala con distancia al goal
    v_raw = kp_v * dist_to_goal_for_v_scaling
    v = max(min(v_raw, v_max), 0.0)
    if 0 < v < v_min:
        v = v_min

    w = max(min(kp_w * bearing, w_max), -w_max)

    return v, w, look
