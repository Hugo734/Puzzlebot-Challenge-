"""Trayectoria parabolica y = a*x^2 con vertice en el robot.

Plan:
    Robot en (0, 0) mirando +x (frame robot).
    Goal en (gx, gy).
    Parabola: y(x) = a*x^2 con a = gy / gx^2  (requiere gx != 0).
    Tangente en vertice (x=0): dy/dx=0 -> alineada con el +x del robot.
    Pasa por el goal por construccion.

Usa odometria para seguir la curva: a medida que el robot se mueve, su
pose (x_r, y_r, th_r) en el frame de planeo (fijado al inicio) se
compara con la curva.
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


class QuadraticPath:
    """y = a*x^2 con vertice en (0,0). Sample-based para closest/lookahead."""

    def __init__(self, goal_xy: Tuple[float, float], n_samples: int = 80) -> None:
        self.gx = float(goal_xy[0])
        self.gy = float(goal_xy[1])
        if abs(self.gx) < 1e-3:
            raise ValueError(
                f'gx={self.gx:.4f} demasiado pequeno para parabola y=ax^2'
            )
        self.a = self.gy / (self.gx ** 2)
        self._n = int(n_samples)

        # Samples a lo largo de x (signed: respeta direccion del gx)
        # Si gx > 0 va de 0 a gx; si gx < 0 va de 0 a gx (negativo).
        xs = np.linspace(0.0, self.gx, self._n)
        ys = self.a * xs * xs
        self._samples = np.stack([xs, ys], axis=1)  # (N, 2)

    # ------------------------------------------------------------------
    def evaluate(self, x: float) -> Tuple[float, float]:
        """y = a*x^2"""
        return float(x), float(self.a * x * x)

    def tangent(self, x: float) -> Tuple[float, float]:
        """Vector tangente unitario (dx/dx, dy/dx) = (1, 2ax)."""
        slope = 2.0 * self.a * x
        norm = math.hypot(1.0, slope)
        return 1.0 / norm, slope / norm

    def heading_at(self, x: float) -> float:
        return math.atan2(2.0 * self.a * x, 1.0)

    def curvature_at(self, x: float) -> float:
        """Curvatura signed de la parabola: kappa(x) = 2a / (1 + (2ax)^2)^(3/2)."""
        denom = (1.0 + (2.0 * self.a * x) ** 2) ** 1.5
        return (2.0 * self.a) / denom

    def samples(self) -> np.ndarray:
        return self._samples

    # ------------------------------------------------------------------
    def closest_index(self, p: np.ndarray) -> int:
        p = np.asarray(p, dtype=np.float64)
        dists = np.linalg.norm(self._samples - p, axis=1)
        return int(np.argmin(dists))

    def lookahead_from(self, p: np.ndarray, d: float) -> Tuple[np.ndarray, int]:
        """Punto en la curva, adelante del closest, a distancia >= d."""
        p = np.asarray(p, dtype=np.float64)
        i0 = self.closest_index(p)
        # 'Adelante' = mayor x si gx > 0, menor x si gx < 0.
        direction = 1 if self.gx >= 0 else -1
        i = i0
        while 0 <= i < self._n:
            sample = self._samples[i]
            if np.linalg.norm(sample - p) >= d:
                return sample, i
            i += direction
        return self._samples[-1 if direction > 0 else 0], i - direction


def plan_quadratic_in_robot_frame(goal_xy: Tuple[float, float]) -> QuadraticPath:
    """Crea parabola y=a*x^2 con vertice en el robot pasando por goal_xy."""
    return QuadraticPath(goal_xy)


def compute_quadratic_trajectory(
    path: QuadraticPath,
    v_const: float,
    control_dt: float,
) -> list:
    """Convierte la parabola en una lista de comandos (v, w) por tick.

    Algoritmo:
      1. Sample la curva en N puntos (ya hecho en path._samples).
      2. Por cada segmento entre samples consecutivos:
           seg_len = distancia euclidea entre samples.
           seg_heading = atan2(dy, dx) del segmento.
           seg_time = seg_len / v_const.
           n_ticks = ceil(seg_time / control_dt).
           dh = wrap(seg_heading - current_heading).
           w = dh / (n_ticks * control_dt).
         Anade (v_const, w) n_ticks veces.
    """
    if v_const <= 0 or control_dt <= 0:
        return []
    samples = path.samples()
    if len(samples) < 2:
        return []
    deltas = samples[1:] - samples[:-1]
    seg_lens = np.linalg.norm(deltas, axis=1)
    seg_headings = np.arctan2(deltas[:, 1], deltas[:, 0])

    trajectory: list = []
    current_heading = seg_headings[0]
    for i, seg_len in enumerate(seg_lens):
        seg_time = seg_len / v_const
        n_ticks = max(1, int(round(seg_time / control_dt)))
        target_heading = seg_headings[i]
        dh = target_heading - current_heading
        dh = math.atan2(math.sin(dh), math.cos(dh))
        w = dh / (n_ticks * control_dt)
        for _ in range(n_ticks):
            trajectory.append((v_const, w))
        current_heading = target_heading
    return trajectory
