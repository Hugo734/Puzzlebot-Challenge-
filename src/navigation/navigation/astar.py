"""
astar.py — Pure A* path planner (no ROS dependencies, fully unit-testable).

Grid cells with occupancy value > 50 are treated as obstacles.
The planner inflates obstacles by `inflation_radius` metres before searching,
then returns a smoothed list of world-frame (x, y) waypoints.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np


class AStarPlanner:
    """A* global path planner over a 2-D occupancy grid."""

    def __init__(
        self,
        occupancy_grid: list[int],
        resolution: float,
        origin_x: float,
        origin_y: float,
        width: int,
        height: int,
        inflation_radius: float = 0.15,
    ) -> None:
        """
        Parameters
        ----------
        occupancy_grid:
            Flat list (row-major) of occupancy values in [0, 100].
            -1 means unknown.
        resolution:
            Metres per cell.
        origin_x, origin_y:
            World coordinates of the bottom-left corner of the grid.
        width, height:
            Grid dimensions in cells.
        inflation_radius:
            Radius (metres) by which obstacles are dilated before planning.
        """
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.inflation_radius = inflation_radius

        raw = np.array(occupancy_grid, dtype=np.int8).reshape((height, width))
        self._grid = raw
        self._inflated: Optional[np.ndarray] = None  # computed lazily

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> Optional[list[tuple[float, float]]]:
        """
        Plan a path from *start* to *goal* in world coordinates.

        Returns a list of (x, y) world-frame waypoints (including start and
        goal) if a path exists, or ``None`` if the goal is unreachable.
        """
        inflated = self._inflate_map()

        sr, sc = self._world_to_grid(*start)
        gr, gc = self._world_to_grid(*goal)

        # Clamp to grid bounds
        sr = max(0, min(self.height - 1, sr))
        sc = max(0, min(self.width - 1, sc))
        gr = max(0, min(self.height - 1, gr))
        gc = max(0, min(self.width - 1, gc))

        # Check that start and goal cells are free
        if inflated[sr, sc] > 50 or inflated[gr, gc] > 50:
            # Try to find the nearest free cell if start/goal is in obstacle
            sr, sc = self._nearest_free(inflated, sr, sc)
            gr, gc = self._nearest_free(inflated, gr, gc)
            if sr is None or gr is None:
                return None

        came_from: dict[tuple[int, int], Optional[tuple[int, int]]] = {}
        g_score: dict[tuple[int, int], float] = {}

        start_cell = (sr, sc)
        goal_cell = (gr, gc)

        g_score[start_cell] = 0.0
        came_from[start_cell] = None

        # Priority queue: (f_score, g_score_tiebreak, cell)
        open_heap: list[tuple[float, float, tuple[int, int]]] = []
        heapq.heappush(
            open_heap,
            (self._heuristic(start_cell, goal_cell), 0.0, start_cell),
        )

        # 8-connected neighbours with diagonal cost
        neighbours = [
            (-1, -1, math.sqrt(2)),
            (-1,  0, 1.0),
            (-1,  1, math.sqrt(2)),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            ( 1, -1, math.sqrt(2)),
            ( 1,  0, 1.0),
            ( 1,  1, math.sqrt(2)),
        ]

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            if current == goal_cell:
                path_cells = self._reconstruct(came_from, current)
                world_path = [self._grid_to_world(r, c) for r, c in path_cells]
                return self._smooth_path(world_path)

            # Skip stale entries
            if g > g_score.get(current, math.inf):
                continue

            cr, cc = current
            for dr, dc, step_cost in neighbours:
                nr, nc = cr + dr, cc + dc
                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue
                if inflated[nr, nc] > 50:
                    continue

                tentative_g = g_score[current] + step_cost * self.resolution
                neighbour = (nr, nc)
                if tentative_g < g_score.get(neighbour, math.inf):
                    g_score[neighbour] = tentative_g
                    came_from[neighbour] = current
                    h = self._heuristic(neighbour, goal_cell)
                    heapq.heappush(
                        open_heap,
                        (tentative_g + h, tentative_g, neighbour),
                    )

        return None  # No path found

    # ------------------------------------------------------------------
    # Map inflation
    # ------------------------------------------------------------------

    def _inflate_map(self) -> np.ndarray:
        """Return (and cache) the obstacle-inflated grid."""
        if self._inflated is not None:
            return self._inflated

        radius_cells = int(math.ceil(self.inflation_radius / self.resolution))
        inflated = self._grid.copy().astype(np.int16)

        obstacle_rows, obstacle_cols = np.where(self._grid > 50)

        for r, c in zip(obstacle_rows.tolist(), obstacle_cols.tolist()):
            r_min = max(0, r - radius_cells)
            r_max = min(self.height, r + radius_cells + 1)
            c_min = max(0, c - radius_cells)
            c_max = min(self.width, c + radius_cells + 1)

            # Use a circular kernel
            for nr in range(r_min, r_max):
                for nc in range(c_min, c_max):
                    dist = math.sqrt((nr - r) ** 2 + (nc - c) ** 2)
                    if dist <= radius_cells:
                        inflated[nr, nc] = 100

        self._inflated = inflated.astype(np.int8)
        return self._inflated

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world (x, y) to grid (row, col)."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def _grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid (row, col) to world (x, y) at cell centre."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        """Euclidean distance heuristic scaled to metres."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) * self.resolution

    def _reconstruct(
        self,
        came_from: dict[tuple[int, int], Optional[tuple[int, int]]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """Trace back from goal to start."""
        path: list[tuple[int, int]] = []
        node: Optional[tuple[int, int]] = current
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path

    def _smooth_path(
        self, path: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        """
        Remove collinear (redundant) intermediate waypoints.

        Two consecutive segments are collinear when the cross product of
        their direction vectors is below a small threshold.
        """
        if len(path) <= 2:
            return path

        result: list[tuple[float, float]] = [path[0]]
        for i in range(1, len(path) - 1):
            px, py = result[-1]
            cx, cy = path[i]
            nx, ny = path[i + 1]

            # Cross product of (current - prev) x (next - current)
            cross = (cx - px) * (ny - cy) - (cy - py) * (nx - cx)
            if abs(cross) > 1e-6:
                result.append(path[i])

        result.append(path[-1])
        return result

    def _nearest_free(
        self, grid: np.ndarray, row: int, col: int
    ) -> tuple[Optional[int], Optional[int]]:
        """BFS to find the nearest free cell to (row, col)."""
        if grid[row, col] <= 50:
            return row, col

        from collections import deque
        visited = set()
        queue: deque[tuple[int, int]] = deque([(row, col)])
        visited.add((row, col))

        while queue:
            r, c = queue.popleft()
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue
                if (nr, nc) in visited:
                    continue
                visited.add((nr, nc))
                if grid[nr, nc] <= 50:
                    return nr, nc
                queue.append((nr, nc))

        return None, None

    # ------------------------------------------------------------------
    # Utility — allow replanning with a new grid
    # ------------------------------------------------------------------

    def update_map(
        self,
        occupancy_grid: list[int],
        resolution: float,
        origin_x: float,
        origin_y: float,
        width: int,
        height: int,
    ) -> None:
        """Update the internal map and invalidate the inflation cache."""
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self._grid = np.array(occupancy_grid, dtype=np.int8).reshape(
            (height, width)
        )
        self._inflated = None  # force recomputation
