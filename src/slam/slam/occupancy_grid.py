import numpy as np


class OccupancyGrid:

    def __init__(self, width, height, resolution,
                 l_occ=0.85, l_free=-0.40, l_min=-5.0, l_max=5.0,
                 display_l_occ=None, display_l_free=None):
        self.width      = width
        self.height     = height
        self.resolution = resolution

        self.l_occ  = l_occ
        self.l_free = l_free
        self.l_min  = l_min
        self.l_max  = l_max

        # Thresholds for to_ros_data() — decoupled from update increments so
        # the evidence required to *display* a cell as occupied/free can be
        # tuned independently without changing the update dynamics.
        self.display_l_occ  = display_l_occ  if display_l_occ  is not None else l_occ
        self.display_l_free = display_l_free if display_l_free is not None else l_free

        self._log = np.zeros((height, width), dtype=np.float32)

        self.origin_x = -(width  * resolution) / 2.0
        self.origin_y = -(height * resolution) / 2.0

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_cell(self, x, y):
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    # ------------------------------------------------------------------
    # Vectorised DDA ray-casting update
    # ------------------------------------------------------------------

    def update_scan(self, robot_x, robot_y, robot_theta,
                    ranges, angle_min, angle_increment,
                    range_min, range_max,
                    laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                    scan_omega=0.0, scan_time=0.0):
        """
        Log-odds update for all rays in one scan — no Python loop over
        rays or cells.

        Free cells are traced via a Digital Differential Analyser (DDA)
        fully vectorised across all N rays × K steps using NumPy
        broadcasting.  Cell counts are accumulated with np.bincount
        (faster than np.add.at for uniform increments).

        Endpoint convention: the hit cell is included in both the free
        trace AND the occupied update (matching the original Bresenham
        implementation), giving it a net positive increment so occupied
        beats free at the wall surface.

        scan_omega / scan_time enable within-scan rotation deskewing:
        the effective heading for ray i is corrected by the rotation
        that elapsed up to ray i's capture instant.  The corresponding
        per-ray LiDAR origin shift is < 0.003 m for this robot geometry
        and is deliberately ignored.
        """
        ranges_arr = np.asarray(ranges, dtype=np.float64)
        n = len(ranges_arr)
        if n == 0:
            return

        valid = np.isfinite(ranges_arr) & (ranges_arr >= range_min)
        hit   = valid & (ranges_arr < range_max)

        if not valid.any():
            return

        # Per-ray deskew heading offsets (zero when no rotation)
        if scan_omega != 0.0 and scan_time > 0.0 and n > 1:
            yaw_offsets = scan_omega * np.linspace(0.0, scan_time, n)
        else:
            yaw_offsets = np.zeros(n)

        # Laser origin in world frame — fixed for all rays in this scan
        cT = np.cos(robot_theta)
        sT = np.sin(robot_theta)
        ox = robot_x + cT * laser_x - sT * laser_y
        oy = robot_y + sT * laser_x + cT * laser_y

        # Per-ray world angles (heading + laser mount + deskew + scan angle)
        ray_angles = (robot_theta + laser_yaw + yaw_offsets
                      + angle_min + np.arange(n, dtype=np.float64) * angle_increment)

        # Endpoint range: actual for hits, range_max for misses
        r_ep = np.where(hit, ranges_arr, np.where(valid, range_max, 0.0))

        # Endpoint world coordinates
        ex = ox + r_ep * np.cos(ray_angles)
        ey = oy + r_ep * np.sin(ray_angles)

        # Convert to grid cells
        inv_res = 1.0 / self.resolution
        ox_c = int((ox - self.origin_x) * inv_res)
        oy_c = int((oy - self.origin_y) * inv_res)
        ex_c = np.floor((ex - self.origin_x) * inv_res).astype(np.int32)
        ey_c = np.floor((ey - self.origin_y) * inv_res).astype(np.int32)

        # DDA step counts: Chebyshev distance from origin to endpoint cell
        dx = ex_c - ox_c  # (n,)
        dy = ey_c - oy_c  # (n,)
        n_steps = np.maximum(np.abs(dx), np.abs(dy))  # (n,)

        max_steps = int(n_steps[valid].max())
        if max_steps == 0:
            return

        # ── Free-cell DDA ───────────────────────────────────────────────
        # Build (n, K) matrices where K = max_steps + 1 (inclusive of the
        # endpoint so the endpoint cell also gets a free vote, matching the
        # original Bresenham behaviour where the hit cell is yielded last).
        k_arr  = np.arange(max_steps + 1, dtype=np.float32)            # (K,)
        s_safe = np.where(n_steps > 0, n_steps, 1).astype(np.float32)  # (n,)
        t      = k_arr[np.newaxis, :] / s_safe[:, np.newaxis]           # (n, K)

        cx = (ox_c + np.round(dx[:, np.newaxis] * t)).astype(np.int32)  # (n, K)
        cy = (oy_c + np.round(dy[:, np.newaxis] * t)).astype(np.int32)  # (n, K)

        in_map = (
            (cx >= 0) & (cx < self.width) &
            (cy >= 0) & (cy < self.height)
        )
        active = (
            valid[:, np.newaxis] &
            (k_arr[np.newaxis, :] <= n_steps[:, np.newaxis]) &
            in_map
        )

        flat_free = cy[active] * self.width + cx[active]
        counts_free = np.bincount(flat_free, minlength=self.width * self.height)
        self._log += counts_free.reshape(self.height, self.width) * self.l_free

        # ── Occupied-cell update ────────────────────────────────────────
        occ = (
            hit &
            (ex_c >= 0) & (ex_c < self.width) &
            (ey_c >= 0) & (ey_c < self.height)
        )
        if occ.any():
            flat_occ = ey_c[occ] * self.width + ex_c[occ]
            counts_occ = np.bincount(flat_occ, minlength=self.width * self.height)
            self._log += counts_occ.reshape(self.height, self.width) * self.l_occ

        np.clip(self._log, self.l_min, self.l_max, out=self._log)

    # ------------------------------------------------------------------
    # Serialise for ROS
    # ------------------------------------------------------------------

    def to_ros_data(self):
        """
        Return flat int8 list for OccupancyGrid.data.
          log > display_l_occ  → 100 (occupied)
          log < display_l_free →   0 (free)
          else                 →  -1 (unknown)

        display_l_occ / display_l_free are set at construction time and
        can be higher/lower than the update increments to require more
        evidence before committing a cell to a displayed classification.
        """
        out = np.full((self.height, self.width), -1, dtype=np.int8)
        out[self._log >  self.display_l_occ]  = 100
        out[self._log <  self.display_l_free] = 0
        return out.flatten().tolist()
