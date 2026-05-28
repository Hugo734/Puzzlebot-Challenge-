import numpy as np


class OccupancyGrid:

    def __init__(self, width, height, resolution,
                 l_occ=0.85, l_free=-0.40, l_min=-5.0, l_max=5.0,
                 display_l_occ=None, display_l_free=None,
                 occupied_stop=None):
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

        # Occupied-stop threshold: when a free-ray DDA would traverse a
        # cell whose log-odds already exceed this, the trace TERMINATES
        # at that cell (and the cell itself is not free-voted).  This is
        # the canonical fix for "walls keep moving / matcher loses lock":
        # a slight pose error makes a ray's geometry pass 1-2 pixels to
        # one side of a previously-mapped wall, so the ray's free votes
        # erase the wall's cells from the inside.  An occupied-stop
        # prevents that — once geometry says "we hit a wall", we trust
        # the wall and stop the free trace there.
        self.occupied_stop = (
            occupied_stop if occupied_stop is not None else self.display_l_occ)

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
                    scan_omega=0.0, scan_time=0.0,
                    ray_dx=None, ray_dy=None, ray_dth=None):
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

        Deskew (caller picks whichever):

          * FULL: ray_dx / ray_dy / ray_dth (length n) — per-ray robot
            pose offset from scan-start in scan-start base frame.  Both
            the per-ray laser ORIGIN and per-ray ray HEADING are derived
            from these, so a robot rotating AND translating during the
            scan still produces a coherent map update.
          * SIMPLE: scan_omega / scan_time — yaw-only per-ray offset,
            single shared origin.  Used as fallback.

        (robot_x, robot_y, robot_theta) is the SLAM pose at scan-start
        (i.e. the reference pose against which ray_dx/dy/dth are taken).
        """
        ranges_arr = np.asarray(ranges, dtype=np.float64)
        n = len(ranges_arr)
        if n == 0:
            return

        valid = np.isfinite(ranges_arr) & (ranges_arr >= range_min)
        hit   = valid & (ranges_arr < range_max)

        if not valid.any():
            return

        cT0 = np.cos(robot_theta)
        sT0 = np.sin(robot_theta)

        if ray_dx is not None and ray_dy is not None and ray_dth is not None:
            # FULL deskew: per-ray robot pose in world frame.
            # base_world = robot_xy + R(theta_0) · ray_dxy
            ray_dx_w = cT0 * ray_dx - sT0 * ray_dy
            ray_dy_w = sT0 * ray_dx + cT0 * ray_dy
            bx_w = robot_x + ray_dx_w
            by_w = robot_y + ray_dy_w
            theta_i = robot_theta + ray_dth
            yaw_offsets = ray_dth        # used by ray-angle formula below
        else:
            # SIMPLE legacy: shared origin + yaw-only per-ray offset.
            if scan_omega != 0.0 and scan_time > 0.0 and n > 1:
                yaw_offsets = scan_omega * np.linspace(0.0, scan_time, n)
            else:
                yaw_offsets = np.zeros(n)
            bx_w = np.full(n, robot_x)
            by_w = np.full(n, robot_y)
            theta_i = np.full(n, robot_theta) + yaw_offsets

        # Per-ray laser origin in world frame
        cTi = np.cos(theta_i)
        sTi = np.sin(theta_i)
        ox_arr = bx_w + cTi * laser_x - sTi * laser_y
        oy_arr = by_w + sTi * laser_x + cTi * laser_y

        # Per-ray world angles (heading + laser mount + scan angle)
        ray_angles = (theta_i + laser_yaw
                      + angle_min + np.arange(n, dtype=np.float64) * angle_increment)

        # Endpoint range: actual for hits, range_max for misses
        r_ep = np.where(hit, ranges_arr, np.where(valid, range_max, 0.0))

        # Endpoint world coordinates
        ex = ox_arr + r_ep * np.cos(ray_angles)
        ey = oy_arr + r_ep * np.sin(ray_angles)

        # Convert to grid cells
        inv_res = 1.0 / self.resolution
        ox_c = np.floor((ox_arr - self.origin_x) * inv_res).astype(np.int32)
        oy_c = np.floor((oy_arr - self.origin_y) * inv_res).astype(np.int32)
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

        cx = (ox_c[:, np.newaxis] + np.round(dx[:, np.newaxis] * t)).astype(np.int32)  # (n, K)
        cy = (oy_c[:, np.newaxis] + np.round(dy[:, np.newaxis] * t)).astype(np.int32)  # (n, K)

        in_map = (
            (cx >= 0) & (cx < self.width) &
            (cy >= 0) & (cy < self.height)
        )
        active = (
            valid[:, np.newaxis] &
            (k_arr[np.newaxis, :] <= n_steps[:, np.newaxis]) &
            in_map
        )

        # ── Occupied-stop ──────────────────────────────────────────────
        # For each ray, find the FIRST already-occupied cell along its
        # path (if any).  The free trace ends at the step just BEFORE
        # that cell — we don't free-vote walls just because pose error
        # made our ray geometry pass a pixel to one side.  Result: real
        # walls stay put across scans, the matcher keeps its anchor.
        cx_safe = np.clip(cx, 0, self.width - 1)
        cy_safe = np.clip(cy, 0, self.height - 1)
        occupied_along = (in_map &
                          (self._log[cy_safe, cx_safe] > self.occupied_stop))
        # Mask out the endpoint cell so a ray that ENDS on its own wall
        # doesn't self-stop one cell short.
        endpoint_mask = (k_arr[np.newaxis, :] == n_steps[:, np.newaxis])
        occupied_along = occupied_along & ~endpoint_mask
        # First occupied step per ray (or n_steps+1 = "never")
        K = max_steps + 1
        first_hit = np.where(
            occupied_along.any(axis=1),
            occupied_along.argmax(axis=1),
            K)
        # Free-vote only steps strictly BEFORE first_hit
        active = active & (k_arr[np.newaxis, :] < first_hit[:, np.newaxis])

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
    # Batched scoring (for the correlative scan matcher)
    # ------------------------------------------------------------------

    def score_points(self, xs, ys, score_floor=0.0):
        """
        Sum the log-odds at the cells under each (x, y) world point.

        Used by the correlative scan matcher to score candidate poses:
        for a given hypothesis we transform the scan into world frame
        and ask "how much positive evidence does this pose accumulate?"

        Points outside the map contribute 0.  `score_floor` clamps the
        per-cell contribution from below (default 0): unknown / free
        cells don't penalise a hypothesis, only occupied cells reward
        it.  Without this, an empty corner of the map would attract
        the search (because it's "less negative" than the wrong wall).
        """
        cx = np.floor((xs - self.origin_x) / self.resolution).astype(np.int32)
        cy = np.floor((ys - self.origin_y) / self.resolution).astype(np.int32)
        in_b = (cx >= 0) & (cx < self.width) & (cy >= 0) & (cy < self.height)
        if not in_b.any():
            return 0.0
        cx = cx[in_b]; cy = cy[in_b]
        vals = self._log[cy, cx]
        vals = np.maximum(vals, score_floor)
        return float(vals.sum())

    # ------------------------------------------------------------------
    # Point-cloud extraction (for scan-to-map ICP)
    # ------------------------------------------------------------------

    def get_occupied_points(self, robot_x=None, robot_y=None,
                            radius=None, threshold=None):
        """
        Return world-frame (x, y) coordinates of every cell whose
        log-odds exceed `threshold` (default: display_l_occ).

        When robot_x/robot_y/radius are given, points outside that
        disc are discarded — the local scan can only match nearby
        map structure, so passing the full grid wastes CPU and
        invites spurious matches at the other end of the room.
        """
        if threshold is None:
            threshold = self.display_l_occ

        mask = self._log > threshold
        if not mask.any():
            return np.empty((0, 2), dtype=np.float64)

        cy_idx, cx_idx = np.nonzero(mask)
        xs = self.origin_x + (cx_idx + 0.5) * self.resolution
        ys = self.origin_y + (cy_idx + 0.5) * self.resolution

        if robot_x is not None and robot_y is not None and radius is not None:
            d2   = (xs - robot_x) ** 2 + (ys - robot_y) ** 2
            keep = d2 <= radius ** 2
            xs   = xs[keep]
            ys   = ys[keep]

        return np.column_stack([xs, ys])

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
