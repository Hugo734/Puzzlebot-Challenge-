import math
import numpy as np

try:
    from scipy.spatial import cKDTree
    _HAS_KDTREE = True
except ImportError:
    _HAS_KDTREE = False


# ──────────────────────────────────────────────────────────────────
# Nearest neighbours
# ──────────────────────────────────────────────────────────────────

def _nearest_neighbors(source, target, target_tree=None):
    """
    Nearest-neighbour search.

    Uses scipy.spatial.cKDTree (O(N log M)) when available, otherwise
    falls back to brute-force NumPy broadcasting (O(N·M)).  When the
    caller already holds a tree built on the target, pass it via
    `target_tree` to avoid rebuilding it every ICP iteration.

    Returns (indices_into_target, squared_distances).
    """
    if _HAS_KDTREE:
        tree = target_tree if target_tree is not None else cKDTree(target)
        dists, idx = tree.query(source, k=1)
        return idx, dists ** 2

    diff  = source[:, np.newaxis, :] - target[np.newaxis, :, :]
    sq    = np.sum(diff ** 2, axis=2)
    idx   = np.argmin(sq, axis=1)
    return idx, sq[np.arange(len(source)), idx]


# ──────────────────────────────────────────────────────────────────
# Surface normals (2D PCA on k-neighbourhoods)
# ──────────────────────────────────────────────────────────────────

def estimate_normals(points, k=8, tree=None):
    """
    Estimate 2D surface normals via PCA on k-nearest neighbourhoods.

    For each point we collect its k nearest neighbours, fit a covariance,
    and take the eigenvector with the smallest eigenvalue as the normal.
    Returns (N, 2) unit normals.  Output is fully vectorised — the only
    Python-level loop is the cKDTree query.
    """
    n = len(points)
    if n < 3:
        return np.zeros((n, 2))
    k = min(k, n)

    if tree is None and _HAS_KDTREE:
        tree = cKDTree(points)

    if _HAS_KDTREE:
        _, idx = tree.query(points, k=k)
    else:
        diff = points[:, np.newaxis, :] - points[np.newaxis, :, :]
        sq   = np.sum(diff ** 2, axis=2)
        idx  = np.argsort(sq, axis=1)[:, :k]

    neighbors = points[idx]                                          # (N, k, 2)
    centered  = neighbors - neighbors.mean(axis=1, keepdims=True)
    cov       = np.einsum('nki,nkj->nij', centered, centered) / max(k - 1, 1)
    # eigh returns eigenvalues ascending; index 0 → smallest → normal direction
    _, eigvecs = np.linalg.eigh(cov)
    return eigvecs[:, :, 0]


# ──────────────────────────────────────────────────────────────────
# Pose / transform helpers
# ──────────────────────────────────────────────────────────────────

def _best_fit_transform(A, B):
    """Closed-form rigid SE(2) transform via SVD (point-to-point)."""
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    return R, t


def _best_fit_point_to_line(source, target, normals):
    """
    Linearised point-to-line solver (2D analogue of point-to-plane).

    Minimises Σ (n_i · (R p_i + t − q_i))² under the small-angle
    approximation R ≈ I + [θ]×, which reduces to a 3×3 normal-equation
    system on x = [t_x, t_y, θ]:

        A_i = [ n_ix , n_iy , n_iy·p_x − n_ix·p_y ]
        b_i =  n_i · (q_i − p_i)

    Solving by least-squares gives translations and a (small) rotation
    update; we then rebuild the exact SE(2) matrix from the recovered θ.
    Point-to-line drastically out-performs point-to-point on long walls
    and corners because sliding along a wall costs nothing in the metric.
    """
    cross = normals[:, 1] * source[:, 0] - normals[:, 0] * source[:, 1]
    A     = np.column_stack([normals[:, 0], normals[:, 1], cross])
    b     = np.sum(normals * (target - source), axis=1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    tx, ty, theta = x
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([tx, ty])
    return R, t


def _apply_transform(points, R, t):
    return (R @ points.T).T + t


def _transform_to_pose(R, t):
    return t[0], t[1], math.atan2(R[1, 0], R[0, 0])


def _pose_to_transform(dx, dy, dtheta):
    c, s = math.cos(dtheta), math.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([dx, dy])
    return R, t


# ──────────────────────────────────────────────────────────────────
# Scan → point cloud
# ──────────────────────────────────────────────────────────────────

def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                   scan_omega=0.0, scan_time=0.0,
                   ray_dx=None, ray_dy=None, ray_dth=None):
    """
    Project a laser scan into the SCAN-START robot base frame, with
    per-ray deskew.

    Two deskew modes (per-ray pose wins when both are given):

      * FULL (preferred): pass `ray_dx`, `ray_dy`, `ray_dth` arrays of
        length n.  Each entry is the robot's pose delta from scan-start
        to that ray's capture time, expressed in the scan-start base
        frame.  Caller computes these by interpolating the odom buffer
        at each ray's timestamp — this gives a clean cloud even when ω
        is non-constant during the scan AND when linear motion adds a
        per-ray laser-origin shift (≈3 cm at 0.3 m/s · 100 ms).

      * SIMPLE (legacy): pass `scan_omega` and `scan_time`.  Each ray's
        angle is rotated by ω·f·dt, assuming ω is constant during the
        scan and ignoring translation.  Adequate when ω is small and
        the robot is roughly stationary; degrades during fast turns.

    Pass nothing to disable deskew entirely (raw single-pose cloud).
    """
    n       = len(ranges)
    angles  = angle_min + np.arange(n) * angle_increment
    ranges  = np.array(ranges, dtype=np.float64)
    valid   = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)

    cL, sL  = math.cos(laser_yaw), math.sin(laser_yaw)

    # Hit point in the LASER frame at each ray's capture time
    cosa = np.cos(angles)
    sina = np.sin(angles)
    rxl = ranges * cosa
    ryl = ranges * sina

    # → BASE-at-capture frame (apply the static base→laser transform)
    bx_cap = laser_x + cL * rxl - sL * ryl
    by_cap = laser_y + sL * rxl + cL * ryl

    if ray_dx is not None and ray_dy is not None and ray_dth is not None:
        # FULL per-ray pose deskew — express each hit in SCAN-START base
        # frame using the robot's pose delta at the ray's capture time.
        cdt = np.cos(ray_dth)
        sdt = np.sin(ray_dth)
        xs = ray_dx + cdt * bx_cap - sdt * by_cap
        ys = ray_dy + sdt * bx_cap + cdt * by_cap
    elif scan_omega != 0.0 and scan_time > 0.0 and n > 1:
        # SIMPLE legacy yaw-only deskew (kept for callers without odom).
        f = np.arange(n, dtype=np.float64) / (n - 1)
        dth = scan_omega * f * scan_time
        cdt = np.cos(dth)
        sdt = np.sin(dth)
        xs = cdt * bx_cap - sdt * by_cap
        ys = sdt * bx_cap + cdt * by_cap
    else:
        xs = bx_cap
        ys = by_cap

    return np.column_stack((xs[valid], ys[valid]))


# ──────────────────────────────────────────────────────────────────
# Range outlier filter (spike rejection)
# ──────────────────────────────────────────────────────────────────

def range_outlier_filter(ranges, max_jump=0.5, range_max=10.0):
    """
    Discard rays whose range differs from BOTH neighbours by more than
    `max_jump` metres.  RPLidar A1 occasionally emits isolated spikes
    (reflections off glass, dust motes, dark cloth) that, if integrated
    into the map, project as ghost walls and pull ICP towards bad
    matches.  This filter runs before scan_to_points and replaces such
    spikes with `inf` (which scan_to_points then drops via its `valid`
    mask).  Two-sided: a true wall edge survives because only ONE
    neighbour differs.
    """
    r = np.array(ranges, dtype=np.float64)
    n = len(r)
    if n < 3:
        return r
    finite = np.isfinite(r) & (r >= 0.0) & (r <= range_max)
    # Forward and backward neighbour differences
    fwd = np.full(n, np.inf)
    bwd = np.full(n, np.inf)
    fwd[:-1] = np.where(finite[:-1] & finite[1:], np.abs(r[:-1] - r[1:]), 0.0)
    bwd[1:]  = np.where(finite[1:]  & finite[:-1], np.abs(r[1:]  - r[:-1]), 0.0)
    # Spike: differs from BOTH neighbours by more than max_jump
    spike = finite & (fwd > max_jump) & (bwd > max_jump)
    if spike.any():
        r = r.copy()
        r[spike] = np.inf
    return r


# ──────────────────────────────────────────────────────────────────
# Correlative Scan Matcher  (Cartographer-style coarse-to-fine)
# ──────────────────────────────────────────────────────────────────

def correlative_scan_match(grid, cloud_base, init_pose,
                           xy_radius=0.30, xy_step=0.05,
                           th_radius=0.20, th_step=0.025,
                           score_floor=0.0):
    """
    Brute-force search for the SE(2) pose that maximises the scan-to-map
    score within a window around `init_pose`.

    Unlike ICP, this is a *global* search inside the window — it cannot
    get trapped by a local minimum, so it recovers cleanly when odometry
    alone is far off from where the scan actually fits the map.  Followed
    by a single ICP step it gives Cartographer-quality matching at a
    fraction of the CPU.

    Parameters
    ----------
    grid       : OccupancyGrid with a `.score_points(xs, ys, floor)` method
    cloud_base : (N, 2) scan in base frame (deskewed)
    init_pose  : (x, y, theta) odometry prediction
    xy_radius  : half-width of (x, y) search window  (m)
    xy_step    : grid step in (x, y)                 (m)
    th_radius  : half-width of θ search window       (rad)
    th_step    : grid step in θ                      (rad)
    score_floor: clamp log-odds below this to 0 when scoring (only
                 positive evidence rewards a match, so empty/unknown
                 cells don't bias the search)

    Returns
    -------
    (best_x, best_y, best_theta, best_score)
    """
    x0, y0, th0 = init_pose

    dxs  = np.arange(-xy_radius, xy_radius + xy_step  / 2.0, xy_step,  dtype=np.float64)
    dys  = np.arange(-xy_radius, xy_radius + xy_step  / 2.0, xy_step,  dtype=np.float64)
    dths = np.arange(-th_radius, th_radius + th_step  / 2.0, th_step,  dtype=np.float64)

    best_score   = -np.inf
    best_dx      = 0.0
    best_dy      = 0.0
    best_dtheta  = 0.0

    # Pre-convert to cell-space step deltas — we shift the lookup
    # indices rather than the world coordinates, which keeps the inner
    # bookkeeping integer.
    res     = grid.resolution
    dx_cells = np.round(dxs / res).astype(np.int32)
    dy_cells = np.round(dys / res).astype(np.int32)

    for dth in dths:
        c, s = math.cos(th0 + dth), math.sin(th0 + dth)
        # Rotate cloud by total heading (th0 + dth) into world axes,
        # then translate by (x0, y0).  We do NOT add (dx, dy) yet —
        # that's the inner-loop shift.
        rot_x = c * cloud_base[:, 0] - s * cloud_base[:, 1]
        rot_y = s * cloud_base[:, 0] + c * cloud_base[:, 1]

        cx_base = np.floor(((rot_x + x0) - grid.origin_x) / res).astype(np.int32)
        cy_base = np.floor(((rot_y + y0) - grid.origin_y) / res).astype(np.int32)

        # Broadcast over (Nx, Ny, Np):  cx[i, j, k] = cx_base[k] + dx_cells[i]
        cx = cx_base[None, None, :] + dx_cells[:, None, None]   # (Nx, 1, Np)
        cy = cy_base[None, None, :] + dy_cells[None, :, None]   # (1, Ny, Np)

        in_b = (cx >= 0) & (cx < grid.width) & (cy >= 0) & (cy < grid.height)
        cx_safe = np.clip(cx, 0, grid.width  - 1)
        cy_safe = np.clip(cy, 0, grid.height - 1)

        vals = grid._log[np.broadcast_to(cy_safe, (len(dx_cells), len(dy_cells), len(cx_base))),
                         np.broadcast_to(cx_safe, (len(dx_cells), len(dy_cells), len(cx_base)))]
        # Clamp negative evidence to 0 — only positive log-odds reward
        np.maximum(vals, score_floor, out=vals)
        vals = vals * in_b
        score_2d = vals.sum(axis=2)   # (Nx, Ny)

        idx = int(np.argmax(score_2d))
        i_x, i_y = divmod(idx, score_2d.shape[1])
        s = float(score_2d[i_x, i_y])
        if s > best_score:
            best_score  = s
            best_dx     = float(dxs[i_x])
            best_dy     = float(dys[i_y])
            best_dtheta = float(dth)

    return (x0 + best_dx,
            y0 + best_dy,
            math.atan2(math.sin(th0 + best_dtheta), math.cos(th0 + best_dtheta)),
            best_score)


def multi_resolution_csm(grid, cloud_base, init_pose,
                         coarse=(0.40, 0.10, 0.20, 0.05),
                         fine  =(0.10, 0.02, 0.05, 0.01),
                         score_floor=0.0):
    """
    Two-stage CSM: coarse window first, then a tighter window around the
    coarse winner.  Costs ~2× a single fine pass but covers a 5× larger
    search area, so it recovers from much larger odom errors without
    blowing the per-scan time budget.
    """
    cx, cy, cth, cs_score = correlative_scan_match(
        grid, cloud_base, init_pose,
        xy_radius=coarse[0], xy_step=coarse[1],
        th_radius=coarse[2], th_step=coarse[3],
        score_floor=score_floor)
    fx, fy, fth, fs_score = correlative_scan_match(
        grid, cloud_base, (cx, cy, cth),
        xy_radius=fine[0],   xy_step=fine[1],
        th_radius=fine[2],   th_step=fine[3],
        score_floor=score_floor)
    return (fx, fy, fth), fs_score


def correlative_scan_match_topk(grid, cloud_base, init_pose,
                                xy_radius, xy_step,
                                th_radius, th_step,
                                top_k=5,
                                exclude_xy_radius=1.0,
                                exclude_th_radius=None,
                                score_floor=0.0):
    """
    Like ``correlative_scan_match`` but returns the top-K spatially-
    distinct maxima instead of just the global one.

    After each maximum is extracted, a neighbourhood of size
    ``exclude_xy_radius`` (m) × ``exclude_th_radius`` (rad) is zeroed
    in the score volume before the next argmax — this guarantees the
    K hypotheses are physically distinct candidates and not three
    copies of the same wall match shifted by one cell.

    Returns a list of ``(x, y, theta, score)`` tuples sorted by score
    descending.  Length ≤ top_k (less when the score volume runs out
    of positive maxima or the exclusion zone has eaten everything).
    """
    x0, y0, th0 = init_pose
    if exclude_th_radius is None:
        exclude_th_radius = 3.0 * th_step

    dxs  = np.arange(-xy_radius, xy_radius + xy_step / 2.0, xy_step,  dtype=np.float64)
    dys  = np.arange(-xy_radius, xy_radius + xy_step / 2.0, xy_step,  dtype=np.float64)
    dths = np.arange(-th_radius, th_radius + th_step / 2.0, th_step,  dtype=np.float64)

    res      = grid.resolution
    dx_cells = np.round(dxs / res).astype(np.int32)
    dy_cells = np.round(dys / res).astype(np.int32)

    # Full (Nth, Nx, Ny) score volume — same inner kernel as the
    # single-best version, just persisted across θ instead of reduced.
    score_3d = np.full((len(dths), len(dxs), len(dys)), -np.inf, dtype=np.float64)
    for k_th, dth in enumerate(dths):
        c, s = math.cos(th0 + dth), math.sin(th0 + dth)
        rot_x = c * cloud_base[:, 0] - s * cloud_base[:, 1]
        rot_y = s * cloud_base[:, 0] + c * cloud_base[:, 1]
        cx_base = np.floor(((rot_x + x0) - grid.origin_x) / res).astype(np.int32)
        cy_base = np.floor(((rot_y + y0) - grid.origin_y) / res).astype(np.int32)
        cx = cx_base[None, None, :] + dx_cells[:, None, None]
        cy = cy_base[None, None, :] + dy_cells[None, :, None]
        in_b = (cx >= 0) & (cx < grid.width) & (cy >= 0) & (cy < grid.height)
        cx_safe = np.clip(cx, 0, grid.width  - 1)
        cy_safe = np.clip(cy, 0, grid.height - 1)
        vals = grid._log[np.broadcast_to(cy_safe, (len(dx_cells), len(dy_cells), len(cx_base))),
                         np.broadcast_to(cx_safe, (len(dx_cells), len(dy_cells), len(cx_base)))]
        np.maximum(vals, score_floor, out=vals)
        vals = vals * in_b
        score_3d[k_th] = vals.sum(axis=2)

    excl_xy = max(1, int(round(exclude_xy_radius / xy_step)))
    excl_th = max(1, int(round(exclude_th_radius / th_step)))
    results = []
    scratch = score_3d.copy()
    for _ in range(top_k):
        idx = int(np.argmax(scratch))
        flat_max = float(scratch.flat[idx])
        if not np.isfinite(flat_max) or flat_max <= 0.0:
            break
        k_th, ij = divmod(idx, scratch.shape[1] * scratch.shape[2])
        i_x, i_y = divmod(ij, scratch.shape[2])
        results.append((
            x0 + float(dxs[i_x]),
            y0 + float(dys[i_y]),
            math.atan2(math.sin(th0 + float(dths[k_th])),
                       math.cos(th0 + float(dths[k_th]))),
            flat_max,
        ))
        k0 = max(0,                  k_th - excl_th)
        k1 = min(scratch.shape[0],   k_th + excl_th + 1)
        i0 = max(0,                  i_x  - excl_xy)
        i1 = min(scratch.shape[1],   i_x  + excl_xy + 1)
        j0 = max(0,                  i_y  - excl_xy)
        j1 = min(scratch.shape[2],   i_y  + excl_xy + 1)
        scratch[k0:k1, i0:i1, j0:j1] = -np.inf

    return results


def global_localization_multi(grid, cloud_base,
                              xy_step=0.30, th_step_deg=8.0,
                              top_k=5,
                              exclude_xy_radius=1.0,
                              score_floor=0.0):
    """
    Multi-hypothesis global localisation.  Stage 1 sweeps the whole map
    × ±180° and returns the top-K spatially-distinct maxima; stages 2
    & 3 then refine each candidate independently with the same coarse-
    to-fine schedule as :func:`global_localization`.

    The caller is expected to verify the winning candidate with a
    quality metric the score itself cannot detect — e.g. inlier
    ratio against the wall cloud.  The score (sum of log-odds at
    scan points) is high enough to be a useful ranking signal but
    cannot tell a genuine match from a partial one against the
    wrong wall.

    Returns a list of ``((x, y, theta), score)`` sorted by score
    descending.  May be empty if Stage 1 found nothing positive.
    """
    half_w    = (grid.width  * grid.resolution) / 2.0
    half_h    = (grid.height * grid.resolution) / 2.0
    radius_xy = max(half_w, half_h)
    th_step   = math.radians(th_step_deg)
    center    = (grid.origin_x + half_w, grid.origin_y + half_h, 0.0)

    coarse = correlative_scan_match_topk(
        grid, cloud_base, center,
        xy_radius=radius_xy, xy_step=xy_step,
        th_radius=math.pi,   th_step=th_step,
        top_k=top_k,
        exclude_xy_radius=exclude_xy_radius,
        exclude_th_radius=0.5,                   # ±29° between candidates
        score_floor=score_floor)
    if not coarse:
        return []

    refined = []
    for (cx, cy, cth, _cs) in coarse:
        # Stage 2 — refine around the coarse winner
        c2_x, c2_y, c2_th, _c2_s = correlative_scan_match(
            grid, cloud_base, (cx, cy, cth),
            xy_radius=xy_step,        xy_step=xy_step / 3.0,
            th_radius=th_step,        th_step=th_step / 3.0,
            score_floor=score_floor)
        # Stage 3 — fine
        f_x, f_y, f_th, f_score = correlative_scan_match(
            grid, cloud_base, (c2_x, c2_y, c2_th),
            xy_radius=xy_step / 3.0,  xy_step=grid.resolution,
            th_radius=th_step / 3.0,  th_step=math.radians(1.0),
            score_floor=score_floor)
        refined.append(((f_x, f_y, f_th), float(f_score)))

    refined.sort(key=lambda r: -r[1])
    return refined


def global_localization(grid, cloud_base,
                        xy_step=0.30, th_step_deg=8.0,
                        center=None,
                        score_floor=0.0):
    """
    Locate the robot inside a pre-loaded map *without* an initial pose
    guess.  Runs a 3-stage coarse-to-fine search:

      1.  Hyper-coarse:  whole map  × ±180°    (xy_step,    th_step)
      2.  Coarse:        ±xy_step   × ±th_step (xy_step/3,  th_step/3)
      3.  Fine:          ±xy_step/3 × ±th_step/3  (1 cell, ≈1°)

    Returns ((x, y, theta), score).  Total cost is ~2-3× one
    multi-resolution_csm call (in our environment, ~100–200 ms).
    The caller decides whether `score` is high enough to trust.
    """
    # Stage 1 — global scan
    half_w = (grid.width  * grid.resolution) / 2.0
    half_h = (grid.height * grid.resolution) / 2.0
    radius_xy = max(half_w, half_h)        # covers the whole map
    th_step   = math.radians(th_step_deg)
    if center is None:
        # Search centered on the map's centre (origin + half-extent).
        center = (grid.origin_x + half_w, grid.origin_y + half_h, 0.0)
    g_x, g_y, g_th, g_score = correlative_scan_match(
        grid, cloud_base, center,
        xy_radius=radius_xy, xy_step=xy_step,
        th_radius=math.pi,   th_step=th_step,
        score_floor=score_floor)

    # Stage 2 — refine ±xy_step / ±th_step around stage-1 winner
    c_x, c_y, c_th, c_score = correlative_scan_match(
        grid, cloud_base, (g_x, g_y, g_th),
        xy_radius=xy_step,        xy_step=xy_step / 3.0,
        th_radius=th_step,        th_step=th_step / 3.0,
        score_floor=score_floor)

    # Stage 3 — fine refinement, one-cell / ≈1° steps
    f_x, f_y, f_th, f_score = correlative_scan_match(
        grid, cloud_base, (c_x, c_y, c_th),
        xy_radius=xy_step / 3.0,  xy_step=grid.resolution,
        th_radius=th_step / 3.0,  th_step=math.radians(1.0),
        score_floor=score_floor)
    return (f_x, f_y, f_th), f_score


def transform_points(points, x, y, theta):
    """Transform points from robot/base frame to world frame."""
    c, s = math.cos(theta), math.sin(theta)
    R    = np.array([[c, -s], [s, c]])
    t    = np.array([x, y])
    return _apply_transform(points, R, t)


# ──────────────────────────────────────────────────────────────────
# Voxel downsampling
# ──────────────────────────────────────────────────────────────────

def voxel_downsample(points, voxel_size):
    """
    Reduce a 2D point cloud to one point per voxel of side `voxel_size`.

    The first point landing in each voxel is kept (cheaper than per-voxel
    means and just as good for ICP).  Used to bound the size of the map
    reference cloud at constant resolution regardless of map growth.
    """
    if len(points) == 0 or voxel_size <= 0.0:
        return points
    voxel_idx   = np.floor(points / voxel_size).astype(np.int64)
    keys        = voxel_idx[:, 0] * 1_000_003 + voxel_idx[:, 1]
    _, uniq     = np.unique(keys, return_index=True)
    return points[uniq]


# ──────────────────────────────────────────────────────────────────
# FFT polar correlation (rotation pre-alignment)
# ──────────────────────────────────────────────────────────────────

def polar_scan_correlation(source_pts, target_pts, n_bins=720,
                           min_range=0.1, max_range=10.0):
    """
    Estimate the rotation between two scans (both expressed in the same
    origin / robot frame) via circular cross-correlation of range-weighted
    polar-angle histograms.

    Returns dtheta in radians such that rotating `source_pts` by +dtheta
    aligns it angularly with `target_pts`.  Range-weighting biases the
    correlation toward distant points, which carry more rotational signal.
    """
    def polar_hist(pts):
        if len(pts) == 0:
            return np.zeros(n_bins)
        r    = np.hypot(pts[:, 0], pts[:, 1])
        a    = np.arctan2(pts[:, 1], pts[:, 0])
        mask = (r >= min_range) & (r <= max_range)
        if not mask.any():
            return np.zeros(n_bins)
        a    = a[mask]
        rr   = r[mask]
        bin_idx = ((a + math.pi) / (2.0 * math.pi) * n_bins).astype(np.int32) % n_bins
        return np.bincount(bin_idx, weights=rr, minlength=n_bins).astype(np.float64)

    h_s = polar_hist(source_pts)
    h_t = polar_hist(target_pts)
    if h_s.sum() == 0.0 or h_t.sum() == 0.0:
        return 0.0

    F_s  = np.fft.rfft(h_s)
    F_t  = np.fft.rfft(h_t)
    corr = np.fft.irfft(np.conj(F_s) * F_t, n=n_bins)
    best = int(np.argmax(corr))
    if best > n_bins // 2:
        best -= n_bins
    return best * (2.0 * math.pi / n_bins)


# ──────────────────────────────────────────────────────────────────
# Scan similarity (cheap change-detector)
# ──────────────────────────────────────────────────────────────────

def scan_diff(ranges_a, ranges_b, range_max=10.0):
    """
    RMS difference between two range vectors after clamping to `range_max`.
    Used as a pose-independent change-detector for the adaptive motion
    gate: if odom says we haven't moved but the scan has changed, we
    still want to refine pose / update the map (odom drift can hide
    real motion).
    """
    a = np.array(ranges_a, dtype=np.float64)
    b = np.array(ranges_b, dtype=np.float64)
    if len(a) != len(b):
        n = min(len(a), len(b))
        a, b = a[:n], b[:n]
    if len(a) == 0:
        return 0.0
    a = np.where(np.isfinite(a), np.clip(a, 0.0, range_max), range_max)
    b = np.where(np.isfinite(b), np.clip(b, 0.0, range_max), range_max)
    return float(np.sqrt(np.mean((a - b) ** 2)))


# ──────────────────────────────────────────────────────────────────
# ICP
# ──────────────────────────────────────────────────────────────────

def icp(source, target,
        init_dx=0.0, init_dy=0.0, init_dtheta=0.0,
        max_iter=20, tolerance=1e-4, reject_dist=0.3,
        min_points=10, use_point_to_line=False,
        normal_neighbors=8,
        target_tree=None, target_normals=None):
    """
    Align source cloud to target cloud.

    Parameters
    ----------
    source, target : (N,2) and (M,2) numpy arrays in the same frame.
    init_dx/dy/dtheta : initial guess (from odometry delta or FFT prealign).
    max_iter   : maximum ICP iterations.
    tolerance  : convergence threshold (per-iteration translation norm).
    reject_dist: drop point pairs further apart than this (m).
    min_points : minimum inlier pairs; returns guess if fewer.
    use_point_to_line : use linearised point-to-line solver (default False).
    normal_neighbors  : k for PCA normal estimation when point-to-line.
    target_tree       : pre-built cKDTree on `target` (avoids rebuild).
    target_normals    : pre-computed normals on `target` (avoids recompute).

    Returns
    -------
    (dx, dy, dtheta, fitness, converged)
        fitness  — RMS inlier distance after the last iteration.
        converged — True if the last iteration's |Δt|, |Δθ| < tolerance.
    """
    if len(source) < min_points or len(target) < min_points:
        return init_dx, init_dy, init_dtheta, float('inf'), False

    if _HAS_KDTREE and target_tree is None:
        target_tree = cKDTree(target)
    if use_point_to_line and target_normals is None:
        target_normals = estimate_normals(target, k=normal_neighbors,
                                          tree=target_tree)

    R_init, t_init = _pose_to_transform(init_dx, init_dy, init_dtheta)
    src     = _apply_transform(source.copy(), R_init, t_init)
    R_total = R_init.copy()
    t_total = t_init.copy()

    fitness   = float('inf')
    converged = False

    for _ in range(max_iter):
        idx, sq = _nearest_neighbors(src, target, target_tree=target_tree)

        mask = sq < reject_dist ** 2
        if mask.sum() < min_points:
            break

        src_m = src[mask]
        tgt_m = target[idx[mask]]

        if use_point_to_line and target_normals is not None:
            n_m = target_normals[idx[mask]]
            R, t = _best_fit_point_to_line(src_m, tgt_m, n_m)
        else:
            R, t = _best_fit_transform(src_m, tgt_m)

        src = _apply_transform(src, R, t)
        R_total = R @ R_total
        t_total = R @ t_total + t

        fitness = math.sqrt(sq[mask].mean())

        if (np.linalg.norm(t) < tolerance
                and abs(math.atan2(R[1, 0], R[0, 0])) < tolerance):
            converged = True
            break

    dx, dy, dtheta = _transform_to_pose(R_total, t_total)
    return dx, dy, dtheta, fitness, converged
