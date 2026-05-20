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
                   laser_x=0.0, laser_y=0.0, laser_yaw=0.0):
    """Project a laser scan into the robot base frame (laser offset applied)."""
    angles  = angle_min + np.arange(len(ranges)) * angle_increment
    ranges  = np.array(ranges, dtype=np.float64)
    valid   = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r       = ranges[valid]
    a       = angles[valid]
    xl      = r * np.cos(a)
    yl      = r * np.sin(a)
    cL, sL  = math.cos(laser_yaw), math.sin(laser_yaw)
    xs      = laser_x + cL * xl - sL * yl
    ys      = laser_y + sL * xl + cL * yl
    return np.column_stack((xs, ys))


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
