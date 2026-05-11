import math
import numpy as np


def _nearest_neighbors(source, target):
    """
    Brute-force nearest-neighbour search.
    Returns indices into target for each source point,
    and the corresponding squared distances.
    """
    # (N,1,2) - (1,M,2) → (N,M,2) → (N,M)
    diff  = source[:, np.newaxis, :] - target[np.newaxis, :, :]
    dists = np.sum(diff ** 2, axis=2)
    idx   = np.argmin(dists, axis=1)
    return idx, dists[np.arange(len(source)), idx]


def _best_fit_transform(A, B):
    """
    Compute the least-squares rigid transform that aligns A to B.
    A, B are (N,2) matched point sets.
    Returns (R 2x2, t 2-vector).
    """
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation (det = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B - R @ centroid_A
    return R, t


def _apply_transform(points, R, t):
    return (R @ points.T).T + t


def _transform_to_pose(R, t):
    """Convert (R, t) to (dx, dy, dtheta)."""
    dx     = t[0]
    dy     = t[1]
    dtheta = math.atan2(R[1, 0], R[0, 0])
    return dx, dy, dtheta


def _pose_to_transform(dx, dy, dtheta):
    """Convert (dx, dy, dtheta) to (R, t)."""
    c = math.cos(dtheta)
    s = math.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([dx, dy])
    return R, t


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   laser_x=0.0, laser_y=0.0, laser_yaw=0.0):
    """
    Convert a laser scan to a 2D point cloud expressed in the robot's base frame.

    (laser_x, laser_y, laser_yaw) is the static SE(2) offset of the LiDAR
    frame relative to the base frame (from TF).
    """
    angles  = angle_min + np.arange(len(ranges)) * angle_increment
    ranges  = np.array(ranges, dtype=np.float64)
    valid   = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r       = ranges[valid]
    a       = angles[valid]
    # Points in laser frame
    xl      = r * np.cos(a)
    yl      = r * np.sin(a)
    # Transform laser → base via the static offset
    cL, sL  = math.cos(laser_yaw), math.sin(laser_yaw)
    xs      = laser_x + cL * xl - sL * yl
    ys      = laser_y + sL * xl + cL * yl
    return np.column_stack((xs, ys))


def transform_points(points, x, y, theta):
    """Transform points from robot frame to world frame."""
    c, s = math.cos(theta), math.sin(theta)
    R    = np.array([[c, -s], [s, c]])
    t    = np.array([x, y])
    return _apply_transform(points, R, t)


def icp(source, target,
        init_dx=0.0, init_dy=0.0, init_dtheta=0.0,
        max_iter=20, tolerance=1e-4, reject_dist=0.3,
        min_points=10):
    """
    Align source cloud to target cloud.

    Parameters
    ----------
    source, target : (N,2) and (M,2) numpy arrays in the same frame
    init_dx/dy/dtheta : initial guess (from odometry delta)
    max_iter   : maximum ICP iterations
    tolerance  : convergence threshold (mean point shift)
    reject_dist: drop point pairs further apart than this (m)
    min_points : minimum inlier pairs; returns guess if fewer

    Returns
    -------
    (dx, dy, dtheta)  — total correction to apply on top of odometry
    fitness           — mean inlier distance after convergence (lower = better)
    converged         — bool
    """
    if len(source) < min_points or len(target) < min_points:
        return init_dx, init_dy, init_dtheta, float('inf'), False

    # Accumulate total transform
    R_total = np.eye(2)
    t_total = np.zeros(2)

    # Apply initial guess
    R_init, t_init = _pose_to_transform(init_dx, init_dy, init_dtheta)
    src = _apply_transform(source.copy(), R_init, t_init)
    R_total = R_init.copy()
    t_total = t_init.copy()

    fitness   = float('inf')
    converged = False

    for _ in range(max_iter):
        idx, sq_dists = _nearest_neighbors(src, target)

        # Reject distant pairs
        mask = sq_dists < reject_dist ** 2
        if mask.sum() < min_points:
            break

        src_matched = src[mask]
        tgt_matched = target[idx[mask]]

        R, t = _best_fit_transform(src_matched, tgt_matched)

        src = _apply_transform(src, R, t)

        # Accumulate
        R_total = R @ R_total
        t_total = R @ t_total + t

        fitness = math.sqrt(sq_dists[mask].mean())

        # Convergence check
        if np.linalg.norm(t) < tolerance and abs(math.atan2(R[1, 0], R[0, 0])) < tolerance:
            converged = True
            break

    dx, dy, dtheta = _transform_to_pose(R_total, t_total)
    return dx, dy, dtheta, fitness, converged
