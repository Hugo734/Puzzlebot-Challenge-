import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

_qos_scan = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid as OccupancyGridMsg
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from rclpy.duration import Duration

try:
    from scipy.spatial import cKDTree
    _HAS_KDTREE = True
except ImportError:
    _HAS_KDTREE = False

from slam.occupancy_grid import OccupancyGrid
from slam.icp import (
    scan_to_points, transform_points, icp,
    voxel_downsample, polar_scan_correlation, scan_diff,
)


# ──────────────────────────────────────────────────────────────────
# SE(2) helpers
# ──────────────────────────────────────────────────────────────────

def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _se2_compose(a, b):
    """Compose SE(2) transforms a ⊕ b. Both are (x, y, theta)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            _wrap(at + bt))


def _se2_inverse(a):
    """Inverse of SE(2) transform a = (x, y, theta)."""
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return (-c * ax - s * ay,
             s * ax - c * ay,
            _wrap(-at))


# ──────────────────────────────────────────────────────────────────
# SLAM node
# ──────────────────────────────────────────────────────────────────

class SLAMNode(Node):

    def __init__(self):
        super().__init__('slam_node')

        # ── Parameters: grid ──────────────────────────────────────
        self.declare_parameter('resolution',         0.05)
        self.declare_parameter('map_width',          400)
        self.declare_parameter('map_height',         400)
        self.declare_parameter('l_occ',              0.85)
        self.declare_parameter('l_free',            -0.40)
        self.declare_parameter('l_min',             -5.0)
        self.declare_parameter('l_max',              5.0)
        self.declare_parameter('display_l_occ',      1.0)
        self.declare_parameter('display_l_free',    -0.5)

        # ── Parameters: scan-matcher (ICP) ────────────────────────
        self.declare_parameter('use_icp',                 True)
        self.declare_parameter('icp_max_iter',            20)
        self.declare_parameter('icp_tolerance',           1e-4)
        self.declare_parameter('icp_reject_dist',         0.3)
        self.declare_parameter('icp_min_points',          20)
        self.declare_parameter('icp_max_fitness',         0.2)
        self.declare_parameter('icp_use_point_to_line',   True)
        self.declare_parameter('icp_normal_neighbors',    8)
        # Scan-to-map matcher: map cloud built from occupied cells
        self.declare_parameter('icp_map_radius',          6.0)   # m around robot
        # -1.0 means "use display_l_occ" — same threshold as the published map
        self.declare_parameter('icp_map_occ_threshold',   -1.0)
        self.declare_parameter('icp_map_voxel',           0.10)  # downsample voxel
        self.declare_parameter('icp_map_max_pts',         3000)  # hard cap on map cloud
        # Scan accumulation (fallback target when map is too sparse)
        self.declare_parameter('icp_scan_accum_n',        4)
        # Adaptive quality gate
        self.declare_parameter('icp_fitness_gate_factor', 3.0)
        self.declare_parameter('icp_fitness_history_n',   20)
        # Pose-jump rejection (per scan)
        self.declare_parameter('icp_jump_xy_max',         0.30)  # m
        self.declare_parameter('icp_jump_theta_max',      0.40)  # rad (~23°)
        # FFT polar pre-alignment
        self.declare_parameter('icp_fft_prealign',        True)
        self.declare_parameter('icp_fft_max_angle_deg',   30.0)
        # Recovery
        self.declare_parameter('recovery_bad_streak',     5)
        self.declare_parameter('pf_n_particles',          40)
        self.declare_parameter('pf_xy_spread',            0.20)
        self.declare_parameter('pf_theta_spread',         0.20)
        self.declare_parameter('pf_sigma',                0.15)

        # ── Parameters: scan / map gating ─────────────────────────
        self.declare_parameter('range_min',          0.12)
        self.declare_parameter('range_max',          10.0)
        self.declare_parameter('map_publish_every',  5)
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('ray_step',           1)
        self.declare_parameter('min_delta_xy',       0.02)
        self.declare_parameter('min_delta_theta',    0.01)
        self.declare_parameter('min_scan_diff',      0.05)  # m RMS; force update on big scene change
        # Map frame initial heading / laser yaw trim
        self.declare_parameter('laser_yaw_trim',     0.0)
        self.declare_parameter('map_initial_heading', 0.0)
        # TF smoothing — EMA on map→odom output
        self.declare_parameter('tf_smoothing_alpha', 0.6)

        res = self.get_parameter('resolution').value
        w   = self.get_parameter('map_width').value
        h   = self.get_parameter('map_height').value

        # ── Occupancy grid ────────────────────────────────────────
        self.grid = OccupancyGrid(
            width=w, height=h, resolution=res,
            l_occ=self.get_parameter('l_occ').value,
            l_free=self.get_parameter('l_free').value,
            l_min=self.get_parameter('l_min').value,
            l_max=self.get_parameter('l_max').value,
            display_l_occ=self.get_parameter('display_l_occ').value,
            display_l_free=self.get_parameter('display_l_free').value,
        )

        # ── State: SLAM / odom poses ──────────────────────────────
        self._sx = 0.0
        self._sy = 0.0
        self._stheta = 0.0

        self._ox = 0.0
        self._oy = 0.0
        self._otheta = 0.0
        self._odom_ready = False

        self._odom_buf = deque(maxlen=200)   # ~10 s of odom samples @ 20 Hz

        # map→odom (seeded with map_initial_heading so the world
        # frame is pre-rotated to room orientation at launch)
        self._tf_x     = 0.0
        self._tf_y     = 0.0
        self._tf_theta = self.get_parameter('map_initial_heading').value
        self._tf_initialised = False

        # Scan history for fallback target & FFT pre-alignment.
        # Each entry: {'world': (N,2), 'base': (N,2), 'ranges': (M,)}
        accum_n = max(1, int(self.get_parameter('icp_scan_accum_n').value))
        self._scan_buf = deque(maxlen=accum_n)
        self._last_ranges = None

        # ICP quality tracking
        hist_n = max(5, int(self.get_parameter('icp_fitness_history_n').value))
        self._fitness_history = deque(maxlen=hist_n)
        self._consecutive_bad = 0
        self._pf_active = False
        self._rng = np.random.default_rng()

        # Monotonic scan counter (NOT len(scan_buf), which saturates at maxlen
        # and would freeze the map publish cadence)
        self._scan_count = 0

        # Pose at which the grid was last updated — used for the
        # minimum-movement gate (stationary jitter suppression).
        self._last_map_x     = 0.0
        self._last_map_y     = 0.0
        self._last_map_theta = 0.0

        # TF: cached base_frame → laser_frame static offset (lazy-resolved)
        self.base_frame    = self.get_parameter('base_frame').value
        self._tf_buffer    = Buffer()
        self._tf_listener  = TransformListener(self._tf_buffer, self)
        self._laser_offset = None

        # ── ROS interfaces ────────────────────────────────────────
        self._map_pub  = self.create_publisher(OccupancyGridMsg, '/map', 1)
        self._pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)
        self._tf_br    = TransformBroadcaster(self)

        # 30 Hz map→odom heartbeat.  Higher than the scan rate so that
        # — even when a long scan callback delays the next timer fire —
        # the published TF never falls more than ~50 ms behind, well
        # under the scan_timestamp_fix offset.  Without this, RViz drops
        # /scan_fixed with "extrapolation into the future" errors.
        self.create_timer(1.0 / 30.0, self._tf_timer_cb)

        self.create_subscription(Odometry, '/odom',
                                 self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_callback, _qos_scan)

        if not _HAS_KDTREE:
            self.get_logger().warn(
                'scipy.spatial.cKDTree NOT available — falling back to '
                'brute-force NN search.  ICP will be much slower.')

        self.get_logger().info(
            f'SLAM node started — {w}x{h} cells @ {res} m/cell '
            f'({w*res:.1f} x {h*res:.1f} m) — '
            f'scan-to-map ICP, point-to-line={self.get_parameter("icp_use_point_to_line").value}')

    # ──────────────────────────────────────────────────────────────
    # TF heartbeat
    # ──────────────────────────────────────────────────────────────

    def _tf_timer_cb(self):
        self._publish_tf(self.get_clock().now().to_msg())

    # ──────────────────────────────────────────────────────────────
    # Odometry callback / interpolation
    # ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        self._ox     = msg.pose.pose.position.x
        self._oy     = msg.pose.pose.position.y
        self._otheta = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_ready = True

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom_buf.append((t, self._ox, self._oy, self._otheta))

    def _odom_at(self, t_query):
        """Linearly interpolate odom pose at ROS time `t_query` (float seconds)."""
        if not self._odom_buf:
            return self._ox, self._oy, self._otheta

        if t_query <= self._odom_buf[0][0]:
            _, x, y, th = self._odom_buf[0]
            return x, y, th
        if t_query >= self._odom_buf[-1][0]:
            _, x, y, th = self._odom_buf[-1]
            return x, y, th

        prev = self._odom_buf[0]
        for sample in self._odom_buf:
            if sample[0] >= t_query:
                t0, x0, y0, th0 = prev
                t1, x1, y1, th1 = sample
                a = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
                dth = _wrap(th1 - th0)
                return (x0 + a * (x1 - x0),
                        y0 + a * (y1 - y0),
                        _wrap(th0 + a * dth))
            prev = sample
        _, x, y, th = self._odom_buf[-1]
        return x, y, th

    # ──────────────────────────────────────────────────────────────
    # Laser → base TF resolution
    # ──────────────────────────────────────────────────────────────

    def _resolve_laser_offset(self, laser_frame):
        if self._laser_offset is not None:
            return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, laser_frame, Time(),
                timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw  = math.atan2(siny, cosy)
        trim = self.get_parameter('laser_yaw_trim').value
        yaw  = _wrap(yaw + trim)
        self._laser_offset = (float(t.x), float(t.y), float(yaw))
        self.get_logger().info(
            f'Laser→{self.base_frame} offset cached: '
            f'x={t.x:.3f} y={t.y:.3f} yaw={math.degrees(yaw):.1f}° '
            f'(trim={math.degrees(trim):.2f}°)')
        return True

    # ──────────────────────────────────────────────────────────────
    # Scan callback
    # ──────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        if not self._odom_ready:
            return
        if not self._resolve_laser_offset(msg.header.frame_id):
            self.get_logger().warn(
                f'Waiting for TF {self.base_frame}→{msg.header.frame_id}',
                throttle_duration_sec=2.0)
            return

        # ── Two odom samples we need: scan-time + latest ──────────
        scan_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        scan_ox, scan_oy, scan_otheta = self._odom_at(scan_t)

        lx, ly, lyaw = self._laser_offset
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value

        ray_step = self.get_parameter('ray_step').value
        if ray_step > 1:
            ranges_sub = np.array(msg.ranges, dtype=np.float64)[::ray_step]
            angle_inc  = msg.angle_increment * ray_step
        else:
            ranges_sub = np.array(msg.ranges, dtype=np.float64)
            angle_inc  = msg.angle_increment

        cloud = scan_to_points(
            ranges_sub, msg.angle_min, angle_inc,
            range_min, range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw)

        if len(cloud) < self.get_parameter('icp_min_points').value:
            return

        # ── Pose propagation: odom-derived initial guess ─────────
        self._propagate_from_odom()

        # ── Scan matcher (ICP) ───────────────────────────────────
        use_icp = self.get_parameter('use_icp').value
        if use_icp:
            self._run_scan_matcher(cloud, ranges_sub)

        # ── Recompute map→odom from refined SLAM pose ────────────
        new_tf = _se2_compose(
            (self._sx, self._sy, self._stheta),
            _se2_inverse((self._ox, self._oy, self._otheta)))

        # EMA smoothing on the TF to suppress per-scan jitter in
        # the displayed robot pose.  We then re-derive the SLAM
        # pose from the smoothed TF so /slam_pose and the TF stay
        # consistent — the smoothed pose is what the map is
        # updated with too, so the displayed robot icon and the
        # map remain registered.
        alpha = self.get_parameter('tf_smoothing_alpha').value
        if alpha < 1.0 and self._tf_initialised:
            self._tf_x = alpha * new_tf[0] + (1.0 - alpha) * self._tf_x
            self._tf_y = alpha * new_tf[1] + (1.0 - alpha) * self._tf_y
            dth = _wrap(new_tf[2] - self._tf_theta)
            self._tf_theta = _wrap(self._tf_theta + alpha * dth)
        else:
            self._tf_x, self._tf_y, self._tf_theta = new_tf
            self._tf_initialised = True

        # Re-derive SLAM pose from smoothed TF (keeps everything coherent)
        self._sx, self._sy, self._stheta = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (self._ox, self._oy, self._otheta))

        # ── Map update at scan-time SLAM pose ────────────────────
        scan_slam = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (scan_ox, scan_oy, scan_otheta))

        scan_dur = msg.scan_time if msg.scan_time > 0.0 else 0.1
        _, _, theta_end = self._odom_at(scan_t + scan_dur)
        scan_omega = (_wrap(theta_end - scan_otheta) / scan_dur
                      if scan_dur > 0 else 0.0)

        self._maybe_update_map(
            scan_slam, ranges_sub, msg.angle_min, angle_inc,
            range_min, range_max, lx, ly, lyaw,
            scan_omega, scan_dur)

        # ── Publish ──────────────────────────────────────────────
        now = self.get_clock().now().to_msg()
        self._publish_pose(now)
        self._publish_tf(now)

        self._scan_count += 1
        if self._scan_count % self.get_parameter('map_publish_every').value == 0:
            self._publish_map(now)

        # Cache current scan (world frame + base frame + raw ranges)
        # for the next iteration's fallback target and FFT prealign.
        cloud_world = transform_points(cloud, self._sx, self._sy, self._stheta)
        self._scan_buf.append({
            'world':  cloud_world,
            'base':   cloud,
            'ranges': np.array(ranges_sub, dtype=np.float64),
        })
        self._last_ranges = np.array(ranges_sub, dtype=np.float64)

    # ──────────────────────────────────────────────────────────────
    # Scan matcher: scan-to-map ICP w/ fallback to scan-to-scan
    # ──────────────────────────────────────────────────────────────

    def _build_target(self):
        """
        Choose ICP target:
          1. occupied-cell point cloud around the robot (scan-to-map)
          2. concatenation of last N scans in world frame (scan-to-scan fallback)
        Returns (target_points, mode_string) or (None, None) if no target.
        """
        min_pts = self.get_parameter('icp_min_points').value

        # Try scan-to-map first
        map_radius = self.get_parameter('icp_map_radius').value
        threshold  = self.get_parameter('icp_map_occ_threshold').value
        if threshold < 0.0:
            threshold = self.grid.display_l_occ
        map_pts = self.grid.get_occupied_points(
            robot_x=self._sx, robot_y=self._sy,
            radius=map_radius, threshold=threshold)

        voxel = self.get_parameter('icp_map_voxel').value
        if voxel > 0.0 and len(map_pts) > 0:
            map_pts = voxel_downsample(map_pts, voxel)

        max_pts = int(self.get_parameter('icp_map_max_pts').value)
        if max_pts > 0 and len(map_pts) > max_pts:
            sel = self._rng.choice(len(map_pts), max_pts, replace=False)
            map_pts = map_pts[sel]

        if len(map_pts) >= min_pts:
            return map_pts, 'map'

        # Fallback: scan-to-scan over accumulated history
        if len(self._scan_buf) > 0:
            stacked = np.vstack([s['world'] for s in self._scan_buf])
            if len(stacked) >= min_pts:
                return stacked, 'scan'

        return None, None

    def _run_scan_matcher(self, cloud, ranges_sub):
        target, mode = self._build_target()
        if target is None:
            return  # not enough reference geometry yet

        # ── Optional FFT pre-alignment (rotation init guess) ─────
        init_dtheta = 0.0
        if (self.get_parameter('icp_fft_prealign').value
                and len(self._scan_buf) > 0):
            prev_base = self._scan_buf[-1]['base']
            fft_dth   = polar_scan_correlation(
                cloud, prev_base, n_bins=720, max_range=10.0)
            max_fft = math.radians(
                self.get_parameter('icp_fft_max_angle_deg').value)
            if abs(fft_dth) <= max_fft:
                init_dtheta = fft_dth

        # ── ICP ──────────────────────────────────────────────────
        curr_world = transform_points(cloud, self._sx, self._sy, self._stheta)
        dx, dy, dtheta, fitness, converged = icp(
            source=curr_world,
            target=target,
            init_dx=0.0, init_dy=0.0, init_dtheta=init_dtheta,
            max_iter=self.get_parameter('icp_max_iter').value,
            tolerance=self.get_parameter('icp_tolerance').value,
            reject_dist=self.get_parameter('icp_reject_dist').value,
            min_points=self.get_parameter('icp_min_points').value,
            use_point_to_line=self.get_parameter('icp_use_point_to_line').value,
            normal_neighbors=self.get_parameter('icp_normal_neighbors').value,
        )

        # ── Quality gates ────────────────────────────────────────
        accept, reason = self._evaluate_icp(converged, fitness, dx, dy, dtheta)

        if accept:
            self._sx, self._sy, self._stheta = _se2_compose(
                (dx, dy, dtheta), (self._sx, self._sy, self._stheta))
            self._fitness_history.append(fitness)
            self._consecutive_bad = 0
            return

        # ── Rejected ─────────────────────────────────────────────
        self._consecutive_bad += 1
        self.get_logger().warn(
            f'ICP rejected ({reason}, mode={mode}): '
            f'converged={converged} fit={fitness:.3f} '
            f'Δ=({dx:+.2f}, {dy:+.2f}, {math.degrees(dtheta):+.1f}°) '
            f'bad_streak={self._consecutive_bad}',
            throttle_duration_sec=2.0)

        # ── Recovery: particle filter snap to best hypothesis ────
        # We always reset the streak after a PF attempt so the next
        # `recovery_bad_streak` scans contribute to the next decision
        # rather than triggering PF every single frame.
        bad_threshold = self.get_parameter('recovery_bad_streak').value
        if self._consecutive_bad >= bad_threshold and mode == 'map':
            applied = self._trigger_particle_recovery(cloud, target)
            if applied:
                self.get_logger().warn(
                    'Particle-filter recovery applied — pose snapped to '
                    f'best-of-{self.get_parameter("pf_n_particles").value}')
            self._consecutive_bad = 0

    def _evaluate_icp(self, converged, fitness, dx, dy, dtheta):
        """
        Return (accept_bool, reason_string).

        We deliberately do NOT require converged=True: with point-to-line
        ICP, the iteration often oscillates at sub-mm without ever hitting
        the per-iteration tolerance, even though the *fitness* is already
        excellent.  The fitness gate alone is the right acceptance test.
        """
        # Adaptive fitness gate — median of recent successful runs × factor,
        # floored at 0.15 m (3× grid resolution) so it never gets tighter
        # than the LiDAR's inherent range noise, and capped at icp_max_fitness.
        hard_max = self.get_parameter('icp_max_fitness').value
        factor   = self.get_parameter('icp_fitness_gate_factor').value
        if len(self._fitness_history) >= 5:
            adaptive = factor * float(np.median(self._fitness_history))
            gate = max(0.15, min(adaptive, hard_max))
        else:
            gate = hard_max
        if fitness > gate:
            return False, f'fitness {fitness:.3f} > gate {gate:.3f}'

        # Pose-jump rejection — ICP corrections larger than physically
        # plausible at 10 Hz are almost always a wrong association.
        jump_xy = self.get_parameter('icp_jump_xy_max').value
        jump_th = self.get_parameter('icp_jump_theta_max').value
        if math.hypot(dx, dy) > jump_xy:
            return False, f'xy-jump {math.hypot(dx, dy):.2f} > {jump_xy:.2f}'
        if abs(dtheta) > jump_th:
            return False, f'θ-jump {math.degrees(dtheta):.1f}° > {math.degrees(jump_th):.1f}°'

        return True, 'ok'

    # ──────────────────────────────────────────────────────────────
    # Particle filter recovery
    # ──────────────────────────────────────────────────────────────

    def _trigger_particle_recovery(self, cloud_base, map_pts):
        """
        Sample N pose hypotheses around the current SLAM estimate, score
        each against the local map, snap to the best.  Returns True if
        a non-identity particle won (i.e. we actually corrected something).
        """
        if not _HAS_KDTREE or len(map_pts) < 50:
            return False

        n     = max(8, int(self.get_parameter('pf_n_particles').value))
        sxy   = self.get_parameter('pf_xy_spread').value
        sth   = self.get_parameter('pf_theta_spread').value
        sigma = self.get_parameter('pf_sigma').value

        # Particle 0 is the current pose (so we never make things worse)
        dxs  = self._rng.normal(0.0, sxy, n); dxs[0]  = 0.0
        dys  = self._rng.normal(0.0, sxy, n); dys[0]  = 0.0
        dths = self._rng.normal(0.0, sth, n); dths[0] = 0.0

        tree = cKDTree(map_pts)

        best_score = -np.inf
        best_idx   = 0
        for i in range(n):
            x  = self._sx + dxs[i]
            y  = self._sy + dys[i]
            th = _wrap(self._stheta + dths[i])
            world_pts = transform_points(cloud_base, x, y, th)
            dists, _  = tree.query(world_pts, k=1)
            # Robust likelihood: sum of bounded Gaussian kernels
            score = float(np.sum(np.exp(-(dists ** 2) / (sigma ** 2))))
            if score > best_score:
                best_score = score
                best_idx   = i

        if best_idx == 0:
            return False

        self._sx     += dxs[best_idx]
        self._sy     += dys[best_idx]
        self._stheta  = _wrap(self._stheta + dths[best_idx])
        return True

    # ──────────────────────────────────────────────────────────────
    # Map update
    # ──────────────────────────────────────────────────────────────

    def _maybe_update_map(self, scan_slam, ranges_sub, angle_min, angle_inc,
                          range_min, range_max, lx, ly, lyaw,
                          scan_omega, scan_dur):
        """
        Apply min-motion + scan-change gate, then run the log-odds update.
        """
        sx, sy, sth = scan_slam
        min_xy    = self.get_parameter('min_delta_xy').value
        min_theta = self.get_parameter('min_delta_theta').value
        min_scan  = self.get_parameter('min_scan_diff').value

        moved = (
            math.hypot(sx - self._last_map_x, sy - self._last_map_y) >= min_xy
            or abs(_wrap(sth - self._last_map_theta)) >= min_theta
        )

        # Adaptive motion gate: even if odom hasn't moved much, force an
        # update when the scan itself changed significantly — protects
        # against silent odom drift / wheel slip.
        scene_changed = False
        if not moved and self._last_ranges is not None and min_scan > 0:
            scene_changed = (
                scan_diff(ranges_sub, self._last_ranges,
                          range_max=range_max) >= min_scan)

        if not (moved or scene_changed):
            return

        self.grid.update_scan(
            sx, sy, sth,
            ranges_sub, angle_min, angle_inc,
            range_min, range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw,
            scan_omega=scan_omega, scan_time=scan_dur)
        self._last_map_x     = sx
        self._last_map_y     = sy
        self._last_map_theta = sth

    # ──────────────────────────────────────────────────────────────
    # Odometry propagation
    # ──────────────────────────────────────────────────────────────

    def _propagate_from_odom(self):
        """SLAM pose ← map→odom ⊕ latest odom (full SE(2))."""
        self._sx, self._sy, self._stheta = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (self._ox, self._oy, self._otheta))

    # ──────────────────────────────────────────────────────────────
    # Publishers
    # ──────────────────────────────────────────────────────────────

    def _publish_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = self._sx
        msg.pose.position.y = self._sy
        msg.pose.position.z = 0.0
        half = self._stheta / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._pose_pub.publish(msg)

    def _publish_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'odom'
        tf.transform.translation.x = self._tf_x
        tf.transform.translation.y = self._tf_y
        tf.transform.translation.z = 0.0
        half = self._tf_theta / 2.0
        tf.transform.rotation.z = math.sin(half)
        tf.transform.rotation.w = math.cos(half)
        self._tf_br.sendTransform(tf)

    def _publish_map(self, stamp):
        msg = OccupancyGridMsg()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'

        msg.info.resolution = self.grid.resolution
        msg.info.width      = self.grid.width
        msg.info.height     = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.to_ros_data()
        self._map_pub.publish(msg)

        self.get_logger().info(
            f'Map published — pose=({self._sx:.2f}, {self._sy:.2f}, '
            f'{math.degrees(self._stheta):.1f}°)  '
            f'fit_hist_med={float(np.median(self._fitness_history)) if self._fitness_history else 0.0:.3f}',
            throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = SLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
