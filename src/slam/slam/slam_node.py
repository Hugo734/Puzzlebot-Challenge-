import math
import os
from collections import deque
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

_qos_scan = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid as OccupancyGridMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
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
    range_outlier_filter, multi_resolution_csm, global_localization,
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
        # Occupied-stop: cells with log-odds above this STOP the free-ray
        # DDA — i.e. a slight pose error never erases established walls.
        # -1 falls back to display_l_occ.
        self.declare_parameter('occupied_stop',     -1.0)

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
        # Threshold (in log-odds) above which a cell may serve as a
        # matcher anchor.  -1 falls back to display_l_occ.  Set
        # STRICTLY HIGHER than display_l_occ so the matcher only locks
        # onto cells with multi-scan agreement — low-evidence cells
        # written by an in-flight bad scan must NEVER attract the
        # next match, otherwise the error self-perpetuates.
        self.declare_parameter('icp_map_occ_threshold',   -1.0)
        self.declare_parameter('icp_map_voxel',           0.10)  # downsample voxel
        self.declare_parameter('icp_map_max_pts',         3000)  # hard cap on map cloud
        # Scan accumulation (fallback target when map is too sparse)
        self.declare_parameter('icp_scan_accum_n',        4)
        # Quality gate: fixed threshold (was adaptive — the rolling
        # median got dragged down by easy stationary matches, which
        # then rejected legitimately-higher-residual matches taken
        # while moving.  Fixed threshold is more predictable.)
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

        # Correlative Scan Matcher (coarse-to-fine grid search).
        # Two window sets:  NORMAL (every scan) and RECOVERY (used on
        # the first scan after the matcher was bypassed for high ω, to
        # absorb whatever drift accumulated during the bypass).
        self.declare_parameter('csm_enabled',             True)
        self.declare_parameter('csm_coarse_xy_radius',    0.40)
        self.declare_parameter('csm_coarse_xy_step',      0.10)
        self.declare_parameter('csm_coarse_th_radius',    0.10)   # ±5.7°
        self.declare_parameter('csm_coarse_th_step',      0.025)  # 1.4°
        self.declare_parameter('csm_fine_xy_radius',      0.10)
        self.declare_parameter('csm_fine_xy_step',        0.02)
        self.declare_parameter('csm_fine_th_radius',      0.025)  # ±1.4°
        self.declare_parameter('csm_fine_th_step',        0.005)  # 0.29°
        # Below this score (sum of log-odds at scan points) the map is
        # too sparse to trust CSM — fall back to odom prop only.
        self.declare_parameter('csm_min_score',           20.0)
        # Map must have at least this many occupied cells in the local
        # disc before CSM is considered useful.
        self.declare_parameter('csm_min_occupied_cells',  80)
        # RECOVERY window — wider, so the matcher can absorb a large
        # accumulated odom drift in one pass when it re-engages after
        # a bypass.  Kept at coarse resolution to stay real-time.
        self.declare_parameter('csm_recovery_xy_radius',   0.80)
        self.declare_parameter('csm_recovery_xy_step',     0.10)
        self.declare_parameter('csm_recovery_th_radius',   0.35)  # ±20°
        self.declare_parameter('csm_recovery_th_step',     0.035) # ~2°

        # Range outlier filter (RPLidar A1 spike rejection)
        self.declare_parameter('outlier_filter_enabled',  True)
        self.declare_parameter('outlier_max_jump',        0.5)

        # Rotation correction clamp — limits per-scan |Δθ_matcher|.
        # Always-on safety net: even if the matcher confidently proposes
        # a 30° correction, we cap it.  Clamp shrinks with the robot's
        # odom-reported angular velocity:
        #   high ω → trust wheel odom more (tighter clamp)
        #   low ω → allow larger correction so accumulated drift can
        #           be undone in one match
        # The CSM RECOVERY window can still apply big corrections — the
        # clamp is the LAST line of defence against a runaway match,
        # not a normal-operation gate.
        self.declare_parameter('rot_clamp_stationary_deg', 15.0)
        self.declare_parameter('rot_clamp_slow_deg',        8.0)
        self.declare_parameter('rot_clamp_fast_deg',        4.0)
        self.declare_parameter('rot_clamp_slow_omega',      0.20)  # rad/s
        self.declare_parameter('rot_clamp_fast_omega',      0.60)  # rad/s

        # ── Matcher omega bypass ─────────────────────────────────
        # When |ω| exceeds this, SKIP the matcher entirely for this
        # scan and ride pure odom.  At very high ω the per-ray deskew
        # is approximate (ω isn't actually constant within the scan
        # period) — running the matcher against a misaligned cloud
        # often locks onto a WRONG local minimum, and the resulting
        # bad pose then contaminates the map.  Riding odom for one or
        # two scans through a fast turn, then re-engaging the matcher
        # with the RECOVERY CSM window, is dramatically more stable
        # than fighting through.
        self.declare_parameter('matcher_omega_max',      2.0)  # rad/s

        # ── Localization mode ─────────────────────────────────────
        # Provide map_yaml to pre-populate the grid from a saved map so ICP
        # has structure from the very first scan.  Set initial_x/y/theta to
        # the robot's approximate starting position in the saved map frame,
        # or use RViz "2D Pose Estimate" → /initialpose to correct at runtime.
        #
        # When auto_localize is true AND a map_yaml is loaded, the node
        # runs a global scan-to-map search (full map × ±180°) on the first
        # scans and snaps the SLAM pose to the best-scoring hypothesis.
        # The robot can start anywhere in the saved map and find itself.
        self.declare_parameter('auto_localize',            True)
        self.declare_parameter('auto_localize_xy_step',    0.30)   # m
        self.declare_parameter('auto_localize_th_step_deg', 8.0)
        # Threshold for accepting the global-CSM result at startup.
        # Empirically real matches in this room score 250-450; bad
        # alignments score <150.  180 is in the safe gap.
        self.declare_parameter('auto_localize_min_score',  180.0)
        # Persistent retry: keep trying global localization until either
        # a scan clears the threshold OR (after this many tries) we snap
        # to the best hypothesis seen so far and let ICP refine.
        # 0 = never accept best-of-N (retry forever).
        self.declare_parameter('auto_localize_max_tries',           20)
        # Throttle between consecutive global-CSM attempts (seconds).
        # Each attempt is ~100–200 ms, so 2 s keeps CPU headroom.
        self.declare_parameter('auto_localize_retry_period',         2.0)
        self.declare_parameter('map_yaml',        '')
        self.declare_parameter('initial_x',       0.0)
        self.declare_parameter('initial_y',       0.0)
        self.declare_parameter('initial_theta',   0.0)
        # Initial mode: 'mapping' (write to grid) | 'navigation' (frozen)
        # Overridden at runtime by /system_mode subscription.
        self.declare_parameter('start_mode',      'navigation')

        # ── Parameters: scan / map gating ─────────────────────────
        self.declare_parameter('range_min',          0.12)
        self.declare_parameter('range_max',          10.0)
        self.declare_parameter('map_publish_every',  5)
        self.declare_parameter('base_frame',         'base_link')
        self.declare_parameter('ray_step',           1)
        self.declare_parameter('min_delta_xy',       0.02)
        self.declare_parameter('min_delta_theta',    0.01)
        self.declare_parameter('min_scan_diff',      0.05)  # m RMS; force update on big scene change
        # Skip log-odds integration when the robot is turning fast — the
        # per-ray deskew assumes ω is constant within the scan period,
        # which breaks down at high |ω| and bakes a sheared cloud into
        # the map.  Set to 0 to disable.  The scan matcher still runs
        # on every scan so the pose stays accurate; only the GRID write
        # is gated, which is what we want for crisp walls.
        self.declare_parameter('map_update_omega_max', 0.6)  # rad/s
        # Integration confidence gates.  Skip integration when:
        #   * matcher fitness is above integrate_max_fitness (tighter
        #     than icp_max_fitness — accept the pose, but don't write)
        #   * matcher xy / θ correction was larger than these limits,
        #     which means odom and the matcher disagreed a lot.  Big
        #     disagreement = the scan was probably deskewed with a
        #     wrong ω (because odom-ω was wrong by the same amount),
        #     so the scan shape itself is biased — writing it would
        #     smear walls.  Wait until things settle and write a crisp
        #     scan instead.
        self.declare_parameter('integrate_max_fitness',    0.10)  # m RMS
        self.declare_parameter('integrate_max_xy_corr',    0.10)  # m
        self.declare_parameter('integrate_max_th_corr',    0.10)  # rad (~5.7°)
        # Map frame initial heading / laser yaw trim
        self.declare_parameter('laser_yaw_trim',     0.0)
        self.declare_parameter('map_initial_heading', 0.0)
        # TF smoothing — EMA on map→odom output
        self.declare_parameter('tf_smoothing_alpha', 0.6)

        res = self.get_parameter('resolution').value
        w   = self.get_parameter('map_width').value
        h   = self.get_parameter('map_height').value

        # ── Callback groups: scan is expensive, keep it off the TF/odom thread
        self._scan_cbg = MutuallyExclusiveCallbackGroup()
        self._main_cbg = MutuallyExclusiveCallbackGroup()

        # ── Occupancy grid ────────────────────────────────────────
        occ_stop = self.get_parameter('occupied_stop').value
        if occ_stop is None or occ_stop < 0.0:
            occ_stop = self.get_parameter('display_l_occ').value
        self.grid = OccupancyGrid(
            width=w, height=h, resolution=res,
            l_occ=self.get_parameter('l_occ').value,
            l_free=self.get_parameter('l_free').value,
            l_min=self.get_parameter('l_min').value,
            l_max=self.get_parameter('l_max').value,
            display_l_occ=self.get_parameter('display_l_occ').value,
            display_l_free=self.get_parameter('display_l_free').value,
            occupied_stop=occ_stop,
        )

        # ── Load saved map into grid (localization mode) ─────────
        map_yaml = self.get_parameter('map_yaml').value
        if map_yaml:
            self._preload_map(os.path.expanduser(map_yaml))

        # Auto-localize state: pending only when a map was loaded AND
        # the user wants the global search.  Skipped during mapping
        # (no preloaded map) because in that case the SLAM frame is
        # defined by the robot's own starting position.
        self._auto_localize_pending = (
            bool(map_yaml)
            and self.get_parameter('auto_localize').value
        )
        self._auto_localize_tries = 0
        self._auto_localize_last_attempt = 0.0           # seconds (ROS time)
        # Best (x, y, th, score) seen so far, used as fallback after N tries.
        self._auto_localize_best: Optional[tuple] = None

        # Localization mode = freeze the grid (no new log-odds writes),
        # because in localization the saved map is fixed and any updates
        # would smear the carefully-built reference.  The matcher still
        # runs every scan (subject to the omega bypass) — that's how the
        # robot stays aligned to the saved map while driving.
        #
        # Initial value follows the `start_mode` parameter, but the node
        # also subscribes to /system_mode to switch at runtime.
        start_mode = str(self.get_parameter('start_mode').value).lower()
        if start_mode not in ('mapping', 'navigation'):
            self.get_logger().warn(
                f'Unknown start_mode "{start_mode}" — defaulting to navigation')
            start_mode = 'navigation'
        self._localization_mode = (start_mode == 'navigation')
        self.get_logger().info(
            f'SLAM start mode: {start_mode.upper()} '
            f'(localization_mode={self._localization_mode})')

        # ── State: SLAM / odom poses ──────────────────────────────
        self._sx = self.get_parameter('initial_x').value
        self._sy = self.get_parameter('initial_y').value
        self._stheta = (self.get_parameter('initial_theta').value
                        or self.get_parameter('map_initial_heading').value)

        self._ox = 0.0
        self._oy = 0.0
        self._otheta = 0.0
        self._omega = 0.0     # latest ω from /odom.twist.twist.angular.z
        self._lin_v = 0.0     # latest v   from /odom.twist.twist.linear.x
        self._odom_ready = False

        # Odom buffer entries: (t, x, y, theta, omega).  We buffer the
        # instantaneous ω so the per-scan deskew can interpolate it at
        # the scan midpoint instead of finite-differencing two poses
        # (which double-amplifies any odom jitter and lags behind the
        # true ω at the moment the scan was taken).
        self._odom_buf = deque(maxlen=200)   # ~10 s of odom samples @ 20 Hz

        # map→odom: computed from initial pose + odom=(0,0,0) at startup
        self._tf_x     = self._sx
        self._tf_y     = self._sy
        self._tf_theta = self._stheta
        self._tf_initialised = bool(map_yaml)

        # Scan history for fallback target & FFT pre-alignment.
        # Each entry: {'world': (N,2), 'base': (N,2), 'ranges': (M,)}
        accum_n = max(1, int(self.get_parameter('icp_scan_accum_n').value))
        self._scan_buf = deque(maxlen=accum_n)
        self._last_ranges = None

        # ICP quality tracking
        hist_n = max(5, int(self.get_parameter('icp_fitness_history_n').value))
        self._fitness_history = deque(maxlen=hist_n)
        self._consecutive_bad = 0
        self._rng = np.random.default_rng()

        # Monotonic scan counter (NOT len(scan_buf), which saturates at maxlen
        # and would freeze the map publish cadence)
        self._scan_count = 0

        # Matcher-bypass state: True for the scan AFTER one where we
        # skipped the matcher due to high ω, so _run_scan_matcher can
        # use the RECOVERY (wider) CSM window for one scan to absorb
        # the drift that accumulated during the bypass.
        self._matcher_recover = False

        # Last accepted match diagnostics — read by the integration
        # gate to decide whether the scan was crisp enough to write
        # into the grid.  A "large correction" means odom and matcher
        # disagreed a lot, which in turn means the scan was likely
        # deskewed with a wrong ω; integrating that scan smears walls.
        self._last_match_fitness   = 0.0
        self._last_match_xy_corr   = 0.0
        self._last_match_th_corr   = 0.0

        # ── Matcher-target cache ─────────────────────────────────
        # In navigation mode the map never changes — recomputing the
        # local map-cloud + KDTree + normals on every scan is pure
        # waste.  We cache them and invalidate only when (a) the map
        # was written to (mapping mode) or (b) the robot has moved
        # past half the local-disc radius from the cache centre, at
        # which point the cached cloud no longer covers the scan.
        self._cache_pts     = None
        self._cache_tree    = None
        self._cache_normals = None
        self._cache_mode    = None        # 'map' | 'scan' | None
        self._cache_rx      = 0.0
        self._cache_ry      = 0.0
        self._map_dirty     = True        # forces first build

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

        # 30 Hz map→odom heartbeat on the fast thread so the scan callback
        # (ICP, CSM) cannot block it.
        self.create_timer(1.0 / 30.0, self._tf_timer_cb,
                          callback_group=self._main_cbg)

        self.create_subscription(Odometry, '/odom',
                                 self._odom_callback, 10,
                                 callback_group=self._main_cbg)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_callback, _qos_scan,
                                 callback_group=self._scan_cbg)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._initialpose_cb, 10,
                                 callback_group=self._main_cbg)

        # /system_mode (latched): runtime switch between MAPPING and NAVIGATION.
        _qos_mode = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            StringMsg, '/system_mode', self._system_mode_cb, _qos_mode,
            callback_group=self._main_cbg,
        )

        # Service: clear the occupancy grid (used when the dashboard asks for
        # a fresh map after switching NAVIGATION → MAPPING).
        self.create_service(
            Trigger, '~/reset_map', self._reset_map_cb,
            callback_group=self._main_cbg,
        )

        # Service: re-arm the global auto-localizer.  Used by the dashboard
        # "Relocate" button (and CLI) when the user sees the laser is not
        # aligned to the saved map.  Equivalent to receiving /initialpose
        # except the search re-runs from scratch and the user doesn't have
        # to guess the pose — we just look again.
        self.create_service(
            Trigger, '~/relocalize', self._relocalize_cb,
            callback_group=self._main_cbg,
        )

        if not _HAS_KDTREE:
            self.get_logger().warn(
                'scipy.spatial.cKDTree NOT available — falling back to '
                'brute-force NN search.  ICP will be much slower.')

        self.get_logger().info(
            f'SLAM node started — {w}x{h} cells @ {res} m/cell '
            f'({w*res:.1f} x {h*res:.1f} m) — '
            f'scan-to-map ICP, point-to-line={self.get_parameter("icp_use_point_to_line").value}')

    # ──────────────────────────────────────────────────────────────
    # Map pre-loading (localization mode)
    # ──────────────────────────────────────────────────────────────

    def _preload_map(self, yaml_path: str) -> None:
        """Populate self.grid._log from a saved PGM+YAML map."""
        import yaml as _yaml
        try:
            with open(yaml_path) as f:
                meta = _yaml.safe_load(f)

            pgm = meta['image']
            if not os.path.isabs(pgm):
                pgm = os.path.join(os.path.dirname(yaml_path), pgm)

            with open(pgm, 'rb') as f:
                raw = f.read()

            # Parse PGM P5 header
            lines, i = [], 0
            while len(lines) < 3:
                end = raw.index(b'\n', i)
                line = raw[i:end].decode().strip()
                i = end + 1
                if not line.startswith('#'):
                    lines.append(line)
            pw, ph = map(int, lines[1].split())

            pixels = np.frombuffer(raw[i:i + pw * ph], dtype=np.uint8)
            pixels = pixels.reshape((ph, pw)).astype(np.float32)

            negate = int(meta.get('negate', 0))
            o_th   = float(meta.get('occupied_thresh', 0.65))
            f_th   = float(meta.get('free_thresh', 0.196))

            occ = 1.0 - pixels / 255.0 if not negate else pixels / 255.0
            occ = np.flipud(occ)   # PGM top=row0 → ROS bottom=row0

            lo = np.zeros((ph, pw), dtype=np.float32)
            lo[occ >= o_th] = float(self.grid.display_l_occ) * 2.0
            lo[occ <  f_th] = float(self.grid.display_l_free) * 2.0

            if pw == self.grid.width and ph == self.grid.height:
                self.grid._log[:] = np.clip(lo, self.grid.l_min, self.grid.l_max)
                self._map_dirty = True
                self.get_logger().info(
                    f'Pre-loaded saved map ({pw}×{ph}) into SLAM grid — '
                    f'localization mode active')
            else:
                self.get_logger().warn(
                    f'Saved map size ({pw}×{ph}) != SLAM grid '
                    f'({self.grid.width}×{self.grid.height}); '
                    'map preload skipped — starting empty')
        except Exception as exc:
            self.get_logger().error(f'_preload_map failed: {exc}')

    def _set_pose(self, x: float, y: float, theta: float) -> None:
        """Snap SLAM pose to (x, y, theta) and recompute map→odom."""
        self._sx      = x
        self._sy      = y
        self._stheta  = theta
        # map→odom such that: map_pose ⊕ odom = slam_pose
        self._tf_x, self._tf_y, self._tf_theta = _se2_compose(
            (x, y, theta),
            _se2_inverse((self._ox, self._oy, self._otheta)))
        self._tf_initialised = True
        self.get_logger().info(
            f'Pose set to ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)')

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """RViz "2D Pose Estimate" → snap SLAM to the given pose."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        # If a manual estimate arrives, treat auto-localize as resolved.
        self._auto_localize_pending = False
        self._set_pose(float(p.x), float(p.y), theta)

    def _system_mode_cb(self, msg: StringMsg) -> None:
        """Switch between MAPPING (writes the grid) and NAVIGATION (frozen)."""
        mode = msg.data.strip().upper()
        if mode not in ('MAPPING', 'NAVIGATION'):
            self.get_logger().warn(f'Ignoring unknown /system_mode: "{mode}"')
            return
        new_loc = (mode == 'NAVIGATION')
        if new_loc == self._localization_mode:
            return
        self._localization_mode = new_loc
        if new_loc:
            self.get_logger().info(
                'Switched to NAVIGATION: map frozen, matcher runs every scan.'
            )
        else:
            self.get_logger().info(
                'Switched to MAPPING: grid updates re-enabled.'
            )

    def _relocalize_cb(self, request, response):
        """Re-arm the global auto-localizer.

        Called by the dashboard "Relocate" button (or CLI) when the user
        sees the laser misaligned to the saved map.  The next scan
        callback will start a fresh 3-stage global search; subsequent
        scans keep retrying until it converges, falling back to the
        best-of-N pose after `auto_localize_max_tries`.
        """
        try:
            self._auto_localize_pending = True
            self._auto_localize_tries = 0
            self._auto_localize_best = None
            self._auto_localize_last_attempt = 0.0
            self.get_logger().info(
                'Relocalize requested — auto-localizer re-armed.')
            response.success = True
            response.message = 'auto-localize re-armed'
        except Exception as exc:
            response.success = False
            response.message = f'relocalize failed: {exc}'
        return response

    def _reset_map_cb(self, request, response):
        """Clear the occupancy grid back to "unknown" (used by the dashboard
        when the user chose to start mapping from scratch).
        """
        try:
            self.grid._log[:] = 0.0
            self._last_map_x = 0.0
            self._last_map_y = 0.0
            self._last_map_theta = 0.0
            self._map_dirty = True
            self.get_logger().info('Occupancy grid reset to empty (unknown).')
            response.success = True
            response.message = 'grid cleared'
        except Exception as exc:
            response.success = False
            response.message = f'reset failed: {exc}'
        return response

    def _try_auto_localize(self, cloud) -> bool:
        """
        Run a 3-stage coarse-to-fine global scan-to-map search and snap
        the SLAM pose to the winning hypothesis if its score clears
        `auto_localize_min_score`.  Returns True iff the pose was set.
        """
        xy_step    = self.get_parameter('auto_localize_xy_step').value
        th_step_d  = self.get_parameter('auto_localize_th_step_deg').value
        min_score  = self.get_parameter('auto_localize_min_score').value
        t0 = self.get_clock().now()
        (x, y, th), score = global_localization(
            self.grid, cloud,
            xy_step=xy_step, th_step_deg=th_step_d,
            score_floor=0.0)
        dt_ms = (self.get_clock().now() - t0).nanoseconds * 1e-6
        # Track best hypothesis across attempts so the throttled-retry loop
        # can fall back to it if the threshold is never cleared.
        if (self._auto_localize_best is None
                or score > self._auto_localize_best[3]):
            self._auto_localize_best = (float(x), float(y), float(th), float(score))

        if score < min_score:
            self.get_logger().warn(
                f'Auto-localize attempt {self._auto_localize_tries + 1}: '
                f'best pose=({x:+.2f}, {y:+.2f}, {math.degrees(th):+.1f}°)  '
                f'score={score:.1f} < {min_score:.1f}  ({dt_ms:.0f} ms)')
            return False
        self.get_logger().info(
            f'Auto-localized in {dt_ms:.0f} ms → pose=({x:+.2f}, {y:+.2f}, '
            f'{math.degrees(th):+.1f}°)  score={score:.1f}')
        self._set_pose(float(x), float(y), float(th))
        return True

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
        self._omega  = float(msg.twist.twist.angular.z)
        self._lin_v  = float(msg.twist.twist.linear.x)
        self._odom_ready = True

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom_buf.append(
            (t, self._ox, self._oy, self._otheta, self._omega))

        # Republish slam_pose at odom rate (20 Hz) so the RViz arrow
        # tracks the robot model between scan callbacks.
        if self._tf_initialised:
            self._propagate_from_odom()
            self._publish_pose(msg.header.stamp)

    def _odom_at(self, t_query):
        """Linearly interpolate odom pose at ROS time `t_query` (float seconds)."""
        if not self._odom_buf:
            return self._ox, self._oy, self._otheta

        if t_query <= self._odom_buf[0][0]:
            _, x, y, th, _ = self._odom_buf[0]
            return x, y, th
        if t_query >= self._odom_buf[-1][0]:
            _, x, y, th, _ = self._odom_buf[-1]
            return x, y, th

        prev = self._odom_buf[0]
        for sample in self._odom_buf:
            if sample[0] >= t_query:
                t0, x0, y0, th0, _ = prev
                t1, x1, y1, th1, _ = sample
                a = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
                dth = _wrap(th1 - th0)
                return (x0 + a * (x1 - x0),
                        y0 + a * (y1 - y0),
                        _wrap(th0 + a * dth))
            prev = sample
        _, x, y, th, _ = self._odom_buf[-1]
        return x, y, th

    def _compute_ray_pose_offsets(self, scan_t, scan_dur, n_rays):
        """
        Per-ray (dx, dy, dθ) of the robot at each ray's capture time,
        expressed in the SCAN-START base frame.

        Sampling: ray i is captured at fraction f_i = i / (n_rays-1)
        through the scan period.  We interpolate the odom buffer at
        each f_i to get the robot's world pose, then express the
        delta from scan-start in scan-start base coordinates.

        Returns three numpy arrays of length n_rays.  If the odom
        buffer is empty (warmup) returns zeros — falls back to
        no-deskew, which is correct at zero velocity.
        """
        if not self._odom_buf or n_rays < 1 or scan_dur <= 0.0:
            z = np.zeros(n_rays)
            return z, z.copy(), z.copy()

        buf = list(self._odom_buf)
        ts  = np.array([b[0] for b in buf])
        xs  = np.array([b[1] for b in buf])
        ys  = np.array([b[2] for b in buf])
        ths = np.unwrap(np.array([b[3] for b in buf]))  # for clean interp

        if n_rays == 1:
            fractions = np.array([0.5])
        else:
            fractions = np.linspace(0.0, 1.0, n_rays)
        qt = np.clip(scan_t + fractions * scan_dur, ts[0], ts[-1])

        xq  = np.interp(qt, ts, xs)
        yq  = np.interp(qt, ts, ys)
        thq = np.interp(qt, ts, ths)

        x0  = float(np.interp(scan_t, ts, xs))
        y0  = float(np.interp(scan_t, ts, ys))
        th0 = float(np.interp(scan_t, ts, ths))

        dxw = xq - x0
        dyw = yq - y0
        # Wrap to [-π, π] (interp on unwrapped is safe; the delta
        # between two unwrapped samples can still grow above π if the
        # robot literally spun multiple turns during the scan —
        # unlikely but cheap to guard).
        dth = (thq - th0 + np.pi) % (2.0 * np.pi) - np.pi

        # World deltas → scan-start base frame
        c, s = math.cos(th0), math.sin(th0)
        dxb =  c * dxw + s * dyw
        dyb = -s * dxw + c * dyw

        return dxb, dyb, dth

    def _omega_at(self, t_query):
        """
        Linearly interpolate the instantaneous angular velocity ω at ROS
        time `t_query` (float seconds).  Reading ω from the odom twist
        directly and interpolating to the scan midpoint produces a much
        better deskew rate than finite-differencing two poses, which
        was sensitive to odom quantisation noise and lagged the actual
        ω at fast turns.
        """
        if not self._odom_buf:
            return self._omega
        if t_query <= self._odom_buf[0][0]:
            return self._odom_buf[0][4]
        if t_query >= self._odom_buf[-1][0]:
            return self._odom_buf[-1][4]
        prev = self._odom_buf[0]
        for sample in self._odom_buf:
            if sample[0] >= t_query:
                t0, _, _, _, om0 = prev
                t1, _, _, _, om1 = sample
                a = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
                return om0 + a * (om1 - om0)
            prev = sample
        return self._odom_buf[-1][4]

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

        scan_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        scan_ox, scan_oy, scan_otheta = self._odom_at(scan_t)

        lx, ly, lyaw = self._laser_offset
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value

        # ── Range vector + outlier filter + sub-sampling ─────────
        ranges_raw = np.array(msg.ranges, dtype=np.float64)
        if self.get_parameter('outlier_filter_enabled').value:
            ranges_raw = range_outlier_filter(
                ranges_raw,
                max_jump=self.get_parameter('outlier_max_jump').value,
                range_max=range_max)

        ray_step = self.get_parameter('ray_step').value
        if ray_step > 1:
            ranges_sub = ranges_raw[::ray_step]
            angle_inc  = msg.angle_increment * ray_step
        else:
            ranges_sub = ranges_raw
            angle_inc  = msg.angle_increment

        # ── Scan timing + instantaneous ω at scan midpoint ───────
        scan_dur = msg.scan_time if msg.scan_time > 0.0 else 0.1
        scan_omega = self._omega_at(scan_t + 0.5 * scan_dur)

        # ── Per-ray pose offsets (FULL deskew) ───────────────────
        # Interpolates the odom buffer at each ray's timestamp and
        # returns (dx, dy, dθ) in scan-start base frame.  Used by
        # scan_to_points (matcher cloud) AND update_scan (map write)
        # so both the matcher and the integration see a coherent,
        # de-sheared cloud.  Costs ~3 ms for 360 rays.
        ray_dx, ray_dy, ray_dth = self._compute_ray_pose_offsets(
            scan_t, scan_dur, len(ranges_sub))

        # ── Build the deskewed point cloud (scan-start base frame) ─
        cloud = scan_to_points(
            ranges_sub, msg.angle_min, angle_inc,
            range_min, range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw,
            ray_dx=ray_dx, ray_dy=ray_dy, ray_dth=ray_dth)

        if len(cloud) < self.get_parameter('icp_min_points').value:
            return

        # ── Auto-localization (throttled retry, localization mode) ─
        # Keep retrying every `auto_localize_retry_period` seconds until a
        # scan clears the threshold; if max_tries is reached without
        # success, snap to the best-scoring hypothesis seen and let ICP
        # refine it. The user can also force a re-run via the
        # `~/relocalize` service or `/initialpose`.
        if self._auto_localize_pending:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            period = float(self.get_parameter('auto_localize_retry_period').value)
            if now_s - self._auto_localize_last_attempt >= period:
                self._auto_localize_last_attempt = now_s
                if self._try_auto_localize(cloud):
                    self._auto_localize_pending = False
                    self._auto_localize_tries = 0
                    self._auto_localize_best = None
                else:
                    self._auto_localize_tries += 1
                    max_tries = int(self.get_parameter('auto_localize_max_tries').value)
                    if (max_tries > 0
                            and self._auto_localize_tries >= max_tries
                            and self._auto_localize_best is not None):
                        bx, by, bth, bscore = self._auto_localize_best
                        self.get_logger().warn(
                            f'Auto-localize: threshold not cleared in '
                            f'{self._auto_localize_tries} tries — accepting '
                            f'best-of pose ({bx:+.2f}, {by:+.2f}, '
                            f'{math.degrees(bth):+.1f}°)  score={bscore:.1f}. '
                            f'ICP will refine from here.')
                        self._set_pose(bx, by, bth)
                        self._auto_localize_pending = False
                        self._auto_localize_tries = 0
                        self._auto_localize_best = None
            now = self.get_clock().now().to_msg()
            self._publish_pose(now)
            self._publish_tf(now)
            return

        # ── Pose propagation: odom-derived initial guess ─────────
        self._propagate_from_odom()
        pre_match_theta = self._stheta

        # ── Matcher omega gate ───────────────────────────────────
        # Above |ω|_max the deskew is approximate and the matcher
        # tends to lock onto wrong local minima.  Skip the matcher
        # for this scan, ride pure odom, and arm a RECOVERY pass for
        # the next-non-bypassed scan so it can absorb the drift in
        # one wider CSM step.
        omega_bypass = self.get_parameter('matcher_omega_max').value
        bypassed = (omega_bypass > 0.0 and abs(scan_omega) > omega_bypass)

        matcher_state = None
        if bypassed:
            self._matcher_recover = True
            self.get_logger().info(
                f'Matcher bypassed — |ω|={abs(scan_omega):.2f} > '
                f'{omega_bypass:.2f} rad/s (riding odom this scan)',
                throttle_duration_sec=1.0)
        elif self.get_parameter('use_icp').value:
            use_recovery = self._matcher_recover
            matcher_state = self._run_scan_matcher(
                cloud, ranges_sub, recovery=use_recovery)
            # The recovery window only fires for ONE scan after a
            # bypass; clear it whether the match was accepted or not.
            self._matcher_recover = False
            if matcher_state is True and use_recovery:
                self.get_logger().info(
                    'Matcher RECOVERY pass succeeded — pose snapped back')

        matcher_ok       = (matcher_state is True)
        matcher_rejected = (matcher_state is False)

        # ── Rotation correction clamp (always-on safety net) ─────
        if matcher_ok:
            self._clamp_rotation_correction(pre_match_theta, scan_omega)

        # ── Recompute map→odom from refined SLAM pose ────────────
        new_tf = _se2_compose(
            (self._sx, self._sy, self._stheta),
            _se2_inverse((self._ox, self._oy, self._otheta)))

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

        # ── Map update at scan-start SLAM pose ────────────────────
        scan_slam = _se2_compose(
            (self._tf_x, self._tf_y, self._tf_theta),
            (scan_ox, scan_oy, scan_otheta))

        # Don't integrate during a matcher bypass — the pose is
        # riding odom only and we have no scan-to-map confidence.
        if not bypassed:
            self._maybe_update_map(
                scan_slam, ranges_sub, msg.angle_min, angle_inc,
                range_min, range_max, lx, ly, lyaw,
                scan_omega, scan_dur,
                ray_dx=ray_dx, ray_dy=ray_dy, ray_dth=ray_dth,
                matcher_rejected=matcher_rejected)

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
    # Scan matcher: CSM (coarse-to-fine) + ICP refinement, scan-to-map
    # with scan-to-scan fallback when the map is still empty
    # ──────────────────────────────────────────────────────────────

    def _build_target(self):
        """
        Choose matcher target and return it ready-to-use:

            (points, mode, tree, normals)

        where `tree` is a cKDTree on `points` and `normals` are per-point
        2D PCA normals (None when scipy isn't available).

        We cache the cloud, the KDTree, and the normals — those three
        are the bulk of the per-scan CPU.  The cache is reused when:

          * the map hasn't been written to since the cache build, AND
          * the robot is still within  0.5 × icp_map_radius  of the
            cache centre (else the cached cloud no longer covers the
            scan and we'd be matching against stale geometry).

        In navigation mode (map frozen) both conditions hold for long
        stretches of motion → cache is essentially permanent inside its
        radius and the matcher cost drops from "rebuild every scan" to
        a single KDTree lookup per scan.
        """
        min_pts    = self.get_parameter('icp_min_points').value
        map_radius = float(self.get_parameter('icp_map_radius').value)

        # ── Cache hit ────────────────────────────────────────────
        # Distance from the cache centre to current SLAM pose.
        if (self._cache_pts is not None
                and self._cache_mode == 'map'
                and not self._map_dirty
                and math.hypot(self._sx - self._cache_rx,
                               self._sy - self._cache_ry) < 0.5 * map_radius):
            return (self._cache_pts, self._cache_mode,
                    self._cache_tree, self._cache_normals)

        # ── Cache miss → rebuild map cloud around robot ──────────
        threshold = self.get_parameter('icp_map_occ_threshold').value
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
            from slam.icp import estimate_normals
            tree = cKDTree(map_pts) if _HAS_KDTREE else None
            normals = (estimate_normals(
                            map_pts,
                            k=int(self.get_parameter('icp_normal_neighbors').value),
                            tree=tree)
                       if self.get_parameter('icp_use_point_to_line').value
                       else None)
            self._cache_pts     = map_pts
            self._cache_tree    = tree
            self._cache_normals = normals
            self._cache_mode    = 'map'
            self._cache_rx      = float(self._sx)
            self._cache_ry      = float(self._sy)
            self._map_dirty     = False
            return map_pts, 'map', tree, normals

        # ── Fallback to scan-to-scan when the map is too sparse ──
        # Scan stack changes every frame, so we DON'T cache here —
        # the buffer is tiny (≤ icp_scan_accum_n scans) anyway.
        if len(self._scan_buf) > 0:
            stacked = np.vstack([s['world'] for s in self._scan_buf])
            if len(stacked) >= min_pts:
                tree = cKDTree(stacked) if _HAS_KDTREE else None
                # No normals: scan fallback uses point-to-point ICP.
                # Invalidate any stale map-cloud cache.
                self._cache_pts = None
                self._cache_mode = 'scan'
                return stacked, 'scan', tree, None

        # Nothing to match against
        self._cache_pts = None
        self._cache_mode = None
        return None, None, None, None

    def _run_scan_matcher(self, cloud, ranges_sub, recovery=False):
        """
        Two-stage matcher operating around the current SLAM pose
        (self._sx/_sy/_stheta, already propagated from latest odom):
          1.  CSM coarse-to-fine — global search in a (xy, θ) window
              over the local map.  Skipped when the map is too sparse.
          2.  Point-to-line ICP refinement on top of CSM (or odom).

        recovery=True swaps the COARSE CSM window for the RECOVERY
        window — wider in both xy and θ.  Used for the first scan
        after the matcher was bypassed due to fast rotation, so it
        can absorb the drift that built up while we were riding odom.

        Modifies self._sx/_sy/_stheta in place when a match is accepted.
        Returns True iff the matcher produced a pose we trust enough to
        commit to /slam_pose AND integrate the scan into the map.
        """
        target, mode, target_tree, target_normals = self._build_target()
        if target is None:
            return None    # didn't run — caller treats as "warmup"

        # ── Stage 1: CSM (only when map is dense enough) ────────
        use_csm = (
            self.get_parameter('csm_enabled').value
            and mode == 'map'
            and len(target) >= self.get_parameter('csm_min_occupied_cells').value)

        csm_pose  = (self._sx, self._sy, self._stheta)
        csm_score = -np.inf
        if use_csm:
            if recovery:
                coarse = (
                    self.get_parameter('csm_recovery_xy_radius').value,
                    self.get_parameter('csm_recovery_xy_step').value,
                    self.get_parameter('csm_recovery_th_radius').value,
                    self.get_parameter('csm_recovery_th_step').value,
                )
            else:
                coarse = (
                    self.get_parameter('csm_coarse_xy_radius').value,
                    self.get_parameter('csm_coarse_xy_step').value,
                    self.get_parameter('csm_coarse_th_radius').value,
                    self.get_parameter('csm_coarse_th_step').value,
                )
            fine = (
                self.get_parameter('csm_fine_xy_radius').value,
                self.get_parameter('csm_fine_xy_step').value,
                self.get_parameter('csm_fine_th_radius').value,
                self.get_parameter('csm_fine_th_step').value,
            )
            csm_pose, csm_score = multi_resolution_csm(
                self.grid, cloud,
                init_pose=(self._sx, self._sy, self._stheta),
                coarse=coarse, fine=fine, score_floor=0.0)

            if csm_score < self.get_parameter('csm_min_score').value:
                csm_pose  = (self._sx, self._sy, self._stheta)
                use_csm   = False
            else:
                # Commit the CSM pose so the ICP refinement starts there.
                self._sx, self._sy, self._stheta = csm_pose

        # ── Stage 2: ICP refinement around CSM (or odom) ─────────
        # FFT prealign helps ONLY when no CSM ran (CSM already handles
        # large rotations; a second prealign would double-count it).
        init_dtheta = 0.0
        if (not use_csm
                and self.get_parameter('icp_fft_prealign').value
                and len(self._scan_buf) > 0):
            prev_base = self._scan_buf[-1]['base']
            fft_dth   = polar_scan_correlation(
                cloud, prev_base, n_bins=720, max_range=10.0)
            max_fft = math.radians(
                self.get_parameter('icp_fft_max_angle_deg').value)
            if abs(fft_dth) <= max_fft:
                init_dtheta = fft_dth

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
            target_tree=target_tree,
            target_normals=target_normals,
        )

        accept, reason = self._evaluate_icp(converged, fitness, dx, dy, dtheta)

        if accept:
            self._sx, self._sy, self._stheta = _se2_compose(
                (dx, dy, dtheta), (self._sx, self._sy, self._stheta))
            self._fitness_history.append(fitness)
            self._consecutive_bad = 0
            # Cache match diagnostics for the integration gate
            self._last_match_fitness = float(fitness)
            self._last_match_xy_corr = math.hypot(dx, dy)
            self._last_match_th_corr = abs(dtheta)
            return True

        # ── Rejected ─────────────────────────────────────────────
        self._consecutive_bad += 1
        self.get_logger().warn(
            f'Matcher rejected ({reason}, mode={mode}, csm_score={csm_score:.1f}): '
            f'fit={fitness:.3f} '
            f'Δicp=({dx:+.2f}, {dy:+.2f}, {math.degrees(dtheta):+.1f}°) '
            f'bad_streak={self._consecutive_bad}',
            throttle_duration_sec=2.0)

        bad_threshold = self.get_parameter('recovery_bad_streak').value
        if self._consecutive_bad >= bad_threshold and mode == 'map':
            applied = self._trigger_particle_recovery(
                cloud, target, target_tree=target_tree)
            if applied:
                self.get_logger().warn(
                    'Particle-filter recovery applied — pose snapped to '
                    f'best-of-{self.get_parameter("pf_n_particles").value}')
            self._consecutive_bad = 0

        return False

    def _evaluate_icp(self, converged, fitness, dx, dy, dtheta):
        """
        Return (accept_bool, reason_string).

        We deliberately do NOT require converged=True: with point-to-line
        ICP, the iteration often oscillates at sub-mm without ever hitting
        the per-iteration tolerance, even though the *fitness* is already
        excellent.  The fitness gate alone is the right acceptance test.

        We use a FIXED fitness gate (icp_max_fitness).  An earlier
        adaptive gate that tracked the median of recent fitness × a
        factor caused brittle behaviour: a long quiet stretch of easy
        stationary matches drove the gate to ~0.05, which then
        rejected legitimately-higher-residual matches the moment the
        robot started moving.  Fixed gate ≈ "what's a reasonable
        residual for THIS grid resolution?" (= 3-4× resolution).
        """
        gate = self.get_parameter('icp_max_fitness').value
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
    # Per-scan rotation correction clamp
    # ──────────────────────────────────────────────────────────────

    def _clamp_rotation_correction(self, pre_theta, scan_omega):
        """
        Limit |θ_matched − θ_odom_predicted| to a per-scan budget that
        depends on the current angular velocity.

        Tuning principle: a wheel-encoder ω of W rad/s integrated over
        one scan period (0.1 s) gives a rotation Wdt with uncertainty
        typically ≪ 1°.  Any matcher disagreement larger than the budget
        therefore reflects a bad scan-to-map registration rather than
        an actual odometry error, and we suppress it.

        We deliberately leave the x/y correction untouched — translation
        is usually well-resolved by the matcher even when rotation gets
        confused, and clamping x/y in lock-step would prevent legitimate
        position drift fixes.
        """
        omega_abs = abs(scan_omega)
        if omega_abs >= self.get_parameter('rot_clamp_fast_omega').value:
            max_deg = self.get_parameter('rot_clamp_fast_deg').value
        elif omega_abs >= self.get_parameter('rot_clamp_slow_omega').value:
            max_deg = self.get_parameter('rot_clamp_slow_deg').value
        else:
            max_deg = self.get_parameter('rot_clamp_stationary_deg').value
        max_dth = math.radians(max_deg)

        dth = _wrap(self._stheta - pre_theta)
        if abs(dth) > max_dth:
            clamped = math.copysign(max_dth, dth)
            self._stheta = _wrap(pre_theta + clamped)
            self.get_logger().info(
                f'Rotation clamp: {math.degrees(dth):+.1f}° → '
                f'{math.degrees(clamped):+.1f}° (|ω|={omega_abs:.2f} rad/s)',
                throttle_duration_sec=2.0)

    # ──────────────────────────────────────────────────────────────
    # Particle filter recovery
    # ──────────────────────────────────────────────────────────────

    def _trigger_particle_recovery(self, cloud_base, map_pts, target_tree=None):
        """
        Sample N pose hypotheses around the current SLAM estimate, score
        each against the local map, snap to the best.  Returns True if a
        non-identity particle won (i.e. we actually corrected something).
        """
        if not _HAS_KDTREE or len(map_pts) < 50:
            return False

        n     = max(8, int(self.get_parameter('pf_n_particles').value))
        sxy   = self.get_parameter('pf_xy_spread').value
        sth   = self.get_parameter('pf_theta_spread').value
        sigma = self.get_parameter('pf_sigma').value

        # Particle 0 is the current pose (so we never make things worse).
        dxs  = self._rng.normal(0.0, sxy, n); dxs[0]  = 0.0
        dys  = self._rng.normal(0.0, sxy, n); dys[0]  = 0.0
        dths = self._rng.normal(0.0, sth, n); dths[0] = 0.0

        tree = target_tree if target_tree is not None else cKDTree(map_pts)

        best_score = -np.inf
        best_idx   = 0
        for i in range(n):
            x  = self._sx + dxs[i]
            y  = self._sy + dys[i]
            th = _wrap(self._stheta + dths[i])
            world_pts = transform_points(cloud_base, x, y, th)
            dists, _  = tree.query(world_pts, k=1)
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
                          scan_omega, scan_dur,
                          ray_dx=None, ray_dy=None, ray_dth=None,
                          matcher_rejected=False):
        """
        Apply motion + confidence gates, then run the log-odds update.

        Localization mode short-circuits everything: the map is frozen.

        Confidence gate:
          - If the matcher ran and EXPLICITLY rejected the scan
            (matcher_rejected=True), we hold off on integration.
            Committing a scan at a wrong pose to the map is the loop
            that produces the rotating-walls effect: bad pose → bad map
            → worse match → worse pose.  Skipping breaks the loop.
          - If the matcher did NOT run (no target — typical during the
            warmup before the local map has any structure) we integrate
            anyway: this is how the map bootstraps from odometry.
            Previously we required matcher_confident=True here, which
            deadlocked the bootstrap because the matcher cannot run
            without a target and the map cannot grow without integration.
        """
        if self._localization_mode:
            return

        # ── Rotational-velocity gate ─────────────────────────────────
        # See declare_parameter docstring: skipping integration at high
        # |ω| is what prevents the "double walls" smear that appears
        # whenever the robot turns through a previously-mapped region.
        # The matcher above already ran and accepted the pose, so we
        # are NOT losing localisation — we're just declining to write
        # a sheared scan into a crisp map.
        omega_max = self.get_parameter('map_update_omega_max').value
        if omega_max > 0.0 and abs(scan_omega) > omega_max:
            self.get_logger().info(
                f'Map update skipped — |ω|={abs(scan_omega):.2f} > '
                f'{omega_max:.2f} rad/s (rotation too fast for clean integration)',
                throttle_duration_sec=2.0)
            return

        # ── Confidence gates ─────────────────────────────────────────
        # Don't write to the map when the last accepted match was
        # marginal (high fitness) or had to apply a big correction:
        # both are indicators that the scan is shaped wrong (slip,
        # bad deskew, partial occlusion, etc.) and integrating it
        # would smear walls.
        fit_max  = self.get_parameter('integrate_max_fitness').value
        xy_max   = self.get_parameter('integrate_max_xy_corr').value
        th_max   = self.get_parameter('integrate_max_th_corr').value
        if self._last_match_fitness > fit_max:
            self.get_logger().info(
                f'Map update skipped — match fitness {self._last_match_fitness:.3f} '
                f'> {fit_max:.3f}',
                throttle_duration_sec=2.0)
            return
        if self._last_match_xy_corr > xy_max:
            self.get_logger().info(
                f'Map update skipped — xy-correction {self._last_match_xy_corr:.2f} m '
                f'> {xy_max:.2f} m (odom/matcher disagreement)',
                throttle_duration_sec=2.0)
            return
        if self._last_match_th_corr > th_max:
            self.get_logger().info(
                f'Map update skipped — θ-correction '
                f'{math.degrees(self._last_match_th_corr):.1f}° '
                f'> {math.degrees(th_max):.1f}° (odom/matcher disagreement)',
                throttle_duration_sec=2.0)
            return

        sx, sy, sth = scan_slam
        min_xy    = self.get_parameter('min_delta_xy').value
        min_theta = self.get_parameter('min_delta_theta').value
        min_scan  = self.get_parameter('min_scan_diff').value

        moved = (
            math.hypot(sx - self._last_map_x, sy - self._last_map_y) >= min_xy
            or abs(_wrap(sth - self._last_map_theta)) >= min_theta
        )

        scene_changed = False
        if not moved and self._last_ranges is not None and min_scan > 0:
            scene_changed = (
                scan_diff(ranges_sub, self._last_ranges,
                          range_max=range_max) >= min_scan)

        if not (moved or scene_changed):
            return

        if matcher_rejected:
            self.get_logger().warn(
                'Map update skipped — matcher rejected this scan',
                throttle_duration_sec=2.0)
            return

        self.grid.update_scan(
            sx, sy, sth,
            ranges_sub, angle_min, angle_inc,
            range_min, range_max,
            laser_x=lx, laser_y=ly, laser_yaw=lyaw,
            scan_omega=scan_omega, scan_time=scan_dur,
            ray_dx=ray_dx, ray_dy=ray_dy, ray_dth=ray_dth)
        self._last_map_x     = sx
        self._last_map_y     = sy
        self._last_map_theta = sth
        # Map mutated → cached target cloud / KDTree / normals are stale.
        self._map_dirty      = True

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
    # Two threads: scan callback (ICP/CSM, slow) + TF/odom heartbeat (fast).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
