"""
nav_node.py — Navigation node for the Puzzlebot AMR.

Loads a pre-saved map + waypoint file at startup (not from /map topic).
Accepts goal waypoint names on /goal_waypoint, runs A* immediately, then
follows the path with a pure-pursuit controller.

Obstacle avoidance: Bug1 algorithm.
  Phase 1 (WALL_FOLLOWING)   — right-hand wall follow; record leave point.
  Phase 2 (RETURNING_LEAVE)  — navigate to leave point; replan A* and resume.

Subscriptions
-------------
/slam_pose      (geometry_msgs/PoseStamped)  — pose from SLAM EKF (frame='map')
/goal_waypoint  (std_msgs/String)            — waypoint name, "" or "stop" to cancel
/scan           (sensor_msgs/LaserScan)      — obstacle detection / wall following

Publications
------------
/cmd_vel        (geometry_msgs/Twist)        — velocity commands
/plan           (nav_msgs/Path)              — planned path (frame='map')
/nav_status     (std_msgs/String)            — IDLE | PLANNING | FOLLOWING:<name>
                                               ARRIVED:<name> | ERROR:<reason>
                                               WALL_FOLLOWING:<name> | RETURNING_LEAVE:<name>
"""

from __future__ import annotations

import math
import os
import struct
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                        qos_profile_sensor_data)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from navigation.map_io import load_map, load_waypoints
from navigation.a_star import plan as astar_plan
from navigation.local_costmap import LocalCostmap

WorldPt = Tuple[float, float]


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

class _State:
    IDLE              = 'IDLE'
    FOLLOWING         = 'FOLLOWING'
    ALIGNING          = 'ALIGNING'          # in-place heading alignment at goal
    ARRIVED           = 'ARRIVED'
    WAITING_FOR_CLEAR = 'WAITING_FOR_CLEAR' # path blocked, waiting for dynamic obstacle
    WALL_FOLLOWING    = 'WALL_FOLLOWING'    # Bug1 phase 1
    RETURNING_LEAVE   = 'RETURNING_LEAVE'   # Bug1 phase 2


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class NavNode(Node):
    """Pure-pursuit + Bug1 obstacle avoidance navigation node."""

    def __init__(self) -> None:
        super().__init__('nav_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('map_yaml',            '~/ros2_maps/warehouse.yaml')
        self.declare_parameter('waypoints_yaml',      '~/ros2_maps/waypoints.yaml')
        self.declare_parameter('inflation_radius',    0.30)
        self.declare_parameter('linear_speed',        0.15)
        self.declare_parameter('angular_kp',          2.0)
        self.declare_parameter('angular_max',         1.2)
        self.declare_parameter('goal_tolerance',      0.20)
        self.declare_parameter('lookahead_distance',  0.40)
        self.declare_parameter('control_rate',        10.0)
        # Drive backwards when the lookahead point sits behind the
        # robot AND is close.  Lets the robot escape tight pockets
        # without sweeping its chassis through whatever forced the
        # tight turn.  reverse_max_dist caps how far we'll drive in
        # reverse before insisting on a forward maneuver.
        self.declare_parameter('allow_reverse',       True)
        self.declare_parameter('reverse_max_dist',    0.60)
        self.declare_parameter('obstacle_distance',   0.50)   # front-arc stop threshold (m)
        self.declare_parameter('obstacle_angle_deg',  55.0)   # half-angle of front arc (deg)
        self.declare_parameter('wall_follow_dist',    0.45)   # target wall distance (m)
        self.declare_parameter('heading_tolerance',   0.12)   # rad (~7°)

        # ── Local costmap (rolling LiDAR memory) ──────────────────────
        self.declare_parameter('local_costmap_radius',        2.0)
        self.declare_parameter('local_costmap_decay_tau',     0.7)
        self.declare_parameter('local_costmap_occ_threshold', 0.4)

        # ── Continuous path validity check ────────────────────────────
        self.declare_parameter('path_validity_lookahead', 3.0)
        self.declare_parameter('path_blocked_hysteresis', 3)

        # ── 360° safety bubble ────────────────────────────────────────
        self.declare_parameter('safety_bubble_radius', 0.25)

        # ── Dynamic-obstacle wait policy ──────────────────────────────
        self.declare_parameter('wait_for_clear_timeout', 5.0)

        # ── System mode (gates goal acceptance) ───────────────────────
        # When mode != NAVIGATION, /goal_waypoint commands are rejected with
        # an ERROR status so we never try to navigate while the operator is
        # actively mapping.
        self.declare_parameter('start_mode', 'navigation')
        sm = str(self.get_parameter('start_mode').value).lower()
        if sm not in ('mapping', 'navigation'):
            sm = 'navigation'
        self._system_mode = 'NAVIGATION' if sm == 'navigation' else 'MAPPING'

        map_yaml       = os.path.expanduser(self.get_parameter('map_yaml').value)
        waypoints_yaml = os.path.expanduser(self.get_parameter('waypoints_yaml').value)

        # ── Load map ────────────────────────────────────────────────────
        self.get_logger().info(f'Loading map from {map_yaml}')
        try:
            self._grid, self._origin_x, self._origin_y, self._resolution = \
                load_map(map_yaml)
            self.get_logger().info(
                f'Map loaded: {self._grid.shape[1]}x{self._grid.shape[0]} cells, '
                f'res={self._resolution} m/cell, '
                f'origin=({self._origin_x}, {self._origin_y})'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load map: {exc}')
            self._grid = None
            self._origin_x = -10.0
            self._origin_y = -10.0
            self._resolution = 0.05

        # ── Load waypoints ─────────────────────────────────────────────
        self.get_logger().info(f'Loading waypoints from {waypoints_yaml}')
        try:
            self._waypoints = load_waypoints(waypoints_yaml)
            self.get_logger().info(
                f'Waypoints loaded: {list(self._waypoints.keys())}'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load waypoints: {exc}')
            self._waypoints = {}

        # ── Navigation state ───────────────────────────────────────────
        self._pose_x: Optional[float] = None
        self._pose_y: Optional[float] = None
        self._pose_theta: Optional[float] = None
        self._latest_scan: Optional[LaserScan] = None

        self._path: List[WorldPt] = []
        self._state: str = _State.IDLE
        self._current_goal_name: str = ''
        self._arrived_timer: Optional[object] = None

        # Bug1 state
        self._q_hit: Optional[WorldPt] = None    # where we first hit the obstacle
        self._q_leave: Optional[WorldPt] = None  # closest-to-goal point found so far
        self._d_leave: float = math.inf           # distance from q_leave to goal
        self._wf_goal: Optional[WorldPt] = None  # goal coordinates when Bug1 started
        self._wf_left_vicinity: bool = False      # True once we've moved away from q_hit
        self._wf_start_time: float = 0.0          # monotonic time when wall-follow started

        # ── Local costmap (rolling LiDAR memory) ───────────────────────
        self._costmap = LocalCostmap(
            radius_m=self.get_parameter('local_costmap_radius').value,
            resolution=self._resolution if self._resolution else 0.05,
            decay_tau=self.get_parameter('local_costmap_decay_tau').value,
            occ_threshold=self.get_parameter('local_costmap_occ_threshold').value,
        )
        self._last_costmap_tick_time: Optional[float] = None

        # Path-validity hysteresis counter (consecutive blocked ticks)
        self._path_blocked_ticks: int = 0

        # WAITING_FOR_CLEAR bookkeeping
        self._wait_start_time: float = 0.0

        # ── QoS ────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Subscriptions ──────────────────────────────────────────────
        self.create_subscription(PoseStamped, '/slam_pose',     self._pose_cb, reliable_qos)
        self.create_subscription(String,      '/goal_waypoint', self._goal_cb, reliable_qos)
        self.create_subscription(LaserScan,   '/scan',          self._scan_cb, qos_profile_sensor_data)

        # /system_mode (latched, transient_local) — dashboard publishes mode
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            String, '/system_mode', self._system_mode_cb, mode_qos,
        )

        # Reload waypoints from disk (called by the dashboard after it
        # writes a new waypoint to waypoints.yaml).
        self.create_service(Trigger, '~/reload_waypoints', self._reload_waypoints_cb)

        # ── Publishers ─────────────────────────────────────────────────
        self._cmd_vel_pub  = self.create_publisher(Twist,       '/cmd_vel',          10)
        self._plan_pub     = self.create_publisher(Path,        '/plan',             10)
        self._status_pub   = self.create_publisher(String,      '/nav_status',       10)
        self._walls_pub    = self.create_publisher(PointCloud2, '/map_walls',         1)
        self._markers_pub  = self.create_publisher(MarkerArray, '/waypoint_markers',  1)

        self._publish_map_walls()
        self._publish_waypoint_markers()
        self.create_timer(5.0, self._publish_map_walls)
        self.create_timer(5.0, self._publish_waypoint_markers)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self._publish_status('IDLE')
        self.get_logger().info('NavNode ready.')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        # Feed the costmap immediately so the next control tick sees fresh data.
        if self._pose_x is not None and self._costmap is not None:
            self._costmap.recenter(self._pose_x, self._pose_y)
            self._costmap.integrate_scan(
                msg, self._pose_x, self._pose_y, self._pose_theta
            )

    def _reload_waypoints_cb(self, request, response):
        """Re-read waypoints.yaml from disk so newly-added waypoints become
        addressable without restarting the node.
        """
        try:
            path = os.path.expanduser(self.get_parameter('waypoints_yaml').value)
            self._waypoints = load_waypoints(path)
            self.get_logger().info(
                f'Waypoints reloaded: {list(self._waypoints.keys())}'
            )
            self._publish_waypoint_markers()
            response.success = True
            response.message = f'{len(self._waypoints)} waypoints loaded'
        except Exception as exc:
            response.success = False
            response.message = f'reload failed: {exc}'
        return response

    def _system_mode_cb(self, msg: String) -> None:
        """Track the current system mode; cancel navigation if we leave it."""
        mode = msg.data.strip().upper()
        if mode not in ('MAPPING', 'NAVIGATION'):
            return
        if mode == self._system_mode:
            return
        prev = self._system_mode
        self._system_mode = mode
        self.get_logger().info(f'system_mode: {prev} → {mode}')
        if mode != 'NAVIGATION' and self._state not in (
            _State.IDLE, _State.ARRIVED
        ):
            self.get_logger().info('Cancelling active navigation due to mode change.')
            self._cancel_navigation()

    def _pose_cb(self, msg: PoseStamped) -> None:
        pos = msg.pose.position
        ori = msg.pose.orientation
        self._pose_x = pos.x
        self._pose_y = pos.y
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self._pose_theta = math.atan2(siny_cosp, cosy_cosp)

    def _goal_cb(self, msg: String) -> None:
        name = msg.data.strip()

        if name in ('', 'stop'):
            self._cancel_navigation()
            return

        # Reject goals while system is in MAPPING mode — we don't want the
        # robot driving around autonomously while the operator is building
        # a map (could collide with un-mapped obstacles or interfere with SLAM).
        if self._system_mode != 'NAVIGATION':
            self.get_logger().warn(
                f'Goal "{name}" rejected: system_mode={self._system_mode}, '
                'expected NAVIGATION.'
            )
            self._publish_status(f'ERROR: not in navigation mode')
            return

        if name not in self._waypoints:
            self.get_logger().error(f'Unknown waypoint: "{name}"')
            self._publish_status(f'ERROR: unknown waypoint {name}')
            return

        wp = self._waypoints[name]
        goal_x, goal_y = wp['x'], wp['y']

        self.get_logger().info(f'Goal received: "{name}" → ({goal_x:.2f}, {goal_y:.2f})')

        if self._pose_x is None:
            self.get_logger().error('No pose available yet — cannot plan.')
            self._publish_status('ERROR: no pose')
            return

        if self._grid is None:
            self.get_logger().error('No map loaded — cannot plan.')
            self._publish_status('ERROR: no map')
            return

        self._publish_status('PLANNING')
        planning_grid = self._grid_with_live_scan(
            self._pose_x, self._pose_y, self._pose_theta
        )
        infl = self.get_parameter('inflation_radius').value
        path = astar_plan(
            planning_grid, self._origin_x, self._origin_y, self._resolution,
            self._pose_x, self._pose_y, goal_x, goal_y,
            inflation_radius=infl,
        )

        if path is None:
            self.get_logger().error(f'A* found no path to "{name}"')
            self._publish_status(f'ERROR: no path to {name}')
            self._state = _State.IDLE
            return

        self.get_logger().info(f'Path found: {len(path)} waypoints to "{name}"')
        self._path = path
        self._current_goal_name = name
        self._path_blocked_ticks = 0
        self._state = _State.FOLLOWING
        self._publish_status(f'FOLLOWING: {name}')

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Dispatches to per-state controllers at control_rate Hz."""
        self._publish_plan()

        if self._pose_x is None:
            return

        # Tick the local costmap (decay + recenter).  This runs in every
        # state so the costmap stays fresh even while waiting or wall-following.
        self._tick_costmap()

        px, py, theta = self._pose_x, self._pose_y, self._pose_theta

        # ── 360° safety bubble: emergency stop in any active driving state ──
        active_driving = self._state in (
            _State.FOLLOWING, _State.RETURNING_LEAVE, _State.WALL_FOLLOWING
        )
        if active_driving and self._safety_bubble_violated():
            self._publish_stop()
            self.get_logger().warn(
                'Safety bubble breached — emergency STOP, entering WAITING_FOR_CLEAR'
            )
            self._enter_waiting(px, py)
            return

        if self._state == _State.WAITING_FOR_CLEAR:
            self._waiting_update(px, py)
            return

        if self._state == _State.WALL_FOLLOWING:
            self._wall_follow_update(px, py)
            return

        if self._state == _State.ALIGNING:
            self._aligning_update(theta)
            return

        if self._state == _State.RETURNING_LEAVE:
            self._returning_leave_update(px, py, theta)
            return

        if self._state != _State.FOLLOWING:
            self._publish_stop()
            return

        # ── FOLLOWING ────────────────────────────────────────────────
        if not self._path:
            self._on_arrived()
            return

        fx, fy = self._path[-1]
        dist_to_goal = math.sqrt((fx - px) ** 2 + (fy - py) ** 2)

        goal_tol = self.get_parameter('goal_tolerance').value
        if dist_to_goal < goal_tol:
            self._on_arrived()
            return

        # Continuous path validity: scan the next N metres of the planned
        # path against the rolling costmap.  Hysteresis avoids flapping.
        if self._path_ahead_blocked(px, py):
            self._path_blocked_ticks += 1
        else:
            self._path_blocked_ticks = 0

        hysteresis = int(self.get_parameter('path_blocked_hysteresis').value)
        if self._path_blocked_ticks >= hysteresis:
            self._publish_stop()
            self.get_logger().info(
                f'Path blocked for {self._path_blocked_ticks} ticks — replanning'
            )
            self._path_blocked_ticks = 0
            if self._replan_from_current(px, py):
                return
            self.get_logger().warn(
                'Replan failed — entering WAITING_FOR_CLEAR (dynamic-obstacle wait)'
            )
            self._enter_waiting(px, py)
            return

        lookahead = self.get_parameter('lookahead_distance').value
        if dist_to_goal < lookahead:
            lx, ly = fx, fy
        else:
            lx, ly = self._find_lookahead(px, py, lookahead)

        heading_error = self._wrap_angle(math.atan2(ly - py, lx - px) - theta)

        kp   = self.get_parameter('angular_kp').value
        wmax = self.get_parameter('angular_max').value
        vmax = self.get_parameter('linear_speed').value

        # ── Reverse maneuver decision ──────────────────────────────
        # When the lookahead point is in the rear hemisphere AND close
        # to the robot, drive backwards instead of pirouetting 180°.
        # This handles the case where the robot is wedged near a wall
        # and the next leg of the plan curls back behind it.
        # Threshold: |heading_error| > π/2 + a small dead-band, and
        # lookahead dist below `reverse_max_dist` so we never drive
        # backwards across long open stretches (less precise control).
        allow_reverse    = self.get_parameter('allow_reverse').value
        reverse_max_dist = self.get_parameter('reverse_max_dist').value
        lookahead_dist   = math.hypot(lx - px, ly - py)
        use_reverse = (
            allow_reverse
            and abs(heading_error) > (math.pi / 2.0 + 0.1)
            and lookahead_dist <= reverse_max_dist
        )

        if use_reverse:
            ctrl_error = self._wrap_angle(heading_error - math.pi)
            sign = -1.0
        else:
            ctrl_error = heading_error
            sign = +1.0

        omega = self._clamp(kp * ctrl_error, -wmax, wmax)
        speed_mag = vmax * max(0.0, math.cos(ctrl_error)) * min(1.0, dist_to_goal / 0.5)
        speed = sign * speed_mag

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self._cmd_vel_pub.publish(cmd)

    # ------------------------------------------------------------------
    # WAITING_FOR_CLEAR — pause for a dynamic obstacle before falling back to Bug1
    # ------------------------------------------------------------------

    def _enter_waiting(self, px: float, py: float) -> None:
        """Stop the robot and start the wait-for-clear timer."""
        self._publish_stop()
        self._wait_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._path_blocked_ticks = 0
        self._state = _State.WAITING_FOR_CLEAR
        self._publish_status(f'WAITING_FOR_CLEAR: {self._current_goal_name}')
        self.get_logger().info(
            f'Pausing up to {self.get_parameter("wait_for_clear_timeout").value:.1f} s '
            'for path to clear before falling back to Bug1.'
        )

    def _waiting_update(self, px: float, py: float) -> None:
        """While waiting: retry replan each tick; on success resume, on
        timeout fall back to Bug1.
        """
        self._publish_stop()
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._wait_start_time
        timeout = self.get_parameter('wait_for_clear_timeout').value

        # Only retry replan if the immediate front is reasonably clear, so
        # we don't spam A* while a person is still inches from the LiDAR.
        bubble_clear = not self._safety_bubble_violated()
        if bubble_clear and self._replan_from_current(px, py):
            self.get_logger().info('Path cleared — resuming FOLLOWING.')
            return

        if elapsed >= timeout:
            self.get_logger().warn(
                f'Wait-for-clear timeout ({timeout:.1f} s) — entering Bug1 wall follow.'
            )
            self._enter_wall_follow(px, py)

    # ------------------------------------------------------------------
    # Bug1 — wall following (phase 1)
    # ------------------------------------------------------------------

    def _enter_wall_follow(self, px: float, py: float) -> None:
        """Start Bug1: record hit point, begin circumnavigation."""
        wp = self._waypoints.get(self._current_goal_name)
        if wp is None:
            self._state = _State.IDLE
            return
        gx, gy = wp['x'], wp['y']

        self._q_hit = (px, py)
        self._q_leave = (px, py)
        self._d_leave = math.sqrt((gx - px) ** 2 + (gy - py) ** 2)
        self._wf_goal = (gx, gy)
        self._wf_left_vicinity = False
        self._wf_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._state = _State.WALL_FOLLOWING
        self._publish_status(f'WALL_FOLLOWING: {self._current_goal_name}')
        self.get_logger().info(
            f'Bug1: hit obstacle at ({px:.2f}, {py:.2f}), starting circumnavigation'
        )

    def _wall_follow_update(self, px: float, py: float) -> None:
        """Bug1 phase 1: right-hand wall follow; track leave point."""
        scan = self._latest_scan
        if scan is None:
            self._publish_stop()
            return

        # Timeout: if stuck near hit point for >25 s, try replanning and give up.
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._wf_start_time
        if elapsed > 25.0 and not self._wf_left_vicinity:
            self.get_logger().warn(
                'Bug1: stuck near hit point for 25 s — attempting live-scan replan'
            )
            if not self._replan_from_current(px, py):
                self._publish_stop()
                self._publish_status(f'ERROR: Bug1 timeout {self._current_goal_name}')
                self._state = _State.IDLE
            return

        gx, gy = self._wf_goal

        # Update leave point (closest position to goal seen so far)
        d_to_goal = math.sqrt((gx - px) ** 2 + (gy - py) ** 2)
        if d_to_goal < self._d_leave:
            self._d_leave = d_to_goal
            self._q_leave = (px, py)

        # Detect when robot has moved away from hit point
        qhx, qhy = self._q_hit
        d_to_hit = math.sqrt((qhx - px) ** 2 + (qhy - py) ** 2)
        if not self._wf_left_vicinity and d_to_hit > 0.50:
            self._wf_left_vicinity = True

        # Full traversal: returned to vicinity of hit point
        if self._wf_left_vicinity and d_to_hit < 0.30:
            self.get_logger().info(
                f'Bug1: traversal complete. '
                f'Leave point: ({self._q_leave[0]:.2f}, {self._q_leave[1]:.2f}), '
                f'd_leave={self._d_leave:.2f}'
            )
            d_hit_to_goal = math.sqrt((gx - qhx) ** 2 + (gy - qhy) ** 2)
            if self._d_leave >= d_hit_to_goal - 0.05:
                self.get_logger().error('Bug1: obstacle surrounds goal — no path')
                self._publish_stop()
                self._publish_status(f'ERROR: Bug1 blocked {self._current_goal_name}')
                self._state = _State.IDLE
                return
            self._state = _State.RETURNING_LEAVE
            self._publish_status(f'RETURNING_LEAVE: {self._current_goal_name}')
            return

        v, omega = self._compute_wall_follow_cmd(scan)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self._cmd_vel_pub.publish(cmd)

    def _compute_wall_follow_cmd(self, scan: LaserScan) -> Tuple[float, float]:
        """Right-hand rule: keep obstacle on the right at wall_follow_dist."""
        obs_dist  = self.get_parameter('obstacle_distance').value
        wall_dist = self.get_parameter('wall_follow_dist').value
        v_max     = self.get_parameter('linear_speed').value
        w_max     = self.get_parameter('angular_max').value

        front       = self._range_at_angle(0.0,          scan, span=math.radians(30))
        front_right = self._range_at_angle(-math.pi / 4, scan, span=math.radians(20))
        right       = self._range_at_angle(-math.pi / 2, scan, span=math.radians(20))

        if front < obs_dist:
            # Blocked ahead — turn left in place
            return 0.0, w_max * 0.7

        if right > wall_dist * 2.0 and front_right > wall_dist * 1.5:
            # Wall disappeared on right (convex corner) — turn right to follow it
            return v_max * 0.5, -w_max * 0.5

        # Maintain lateral distance to right wall
        dist_error = wall_dist - right   # positive → too close → steer left
        kp_wall = 1.5
        omega = self._clamp(kp_wall * dist_error, -w_max, w_max)
        return v_max * 0.7, omega

    # ------------------------------------------------------------------
    # Bug1 — return to leave point (phase 2)
    # ------------------------------------------------------------------

    def _returning_leave_update(self, px: float, py: float, theta: float) -> None:
        """Bug1 phase 2: go to leave point then replan A*."""
        lx, ly = self._q_leave
        dx, dy = lx - px, ly - py
        dist = math.sqrt(dx ** 2 + dy ** 2)

        tol = self.get_parameter('goal_tolerance').value
        if dist < tol:
            self.get_logger().info('Bug1: arrived at leave point, replanning A*…')
            self._replan_from_current(px, py)
            return

        heading_error = self._wrap_angle(math.atan2(dy, dx) - theta)
        kp   = self.get_parameter('angular_kp').value
        wmax = self.get_parameter('angular_max').value
        vmax = self.get_parameter('linear_speed').value

        allow_reverse    = self.get_parameter('allow_reverse').value
        reverse_max_dist = self.get_parameter('reverse_max_dist').value
        use_reverse = (
            allow_reverse
            and abs(heading_error) > (math.pi / 2.0 + 0.1)
            and dist <= reverse_max_dist
        )
        if use_reverse:
            ctrl_error = self._wrap_angle(heading_error - math.pi)
            sign = -1.0
        else:
            ctrl_error = heading_error
            sign = +1.0

        omega = self._clamp(kp * ctrl_error, -wmax, wmax)
        speed = sign * vmax * 0.7 * max(0.0, math.cos(ctrl_error)) * min(1.0, dist / 0.3)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self._cmd_vel_pub.publish(cmd)

    def _replan_from_current(self, px: float, py: float) -> bool:
        """Run A* (with live scan overlay) from current position to current goal.

        Returns True and transitions to FOLLOWING if a path is found.
        Returns False and leaves state unchanged if no path exists.
        """
        if self._current_goal_name not in self._waypoints:
            self._state = _State.IDLE
            self._publish_status('IDLE')
            return False

        wp = self._waypoints[self._current_goal_name]
        gx, gy = wp['x'], wp['y']
        infl = self.get_parameter('inflation_radius').value

        planning_grid = self._grid_with_live_scan(px, py, self._pose_theta)
        path = astar_plan(
            planning_grid, self._origin_x, self._origin_y, self._resolution,
            px, py, gx, gy, inflation_radius=infl,
        )

        if path is None:
            self.get_logger().warn('Replan failed — no path in augmented grid')
            return False

        self.get_logger().info(f'Replanned path ({len(path)} pts), resuming FOLLOWING')
        self._path = path
        self._state = _State.FOLLOWING
        self._publish_status(f'FOLLOWING: {self._current_goal_name}')
        return True

    # ------------------------------------------------------------------
    # Local costmap helpers
    # ------------------------------------------------------------------

    def _tick_costmap(self) -> None:
        """Recenter on robot pose and apply temporal decay each control tick."""
        if self._pose_x is None or self._costmap is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_costmap_tick_time is None:
            dt = 1.0 / self.get_parameter('control_rate').value
        else:
            dt = max(0.0, now - self._last_costmap_tick_time)
        self._last_costmap_tick_time = now
        self._costmap.recenter(self._pose_x, self._pose_y)
        self._costmap.decay(dt)

    def _grid_with_live_scan(self, rx: float, ry: float,
                              rtheta: float) -> np.ndarray:
        """Return a copy of the static map with the rolling costmap overlaid.

        Unlike the old single-frame overlay this uses the decay-filtered
        local costmap, so transient blips don't poison the global plan and
        persistent obstacles (rack legs, unexpected boxes) do.
        """
        if self._grid is None:
            return self._grid
        if self._costmap is None:
            return self._grid.copy()

        augmented = self._grid.copy()
        h, w = augmented.shape
        res = self._resolution
        ox, oy = self._origin_x, self._origin_y

        # Project occupied costmap cells into the global grid.
        occ = self._costmap.occupied_mask()
        if not np.any(occ):
            return augmented

        cm_side = self._costmap.side_cells
        cm_ox, cm_oy = self._costmap.origin
        cm_res = self._costmap.resolution

        # Indices of occupied cells in the costmap
        ys, xs = np.where(occ)
        # World coords of those cells
        wxs = cm_ox + (xs + 0.5) * cm_res
        wys = cm_oy + (ys + 0.5) * cm_res
        # Map to global grid indices
        gx = ((wxs - ox) / res).astype(np.int32)
        gy = ((wys - oy) / res).astype(np.int32)
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        if np.any(inside):
            augmented[gy[inside], gx[inside]] = 100

        return augmented

    # ------------------------------------------------------------------
    # Safety bubble (360°)
    # ------------------------------------------------------------------

    def _safety_bubble_violated(self) -> bool:
        """True if any scan ray in 360° is closer than safety_bubble_radius."""
        scan = self._latest_scan
        if scan is None:
            return False
        radius = self.get_parameter('safety_bubble_radius').value
        for r in scan.ranges:
            if not math.isfinite(r):
                continue
            if scan.range_min < r < radius:
                return True
        return False

    # ------------------------------------------------------------------
    # Continuous path validity check
    # ------------------------------------------------------------------

    def _path_ahead_blocked(self, px: float, py: float) -> bool:
        """Return True if the planned path within `path_validity_lookahead`
        metres ahead crosses any occupied cell in the local costmap.
        """
        if not self._path or self._costmap is None:
            return False

        lookahead = self.get_parameter('path_validity_lookahead').value
        if lookahead <= 0.0:
            return False

        # Inflation in costmap cells, derived from inflation_radius parameter.
        infl_m = self.get_parameter('inflation_radius').value
        infl_cells = max(0, int(round(infl_m / self._costmap.resolution)))

        # Find the closest point on the path to the robot, then walk forward
        # along the path until we've covered `lookahead` metres.
        # Use the same projection logic as _find_lookahead but accumulate
        # segments and bresenham-test each one.
        best_seg = 0
        best_t = 0.0
        best_d2 = math.inf
        for i in range(len(self._path) - 1):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-9:
                continue
            t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx, cy = ax + t * dx, ay + t * dy
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_seg = i
                best_t = t

        ax, ay = self._path[best_seg]
        bx, by = self._path[best_seg + 1] if best_seg + 1 < len(self._path) else self._path[best_seg]
        cur_x = ax + best_t * (bx - ax)
        cur_y = ay + best_t * (by - ay)

        remaining = lookahead
        i = best_seg
        while remaining > 0.0 and i < len(self._path) - 1:
            nx, ny = self._path[i + 1]
            seg_len = math.hypot(nx - cur_x, ny - cur_y)
            if seg_len < 1e-6:
                i += 1
                continue

            if seg_len <= remaining:
                end_x, end_y = nx, ny
                step = seg_len
            else:
                frac = remaining / seg_len
                end_x = cur_x + frac * (nx - cur_x)
                end_y = cur_y + frac * (ny - cur_y)
                step = remaining

            if self._costmap.segment_blocked(
                cur_x, cur_y, end_x, end_y, inflation_cells=infl_cells
            ):
                return True

            cur_x, cur_y = end_x, end_y
            remaining -= step
            i += 1

        return False

    # ------------------------------------------------------------------
    # Pure pursuit helpers
    # ------------------------------------------------------------------

    def _find_lookahead(self, px: float, py: float, dist: float) -> WorldPt:
        """Project robot onto the nearest path segment, advance dist metres forward."""
        if not self._path:
            return (px, py)
        if len(self._path) == 1:
            return self._path[0]

        best_seg = 0
        best_t   = 0.0
        best_d2  = math.inf

        for i in range(len(self._path) - 1):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            dx, dy = bx - ax, by - ay
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < 1e-9:
                continue
            t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx, cy = ax + t * dx, ay + t * dy
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            if d2 < best_d2:
                best_d2  = d2
                best_seg = i
                best_t   = t

        ax, ay = self._path[best_seg]
        bx, by = self._path[best_seg + 1]
        seg_len   = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
        remaining = (1.0 - best_t) * seg_len

        if remaining >= dist:
            t_new = best_t + dist / seg_len if seg_len > 1e-9 else 1.0
            return (ax + t_new * (bx - ax), ay + t_new * (by - ay))

        cumulative = remaining
        for i in range(best_seg + 1, len(self._path) - 1):
            ax2, ay2 = self._path[i]
            bx2, by2 = self._path[i + 1]
            seg2 = math.sqrt((bx2 - ax2) ** 2 + (by2 - ay2) ** 2)
            if cumulative + seg2 >= dist:
                need = dist - cumulative
                frac = need / seg2 if seg2 > 1e-9 else 1.0
                return (ax2 + frac * (bx2 - ax2), ay2 + frac * (by2 - ay2))
            cumulative += seg2

        return self._path[-1]

    # ------------------------------------------------------------------
    # Obstacle detection helpers
    # ------------------------------------------------------------------

    def _front_obstacle_detected(self) -> bool:
        """True if any scan ray in the front arc is closer than obstacle_distance."""
        scan = self._latest_scan
        if scan is None:
            return False
        threshold  = self.get_parameter('obstacle_distance').value
        half_angle = math.radians(self.get_parameter('obstacle_angle_deg').value)
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) <= half_angle:
                if scan.range_min < r < threshold:
                    return True
            angle += scan.angle_increment
        return False

    def _range_at_angle(self, target_angle: float, scan: LaserScan,
                        span: float = 0.15) -> float:
        """Average of valid ranges within span radians of target_angle."""
        angle = scan.angle_min
        vals: List[float] = []
        for r in scan.ranges:
            if abs(angle - target_angle) <= span:
                if scan.range_min < r < scan.range_max:
                    vals.append(r)
            angle += scan.angle_increment
        return sum(vals) / len(vals) if vals else scan.range_max

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _on_arrived(self) -> None:
        """Position goal reached — start heading alignment if waypoint has theta."""
        self._publish_stop()
        wp = self._waypoints.get(self._current_goal_name)
        if wp and 'theta' in wp:
            self._state = _State.ALIGNING
            self._publish_status(f'ALIGNING: {self._current_goal_name}')
            self.get_logger().info(
                f'Position reached for "{self._current_goal_name}", '
                f'aligning to θ={math.degrees(wp["theta"]):.1f}°'
            )
        else:
            self._declare_arrived()

    def _aligning_update(self, theta: float) -> None:
        """Rotate in place until heading matches the waypoint's theta."""
        wp = self._waypoints.get(self._current_goal_name)
        if wp is None:
            self._declare_arrived()
            return

        target_theta  = wp['theta']
        heading_error = self._wrap_angle(target_theta - theta)
        tol = self.get_parameter('heading_tolerance').value

        if abs(heading_error) < tol:
            self._declare_arrived()
            return

        kp   = self.get_parameter('angular_kp').value
        wmax = self.get_parameter('angular_max').value
        omega = self._clamp(kp * heading_error, -wmax, wmax)

        # Ensure minimum angular speed so the robot doesn't stall near tolerance.
        min_omega = 0.15
        if abs(omega) < min_omega:
            omega = math.copysign(min_omega, omega)

        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = omega
        self._cmd_vel_pub.publish(cmd)

    def _declare_arrived(self) -> None:
        """Transition to ARRIVED and schedule return to IDLE."""
        self._publish_stop()
        name = self._current_goal_name
        self._state = _State.ARRIVED
        self._publish_status(f'ARRIVED: {name}')
        self.get_logger().info(f'Arrived at "{name}" with correct heading.')
        self._arrived_timer = self.create_timer(1.0, self._arrived_timeout)

    def _arrived_timeout(self) -> None:
        if self._arrived_timer is not None:
            self._arrived_timer.cancel()
            self._arrived_timer = None
        self._path = []
        self._state = _State.IDLE
        self._publish_status('IDLE')

    def _cancel_navigation(self) -> None:
        self._publish_stop()
        self._path = []
        self._state = _State.IDLE
        self._current_goal_name = ''
        self._path_blocked_ticks = 0
        self._publish_status('IDLE')
        self.get_logger().info('Navigation cancelled.')

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _publish_plan(self) -> None:
        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for wx, wy in self._path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._plan_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # Visualization helpers
    # ------------------------------------------------------------------

    def _publish_map_walls(self) -> None:
        if self._grid is None:
            return
        grid = self._grid
        h, w = grid.shape
        res  = self._resolution
        ox, oy = self._origin_x, self._origin_y
        buf = bytearray()
        for row in range(h):
            for col in range(w):
                if grid[row, col] == 100:
                    x = ox + (col + 0.5) * res
                    y = oy + (row + 0.5) * res
                    buf += struct.pack('fff', x, y, 0.0)
        msg = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height    = 1
        msg.width     = len(buf) // 12
        msg.fields    = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step   = 12
        msg.row_step     = len(buf)
        msg.data         = bytes(buf)
        msg.is_dense     = True
        self._walls_pub.publish(msg)

    def _publish_waypoint_markers(self) -> None:
        array = MarkerArray()
        now   = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action          = Marker.DELETEALL
        clear.header.frame_id = 'map'
        clear.header.stamp    = now
        array.markers.append(clear)
        for idx, (label, wp) in enumerate(self._waypoints.items()):
            x, y, theta = wp['x'], wp['y'], wp['theta']
            half = theta / 2.0
            qz, qw = math.sin(half), math.cos(half)

            arrow            = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp    = now
            arrow.ns, arrow.id    = 'wp_arrows', idx * 2
            arrow.type            = Marker.ARROW
            arrow.action          = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 0.08, 0.08
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.9, 0.3, 1.0
            array.markers.append(arrow)

            text             = Marker()
            text.header.frame_id = 'map'
            text.header.stamp    = now
            text.ns, text.id     = 'wp_labels', idx * 2 + 1
            text.type            = Marker.TEXT_VIEW_FACING
            text.action          = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z            = 0.25
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.text = label
            array.markers.append(text)
        self._markers_pub.publish(array)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
