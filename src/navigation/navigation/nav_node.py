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
from visualization_msgs.msg import Marker, MarkerArray

from navigation.map_io import load_map, load_waypoints
from navigation.a_star import plan as astar_plan

WorldPt = Tuple[float, float]


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

class _State:
    IDLE             = 'IDLE'
    FOLLOWING        = 'FOLLOWING'
    ALIGNING         = 'ALIGNING'         # in-place heading alignment at goal
    ARRIVED          = 'ARRIVED'
    WALL_FOLLOWING   = 'WALL_FOLLOWING'   # Bug1 phase 1
    RETURNING_LEAVE  = 'RETURNING_LEAVE'  # Bug1 phase 2


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
        self.declare_parameter('obstacle_distance',   0.50)   # front-arc stop threshold (m)
        self.declare_parameter('obstacle_angle_deg',  55.0)   # half-angle of front arc (deg)
        self.declare_parameter('wall_follow_dist',    0.45)   # target wall distance (m)
        self.declare_parameter('heading_tolerance',   0.12)   # rad (~7°)

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

        # Obstacle cooldown: ticks remaining before obstacle check is re-enabled
        # after a successful live-scan replan (prevents immediate re-trigger).
        self._obstacle_cooldown: int = 0

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

        px, py, theta = self._pose_x, self._pose_y, self._pose_theta

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

        if self._obstacle_cooldown > 0:
            self._obstacle_cooldown -= 1
        elif self._front_obstacle_detected():
            self._publish_stop()
            self.get_logger().info('Obstacle detected — replanning with live scan')
            if self._replan_from_current(px, py):
                # Give the robot time to turn away before the next obstacle check.
                self._obstacle_cooldown = 15  # 1.5 s at 10 Hz
            else:
                self.get_logger().warn('Replan failed — entering Bug1 wall follow')
                self._enter_wall_follow(px, py)
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

        omega = self._clamp(kp * heading_error, -wmax, wmax)
        speed = vmax * max(0.0, math.cos(heading_error)) * min(1.0, dist_to_goal / 0.5)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self._cmd_vel_pub.publish(cmd)

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

        omega = self._clamp(kp * heading_error, -wmax, wmax)
        speed = vmax * 0.7 * max(0.0, math.cos(heading_error)) * min(1.0, dist / 0.3)

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
    # Live-scan grid augmentation
    # ------------------------------------------------------------------

    def _grid_with_live_scan(self, rx: float, ry: float,
                              rtheta: float) -> np.ndarray:
        """Return a copy of the static map with current LiDAR hits marked occupied.

        This lets A* route around obstacles (like thin rack legs) that the LiDAR
        can see now but that weren't captured during the original mapping session.
        Only rays within range_max are used; rays beyond scan.range_max are ignored
        (open space — no new obstacle to mark).
        """
        scan = self._latest_scan
        if scan is None or self._grid is None:
            return self._grid

        augmented = self._grid.copy()
        h, w = augmented.shape
        res = self._resolution
        ox, oy = self._origin_x, self._origin_y

        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                wx = rx + r * math.cos(rtheta + angle)
                wy = ry + r * math.sin(rtheta + angle)
                cx = int((wx - ox) / res)
                cy = int((wy - oy) / res)
                if 0 <= cx < w and 0 <= cy < h:
                    augmented[cy, cx] = 100
            angle += scan.angle_increment

        return augmented

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
