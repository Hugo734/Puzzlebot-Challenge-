"""
navigation_node.py — ROS2 node that fuses A*, Bug1, and PID path-following
into an autonomous navigation stack for the Puzzlebot AMR.

Topic subscriptions:
    /robot_pose         (geometry_msgs/PoseStamped)  — EKF pose
    /map                (nav_msgs/OccupancyGrid)     — SLAM map
    /scan               (sensor_msgs/LaserScan)      — LiDAR
    /navigation/goal    (geometry_msgs/PoseStamped)  — requested destination

Publications:
    /cmd_vel            (geometry_msgs/Twist)        — velocity commands
    /navigation/status  (std_msgs/String)            — IDLE|NAVIGATING|ARRIVED|STUCK

Parameters (nav_params.yaml):
    goal_tolerance, waypoint_tolerance, inflation_radius,
    max_linear_vel, max_angular_vel,
    kp_angular, ki_angular, kd_angular, kp_linear,
    obstacle_threshold, obstacle_front_angle_deg,
    wall_follow_distance, control_frequency, stuck_timeout
"""

from __future__ import annotations

import json
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from navigation.astar import AStarPlanner
from navigation.bug1 import Bug1
from navigation.path_follower import PathFollower


class _NavStatus:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    ARRIVED = 'ARRIVED'
    STUCK = 'STUCK'


class NavigationNode(Node):
    """Autonomous navigation node: A* + Bug1 + PID for the Puzzlebot AMR."""

    def __init__(self) -> None:
        super().__init__('navigation_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('inflation_radius', 0.15)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.1)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('obstacle_front_angle_deg', 30.0)
        self.declare_parameter('wall_follow_distance', 0.3)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('stuck_timeout', 10.0)

        p = self._params()

        # ── Component instantiation ────────────────────────────────────
        self._follower = PathFollower(
            kp_angular=p['kp_angular'],
            ki_angular=p['ki_angular'],
            kd_angular=p['kd_angular'],
            kp_linear=p['kp_linear'],
            max_linear=p['max_linear_vel'],
            max_angular=p['max_angular_vel'],
            goal_tolerance=p['waypoint_tolerance'],
        )
        self._bug1 = Bug1(
            node=self,
            obstacle_threshold=p['obstacle_threshold'],
            follow_distance=p['wall_follow_distance'],
            front_angle_deg=p['obstacle_front_angle_deg'],
            max_linear=p['max_linear_vel'] * 0.5,
            max_angular=p['max_angular_vel'],
        )
        self._planner: Optional[AStarPlanner] = None

        # ── State ─────────────────────────────────────────────────────
        self._path: list[tuple[float, float]] = []
        self._waypoint_idx: int = 0
        self._status: str = _NavStatus.IDLE
        self._goal: Optional[PoseStamped] = None

        # Most-recent sensor data
        self._robot_pose: Optional[PoseStamped] = None
        self._map: Optional[OccupancyGrid] = None
        self._scan: Optional[LaserScan] = None

        # Stuck detection
        self._last_progress_time: Optional[float] = None
        self._last_progress_position: Optional[tuple[float, float]] = None
        self._stuck_timeout: float = p['stuck_timeout']
        self._progress_threshold: float = 0.05  # metres of movement = progress

        # Control loop period
        self._control_dt: float = 1.0 / p['control_frequency']

        # ── QoS ───────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self._pose_callback,
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            '/navigation/goal',
            self._goal_callback,
            reliable_qos,
        )

        # ── Publishers ────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)
        self._status_pub = self.create_publisher(String, '/navigation/status', 10)

        # ── Control timer ─────────────────────────────────────────────
        self.create_timer(self._control_dt, self._control_loop)

        self._publish_status(_NavStatus.IDLE)
        self.get_logger().info('NavigationNode started.')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _pose_callback(self, msg: PoseStamped) -> None:
        self._robot_pose = msg

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = msg
        # Invalidate planner so it will be rebuilt with new map data
        self._planner = None
        self.get_logger().debug(
            f'Map received: {msg.info.width}x{msg.info.height} cells, '
            f'res={msg.info.resolution:.3f} m/cell'
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan = msg

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )
        self._goal = msg
        self._start_planning()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _start_planning(self) -> None:
        """Run A* from current pose to goal and initialise path tracking."""
        if self._robot_pose is None:
            self.get_logger().warn('No robot pose available yet — cannot plan.')
            return
        if self._goal is None:
            return

        goal_x = self._goal.pose.position.x
        goal_y = self._goal.pose.position.y
        start_x = self._robot_pose.pose.position.x
        start_y = self._robot_pose.pose.position.y

        # If we have a map, use A*; otherwise go straight to goal
        if self._map is not None:
            planner = self._get_planner()
            path = planner.plan(
                start=(start_x, start_y),
                goal=(goal_x, goal_y),
            )
            if path is None:
                self.get_logger().warn(
                    'A* found no path to goal — will attempt direct approach.'
                )
                path = [(start_x, start_y), (goal_x, goal_y)]
        else:
            self.get_logger().warn(
                'No map available — using direct line to goal.'
            )
            path = [(start_x, start_y), (goal_x, goal_y)]

        # Ensure goal pose theta is the final approach heading
        self._path = path
        self._waypoint_idx = 0
        self._follower.reset()
        self._bug1.reset()

        # Initialise stuck detector
        self._last_progress_time = self.get_clock().now().nanoseconds * 1e-9
        self._last_progress_position = (start_x, start_y)

        self._publish_status(_NavStatus.NAVIGATING)
        self.get_logger().info(
            f'Path planned: {len(path)} waypoints, '
            f'first=({path[0][0]:.2f},{path[0][1]:.2f}), '
            f'last=({path[-1][0]:.2f},{path[-1][1]:.2f})'
        )

    def _get_planner(self) -> AStarPlanner:
        """Return (and cache) the A* planner for the current map."""
        if self._planner is not None:
            return self._planner

        m = self._map
        self._planner = AStarPlanner(
            occupancy_grid=list(m.data),
            resolution=m.info.resolution,
            origin_x=m.info.origin.position.x,
            origin_y=m.info.origin.position.y,
            width=m.info.width,
            height=m.info.height,
            inflation_radius=self.get_parameter('inflation_radius').value,
        )
        return self._planner

    # ------------------------------------------------------------------
    # Control loop (10 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Main 10 Hz callback: path following + Bug1 + stuck detection."""
        if self._status in (_NavStatus.IDLE, _NavStatus.ARRIVED, _NavStatus.STUCK):
            return
        if not self._path or self._robot_pose is None:
            return

        robot_pose = self._robot_pose.pose
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # ── Bug1 obstacle check ────────────────────────────────────────
        if self._scan is not None:
            goal_point = Point()
            final_wp = self._path[-1]
            goal_point.x = final_wp[0]
            goal_point.y = final_wp[1]

            bug_twist = self._bug1.update(self._scan, robot_pose, goal_point)
            if bug_twist is not None:
                self._cmd_vel_pub.publish(bug_twist)
                self._update_stuck_detector(robot_pose, current_time)
                return  # Bug1 is in control

        # ── Path follower ─────────────────────────────────────────────
        if self._waypoint_idx >= len(self._path):
            self._on_arrived()
            return

        waypoint = self._path[self._waypoint_idx]
        is_final = (self._waypoint_idx == len(self._path) - 1)

        # Use tighter tolerance for the final waypoint
        if is_final:
            self._follower.goal_tolerance = self.get_parameter(
                'goal_tolerance'
            ).value
        else:
            self._follower.goal_tolerance = self.get_parameter(
                'waypoint_tolerance'
            ).value

        twist, reached = self._follower.compute_velocity(
            robot_pose=robot_pose,
            waypoint=waypoint,
            dt=self._control_dt,
        )

        if reached:
            if is_final:
                self._on_arrived()
                return
            else:
                self._waypoint_idx += 1
                self._follower.reset()
                self.get_logger().debug(
                    f'Waypoint {self._waypoint_idx - 1} reached, '
                    f'advancing to {self._waypoint_idx}/{len(self._path) - 1}'
                )
                return

        self._cmd_vel_pub.publish(twist)
        self._update_stuck_detector(robot_pose, current_time)

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _on_arrived(self) -> None:
        """Robot has reached the final waypoint."""
        self._stop_robot()
        self._path = []
        self._waypoint_idx = 0
        self._bug1.reset()
        self._follower.reset()
        self._publish_status(_NavStatus.ARRIVED)
        self.get_logger().info('Goal reached — ARRIVED.')

    def _on_stuck(self) -> None:
        """Robot has made no progress within stuck_timeout seconds."""
        self._stop_robot()
        self._publish_status(_NavStatus.STUCK)
        self.get_logger().error(
            'No progress detected — robot is STUCK. '
            'Waiting for new goal or recovery command.'
        )

    def _stop_robot(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Stuck detection
    # ------------------------------------------------------------------

    def _update_stuck_detector(self, robot_pose, current_time: float) -> None:
        """Track position progress; declare STUCK if no movement for stuck_timeout."""
        rx = robot_pose.position.x
        ry = robot_pose.position.y

        if self._last_progress_position is None:
            self._last_progress_position = (rx, ry)
            self._last_progress_time = current_time
            return

        dist_moved = math.sqrt(
            (rx - self._last_progress_position[0]) ** 2
            + (ry - self._last_progress_position[1]) ** 2
        )

        if dist_moved > self._progress_threshold:
            # Progress made — reset timer
            self._last_progress_time = current_time
            self._last_progress_position = (rx, ry)
        else:
            elapsed = current_time - (self._last_progress_time or current_time)
            if elapsed > self._stuck_timeout:
                self._on_stuck()

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_status(self, status: str) -> None:
        if status == self._status and status not in (
            _NavStatus.NAVIGATING,
        ):
            return  # avoid duplicate transitions
        self._status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'Navigation status: {status}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _params(self) -> dict[str, float]:
        names = [
            'goal_tolerance', 'waypoint_tolerance', 'inflation_radius',
            'max_linear_vel', 'max_angular_vel',
            'kp_angular', 'ki_angular', 'kd_angular', 'kp_linear',
            'obstacle_threshold', 'obstacle_front_angle_deg',
            'wall_follow_distance', 'control_frequency', 'stuck_timeout',
        ]
        return {n: self.get_parameter(n).value for n in names}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
