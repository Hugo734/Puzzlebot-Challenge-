"""
bug1.py — Bug1 reactive obstacle avoidance for a differential-drive robot.

State machine:
    FREE              → robot drives straight toward goal
    CIRCUMNAVIGATING  → robot follows the wall of an obstacle, tracking the
                        point on the boundary closest to the goal
    BACK_ON_TRACK     → robot has found the minimum-distance point and is
                        returning to it before resuming straight-line travel

The update() method is called every control cycle.  It returns a Twist when
the robot must avoid an obstacle, or None when the path ahead is clear and
normal path-following should take over.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional

from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan


class _BugState(Enum):
    FREE = auto()
    CIRCUMNAVIGATING = auto()
    BACK_ON_TRACK = auto()


class Bug1:
    """
    Bug1 reactive obstacle avoider.

    Parameters
    ----------
    node:
        The owning rclpy Node (used for logging only).
    obstacle_threshold:
        Distance (m) at which a scan reading triggers obstacle detection.
    follow_distance:
        Desired gap (m) between the robot and the wall during circumnavigation.
    front_angle_deg:
        Half-angle (degrees) of the front sector used for obstacle detection.
        A value of 30 covers ±30° (60° total cone ahead).
    kp_wall:
        Proportional gain for the wall-follow PD controller.
    kd_wall:
        Derivative gain for the wall-follow PD controller.
    max_linear:
        Maximum linear speed (m/s) during wall following.
    max_angular:
        Maximum angular speed (rad/s).
    """

    def __init__(
        self,
        node,
        obstacle_threshold: float = 0.4,
        follow_distance: float = 0.3,
        front_angle_deg: float = 30.0,
        kp_wall: float = 1.8,
        kd_wall: float = 0.2,
        max_linear: float = 0.15,
        max_angular: float = 1.0,
    ) -> None:
        self._node = node
        self.obstacle_threshold = obstacle_threshold
        self.follow_distance = follow_distance
        self.front_angle_deg = front_angle_deg
        self.kp_wall = kp_wall
        self.kd_wall = kd_wall
        self.max_linear = max_linear
        self.max_angular = max_angular

        self._state: _BugState = _BugState.FREE

        # Point on the obstacle boundary where the robot first collided
        self._hit_point: Optional[tuple[float, float]] = None
        # Distance from hit point to goal
        self._hit_dist_to_goal: float = math.inf
        # Minimum distance to goal observed while circumnavigating
        self._min_dist_to_goal: float = math.inf
        # Grid cell (world coords) at which minimum was recorded
        self._min_point: Optional[tuple[float, float]] = None
        # Whether the robot has completed at least one pass and is returning
        self._returning_to_min: bool = False
        # Previous wall-follow error (for derivative term)
        self._prev_wall_error: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        scan: LaserScan,
        robot_pose: Pose,
        goal: Point,
    ) -> Optional[Twist]:
        """
        Decide whether obstacle avoidance is needed.

        Returns
        -------
        Twist
            A velocity command for obstacle avoidance, when active.
        None
            When the path ahead is clear — caller should use path follower.
        """
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        dist_to_goal = math.sqrt(
            (robot_x - goal.x) ** 2 + (robot_y - goal.y) ** 2
        )

        if self._state == _BugState.FREE:
            if self.is_obstacle_ahead(scan):
                self._state = _BugState.CIRCUMNAVIGATING
                self._hit_point = (robot_x, robot_y)
                self._hit_dist_to_goal = dist_to_goal
                self._min_dist_to_goal = dist_to_goal
                self._min_point = (robot_x, robot_y)
                self._returning_to_min = False
                self._prev_wall_error = 0.0
                self._node.get_logger().info(
                    f'Bug1: obstacle detected at ({robot_x:.2f},{robot_y:.2f}), '
                    f'circumnavigating (d_goal={dist_to_goal:.2f} m)'
                )
                return self.wall_follow_velocity(scan)
            return None

        if self._state == _BugState.CIRCUMNAVIGATING:
            # Track minimum-distance point
            if dist_to_goal < self._min_dist_to_goal:
                self._min_dist_to_goal = dist_to_goal
                self._min_point = (robot_x, robot_y)

            # Check if robot has gone around enough and is back near hit point
            if self._hit_point is not None:
                dist_to_hit = math.sqrt(
                    (robot_x - self._hit_point[0]) ** 2
                    + (robot_y - self._hit_point[1]) ** 2
                )
                # After travelling at least 0.5 m, if close to hit point →
                # full circumnavigation complete, head to min-dist point
                if dist_to_hit < 0.15 and dist_to_goal < self._hit_dist_to_goal:
                    self._state = _BugState.BACK_ON_TRACK
                    self._node.get_logger().info(
                        'Bug1: circumnavigation complete, returning to min-dist point '
                        f'({self._min_point[0]:.2f},{self._min_point[1]:.2f})'
                    )
                    return self._turn_toward(robot_pose, self._min_point)

            return self.wall_follow_velocity(scan)

        if self._state == _BugState.BACK_ON_TRACK:
            if self._min_point is None:
                self._state = _BugState.FREE
                return None

            dist_to_min = math.sqrt(
                (robot_x - self._min_point[0]) ** 2
                + (robot_y - self._min_point[1]) ** 2
            )
            if dist_to_min < 0.12:
                # Arrived at leave point — resume free travel
                self._state = _BugState.FREE
                self._hit_point = None
                self._min_point = None
                self._node.get_logger().info(
                    'Bug1: reached leave point, resuming free navigation'
                )
                return None

            # Still driving toward the min-dist point
            return self._turn_toward(robot_pose, self._min_point)

        return None

    def is_obstacle_ahead(self, scan: LaserScan) -> bool:
        """
        Return True if any scan reading within the forward cone is closer
        than *obstacle_threshold*.

        The forward cone spans ±front_angle_deg degrees centred on angle=0
        (directly ahead of the robot as reported by the LiDAR frame).
        """
        if scan is None or len(scan.ranges) == 0:
            return False

        n = len(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        threshold_rad = math.radians(self.front_angle_deg)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = angle_min + i * angle_inc
            # Normalise to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= threshold_rad:
                if r < self.obstacle_threshold:
                    return True
        return False

    def wall_follow_velocity(self, scan: LaserScan) -> Twist:
        """
        PD controller: maintain *follow_distance* from the nearest wall.

        The robot keeps the wall on its right side (classic Bug1 convention).
        The right side is approximately the 270°–360° / 0°–90° right half of
        the scan (angle range ≈ [-π/2, 0] in ROS convention where 0 is ahead).
        """
        right_dist = self._min_range_in_sector(
            scan, angle_min_deg=-90.0, angle_max_deg=-10.0
        )
        front_dist = self._min_range_in_sector(
            scan, angle_min_deg=-30.0, angle_max_deg=30.0
        )

        # Wall-follow error: positive → too far from wall, negative → too close
        wall_error = right_dist - self.follow_distance
        d_error = wall_error - self._prev_wall_error
        self._prev_wall_error = wall_error

        angular = self.kp_wall * (-wall_error) + self.kd_wall * (-d_error)
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # Slow down when something is close in front
        if front_dist < self.obstacle_threshold:
            linear = 0.0
            angular = self.max_angular  # turn left to avoid
        else:
            linear = self.max_linear

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def reset(self) -> None:
        """Reset the Bug1 state machine to FREE."""
        self._state = _BugState.FREE
        self._hit_point = None
        self._hit_dist_to_goal = math.inf
        self._min_dist_to_goal = math.inf
        self._min_point = None
        self._returning_to_min = False
        self._prev_wall_error = 0.0

    @property
    def is_active(self) -> bool:
        """True when Bug1 is actively avoiding an obstacle."""
        return self._state != _BugState.FREE

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _min_range_in_sector(
        self,
        scan: LaserScan,
        angle_min_deg: float,
        angle_max_deg: float,
    ) -> float:
        """Return the minimum valid range within an angular sector (degrees)."""
        if scan is None or len(scan.ranges) == 0:
            return self.obstacle_threshold * 2.0

        a_min = math.radians(angle_min_deg)
        a_max = math.radians(angle_max_deg)
        angle_inc = scan.angle_increment
        angle_start = scan.angle_min

        min_r = math.inf
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = angle_start + i * angle_inc
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if a_min <= angle <= a_max:
                min_r = min(min_r, r)

        return min_r if math.isfinite(min_r) else self.obstacle_threshold * 2.0

    def _robot_yaw(self, pose: Pose) -> float:
        """Extract yaw from a Pose quaternion."""
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _turn_toward(
        self,
        robot_pose: Pose,
        target: tuple[float, float],
    ) -> Twist:
        """Generate a Twist that turns the robot toward *target*."""
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        yaw = self._robot_yaw(robot_pose)

        dx = target[0] - rx
        dy = target[1] - ry
        desired_heading = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(desired_heading - yaw),
            math.cos(desired_heading - yaw),
        )

        twist = Twist()
        twist.linear.x = self.max_linear * max(0.0, math.cos(error))
        twist.angular.z = max(
            -self.max_angular, min(self.max_angular, self.kp_wall * error)
        )
        return twist
