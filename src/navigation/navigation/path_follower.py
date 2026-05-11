"""
path_follower.py — PID heading follower for a differential-drive robot.

The follower implements a pure-pursuit-style controller:
  1. Compute the heading error (angle to waypoint minus robot yaw).
  2. PID on heading error → angular velocity.
  3. Linear velocity proportional to alignment (cosine of heading error),
     clamped so the robot slows when turning sharply.
  4. Declare the waypoint reached when within *goal_tolerance* metres.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Pose, Twist


class PathFollower:
    """
    PID heading follower for a single waypoint at a time.

    Parameters
    ----------
    kp_angular, ki_angular, kd_angular:
        PID gains for the heading error.
    kp_linear:
        Proportional gain scaling the cosine alignment into linear speed.
    max_linear:
        Hard cap on linear velocity (m/s).
    max_angular:
        Hard cap on angular velocity (rad/s).
    goal_tolerance:
        Distance (m) at which the waypoint is considered reached.
    heading_tolerance:
        Heading error (rad) below which linear velocity is allowed.
        Above this threshold the robot turns in place before moving forward.
    """

    def __init__(
        self,
        kp_angular: float = 1.5,
        ki_angular: float = 0.0,
        kd_angular: float = 0.1,
        kp_linear: float = 0.5,
        max_linear: float = 0.3,
        max_angular: float = 1.0,
        goal_tolerance: float = 0.10,
        heading_tolerance: float = 0.2,
    ) -> None:
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        self.kp_linear = kp_linear
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.goal_tolerance = goal_tolerance
        self.heading_tolerance = heading_tolerance

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._dt: float = 0.1  # assumed control period; caller can override

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute_velocity(
        self,
        robot_pose: Pose,
        waypoint: tuple[float, float],
        dt: float = 0.1,
    ) -> tuple[Twist, bool]:
        """
        Compute the velocity command to reach *waypoint*.

        Parameters
        ----------
        robot_pose:
            Current robot pose (position + orientation quaternion).
        waypoint:
            Target (x, y) in world frame.
        dt:
            Control period (seconds) for the integrator and derivative.

        Returns
        -------
        twist:
            Velocity command to publish on /cmd_vel.
        reached:
            True when the robot is within *goal_tolerance* of *waypoint*.
        """
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        yaw = self._yaw_from_pose(robot_pose)

        dx = waypoint[0] - rx
        dy = waypoint[1] - ry
        distance = math.sqrt(dx * dx + dy * dy)

        # Reached check
        if distance < self.goal_tolerance:
            self.reset()
            return self._zero_twist(), True

        # Desired heading
        desired_heading = math.atan2(dy, dx)

        # Heading error — wrapped to [-pi, pi]
        heading_error = math.atan2(
            math.sin(desired_heading - yaw),
            math.cos(desired_heading - yaw),
        )

        # PID on heading error
        self._integral += heading_error * dt
        # Anti-windup: clamp integrator
        max_integral = 0.5
        self._integral = max(-max_integral, min(max_integral, self._integral))

        derivative = (heading_error - self._prev_error) / max(dt, 1e-6)
        self._prev_error = heading_error

        angular = (
            self.kp_angular * heading_error
            + self.ki_angular * self._integral
            + self.kd_angular * derivative
        )
        angular = max(-self.max_angular, min(self.max_angular, angular))

        # Linear velocity: proportional to alignment, suppressed when turning
        if abs(heading_error) > self.heading_tolerance:
            # Turn in place when misaligned
            linear = 0.0
        else:
            alignment = math.cos(heading_error)  # 1.0 when perfectly aligned
            linear = self.kp_linear * distance * alignment
            linear = max(0.0, min(self.max_linear, linear))

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist, False

    def reset(self) -> None:
        """Reset PID integrators and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_from_pose(pose: Pose) -> float:
        """Extract yaw (heading) from a geometry_msgs/Pose quaternion."""
        q = pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _zero_twist() -> Twist:
        """Return a zero-velocity Twist."""
        return Twist()
