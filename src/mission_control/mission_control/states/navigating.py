"""
navigating.py
-------------
Navigating state — publishes a navigation goal and waits for the result.

Used for both legs of a mission:
    phase="to_source"  → reads blackboard["current_mission"]["source"]
    phase="to_dest"    → reads blackboard["current_mission"]["destination"]

The ROS node writes the navigation stack's status string to
blackboard["nav_status"]:  "IDLE" | "NAVIGATING" | "ARRIVED" | "STUCK"
"""

import math
import time
import logging
from typing import Callable

from yasmin import State, Blackboard

from mission_control.mission_parser import get_waypoint_pose

logger = logging.getLogger(__name__)

# Seconds between successive status polls
_POLL_INTERVAL = 0.1


def _build_pose_stamped_msg(x: float, y: float, theta: float):
    """Build a geometry_msgs/PoseStamped dict-like object.

    We return a lambda that the node will call with a real rclpy Clock so that
    the header stamp is always fresh at publish time.  The state stores the
    callable in the blackboard; the node picks it up and publishes.
    """
    # Returned as a plain tuple; the node converts it to PoseStamped.
    return (x, y, theta)


class Navigating(State):
    """Navigate the robot to a waypoint.

    Parameters
    ----------
    publish_goal_fn:
        Callable ``(x, y, theta) -> None`` that publishes a PoseStamped to
        ``/navigation/goal``.  Injected by the node to keep the state free of
        direct ROS imports.
    phase:
        ``"to_source"`` or ``"to_dest"``; determines which blackboard field
        holds the target zone name.

    Outcomes:
        arrived — the navigation stack reported ARRIVED.
        stuck   — the navigation stack reported STUCK (obstacle or timeout).
        stop    — global stop flag was raised.
    """

    def __init__(self, publish_goal_fn: Callable[[float, float, float], None], phase: str) -> None:
        if phase not in ("to_source", "to_dest"):
            raise ValueError(f"phase must be 'to_source' or 'to_dest', got {phase!r}")
        super().__init__(outcomes=["arrived", "stuck", "stop"])
        self._publish_goal = publish_goal_fn
        self._phase = phase

    def execute(self, blackboard: Blackboard) -> str:
        mission: dict = blackboard["current_mission"]

        zone_key = "source" if self._phase == "to_source" else "destination"
        zone_name: str = mission[zone_key]

        pose = get_waypoint_pose(zone_name)
        if pose is None:
            logger.error(
                f"[Navigating/{self._phase}] Unknown waypoint '{zone_name}'. "
                "Treating as stuck."
            )
            return "stuck"

        x, y, theta = pose
        logger.info(
            f"[Navigating/{self._phase}] Sending goal → "
            f"{zone_name} ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)"
        )

        # Reset nav_status before publishing so stale ARRIVED doesn't fire
        # immediately if the previous navigation finished at the same waypoint.
        blackboard["nav_status"] = "NAVIGATING"
        self._publish_goal(x, y, theta)

        while True:
            if blackboard.get("stop_flag", False):
                logger.info(f"[Navigating/{self._phase}] Stop flag — aborting navigation.")
                return "stop"

            status: str = blackboard.get("nav_status", "NAVIGATING")

            if status == "ARRIVED":
                logger.info(f"[Navigating/{self._phase}] Arrived at {zone_name}.")
                return "arrived"

            if status == "STUCK":
                logger.warning(
                    f"[Navigating/{self._phase}] Navigation stuck at {zone_name}."
                )
                return "stuck"

            time.sleep(_POLL_INTERVAL)
