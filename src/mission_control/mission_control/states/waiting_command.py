"""
waiting_command.py
------------------
WaitingCommand state — idles until a mission JSON arrives on the blackboard.

The ROS node writes incoming /mission messages to blackboard["mission"].
This state polls that key, validates the payload, and stores the parsed
mission dict at blackboard["current_mission"] before transitioning.
"""

import time
import logging

from yasmin import State, Blackboard

from mission_control.mission_parser import parse_mission

logger = logging.getLogger(__name__)

# Seconds between successive blackboard polls
_POLL_INTERVAL = 0.1


class WaitingCommand(State):
    """Waits for a valid mission to appear on the blackboard.

    Outcomes:
        mission_received — a valid mission was parsed and stored.
        stop             — global stop flag was raised.
    """

    def __init__(self) -> None:
        super().__init__(outcomes=["mission_received", "stop"])

    def execute(self, blackboard: Blackboard) -> str:
        """Block until a mission is received or a stop signal is raised.

        The state clears the stale ``mission`` key after consuming it so that
        re-entry does not immediately re-trigger on the previous mission.
        """
        # Clear any leftover mission from the previous cycle so we wait for a
        # genuinely new one.
        blackboard["mission"] = None
        blackboard["current_mission"] = None

        logger.info("[WaitingCommand] Waiting for mission...")

        while True:
            # Check global stop flag first
            if blackboard.get("stop_flag", False):
                logger.info("[WaitingCommand] Stop flag detected — halting.")
                return "stop"

            raw_mission: str | None = blackboard.get("mission")
            if raw_mission:
                mission = parse_mission(raw_mission)
                if mission is not None:
                    blackboard["current_mission"] = mission
                    # Consume the raw payload so a re-entry doesn't replay it
                    blackboard["mission"] = None
                    logger.info(
                        "[WaitingCommand] Mission accepted: "
                        f"pallet={mission['pallet_id']} "
                        f"source={mission['source']} (level {mission['source_level']}) "
                        f"→ dest={mission['destination']}"
                    )
                    return "mission_received"
                else:
                    # Bad payload — discard and keep waiting
                    logger.warning(
                        f"[WaitingCommand] Invalid mission payload discarded: {raw_mission!r}"
                    )
                    blackboard["mission"] = None

            time.sleep(_POLL_INTERVAL)
