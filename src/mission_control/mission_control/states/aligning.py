"""
aligning.py
-----------
Aligning state — waits for the perception system to confirm alignment.

The vision node publishes a geometry_msgs/Point to /alignment_error:
    x  — pixel error in the horizontal axis
    y  — pixel error in the vertical axis
    z  — 1.0 when aligned, 0.0 when still correcting

The ROS node copies this message into blackboard["alignment_error"] as the
raw Point message object.

A dedicated PID controller (in the navigation / vision stack) handles the
actual actuation; this state only monitors the alignment flag.
"""

import time
import logging
from typing import Any

from yasmin import State, Blackboard

logger = logging.getLogger(__name__)

# Polling cadence (seconds)
_POLL_INTERVAL = 0.1

# Default timeout before declaring alignment failed (seconds)
_DEFAULT_TIMEOUT = 30.0


class Aligning(State):
    """Monitor the alignment_error topic until the robot is aligned.

    Parameters
    ----------
    timeout:
        Maximum seconds to wait for alignment before returning ``"failed"``.

    Outcomes:
        aligned — ``alignment_error.z == 1.0`` received within timeout.
        failed  — timeout elapsed without alignment being achieved.
        stop    — global stop flag was raised.
    """

    def __init__(self, timeout: float = _DEFAULT_TIMEOUT) -> None:
        super().__init__(outcomes=["aligned", "failed", "stop"])
        self._timeout = timeout

    def execute(self, blackboard: Blackboard) -> str:
        logger.info(
            f"[Aligning] Starting alignment — timeout={self._timeout:.1f}s"
        )

        deadline = time.monotonic() + self._timeout

        while True:
            if blackboard.get("stop_flag", False):
                logger.info("[Aligning] Stop flag raised — aborting alignment.")
                return "stop"

            if time.monotonic() > deadline:
                logger.warning(
                    f"[Aligning] Alignment timed out after {self._timeout:.1f}s."
                )
                return "failed"

            error: Any = blackboard.get("alignment_error")
            if error is not None:
                # geometry_msgs/Point: z == 1.0 signals aligned
                try:
                    aligned_flag = float(error.z)
                except (AttributeError, TypeError, ValueError):
                    # Message not yet populated or unexpected type — keep waiting
                    time.sleep(_POLL_INTERVAL)
                    continue

                if aligned_flag == 1.0:
                    logger.info("[Aligning] Alignment confirmed.")
                    return "aligned"

            time.sleep(_POLL_INTERVAL)
