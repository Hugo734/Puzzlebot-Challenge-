"""
placing_pallet.py
-----------------
PlacingPallet state — lifter sequence for setting a pallet down at the
destination zone.

Lifter levels:
    0 — rest / fully lowered (forks completely retracted)
    3 — place height (lower forks until pallet rests on destination surface)

Sequence:
    1. Lower to 3  — bring pallet down to just above the surface.
    2. Wait 1.5 s  — FPGA completes the downward stroke.
    3. Lower to 0  — fully retract forks (pallet now resting on surface).
    4. Wait 1.0 s  — FPGA settles and forks clear the pallet openings.
"""

import time
import logging
from typing import Callable

from yasmin import State, Blackboard

logger = logging.getLogger(__name__)


class PlacingPallet(State):
    """Execute the place-down sequence at the destination zone.

    Parameters
    ----------
    publish_lifter_fn:
        Callable ``(level: int) -> None`` that publishes a std_msgs/UInt8 to
        ``/lifter_level``.  Injected by the node.

    Outcomes:
        placed — sequence completed successfully.
        stop   — global stop flag was raised mid-sequence.
    """

    def __init__(self, publish_lifter_fn: Callable[[int], None]) -> None:
        super().__init__(outcomes=["placed", "stop"])
        self._publish_lifter = publish_lifter_fn

    def _publish_and_wait(
        self,
        level: int,
        wait: float,
        blackboard: Blackboard,
    ) -> bool:
        """Publish a lifter level, then wait.

        Returns True if completed normally, False if stop flag raised.
        """
        self._publish_lifter(level)
        logger.debug(f"[PlacingPallet] Lifter → {level}, waiting {wait:.1f}s")
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if blackboard.get("stop_flag", False):
                return False
            time.sleep(0.05)
        return True

    def execute(self, blackboard: Blackboard) -> str:
        mission: dict = blackboard.get("current_mission", {})
        dest = mission.get("destination", "unknown")
        logger.info(f"[PlacingPallet] Starting place sequence at '{dest}'.")

        steps: list[tuple[int, float]] = [
            (3, 1.5),   # Lower to place height — pallet contacts surface
            (0, 1.0),   # Fully retract forks — pallet stays on surface
        ]

        for level, wait in steps:
            if not self._publish_and_wait(level, wait, blackboard):
                logger.info("[PlacingPallet] Stop flag — aborting place sequence.")
                return "stop"

        logger.info(f"[PlacingPallet] Pallet placed at '{dest}'.")
        return "placed"
