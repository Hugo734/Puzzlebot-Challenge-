"""
picking_floor.py
----------------
PickingFloor state — lifter sequence for floor-level or rack-level-1 pallets.

Lifter levels (encoded as 3-bit value sent to FPGA via Jetson GPIO):
    0 — rest / fully lowered
    1 — transport height (carry pallet safely while moving)
    3 — floor pick height (insert forks under a floor-level pallet)

Sequence:
    1. Lower to 0  — ensure forks are fully down before approaching.
    2. Wait 1.0 s  — allow FPGA to complete movement.
    3. Raise to 3  — insert forks under the pallet.
    4. Wait 1.5 s  — FPGA finishes lifting.
    5. Lower to 1  — transport height (safe for travel).
    6. Wait 0.5 s  — FPGA settles.
"""

import time
import logging
from typing import Callable

from yasmin import State, Blackboard

logger = logging.getLogger(__name__)


class PickingFloor(State):
    """Execute the pick sequence for a floor / rack-level-1 pallet.

    Parameters
    ----------
    publish_lifter_fn:
        Callable ``(level: int) -> None`` that publishes a std_msgs/UInt8 to
        ``/lifter_level``.  Injected by the node.

    Outcomes:
        picked — sequence completed successfully.
        stop   — global stop flag was raised mid-sequence.
    """

    def __init__(self, publish_lifter_fn: Callable[[int], None]) -> None:
        super().__init__(outcomes=["picked", "stop"])
        self._publish_lifter = publish_lifter_fn

    def _publish_and_wait(
        self,
        level: int,
        wait: float,
        blackboard: Blackboard,
    ) -> bool:
        """Publish a lifter level, then wait.

        Returns True if the wait completed normally, False if interrupted by
        the stop flag.
        """
        self._publish_lifter(level)
        logger.debug(f"[PickingFloor] Lifter → {level}, waiting {wait:.1f}s")
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if blackboard.get("stop_flag", False):
                return False
            time.sleep(0.05)
        return True

    def execute(self, blackboard: Blackboard) -> str:
        logger.info("[PickingFloor] Starting floor pick sequence.")

        steps: list[tuple[int, float]] = [
            (0, 1.0),   # Ensure fully lowered
            (3, 1.5),   # Pick height — insert forks
            (1, 0.5),   # Transport height
        ]

        for level, wait in steps:
            if not self._publish_and_wait(level, wait, blackboard):
                logger.info("[PickingFloor] Stop flag — aborting pick sequence.")
                return "stop"

        logger.info("[PickingFloor] Floor pick complete.")
        return "picked"
