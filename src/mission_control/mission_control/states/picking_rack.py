"""
picking_rack.py
---------------
PickingRack state — lifter sequence for rack level-2 pallets.

Lifter levels:
    4 — carry height (between actions, safe for travel)
    5 — rack level-2 pick height (insert forks at the elevated shelf)

Sequence:
    1. Raise to 5  — reach rack level-2 shelf, insert forks under pallet.
    2. Wait 2.0 s  — FPGA extends lifter fully.
    3. Lower to 4  — carry height (pallet clear of shelf, safe to move).
    4. Wait 0.5 s  — FPGA settles.
"""

import time
import logging
from typing import Callable

from yasmin import State, Blackboard

logger = logging.getLogger(__name__)


class PickingRack(State):
    """Execute the pick sequence for a rack level-2 pallet.

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

        Returns True if completed normally, False if stop flag raised.
        """
        self._publish_lifter(level)
        logger.debug(f"[PickingRack] Lifter → {level}, waiting {wait:.1f}s")
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if blackboard.get("stop_flag", False):
                return False
            time.sleep(0.05)
        return True

    def execute(self, blackboard: Blackboard) -> str:
        logger.info("[PickingRack] Starting rack level-2 pick sequence.")

        steps: list[tuple[int, float]] = [
            (5, 2.0),   # Rack level-2 pick height
            (4, 0.5),   # Carry height — pallet clear of shelf
        ]

        for level, wait in steps:
            if not self._publish_and_wait(level, wait, blackboard):
                logger.info("[PickingRack] Stop flag — aborting pick sequence.")
                return "stop"

        logger.info("[PickingRack] Rack pick complete.")
        return "picked"
