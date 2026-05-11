"""
picking_truck.py
----------------
PickingTruck state — lifter sequence for truck loading-bay pallets.

Lifter levels:
    3 — truck pick height (forks under pallet on truck floor)
    4 — carry height (pallet clear of truck bed, safe for travel)

Sequence:
    1. Raise to 3  — insert forks under the pallet inside the truck.
    2. Wait 2.0 s  — FPGA finishes lifting.
    3. Raise to 4  — carry height (pallet clear of truck floor).
    4. Wait 0.5 s  — FPGA settles.
"""

import time
import logging
from typing import Callable

from yasmin import State, Blackboard

logger = logging.getLogger(__name__)


class PickingTruck(State):
    """Execute the pick sequence for a truck loading-bay pallet.

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
        logger.debug(f"[PickingTruck] Lifter → {level}, waiting {wait:.1f}s")
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if blackboard.get("stop_flag", False):
                return False
            time.sleep(0.05)
        return True

    def execute(self, blackboard: Blackboard) -> str:
        logger.info("[PickingTruck] Starting truck pick sequence.")

        steps: list[tuple[int, float]] = [
            (3, 2.0),   # Truck pick height — insert forks
            (4, 0.5),   # Carry height — pallet clear of truck bed
        ]

        for level, wait in steps:
            if not self._publish_and_wait(level, wait, blackboard):
                logger.info("[PickingTruck] Stop flag — aborting pick sequence.")
                return "stop"

        logger.info("[PickingTruck] Truck pick complete.")
        return "picked"
