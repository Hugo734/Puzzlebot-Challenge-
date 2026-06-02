"""LIFT_PLACE — lower the lifter to the mission's place level."""

from __future__ import annotations

import logging
import time
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.1


class LiftPlace(DebuggableState):
    """Drive the lifter down to ``place_level`` and wait for ack.

    Outcomes:
        placed      — lifter_status reached place_level.
        lifter_fail — timeout waiting for ack.
        stop        — abort raised.
    """

    def __init__(
        self,
        debug_ctx: DebugContext,
        publish_lifter_fn: Callable[[int], None],
        timeout: float,
        **kwargs,
    ) -> None:
        super().__init__("LIFT_PLACE", ["placed", "lifter_fail", "stop"], debug_ctx, **kwargs)
        self._publish_lifter = publish_lifter_fn
        self._timeout = float(timeout)

    def run(self, blackboard: Blackboard) -> str:
        mission = blackboard["current_mission"]
        target = int(mission["place_level"])
        self._publish_lifter(target)
        deadline = time.monotonic() + self._timeout

        logger.info("[LIFT_PLACE] Commanding level %d (timeout=%.1fs).", target, self._timeout)

        while True:
            if self._debug.aborted:
                return "stop"

            self._debug.wait_if_paused()

            status = bb_get(blackboard, "lifter_status")
            if status is not None and int(status) == target:
                logger.info("[LIFT_PLACE] Lifter reached level %d.", target)
                return "placed"

            if time.monotonic() > deadline:
                logger.warning(
                    "[LIFT_PLACE] Timed out waiting for lifter_status=%d (last=%s).",
                    target, status,
                )
                blackboard["mission_error_reason"] = (
                    f"lifter did not reach level {target} (last status={status})"
                )
                return "lifter_fail"

            time.sleep(_POLL_INTERVAL)
