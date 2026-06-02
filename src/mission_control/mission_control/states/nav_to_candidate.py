"""NAV_TO_CANDIDATE — send the next candidate waypoint to nav_node."""

from __future__ import annotations

import logging
import time
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.1


class NavToCandidate(DebuggableState):
    """Pop the head of candidate_queue and navigate there.

    Outcomes:
        arrived — nav_node reported ARRIVED.
        stuck   — nav_node reported ERROR.
        stop    — abort raised.
    """

    def __init__(
        self,
        debug_ctx: DebugContext,
        publish_goal_fn: Callable[[str], None],
        **kwargs,
    ) -> None:
        super().__init__("NAV_TO_CANDIDATE", ["arrived", "stuck", "stop"], debug_ctx, **kwargs)
        self._publish_goal = publish_goal_fn

    def run(self, blackboard: Blackboard) -> str:
        queue = blackboard["candidate_queue"]
        if not queue:
            # Defensive: NEXT_CANDIDATE should have routed to FAIL already.
            blackboard["mission_error_reason"] = "candidate_queue emptied unexpectedly"
            return "stuck"

        candidate = queue.popleft()
        blackboard["current_candidate"] = candidate
        blackboard["nav_status_prefix"] = "PLANNING"

        logger.info("[NAV_TO_CANDIDATE] → %s (remaining=%d)", candidate, len(queue))
        self._publish_goal(candidate)

        while True:
            if self._debug.aborted:
                self._publish_goal("stop")
                return "stop"

            self._debug.wait_if_paused()

            prefix = bb_get(blackboard, "nav_status_prefix", "PLANNING")
            if prefix == "ARRIVED":
                logger.info("[NAV_TO_CANDIDATE] Arrived at %s.", candidate)
                return "arrived"
            if prefix == "ERROR":
                logger.warning(
                    "[NAV_TO_CANDIDATE] nav_node ERROR at %s (full=%s)",
                    candidate, bb_get(blackboard, "nav_status_full"),
                )
                blackboard["mission_error_reason"] = (
                    f"navigation error at {candidate}: {bb_get(blackboard, 'nav_status_full')}"
                )
                return "stuck"

            time.sleep(_POLL_INTERVAL)
