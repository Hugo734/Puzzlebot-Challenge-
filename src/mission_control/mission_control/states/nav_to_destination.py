"""NAV_TO_DESTINATION — go to resolved_dest with a pallet on board."""

from __future__ import annotations

import logging
import time
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.1


class NavToDestination(DebuggableState):
    """Send ``blackboard['resolved_dest']`` as a waypoint and wait.

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
        super().__init__("NAV_TO_DESTINATION", ["arrived", "stuck", "stop"], debug_ctx, **kwargs)
        self._publish_goal = publish_goal_fn

    def run(self, blackboard: Blackboard) -> str:
        dest = bb_get(blackboard, "resolved_dest")
        if not dest:
            blackboard["mission_error_reason"] = "resolved_dest is None at NAV_TO_DESTINATION"
            return "stuck"

        blackboard["nav_status_prefix"] = "PLANNING"
        logger.info("[NAV_TO_DESTINATION] → %s", dest)
        self._publish_goal(dest)

        while True:
            if self._debug.aborted:
                self._publish_goal("stop")
                return "stop"

            self._debug.wait_if_paused()

            prefix = bb_get(blackboard, "nav_status_prefix", "PLANNING")
            if prefix == "ARRIVED":
                logger.info("[NAV_TO_DESTINATION] Arrived at %s.", dest)
                return "arrived"
            if prefix == "ERROR":
                logger.warning(
                    "[NAV_TO_DESTINATION] nav_node ERROR (full=%s).",
                    bb_get(blackboard, "nav_status_full"),
                )
                blackboard["mission_error_reason"] = (
                    f"navigation error to {dest}: {bb_get(blackboard, 'nav_status_full')}"
                )
                return "stuck"

            time.sleep(_POLL_INTERVAL)
