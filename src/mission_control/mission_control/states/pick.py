"""PICK — mission step 2: dock onto the pallet then raise the lifter.

Merges the old ALIGN_TO_PALLET + LIFT_PICKUP states: align (via qr_quad_alignment)
then lift to the mission's pickup level, in one state.
"""

from __future__ import annotations

import logging
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get
from mission_control.states._actions import run_alignment, drive_lifter

logger = logging.getLogger(__name__)


class Pick(DebuggableState):
    """Step 2 — *recoge*: align to the pallet, then lift it.

    Outcomes:
        picked — aligned (or skipped) AND lifter reached pickup_level.
        failed — alignment failed/timeout, or lifter timeout.
        stop   — abort raised.
    """

    def __init__(
        self,
        debug_ctx: DebugContext,
        publish_alignment_start_fn: Callable[[bool], None],
        publish_lifter_fn: Callable[[int], None],
        alignment_timeout: float,
        lifter_timeout: float,
        **kwargs,
    ) -> None:
        super().__init__(
            "PICK", ["picked", "failed", "stop"], debug_ctx,
            abort_outcome="stop", **kwargs,
        )
        self._publish_align = publish_alignment_start_fn
        self._publish_lifter = publish_lifter_fn
        self._align_timeout = float(alignment_timeout)
        self._lifter_timeout = float(lifter_timeout)

    def run(self, blackboard: Blackboard) -> str:
        mission = bb_get(blackboard, "current_mission") or {}

        # --- align onto the pallet ---
        if mission.get("skip_alignment"):
            logger.info("[PICK] skip_alignment=true — skipping docking.")
        else:
            outcome = run_alignment(
                self._debug, blackboard, self._publish_align,
                self._align_timeout, tag="PICK",
            )
            if outcome == "stop":
                return "stop"
            if outcome == "failed":
                return "failed"

        # --- lift the pallet ---
        level = int(mission.get("pickup_level", 1))
        outcome = drive_lifter(
            self._debug, blackboard, self._publish_lifter,
            level, self._lifter_timeout, tag="PICK",
        )
        if outcome == "stop":
            return "stop"
        if outcome == "timeout":
            return "failed"
        return "picked"
