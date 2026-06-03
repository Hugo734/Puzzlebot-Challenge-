"""PICK — mission step 2: dock onto the pallet, then a timed pick maneuver.

Sequence (open-loop, time-based):
    1. QR alignment (qr_quad_alignment docking) to centre on the pallet.
    2. Set the lifter to ``entry_level`` (fork height to slide under the pallet).
    3. Drive forward for ``forward_time`` s to get the forks under the pallet.
    4. Set the lifter to ``lift_level`` to take the pallet's weight.
    5. Drive backward for ``reverse_time`` s to pull the pallet clear.
"""

from __future__ import annotations

import logging
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get
from mission_control.states._actions import (
    run_alignment, drive_for_time, drive_until_stall, drive_lifter,
)

logger = logging.getLogger(__name__)


class Pick(DebuggableState):
    """Step 2 — *recoge*: align, drive in, lift, back out.

    Outcomes:
        picked — full maneuver done → continue mission.
        done   — same, but PICK_ONLY test → end mission.
        failed — alignment failed/timeout, or lifter timeout.
        stop   — abort raised.
    """

    def __init__(
        self,
        debug_ctx: DebugContext,
        publish_alignment_start_fn: Callable[[bool], None],
        publish_lifter_fn: Callable[[int], None],
        publish_cmd_fn: Callable[[float, float], None],
        alignment_timeout: float,
        lifter_timeout: float,
        drive_speed: float,
        forward_time: float,
        reverse_time: float,
        entry_level: int,
        lift_level: int,
        stall_grace: float,
        stall_speed: float,
        stall_ticks: int,
        **kwargs,
    ) -> None:
        super().__init__(
            "PICK", ["picked", "done", "failed", "stop"], debug_ctx,
            abort_outcome="stop", **kwargs,
        )
        self._publish_align = publish_alignment_start_fn
        self._publish_lifter = publish_lifter_fn
        self._publish_cmd = publish_cmd_fn
        self._align_timeout = float(alignment_timeout)
        self._lifter_timeout = float(lifter_timeout)
        self._drive_speed = float(drive_speed)
        self._forward_time = float(forward_time)
        self._reverse_time = float(reverse_time)
        self._entry_level = int(entry_level)
        self._lift_level = int(lift_level)
        self._stall_grace = float(stall_grace)
        self._stall_speed = float(stall_speed)
        self._stall_ticks = int(stall_ticks)

    def run(self, blackboard: Blackboard) -> str:
        mission = bb_get(blackboard, "current_mission") or {}

        # --- 1) align onto the pallet (QR docking) ---
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

        # --- 2) raise forks to the entry height (slide under the pallet) ---
        outcome = drive_lifter(
            self._debug, blackboard, self._publish_lifter,
            self._entry_level, self._lifter_timeout, tag="PICK entry",
        )
        if outcome == "stop":
            return "stop"
        if outcome == "timeout":
            return "failed"

        # --- 3) drive forward into the pallet: stop as soon as the wheels stall
        #         (blocked by the pallet) or the time limit is hit — both count
        #         as "reached" so the motor driver isn't left stalling. ---
        if drive_until_stall(self._debug, blackboard, self._publish_cmd,
                             self._drive_speed, 0.0, self._forward_time,
                             grace=self._stall_grace, stall_speed=self._stall_speed,
                             stall_ticks=self._stall_ticks, tag="PICK fwd") == "stop":
            return "stop"

        # --- 4) lift the pallet (change to lift level) ---
        outcome = drive_lifter(
            self._debug, blackboard, self._publish_lifter,
            self._lift_level, self._lifter_timeout, tag="PICK lift",
        )
        if outcome == "stop":
            return "stop"
        if outcome == "timeout":
            return "failed"

        # --- 5) back out with the pallet (timed reverse) ---
        if drive_for_time(self._debug, blackboard, self._publish_cmd,
                          -self._drive_speed, 0.0, self._reverse_time,
                          tag="PICK rev") == "stop":
            return "stop"

        return "done" if mission.get("pick_only") else "picked"
