"""ALIGN_TO_PALLET — trigger the QR docking maneuver and wait for DONE."""

from __future__ import annotations

import logging
import time
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.1


class AlignToPallet(DebuggableState):
    """Activate qr_quad_alignment and wait until it reports DONE.

    The alignment node consumes ``/alignment_start`` (Bool) and publishes its
    own state on ``/alignment_state`` (String). The bridge mirrors that into
    ``blackboard['alignment_state']``.

    Outcomes:
        aligned — alignment node reported DONE.
        failed  — alignment node reported LOST, or timeout.
        stop    — abort raised.
    """

    def __init__(
        self,
        debug_ctx: DebugContext,
        publish_alignment_start_fn: Callable[[bool], None],
        timeout: float,
        **kwargs,
    ) -> None:
        super().__init__("ALIGN_TO_PALLET", ["aligned", "failed", "stop"], debug_ctx, **kwargs)
        self._publish_start = publish_alignment_start_fn
        self._timeout = float(timeout)

    def run(self, blackboard: Blackboard) -> str:
        mission = blackboard["current_mission"]
        if mission.get("skip_alignment"):
            logger.info("[ALIGN_TO_PALLET] skip_alignment=true — bypassing.")
            return "aligned"

        # Reset the alignment state so a stale DONE from before doesn't fire.
        blackboard["alignment_state"] = "IDLE"
        self._publish_start(True)
        deadline = time.monotonic() + self._timeout

        logger.info("[ALIGN_TO_PALLET] Started alignment (timeout=%.1fs).", self._timeout)

        try:
            while True:
                if self._debug.aborted:
                    return "stop"

                self._debug.wait_if_paused()

                state = bb_get(blackboard, "alignment_state", "IDLE")
                if state == "DONE":
                    logger.info("[ALIGN_TO_PALLET] Alignment DONE.")
                    return "aligned"
                if state == "LOST":
                    logger.warning("[ALIGN_TO_PALLET] Alignment LOST.")
                    blackboard["mission_error_reason"] = "alignment LOST"
                    return "failed"

                if time.monotonic() > deadline:
                    logger.warning("[ALIGN_TO_PALLET] Timed out after %.1fs.", self._timeout)
                    blackboard["mission_error_reason"] = "alignment timeout"
                    return "failed"

                time.sleep(_POLL_INTERVAL)
        finally:
            self._publish_start(False)
