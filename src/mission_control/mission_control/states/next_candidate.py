"""NEXT_CANDIDATE — branch on whether more candidates remain to visit."""

from __future__ import annotations

import logging

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)


class NextCandidate(DebuggableState):
    """Inspect candidate_queue; route to NAV or to MISSION_FAILED.

    Outcomes:
        more      — at least one candidate remaining.
        exhausted — queue is empty, mission cannot find its pallet.
    """

    def __init__(self, debug_ctx: DebugContext, **kwargs) -> None:
        super().__init__("NEXT_CANDIDATE", ["more", "exhausted"], debug_ctx, abort_outcome="exhausted", **kwargs)

    def run(self, blackboard: Blackboard) -> str:
        queue = bb_get(blackboard, "candidate_queue")
        remaining = len(queue) if queue is not None else 0
        if remaining > 0:
            logger.info("[NEXT_CANDIDATE] %d candidate(s) remaining.", remaining)
            return "more"

        logger.warning("[NEXT_CANDIDATE] Candidate queue exhausted.")
        blackboard["mission_error_reason"] = "No QR pallet found in any candidate location."
        return "exhausted"
