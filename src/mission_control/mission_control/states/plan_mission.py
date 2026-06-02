"""PLAN_MISSION — validate the parsed mission and seed the candidate queue."""

from __future__ import annotations

import logging
from collections import deque

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)


class PlanMission(DebuggableState):
    """Promote a parsed mission into execution state.

    Outcomes:
        ok       — candidate_queue is non-empty and ready.
        invalid  — parser returned None, or the queue is empty.
    """

    def __init__(self, debug_ctx: DebugContext, **kwargs) -> None:
        super().__init__("PLAN_MISSION", ["ok", "invalid"], debug_ctx, abort_outcome="invalid", **kwargs)

    def run(self, blackboard: Blackboard) -> str:
        mission = bb_get(blackboard, "current_mission")
        if mission is None:
            blackboard["mission_error_reason"] = "JSON failed to parse or violated schema."
            return "invalid"

        queue = mission["candidate_queue"]
        if not isinstance(queue, deque) or not queue:
            blackboard["mission_error_reason"] = "Candidate queue is empty."
            return "invalid"

        # Reset transient fields for this mission.
        blackboard["candidate_queue"] = queue
        blackboard["current_candidate"] = None
        blackboard["qr_value"] = None
        blackboard["resolved_dest"] = mission.get("destination")

        logger.info(
            "[PLAN_MISSION] Mission %s ready — queue=%s scan_qr=%s",
            mission["id"], list(queue), mission["scan_qr"],
        )
        return "ok"
