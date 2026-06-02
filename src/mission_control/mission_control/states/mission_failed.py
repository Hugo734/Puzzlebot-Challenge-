"""MISSION_FAILED — terminal failure state."""

from __future__ import annotations

import logging

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)


class MissionFailed(DebuggableState):
    """Log the failure reason and return to IDLE."""

    def __init__(self, debug_ctx: DebugContext, **kwargs) -> None:
        super().__init__("MISSION_FAILED", ["ok"], debug_ctx, abort_outcome="ok", **kwargs)

    def run(self, blackboard: Blackboard) -> str:
        mission = bb_get(blackboard, "current_mission") or {}
        reason = bb_get(blackboard, "mission_error_reason") or "unspecified"
        logger.error(
            "[MISSION_FAILED] Mission %s aborted: %s",
            mission.get("id"), reason,
        )
        return "ok"
