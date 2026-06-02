"""RESOLVE_DESTINATION — turn a decoded QR string into a truck waypoint."""

from __future__ import annotations

import logging

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get
from mission_control.mission_parser import resolve_qr_to_waypoint

logger = logging.getLogger(__name__)


class ResolveDestination(DebuggableState):
    """Translate ``blackboard['qr_value']`` into ``blackboard['resolved_dest']``.

    Outcomes:
        resolved — qr_aliases mapped the payload to a known truck.
        invalid  — QR did not match any registered alias.
    """

    def __init__(self, debug_ctx: DebugContext, zones_data: dict, **kwargs) -> None:
        super().__init__("RESOLVE_DESTINATION", ["resolved", "invalid"], debug_ctx, abort_outcome="invalid", **kwargs)
        self._zones = zones_data

    def run(self, blackboard: Blackboard) -> str:
        mission = blackboard["current_mission"]
        # If the mission already had a hard-coded destination (CUSTOM), keep it.
        if mission.get("destination"):
            blackboard["resolved_dest"] = mission["destination"]
            logger.info("[RESOLVE_DESTINATION] Using pre-set destination %s.", mission["destination"])
            return "resolved"

        qr = bb_get(blackboard, "qr_value")
        target = resolve_qr_to_waypoint(self._zones, qr or "")
        if target is None:
            logger.error("[RESOLVE_DESTINATION] QR payload %r not in qr_aliases.", qr)
            blackboard["mission_error_reason"] = f"QR payload {qr!r} not in qr_aliases"
            return "invalid"

        blackboard["resolved_dest"] = target
        logger.info("[RESOLVE_DESTINATION] QR %r → %s.", qr, target)
        return "resolved"
