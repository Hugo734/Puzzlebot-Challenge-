"""SCAN_QR — wait for /qr_detected, with timeout."""

from __future__ import annotations

import logging
import time

from yasmin import Blackboard

from mission_control.debug_wrapper import DebuggableState, DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.1


class ScanQR(DebuggableState):
    """Look for a decoded QR payload while parked at the current candidate.

    Outcomes:
        qr_found     — fresh /qr_detected payload received within timeout.
        qr_not_found — timeout expired, no payload.
        stop         — abort raised.
    """

    def __init__(self, debug_ctx: DebugContext, timeout: float, **kwargs) -> None:
        super().__init__("SCAN_QR", ["qr_found", "qr_not_found", "stop"], debug_ctx, **kwargs)
        self._timeout = float(timeout)

    def run(self, blackboard: Blackboard) -> str:
        # CUSTOM missions can opt out of scanning entirely.
        mission = blackboard["current_mission"]
        if mission and not mission.get("scan_qr", True):
            logger.info("[SCAN_QR] scan_qr=false — skipping.")
            return "qr_found"

        # Treat any payload that arrived during nav/before-we-got-here as
        # stale and require a fresh one. The bridge stores arrival time too.
        scan_started_at = time.monotonic()
        blackboard["qr_value"] = None
        deadline = scan_started_at + self._timeout

        candidate = bb_get(blackboard, "current_candidate")
        logger.info("[SCAN_QR] Scanning at %s (timeout=%.1fs)", candidate, self._timeout)

        while True:
            if self._debug.aborted:
                return "stop"

            self._debug.wait_if_paused()

            qr = bb_get(blackboard, "qr_detected")
            qr_t = bb_get(blackboard, "qr_detected_at", 0.0)
            if qr and qr_t >= scan_started_at:
                logger.info("[SCAN_QR] Found QR payload %r at %s.", qr, candidate)
                blackboard["qr_value"] = qr
                return "qr_found"

            if time.monotonic() > deadline:
                logger.info("[SCAN_QR] No QR at %s within %.1fs.", candidate, self._timeout)
                return "qr_not_found"

            time.sleep(_POLL_INTERVAL)
