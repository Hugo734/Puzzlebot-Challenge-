"""Shared blocking action helpers used by mission states.

Each helper runs a 10 Hz poll loop that honours the DebugContext (abort + pause)
and returns a short outcome string, so state ``run()`` methods stay tiny and the
navigation / lifter / alignment logic lives in exactly one place (the old design
duplicated each of these across two near-identical states).
"""

from __future__ import annotations

import logging
import time
from typing import Callable

from yasmin import Blackboard

from mission_control.debug_wrapper import DebugContext
from mission_control.bb_helpers import bb_get

logger = logging.getLogger(__name__)

POLL_INTERVAL = 0.1


def navigate(
    debug: DebugContext,
    blackboard: Blackboard,
    publish_goal: Callable[[str], None],
    target: str,
    *,
    tag: str,
) -> str:
    """Publish ``target`` on /goal_waypoint and poll /nav_status.

    Returns 'arrived' | 'error' | 'stop'. On abort the goal is cancelled.
    """
    blackboard["nav_status_prefix"] = "PLANNING"
    logger.info("[%s] → navigating to %s", tag, target)
    publish_goal(target)
    while True:
        if debug.aborted:
            publish_goal("stop")
            return "stop"
        debug.wait_if_paused()

        prefix = bb_get(blackboard, "nav_status_prefix", "PLANNING")
        if prefix == "ARRIVED":
            logger.info("[%s] arrived at %s", tag, target)
            return "arrived"
        if prefix == "ERROR":
            blackboard["mission_error_reason"] = (
                f"navigation error at {target}: {bb_get(blackboard, 'nav_status_full')}"
            )
            return "error"

        time.sleep(POLL_INTERVAL)


def drive_lifter(
    debug: DebugContext,
    blackboard: Blackboard,
    publish_lifter: Callable[[int], None],
    level: int,
    timeout: float,
    *,
    tag: str,
) -> str:
    """Command a lifter ``level`` and wait for /lifter_status to match.

    Returns 'done' | 'timeout' | 'stop'.
    """
    target = max(0, min(7, int(level)))
    logger.info("[%s] lifter → %d (timeout=%.1fs)", tag, target, timeout)
    publish_lifter(target)
    deadline = time.monotonic() + timeout
    while True:
        if debug.aborted:
            return "stop"
        debug.wait_if_paused()

        status = bb_get(blackboard, "lifter_status")
        if status is not None and int(status) == target:
            logger.info("[%s] lifter reached %d", tag, target)
            return "done"
        if time.monotonic() > deadline:
            blackboard["mission_error_reason"] = (
                f"lifter did not reach level {target} (last status={status})"
            )
            return "timeout"

        time.sleep(POLL_INTERVAL)


def drive_for_time(
    debug: DebugContext,
    blackboard: Blackboard,
    publish_cmd: Callable[[float, float], None],
    v: float,
    w: float,
    duration: float,
    *,
    tag: str,
) -> str:
    """Open-loop drive at (v, w) for ``duration`` seconds, then stop.

    Returns 'ok' | 'stop'. Honours abort/pause and always sends a zero command
    on exit. Used by PICK for the timed forward-into-pallet and back-out moves.
    """
    deadline = time.monotonic() + duration
    logger.info("[%s] drive v=%.3f w=%.3f for %.1fs", tag, v, w, duration)
    try:
        while time.monotonic() < deadline:
            if debug.aborted:
                return "stop"
            debug.wait_if_paused()
            publish_cmd(v, w)
            time.sleep(POLL_INTERVAL)
        return "ok"
    finally:
        publish_cmd(0.0, 0.0)


def drive_until_stall(
    debug: DebugContext,
    blackboard: Blackboard,
    publish_cmd: Callable[[float, float], None],
    v: float,
    w: float,
    max_duration: float,
    *,
    grace: float,
    stall_speed: float,
    stall_ticks: int,
    tag: str,
) -> str:
    """Drive at (v, w) until the wheels stall (robot blocked, e.g. pressed into
    the pallet) OR ``max_duration`` s elapses, then stop.

    Stall = wheel speed below ``stall_speed`` (rad/s, from blackboard['wheel_speed'])
    for ``stall_ticks`` consecutive ticks, only checked after a ``grace`` spin-up
    window so the initial ramp isn't mistaken for a stall. Returns 'stalled' |
    'timeout' | 'stop'. Both 'stalled' and 'timeout' mean the move finished
    normally — stalling against the pallet IS the success condition here, and
    stopping at once protects the motor driver from a long stall-current draw.
    """
    t0 = time.monotonic()
    deadline = t0 + max_duration
    grace_until = t0 + grace
    stalled = 0
    logger.info("[%s] drive v=%.3f for <=%.1fs (stop on wheel stall < %.2f rad/s)",
                tag, v, max_duration, stall_speed)
    try:
        while True:
            if debug.aborted:
                return "stop"
            debug.wait_if_paused()
            publish_cmd(v, w)

            now = time.monotonic()
            if now >= grace_until:
                ws = bb_get(blackboard, "wheel_speed")
                if ws is not None and abs(ws) < stall_speed:
                    stalled += 1
                    if stalled >= stall_ticks:
                        logger.info("[%s] wheels stalled (%.2f rad/s) — blocked, "
                                    "treating step as reached.", tag, ws)
                        return "stalled"
                else:
                    stalled = 0

            if now >= deadline:
                logger.info("[%s] reached time limit %.1fs.", tag, max_duration)
                return "timeout"
            time.sleep(POLL_INTERVAL)
    finally:
        publish_cmd(0.0, 0.0)


def run_alignment(
    debug: DebugContext,
    blackboard: Blackboard,
    publish_align: Callable[[bool], None],
    timeout: float,
    *,
    tag: str,
) -> str:
    """Trigger qr_quad_alignment and wait for DONE / LOST / timeout.

    Always sends /alignment_start False on exit so the docking node returns to
    IDLE. Returns 'aligned' | 'failed' | 'stop'.
    """
    blackboard["alignment_state"] = "IDLE"
    publish_align(True)
    deadline = time.monotonic() + timeout
    logger.info("[%s] alignment started (timeout=%.1fs)", tag, timeout)
    try:
        while True:
            if debug.aborted:
                return "stop"
            debug.wait_if_paused()

            state = bb_get(blackboard, "alignment_state", "IDLE")
            if state == "DONE":
                logger.info("[%s] alignment DONE", tag)
                return "aligned"
            if state == "LOST":
                blackboard["mission_error_reason"] = "alignment LOST"
                return "failed"
            if time.monotonic() > deadline:
                blackboard["mission_error_reason"] = "alignment timeout"
                return "failed"

            time.sleep(POLL_INTERVAL)
    finally:
        publish_align(False)
