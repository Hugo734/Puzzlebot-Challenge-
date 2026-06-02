"""
mission_parser.py
-----------------
Utilities for parsing mission JSON payloads and resolving waypoints.
No ROS dependencies — pure Python so it can be unit-tested standalone.
"""

import json
from typing import Optional

# ---------------------------------------------------------------------------
# Waypoint registry
# (x [m], y [m], theta [rad])  — theta is the robot heading at the goal
# ---------------------------------------------------------------------------
WAYPOINTS: dict[str, tuple[float, float, float]] = {
    "truck_1":  (1.50, 0.50, 0.00),
    "truck_2":  (1.50, 1.50, 0.00),
    "truck_3":  (1.50, 2.50, 0.00),
    "rack_1":   (0.80, 0.50, 1.57),
    "rack_2":   (0.80, 1.50, 1.57),
    "roller_1": (2.00, 1.00, 3.14),
    "roller_2": (2.00, 2.00, 3.14),
    "home":     (0.00, 0.00, 0.00),
}

# Mapping from source_level integer → picking state name
_LEVEL_TO_STATE: dict[int, str] = {
    1: "PICKING_FLOOR",   # floor / rack level 1
    2: "PICKING_RACK",    # rack level 2
    3: "PICKING_TRUCK",   # truck loading bay
}

# Required keys and their expected types
_REQUIRED_FIELDS: dict[str, type] = {
    "pallet_id":    str,
    "source":       str,
    "source_level": int,
    "destination":  str,
}


def parse_mission(json_str: str) -> Optional[dict]:
    """Parse and validate a mission JSON string.

    Expected format:
        {"pallet_id": "A1", "source": "rack_1", "source_level": 1,
         "destination": "truck_2"}

    source_level:
        1 — floor or rack level 1
        2 — rack level 2
        3 — truck loading bay

    Returns a validated dict on success, or None on any parse/validation error.
    """
    if not json_str or not json_str.strip():
        return None

    try:
        data: dict = json.loads(json_str)
    except json.JSONDecodeError:
        return None

    if not isinstance(data, dict):
        return None

    # Validate required fields exist and have correct types
    for field, expected_type in _REQUIRED_FIELDS.items():
        if field not in data:
            return None
        value = data[field]
        # source_level may arrive as a float from JSON; cast to int if whole
        if expected_type is int and isinstance(value, float) and value.is_integer():
            data[field] = int(value)
            value = data[field]
        if not isinstance(value, expected_type):
            return None

    # Validate source_level range
    if data["source_level"] not in _LEVEL_TO_STATE:
        return None

    # Validate that source and destination are known waypoints
    if data["source"] not in WAYPOINTS:
        return None
    if data["destination"] not in WAYPOINTS:
        return None

    return data


def get_waypoint_pose(zone_name: str) -> Optional[tuple[float, float, float]]:
    """Return the (x, y, theta) pose for a named waypoint zone.

    Returns None if the zone is not registered.
    """
    return WAYPOINTS.get(zone_name)


def pick_state_for_level(source_level: int) -> str:
    """Map a source_level integer to the name of the appropriate picking state.

    1 → "PICKING_FLOOR"
    2 → "PICKING_RACK"
    3 → "PICKING_TRUCK"

    Raises ValueError for unrecognised levels.
    """
    state = _LEVEL_TO_STATE.get(source_level)
    if state is None:
        raise ValueError(
            f"Unknown source_level {source_level!r}. "
            f"Valid values: {list(_LEVEL_TO_STATE.keys())}"
        )
    return state
