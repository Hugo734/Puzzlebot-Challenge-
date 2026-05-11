"""
Flask web application for the AMR Warehouse Robot Dashboard.

This module is imported by dashboard_node.py which sets `ros_bridge`
before starting the Flask/SocketIO server.
"""

import base64
import json
import os
import threading
import time
from typing import Optional

from flask import Flask, Response, jsonify, render_template, request
from flask_socketio import SocketIO

# ---------------------------------------------------------------------------
# App configuration
# ---------------------------------------------------------------------------

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_TEMPLATE_DIR = os.path.join(_THIS_DIR, "web", "templates")
_STATIC_DIR = os.path.join(_THIS_DIR, "web", "static")

app = Flask(
    __name__,
    template_folder=_TEMPLATE_DIR,
    static_folder=_STATIC_DIR,
)
app.config["SECRET_KEY"] = os.environ.get("DASHBOARD_SECRET", "amr-dashboard-secret")

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# Set by dashboard_node before the server starts
ros_bridge = None  # type: Optional[object]

# ---------------------------------------------------------------------------
# Valid location choices for mission form validation
# ---------------------------------------------------------------------------

VALID_LOCATIONS = {
    "rack_1", "rack_2", "roller_1", "roller_2",
    "truck_1", "truck_2", "truck_3",
}
VALID_LEVELS = {1, 2, 3}

# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def get_state():
    """Return current robot state as JSON."""
    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503

    state = ros_bridge.get_state()
    return jsonify(
        {
            "pose": state.pose,
            "state": state.state,
            "mission": state.mission,
            "velocity": state.velocity,
            "lifter_level": state.lifter_level,
        }
    )


@app.route("/api/mission", methods=["POST"])
def send_mission():
    """
    Accept a mission JSON and publish it to /mission.

    Expected body:
        {
            "pallet_id": "P001",
            "source": "truck_1",
            "source_level": 3,
            "destination": "rack_2"
        }
    """
    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503

    body = request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Invalid JSON body"}), 400

    # --- Validation ---
    pallet_id = body.get("pallet_id", "").strip()
    source = body.get("source", "")
    destination = body.get("destination", "")
    try:
        source_level = int(body.get("source_level", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "source_level must be an integer"}), 400

    errors = []
    if not pallet_id:
        errors.append("pallet_id is required")
    if source not in VALID_LOCATIONS:
        errors.append(f"source must be one of {sorted(VALID_LOCATIONS)}")
    if destination not in VALID_LOCATIONS:
        errors.append(f"destination must be one of {sorted(VALID_LOCATIONS)}")
    if source_level not in VALID_LEVELS:
        errors.append(f"source_level must be one of {sorted(VALID_LEVELS)}")
    if source == destination:
        errors.append("source and destination must be different")

    if errors:
        return jsonify({"error": "; ".join(errors)}), 400

    mission = {
        "pallet_id": pallet_id,
        "source": source,
        "source_level": source_level,
        "destination": destination,
    }
    ros_bridge.publish_mission(json.dumps(mission))
    return jsonify({"status": "ok", "mission": mission})


@app.route("/api/map")
def get_map():
    """Return the latest occupancy grid map as a PNG image."""
    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503

    png = ros_bridge.get_latest_map()
    if png is None:
        return jsonify({"error": "No map available yet"}), 404

    return Response(png, mimetype="image/png")


@app.route("/video_feed")
def video_feed():
    """MJPEG stream from the camera."""
    return Response(
        _mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ---------------------------------------------------------------------------
# MJPEG generator
# ---------------------------------------------------------------------------

_PLACEHOLDER_JPEG: Optional[bytes] = None


def _get_placeholder_jpeg() -> bytes:
    """Return a small gray 320×240 JPEG used when no camera frame is available."""
    global _PLACEHOLDER_JPEG
    if _PLACEHOLDER_JPEG is not None:
        return _PLACEHOLDER_JPEG

    try:
        import numpy as np

        gray = (128 * np.ones((240, 320, 3), dtype=np.uint8))
        # Add "No Signal" text via OpenCV if available
        try:
            import cv2

            cv2.putText(
                gray, "No Signal", (90, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 136), 2,
            )
            _, buf = cv2.imencode(".jpg", gray)
            _PLACEHOLDER_JPEG = bytes(buf)
        except ImportError:
            from PIL import Image as PILImage, ImageDraw
            import io

            img = PILImage.fromarray(gray.astype("uint8"), "RGB")
            draw = ImageDraw.Draw(img)
            draw.text((90, 110), "No Signal", fill=(0, 255, 136))
            buf = io.BytesIO()
            img.save(buf, format="JPEG")
            _PLACEHOLDER_JPEG = buf.getvalue()
    except Exception:  # noqa: BLE001
        # Absolute fallback
        from dashboard.ros_bridge import RosBridge
        _PLACEHOLDER_JPEG = RosBridge._minimal_jpeg()

    return _PLACEHOLDER_JPEG


def _mjpeg_generator():
    """Yield MJPEG frames at ~15 fps."""
    while True:
        if ros_bridge is not None:
            frame = ros_bridge.get_latest_frame()
        else:
            frame = None

        if frame is None:
            frame = _get_placeholder_jpeg()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )
        time.sleep(1.0 / 15.0)


# ---------------------------------------------------------------------------
# SocketIO background emitter
# ---------------------------------------------------------------------------

def _start_background_emitter() -> None:
    """Spawn the SocketIO emitter thread. Called once by dashboard_node."""
    t = threading.Thread(target=_background_socketio_emitter, daemon=True)
    t.start()


def _background_socketio_emitter() -> None:
    """
    Emit SocketIO events to all connected clients.

    - robot_pose   → 5 Hz
    - robot_state  → on change (polled at 5 Hz)
    - telemetry    → 1 Hz
    """
    last_state_str = ""
    last_telemetry_time = 0.0
    interval = 0.2  # 5 Hz base tick

    while True:
        time.sleep(interval)

        if ros_bridge is None:
            continue

        try:
            state = ros_bridge.get_state()
        except Exception:  # noqa: BLE001
            continue

        # 5 Hz — pose
        socketio.emit(
            "robot_pose",
            {
                "x": state.pose["x"],
                "y": state.pose["y"],
                "theta": state.pose["theta"],
            },
        )

        # On change — robot state + mission
        state_key = f"{state.state}|{json.dumps(state.mission, sort_keys=True)}"
        if state_key != last_state_str:
            last_state_str = state_key
            socketio.emit(
                "robot_state",
                {"state": state.state, "mission": state.mission},
            )

        # 1 Hz — telemetry
        now = time.monotonic()
        if now - last_telemetry_time >= 1.0:
            last_telemetry_time = now

            # Map as base64-encoded PNG (send only when updated)
            map_png = ros_bridge.get_latest_map()
            map_b64 = (
                base64.b64encode(map_png).decode("ascii") if map_png else None
            )

            socketio.emit(
                "telemetry",
                {
                    "linear_vel": state.velocity["linear"],
                    "angular_vel": state.velocity["angular"],
                    "lifter_level": state.lifter_level,
                    "map_png": map_b64,
                },
            )


# ---------------------------------------------------------------------------
# SocketIO connect/disconnect
# ---------------------------------------------------------------------------


@socketio.on("connect")
def on_connect():
    """Push full state to a freshly connected client."""
    if ros_bridge is None:
        return
    state = ros_bridge.get_state()
    socketio.emit(
        "robot_state",
        {"state": state.state, "mission": state.mission},
    )
    socketio.emit(
        "robot_pose",
        {"x": state.pose["x"], "y": state.pose["y"], "theta": state.pose["theta"]},
    )
    socketio.emit(
        "telemetry",
        {
            "linear_vel": state.velocity["linear"],
            "angular_vel": state.velocity["angular"],
            "lifter_level": state.lifter_level,
            "map_png": None,
        },
    )
