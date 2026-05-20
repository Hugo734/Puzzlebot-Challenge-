"""
Flask web application for the AMR Warehouse Robot Dashboard.

This module is imported by dashboard_node.py which sets `ros_bridge`
before starting the Flask/SocketIO server.
"""

import base64
import io
import json
import os
import threading
import time
from typing import Optional

import numpy as np
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
# HMM voice recognizer (lazy-loaded on first /api/voice call)
# ---------------------------------------------------------------------------

_hmm_recognizer = None
_hmm_lock = threading.Lock()


def _get_hmm_recognizer():
    global _hmm_recognizer
    if _hmm_recognizer is not None:
        return _hmm_recognizer
    with _hmm_lock:
        if _hmm_recognizer is not None:
            return _hmm_recognizer
        try:
            from voice_control.hmm_recognizer import HMMRecognizer  # type: ignore[import]
            r = HMMRecognizer()
            if r.load():
                _hmm_recognizer = r
            else:
                # Try source tree path as fallback
                src_models = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    '..', '..', '..', 'voice_control', 'models',
                )
                r2 = HMMRecognizer(models_dir=os.path.normpath(src_models))
                if r2.load():
                    _hmm_recognizer = r2
        except Exception:  # noqa: BLE001
            pass
    return _hmm_recognizer

# ---------------------------------------------------------------------------
# Valid location choices for mission form validation
# ---------------------------------------------------------------------------

VALID_LOCATIONS = {
    "rack_1", "rack_2", "roller_1", "roller_2",
    "truck_1", "truck_2", "truck_3",
}
VALID_LEVELS = {1, 2, 3}

# ---------------------------------------------------------------------------
# Teleop watchdog state
# ---------------------------------------------------------------------------

_last_teleop_time: float = 0.0
_teleop_active: bool = False
_teleop_lock = threading.Lock()


def _teleop_watchdog() -> None:
    """Stop the robot if no teleop command received for 400ms."""
    global _teleop_active
    while True:
        time.sleep(0.05)
        with _teleop_lock:
            active = _teleop_active
            last = _last_teleop_time
        if active and ros_bridge is not None:
            if time.monotonic() - last > 0.35:
                try:
                    ros_bridge.publish_cmd_vel(0.0, 0.0)
                except Exception:  # noqa: BLE001
                    pass
                with _teleop_lock:
                    _teleop_active = False


# Start watchdog thread eagerly (daemon, won't block shutdown)
_watchdog_thread = threading.Thread(target=_teleop_watchdog, daemon=True)
_watchdog_thread.start()

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


@app.route("/api/waypoints", methods=["GET"])
def get_waypoints():
    """Return waypoints with full position data {name: {x, y, theta}}."""
    if ros_bridge is None:
        return jsonify({"waypoints": {}})
    wps = ros_bridge.get_waypoints()
    result = {}
    for name, data in wps.items():
        if isinstance(data, dict):
            result[name] = {
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "theta": float(data.get("theta", 0.0)),
            }
    return jsonify({"waypoints": result})


@app.route("/api/waypoints", methods=["POST"])
def add_waypoint():
    """Add or update a waypoint and persist to YAML."""
    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503
    body = request.get_json(silent=True) or {}
    name = (body.get("name") or "").strip()
    if not name:
        return jsonify({"error": "name is required"}), 400
    try:
        x = float(body.get("x", 0.0))
        y = float(body.get("y", 0.0))
        theta = float(body.get("theta", 0.0))
    except (TypeError, ValueError):
        return jsonify({"error": "x, y, theta must be numbers"}), 400
    ros_bridge.add_waypoint(name, x, y, theta)
    return jsonify({"ok": True})


@app.route("/api/goal", methods=["POST"])
def send_goal():
    """
    Send a navigation goal waypoint.

    Body: {"waypoint": "truck_1"} or {"waypoint": ""} to cancel.
    """
    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503

    body = request.get_json(silent=True) or {}
    waypoint = body.get("waypoint", "")

    ros_bridge.publish_goal(waypoint)
    return jsonify({"ok": True})


@app.route("/api/teleop", methods=["POST"])
def send_teleop():
    """
    Send a teleop velocity command.

    Body: {"linear": 0.15, "angular": 0.0}
    """
    global _last_teleop_time, _teleop_active

    if ros_bridge is None:
        return jsonify({"error": "ROS bridge not initialised"}), 503

    body = request.get_json(silent=True) or {}
    try:
        linear = float(body.get("linear", 0.0))
        angular = float(body.get("angular", 0.0))
    except (TypeError, ValueError):
        return jsonify({"error": "linear and angular must be numbers"}), 400

    ros_bridge.publish_cmd_vel(linear, angular)

    with _teleop_lock:
        _last_teleop_time = time.monotonic()
        _teleop_active = True

    return jsonify({"ok": True})


@app.route("/api/voice", methods=["POST"])
def voice_command():
    """
    Receive an audio blob from the browser, run HMM recognition,
    publish the result to /voice_command, and return the recognised word.

    The browser should POST multipart/form-data with field 'audio'
    containing a WebM/OGG audio blob, or raw WAV bytes with
    Content-Type: audio/wav.
    """
    recognizer = _get_hmm_recognizer()
    if recognizer is None:
        return jsonify({"error": "HMM models not loaded"}), 503

    # ── Decode incoming audio ──────────────────────────────────────
    raw = None
    content_type = request.content_type or ""

    if "multipart/form-data" in content_type:
        f = request.files.get("audio")
        if f is None:
            return jsonify({"error": "No audio field in form"}), 400
        raw = f.read()
        fname = f.filename or ""
    else:
        raw = request.get_data()
        fname = ""

    if not raw:
        return jsonify({"error": "Empty audio data"}), 400

    # ── Convert to float32 numpy array at 16 kHz ──────────────────
    signal = None
    fs_in  = 16000

    # Try WAV first (scipy — no extra deps)
    try:
        import scipy.io.wavfile as wav_io
        fs_in, data = wav_io.read(io.BytesIO(raw))
        data = data.astype(np.float32)
        if data.ndim > 1:
            data = data.mean(axis=1)
        if data.max() > 1.5:
            data /= 32768.0
        signal = data
    except Exception:  # noqa: BLE001
        pass

    # Fallback: pydub (handles webm, ogg, mp3…)
    if signal is None:
        try:
            from pydub import AudioSegment
            seg = (
                AudioSegment.from_file(io.BytesIO(raw))
                .set_channels(1)
                .set_frame_rate(16000)
            )
            arr = np.array(seg.get_array_of_samples(), dtype=np.float32)
            arr /= 2 ** (seg.sample_width * 8 - 1)
            signal = arr
            fs_in  = 16000
        except Exception:  # noqa: BLE001
            return jsonify({"error": "Could not decode audio (WAV or WebM/pydub required)"}), 400

    # Resample if needed
    if fs_in != 16000:
        import scipy.signal as sp_signal
        signal = sp_signal.resample(signal, int(len(signal) * 16000 / fs_in)).astype(np.float32)

    if len(signal) < 800:
        return jsonify({"error": "Audio too short"}), 400

    # ── Recognise ─────────────────────────────────────────────────
    word = recognizer.recognize(signal, fs=16000)
    if word is None:
        return jsonify({"error": "Recognition failed"}), 500

    # ── Publish to ROS2 ───────────────────────────────────────────
    if ros_bridge is not None:
        try:
            ros_bridge.publish_voice_command(word)
        except Exception:  # noqa: BLE001
            pass

    socketio.emit("voice_result", {"word": word})
    return jsonify({"word": word})


@app.route("/api/voice/status")
def voice_status():
    """Return whether the HMM recognizer is loaded and which words it knows."""
    r = _get_hmm_recognizer()
    if r is None:
        return jsonify({"ready": False, "vocabulary": []})
    return jsonify({"ready": True, "vocabulary": r.vocabulary})


@app.route("/api/teleop/stop", methods=["POST"])
def stop_teleop():
    """Immediately stop the robot."""
    global _teleop_active

    if ros_bridge is not None:
        ros_bridge.publish_cmd_vel(0.0, 0.0)

    with _teleop_lock:
        _teleop_active = False

    return jsonify({"ok": True})


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
    - scan_data    → 5 Hz (every other 200ms tick)
    - nav_plan     → on change
    """
    last_state_str = ""
    last_telemetry_time = 0.0
    last_nav_plan_str = ""
    tick = 0
    interval = 0.2  # 5 Hz base tick

    while True:
        time.sleep(interval)
        tick += 1

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

        # 5 Hz (every other tick ~2.5 Hz effectively, but emit every tick for simplicity)
        # Emit scan_data at 5 Hz when available
        if tick % 2 == 0 and state.scan is not None:
            socketio.emit("scan_data", state.scan)

        # Nav plan — on change
        nav_plan_str = json.dumps(state.nav_plan, sort_keys=True)
        if nav_plan_str != last_nav_plan_str:
            last_nav_plan_str = nav_plan_str
            socketio.emit(
                "nav_plan",
                {"points": state.nav_plan or []},
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
