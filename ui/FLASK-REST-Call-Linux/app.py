from flask import Flask, Response, jsonify, request
import requests
import time
import threading

app = Flask(__name__)
WRAPPER = 'http://127.0.0.1:7043'

_latest_frame = None
_frame_lock = threading.Lock()


def _frame_poller():
    global _latest_frame
    while True:
        try:
            r = requests.get(f'{WRAPPER}/image', timeout=0.5)
            if r.status_code == 200:
                with _frame_lock:
                    _latest_frame = r.content
        except Exception:
            pass
        time.sleep(0.033)


threading.Thread(target=_frame_poller, daemon=True).start()


@app.route('/')
def index():
    html_path = app.root_path + '/templates/result.html'
    with open(html_path, 'r') as f:
        return f.read(), 200, {'Content-Type': 'text/html; charset=utf-8'}


@app.route('/api/urdf/<path:filename>')
def serve_urdf(filename):
    import os
    import subprocess
    # Security: only allow URDF files
    if not filename.endswith(('.urdf', '.xacro')):
        return jsonify({"error": "Invalid file"}), 400

    # app.root_path is /home/jjj/Documents/Puzzlebot-Challenge-/src/ui/FLASK-REST-Call-Linux
    urdf_path = os.path.join(app.root_path, '..', '..', 'puzzlebot_description', 'urdf', filename)

    if not os.path.exists(urdf_path):
        return jsonify({"error": "File not found"}), 404

    # If xacro file, process it to URDF
    if filename.endswith('.xacro'):
        try:
            result = subprocess.run(['xacro', urdf_path], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout, 200, {'Content-Type': 'application/xml'}
            else:
                return jsonify({"error": result.stderr}), 500
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    # Serve URDF directly
    with open(urdf_path, 'r') as f:
        return f.read(), 200, {'Content-Type': 'application/xml'}


@app.route('/api/mesh/<path:filename>')
def serve_mesh(filename):
    import os
    # Security: only allow STL and DAE files
    if not filename.endswith(('.stl', '.dae')):
        return jsonify({"error": "Invalid file"}), 400

    mesh_path = os.path.join(app.root_path, '..', '..', 'puzzlebot_description', 'meshes', filename)

    if not os.path.exists(mesh_path):
        return jsonify({"error": "File not found"}), 404

    with open(mesh_path, 'rb') as f:
        content = f.read()

    mime_type = 'application/octet-stream'
    if filename.endswith('.stl'):
        mime_type = 'application/octet-stream'
    elif filename.endswith('.dae'):
        mime_type = 'application/collada+xml'

    return content, 200, {'Content-Type': mime_type}


@app.route('/api/odom')
def odom():
    try:
        r = requests.get(f'{WRAPPER}/odom', timeout=0.5)
        return r.json()
    except Exception:
        return jsonify({"x": 0.0, "y": 0.0, "theta": 0.0})


@app.route('/api/map')
def map_data():
    try:
        r = requests.get(f'{WRAPPER}/map', timeout=1.0)
        return r.json()
    except Exception:
        return jsonify({"loaded": False}), 503


@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    data = request.get_json(silent=True) or {}
    try:
        requests.post(f'{WRAPPER}/cmd_vel', json=data, timeout=0.2)
    except Exception:
        pass
    return jsonify({"ok": True})


@app.route('/api/teleop/enable', methods=['POST'])
def teleop_enable():
    try:
        r = requests.post(f'{WRAPPER}/teleop/enable', timeout=0.5)
        return r.json()
    except Exception:
        return jsonify({"ok": False}), 503


@app.route('/api/teleop/disable', methods=['POST'])
def teleop_disable():
    try:
        r = requests.post(f'{WRAPPER}/teleop/disable', timeout=0.5)
        return r.json()
    except Exception:
        return jsonify({"ok": False}), 503


@app.route('/api/lifter', methods=['POST'])
def lifter():
    data = request.get_json(silent=True) or {}
    try:
        r = requests.post(f'{WRAPPER}/lifter', json=data, timeout=0.5)
        return r.json()
    except Exception:
        return jsonify({"ok": False}), 503


def _gen_mjpeg():
    while True:
        with _frame_lock:
            frame = _latest_frame
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)


@app.route('/api/camera')
def camera():
    return Response(_gen_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(debug=False, port=8002, threaded=True)
