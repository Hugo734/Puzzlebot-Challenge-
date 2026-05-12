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


@app.route('/api/odom')
def odom():
    try:
        r = requests.get(f'{WRAPPER}/odom', timeout=0.5)
        return r.json()
    except Exception:
        return jsonify({"x": 0.0, "y": 0.0, "theta": 0.0})


@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    data = request.get_json(silent=True) or {}
    try:
        requests.post(f'{WRAPPER}/cmd_vel', json=data, timeout=0.2)
    except Exception:
        pass
    return jsonify({"ok": True})


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
