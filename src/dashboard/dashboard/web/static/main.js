/**
 * AMR Warehouse Robot Dashboard — main.js
 *
 * Responsibilities:
 *  - Establish Socket.IO connection with auto-reconnect
 *  - Handle robot_pose, robot_state, telemetry, scan_data, nav_plan events
 *  - Update all UI elements reactively
 *  - Submit mission form via fetch POST /api/mission
 *  - Track session stats (missions sent, uptime, WS events)
 *  - Tab switching (MISIÓN / NAVEGAR / TELEOP)
 *  - Waypoint navigation control
 *  - WASD teleop with watchdog
 *  - Canvas map overlay (robot arrow + scan dots + path)
 */

'use strict';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const MAX_LINEAR_VEL  = 0.5;   // m/s — for bar scaling
const MAX_ANGULAR_VEL = 2.0;   // rad/s
const MAX_HISTORY     = 20;

const TELEOP_LINEAR  = 0.18;   // m/s
const TELEOP_ANGULAR = 0.9;    // rad/s

// Map parameters (match warehouse.yaml)
const MAP_ORIGIN_X   = -10.0;  // m
const MAP_ORIGIN_Y   = -10.0;  // m
const MAP_RESOLUTION = 0.05;   // m/pixel
const MAP_SIZE       = 400;    // pixels

// ---------------------------------------------------------------------------
// Session counters
// ---------------------------------------------------------------------------

let _missionCount = 0;
let _eventCount   = 0;
let _startTime    = Date.now();

// ---------------------------------------------------------------------------
// Map overlay state
// ---------------------------------------------------------------------------

let _currentPose = {x: 0, y: 0, theta: 0};
let _currentScan = null;
let _currentPlan = [];

// ---------------------------------------------------------------------------
// Teleop state
// ---------------------------------------------------------------------------

let _keysDown       = new Set();
let _teleopInterval = null;

// ---------------------------------------------------------------------------
// Socket.IO setup
// ---------------------------------------------------------------------------

const socket = io({
  reconnection:        true,
  reconnectionAttempts: Infinity,
  reconnectionDelay:   1000,
  reconnectionDelayMax: 10000,
});

socket.on('connect', () => {
  setConnectionStatus(true);
});

socket.on('disconnect', () => {
  setConnectionStatus(false);
});

socket.on('robot_pose', (data) => {
  _eventCount++;
  _currentPose = {x: data.x, y: data.y, theta: data.theta};
  updatePose(data);
});

socket.on('robot_state', (data) => {
  _eventCount++;
  updateState(data.state, data.mission);
  updateNavStatus(data.state);
});

socket.on('telemetry', (data) => {
  _eventCount++;
  updateTelemetry(data);
});

socket.on('scan_data', (data) => {
  _currentScan = data;
});

socket.on('nav_plan', (data) => {
  _currentPlan = data.points || [];
});

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------

function setConnectionStatus(connected) {
  const dot   = document.getElementById('connDot');
  const label = document.getElementById('connLabel');
  const fDot  = document.getElementById('footerDot');
  const fLbl  = document.getElementById('footerLabel');

  if (connected) {
    dot.className  = 'conn-dot connected';
    fDot.className = 'conn-dot connected';
    label.textContent = 'Connected';
    fLbl.textContent  = 'Connected';
  } else {
    dot.className  = 'conn-dot disconnected';
    fDot.className = 'conn-dot disconnected';
    label.textContent = 'Disconnected — reconnecting…';
    fLbl.textContent  = 'Disconnected';
  }
}

// ---------------------------------------------------------------------------
// Pose update
// ---------------------------------------------------------------------------

function updatePose(data) {
  setText('poseX',     `${data.x.toFixed(3)} m`);
  setText('poseY',     `${data.y.toFixed(3)} m`);
  setText('poseTheta', `${data.theta.toFixed(4)} rad`);
}

// ---------------------------------------------------------------------------
// Robot state + mission update
// ---------------------------------------------------------------------------

function updateState(state, mission) {
  const badge = document.getElementById('stateBadge');
  if (!badge) return;

  badge.textContent = state || 'UNKNOWN';
  // Remove all state classes, add current one
  badge.className = 'state-badge ' + (state || '');

  // Mission box
  const box = document.getElementById('missionBox');
  if (!box) return;

  if (!mission || typeof mission !== 'object') {
    box.innerHTML = '<p class="dim">No active mission</p>';
    return;
  }

  const rows = [
    ['Pallet', mission.pallet_id || '—'],
    ['Source', formatLocation(mission.source)],
    ['Level',  String(mission.source_level || '—')],
    ['Dest',   formatLocation(mission.destination)],
  ];

  box.innerHTML = rows.map(([k, v]) =>
    `<div class="mission-row">
       <span class="mission-key">${k}</span>
       <span class="mission-val">${v}</span>
     </div>`
  ).join('');
}

// ---------------------------------------------------------------------------
// Nav status badge update
// ---------------------------------------------------------------------------

function updateNavStatus(state) {
  const badge = document.getElementById('navStatusBadge');
  if (!badge) return;
  badge.textContent = state || 'UNKNOWN';
  // Extract base status before colon (e.g. "FOLLOWING:truck_1" → "FOLLOWING")
  const base = (state || '').split(':')[0];
  badge.className = 'nav-status-badge ' + base;
}

// ---------------------------------------------------------------------------
// Telemetry update
// ---------------------------------------------------------------------------

function updateTelemetry(data) {
  // Velocities
  const lin = data.linear_vel  ?? 0;
  const ang = data.angular_vel ?? 0;
  setText('velLinear',  `${lin.toFixed(3)} m/s`);
  setText('velAngular', `${ang.toFixed(3)} rad/s`);

  const linPct = Math.min(Math.abs(lin) / MAX_LINEAR_VEL  * 100, 100);
  const angPct = Math.min(Math.abs(ang) / MAX_ANGULAR_VEL * 100, 100);
  setStyle('barLinear',  'width', `${linPct}%`);
  setStyle('barAngular', 'width', `${angPct}%`);

  // Lifter
  const level = data.lifter_level ?? 0;
  updateLifter(level);

  // Map update via base64 PNG
  if (data.map_png) {
    const mapImg    = document.getElementById('mapImg');
    const noDataDiv = document.getElementById('mapNoData');
    if (mapImg) {
      mapImg.src = 'data:image/png;base64,' + data.map_png;
      mapImg.style.display = 'block';
    }
    if (noDataDiv) noDataDiv.style.display = 'none';
  }
}

// ---------------------------------------------------------------------------
// Lifter indicator
// ---------------------------------------------------------------------------

/** Build 8 lifter segments once on load, then toggle .active class. */
function initLifter() {
  const col = document.getElementById('lifterColumn');
  if (!col) return;

  col.innerHTML = '';
  // Segments drawn top=7 → bottom=0
  for (let i = 7; i >= 0; i--) {
    const seg = document.createElement('div');
    seg.className = 'lifter-segment';
    seg.id = `seg${i}`;
    col.appendChild(seg);
  }
}

function updateLifter(level) {
  for (let i = 0; i <= 7; i++) {
    const seg = document.getElementById(`seg${i}`);
    if (!seg) continue;
    if (i <= level) {
      seg.classList.add('active');
    } else {
      seg.classList.remove('active');
    }
  }
  setText('lifterText', String(level));
}

// ---------------------------------------------------------------------------
// Tab switching
// ---------------------------------------------------------------------------

function initTabs() {
  const buttons = document.querySelectorAll('.tab-btn');
  buttons.forEach((btn) => {
    btn.addEventListener('click', () => {
      // Deactivate all tabs
      buttons.forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach((p) => p.classList.add('hidden'));

      // Activate clicked tab
      btn.classList.add('active');
      const tabName = btn.getAttribute('data-tab');
      const pane = document.getElementById(`tab-${tabName}`);
      if (pane) pane.classList.remove('hidden');
    });
  });
}

// ---------------------------------------------------------------------------
// Waypoints dropdown
// ---------------------------------------------------------------------------

async function loadWaypoints() {
  try {
    const res = await fetch('/api/waypoints');
    const data = await res.json();
    const select = document.getElementById('waypointSelect');
    if (!select) return;

    select.innerHTML = '<option value="">— select waypoint —</option>';
    (data.waypoints || []).forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      select.appendChild(opt);
    });
  } catch (err) {
    console.warn('Could not load waypoints:', err);
  }
}

// ---------------------------------------------------------------------------
// Navigation control
// ---------------------------------------------------------------------------

function initNavControls() {
  const btnGo = document.getElementById('btnGoWaypoint');
  const btnCancel = document.getElementById('btnCancelNav');

  if (btnGo) {
    btnGo.addEventListener('click', async () => {
      const select = document.getElementById('waypointSelect');
      const waypoint = select ? select.value : '';
      if (!waypoint) {
        showToast('Selecciona un waypoint primero.', 'error');
        return;
      }
      try {
        const res = await fetch('/api/goal', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({waypoint}),
        });
        const json = await res.json();
        if (json.ok) {
          showToast(`Navegando a: ${waypoint}`, 'success');
        } else {
          showToast(`Error: ${json.error || 'unknown'}`, 'error');
        }
      } catch (err) {
        showToast(`Error de red: ${err.message}`, 'error');
      }
    });
  }

  if (btnCancel) {
    btnCancel.addEventListener('click', async () => {
      try {
        await fetch('/api/goal', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({waypoint: ''}),
        });
        showToast('Navegación cancelada', 'success');
      } catch (err) {
        showToast(`Error de red: ${err.message}`, 'error');
      }
    });
  }
}

// ---------------------------------------------------------------------------
// Teleop — WASD
// ---------------------------------------------------------------------------

function _isInputFocused() {
  const tag = document.activeElement ? document.activeElement.tagName : '';
  return tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA';
}

function _computeTeleopVelocities() {
  let linear  = 0;
  let angular = 0;

  if (_keysDown.has('w') || _keysDown.has('arrowup'))    linear  += TELEOP_LINEAR;
  if (_keysDown.has('s') || _keysDown.has('arrowdown'))  linear  -= TELEOP_LINEAR;
  if (_keysDown.has('a') || _keysDown.has('arrowleft'))  angular += TELEOP_ANGULAR;
  if (_keysDown.has('d') || _keysDown.has('arrowright')) angular -= TELEOP_ANGULAR;

  return {linear, angular};
}

function _startTeleopInterval() {
  if (_teleopInterval !== null) return;
  _teleopInterval = setInterval(async () => {
    if (_keysDown.size === 0) {
      _stopTeleopInterval();
      return;
    }
    const {linear, angular} = _computeTeleopVelocities();
    updateTeleopBars(linear, angular);
    try {
      await fetch('/api/teleop', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({linear, angular}),
      });
    } catch (_) {}
  }, 100);
}

function _stopTeleopInterval() {
  if (_teleopInterval !== null) {
    clearInterval(_teleopInterval);
    _teleopInterval = null;
  }
}

function _highlightKeys() {
  const map = {w: 'keyW', a: 'keyA', s: 'keyS', d: 'keyD'};
  Object.entries(map).forEach(([k, id]) => {
    const el = document.getElementById(id);
    if (!el) return;
    if (_keysDown.has(k)) {
      el.classList.add('pressed');
    } else {
      el.classList.remove('pressed');
    }
  });
}

function initTeleop() {
  document.addEventListener('keydown', (e) => {
    if (_isInputFocused()) return;

    const key = e.key.toLowerCase();

    // E-stop on spacebar
    if (key === ' ') {
      e.preventDefault();
      triggerEstop();
      return;
    }

    const validKeys = new Set(['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright']);
    if (!validKeys.has(key)) return;

    e.preventDefault();
    _keysDown.add(key);
    _highlightKeys();
    _startTeleopInterval();
  });

  document.addEventListener('keyup', (e) => {
    if (_isInputFocused()) return;

    const key = e.key.toLowerCase();
    _keysDown.delete(key);
    _highlightKeys();

    if (_keysDown.size === 0) {
      _stopTeleopInterval();
      updateTeleopBars(0, 0);
      fetch('/api/teleop/stop', {method: 'POST'}).catch(() => {});
    }
  });

  const btnEstop = document.getElementById('btnEstop');
  if (btnEstop) {
    btnEstop.addEventListener('click', triggerEstop);
  }
}

function triggerEstop() {
  _keysDown.clear();
  _stopTeleopInterval();
  _highlightKeys();
  updateTeleopBars(0, 0);
  fetch('/api/teleop/stop', {method: 'POST'}).catch(() => {});
}

// ---------------------------------------------------------------------------
// Teleop velocity bars (bipolar)
// ---------------------------------------------------------------------------

function updateTeleopBars(linear, angular) {
  const linPct = Math.min(Math.abs(linear) / TELEOP_LINEAR * 50, 50);
  const angPct = Math.min(Math.abs(angular) / TELEOP_ANGULAR * 50, 50);

  const tBarL = document.getElementById('tBarLinear');
  const tBarA = document.getElementById('tBarAngular');

  if (tBarL) {
    tBarL.style.width = linPct + '%';
    tBarL.style.left  = linear >= 0 ? '50%' : (50 - linPct) + '%';
  }
  if (tBarA) {
    tBarA.style.width = angPct + '%';
    tBarA.style.left  = angular >= 0 ? '50%' : (50 - angPct) + '%';
  }

  setText('tVelLinear',  linear.toFixed(2));
  setText('tVelAngular', angular.toFixed(2));
}

// ---------------------------------------------------------------------------
// Map overlay — canvas drawing
// ---------------------------------------------------------------------------

function worldToCanvas(wx, wy, canvasW, canvasH) {
  const px = (wx - MAP_ORIGIN_X) / MAP_RESOLUTION;
  const py = MAP_SIZE - (wy - MAP_ORIGIN_Y) / MAP_RESOLUTION;
  return [px * canvasW / MAP_SIZE, py * canvasH / MAP_SIZE];
}

function drawMapOverlay() {
  const canvas = document.getElementById('mapOverlay');
  const mapImg = document.getElementById('mapImg');
  if (!canvas || !mapImg) return;

  // Resize canvas to match rendered image
  const rect = mapImg.getBoundingClientRect();
  canvas.width  = rect.width  || mapImg.offsetWidth;
  canvas.height = rect.height || mapImg.offsetHeight;
  if (!canvas.width || !canvas.height) return;

  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const cw = canvas.width;
  const ch = canvas.height;

  // Draw path (orange line)
  if (_currentPlan && _currentPlan.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = '#ff6600';
    ctx.lineWidth = 2;
    _currentPlan.forEach((pt, i) => {
      const [cx, cy] = worldToCanvas(pt.x, pt.y, cw, ch);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    // Draw dots at each waypoint
    ctx.fillStyle = '#ff9900';
    _currentPlan.forEach((pt) => {
      const [cx, cy] = worldToCanvas(pt.x, pt.y, cw, ch);
      ctx.beginPath();
      ctx.arc(cx, cy, 3, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // Draw scan (red dots)
  if (_currentScan && _currentScan.ranges) {
    const {ranges, angle_min, angle_increment} = _currentScan;
    ctx.fillStyle = 'rgba(255, 60, 60, 0.7)';
    ranges.forEach((r, i) => {
      if (r >= (_currentScan.range_max || 12)) return;
      if (r <= 0) return;
      const angle = _currentPose.theta + angle_min + i * angle_increment;
      const wx = _currentPose.x + r * Math.cos(angle);
      const wy = _currentPose.y + r * Math.sin(angle);
      const [cx, cy] = worldToCanvas(wx, wy, cw, ch);
      ctx.beginPath();
      ctx.arc(cx, cy, 1.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // Draw robot (green arrow)
  const [rx, ry] = worldToCanvas(_currentPose.x, _currentPose.y, cw, ch);
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-_currentPose.theta);  // canvas Y is inverted
  ctx.fillStyle = '#00ff88';
  ctx.strokeStyle = '#00aa55';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(8, 0);
  ctx.lineTo(-5, -5);
  ctx.lineTo(-3, 0);
  ctx.lineTo(-5, 5);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

// Redraw map overlay at 10 Hz
setInterval(drawMapOverlay, 100);

// ---------------------------------------------------------------------------
// Mission form
// ---------------------------------------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
  initLifter();
  startUptimeClock();
  initTabs();
  loadWaypoints();
  initNavControls();
  initTeleop();

  const form = document.getElementById('missionForm');
  if (!form) return;

  form.addEventListener('submit', async (e) => {
    e.preventDefault();

    const palletId    = document.getElementById('palletId').value.trim();
    const source      = document.getElementById('sourceSelect').value;
    const sourceLevel = parseInt(document.getElementById('sourceLevelSelect').value, 10);
    const destination = document.getElementById('destSelect').value;

    if (!palletId || !source || !sourceLevel || !destination) {
      showToast('Please fill in all fields.', 'error');
      return;
    }
    if (source === destination) {
      showToast('Source and destination must be different.', 'error');
      return;
    }

    const btn = form.querySelector('.btn-submit');
    btn.disabled = true;

    try {
      const res = await fetch('/api/mission', {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pallet_id: palletId, source, source_level: sourceLevel, destination }),
      });

      const json = await res.json();

      if (!res.ok || json.error) {
        showToast(`Error: ${json.error || 'Unknown error'}`, 'error');
        return;
      }

      showToast(`Mission sent: ${palletId} from ${formatLocation(source)} to ${formatLocation(destination)}`, 'success');
      addHistoryItem(json.mission);
      _missionCount++;
      setText('statMissions', String(_missionCount));
      form.reset();

    } catch (err) {
      showToast(`Network error: ${err.message}`, 'error');
    } finally {
      btn.disabled = false;
    }
  });
});

// ---------------------------------------------------------------------------
// History
// ---------------------------------------------------------------------------

function addHistoryItem(mission) {
  const list = document.getElementById('historyList');
  if (!list) return;

  // Remove placeholder
  const placeholder = list.querySelector('.dim');
  if (placeholder) placeholder.remove();

  const li = document.createElement('li');
  li.className = 'history-item';
  li.innerHTML =
    `<span class="hist-id">${mission.pallet_id}</span>
     <span class="hist-meta"> · ${formatLocation(mission.source)} (L${mission.source_level}) → ${formatLocation(mission.destination)}</span>`;

  list.insertBefore(li, list.firstChild);

  // Trim to max
  while (list.children.length > MAX_HISTORY) {
    list.removeChild(list.lastChild);
  }
}

// ---------------------------------------------------------------------------
// Toast
// ---------------------------------------------------------------------------

let _toastTimer = null;

function showToast(msg, type = 'success') {
  const toast = document.getElementById('toast');
  if (!toast) return;

  toast.textContent = msg;
  toast.className   = `toast ${type}`;

  if (_toastTimer) clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => {
    toast.className = 'toast hidden';
  }, 4000);
}

// ---------------------------------------------------------------------------
// Uptime clock
// ---------------------------------------------------------------------------

function startUptimeClock() {
  setInterval(() => {
    const elapsed = Math.floor((Date.now() - _startTime) / 1000);
    const h = Math.floor(elapsed / 3600);
    const m = Math.floor((elapsed % 3600) / 60);
    const s = elapsed % 60;
    setText('statUptime', `${pad(h)}:${pad(m)}:${pad(s)}`);
    setText('statEvents', String(_eventCount));
  }, 1000);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function setStyle(id, prop, value) {
  const el = document.getElementById(id);
  if (el) el.style[prop] = value;
}

function pad(n) {
  return String(n).padStart(2, '0');
}

/**
 * Format a location identifier for human display.
 * "truck_1" → "Truck 1", "rack_2" → "Rack 2"
 */
function formatLocation(loc) {
  if (!loc) return '—';
  return loc
    .replace('_', ' ')
    .replace(/\b\w/g, (c) => c.toUpperCase());
}
