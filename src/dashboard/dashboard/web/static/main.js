'use strict';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const MAX_LINEAR_VEL  = 0.5;
const MAX_ANGULAR_VEL = 2.0;
const MAX_HISTORY     = 20;

const TELEOP_LINEAR  = 0.18;
const TELEOP_ANGULAR = 0.9;

// Map world-space parameters — updated dynamically from /api/map/info
let MAP_ORIGIN_X   = -10.0;  // m
let MAP_ORIGIN_Y   = -10.0;  // m
let MAP_RESOLUTION = 0.05;   // m/pixel

// ---------------------------------------------------------------------------
// Session counters
// ---------------------------------------------------------------------------

let _missionCount = 0;
let _eventCount   = 0;
let _startTime    = Date.now();

// ---------------------------------------------------------------------------
// Map state
// ---------------------------------------------------------------------------

// Single Image element reused for map updates (avoids object churn)
const _mapImg = new Image();

// Letterbox rect computed each draw frame: {drawX, drawY, drawW, drawH, mapW, mapH}
let _mapRender = null;

let _currentPose = {x: 0, y: 0, theta: 0};
let _currentScan = null;
let _currentPlan = [];

// Waypoints: {name: {x, y, theta}}
let _waypoints = {};

// Edit mode
let _editMode    = false;
let _pendingWp   = null;  // {x, y, theta} world coords of pending placement
let _dragWp      = null;  // {wx0, wy0, wx1, wy1} live drag preview

// ---------------------------------------------------------------------------
// Teleop state
// ---------------------------------------------------------------------------

let _keysDown       = new Set();
let _teleopInterval = null;

// ---------------------------------------------------------------------------
// Socket.IO
// ---------------------------------------------------------------------------

const socket = io({
  reconnection: true,
  reconnectionAttempts: Infinity,
  reconnectionDelay: 1000,
  reconnectionDelayMax: 10000,
});

socket.on('connect',    () => setConnectionStatus(true));
socket.on('disconnect', () => setConnectionStatus(false));

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
  if (data && data.system_mode && data.system_mode !== _systemMode) {
    applyModeToUI(data.system_mode);
  }
});

socket.on('scan_data', (data) => { _currentScan = data; });

socket.on('nav_plan', (data) => { _currentPlan = data.points || []; });

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------

function setConnectionStatus(connected) {
  const cls = connected ? 'conn-dot connected' : 'conn-dot disconnected';
  const lbl = connected ? 'Connected' : 'Disconnected — reconnecting…';
  document.getElementById('connDot').className   = cls;
  document.getElementById('footerDot').className = cls;
  document.getElementById('connLabel').textContent  = lbl;
  document.getElementById('footerLabel').textContent = connected ? 'Connected' : 'Disconnected';
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
// Robot state + mission
// ---------------------------------------------------------------------------

function updateState(state, mission) {
  const badge = document.getElementById('stateBadge');
  if (!badge) return;
  badge.textContent = state || 'UNKNOWN';
  badge.className   = 'state-badge ' + (state || '');

  const box = document.getElementById('missionBox');
  if (!box) return;
  if (!mission || typeof mission !== 'object') {
    box.innerHTML = '<p class="dim">No active mission</p>';
    return;
  }
  box.innerHTML = [
    ['Pallet', mission.pallet_id || '—'],
    ['Source', formatLocation(mission.source)],
    ['Level',  String(mission.source_level || '—')],
    ['Dest',   formatLocation(mission.destination)],
  ].map(([k, v]) =>
    `<div class="mission-row"><span class="mission-key">${k}</span><span class="mission-val">${v}</span></div>`
  ).join('');
}

// ---------------------------------------------------------------------------
// Nav status badge
// ---------------------------------------------------------------------------

function updateNavStatus(state) {
  const badge = document.getElementById('navStatusBadge');
  if (!badge) return;
  badge.textContent = state || 'UNKNOWN';
  badge.className   = 'nav-status-badge ' + (state || '').split(':')[0];
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

function updateTelemetry(data) {
  const lin = data.linear_vel  ?? 0;
  const ang = data.angular_vel ?? 0;
  setText('velLinear',  `${lin.toFixed(3)} m/s`);
  setText('velAngular', `${ang.toFixed(3)} rad/s`);
  setStyle('barLinear',  'width', `${Math.min(Math.abs(lin) / MAX_LINEAR_VEL  * 100, 100)}%`);
  setStyle('barAngular', 'width', `${Math.min(Math.abs(ang) / MAX_ANGULAR_VEL * 100, 100)}%`);

  updateLifter(data.lifter_level ?? 0);

  if (data.map_png) {
    _mapImg.src = 'data:image/png;base64,' + data.map_png;
  }
  if (data.map_info) {
    applyMapInfo(data.map_info);
  }
}

function applyMapInfo(info) {
  if (typeof info.origin_x   === 'number') MAP_ORIGIN_X   = info.origin_x;
  if (typeof info.origin_y   === 'number') MAP_ORIGIN_Y   = info.origin_y;
  if (typeof info.resolution === 'number') MAP_RESOLUTION = info.resolution;

  const badge = document.getElementById('mapSourceBadge');
  if (badge) {
    const live = info.source === 'live';
    badge.textContent = live ? 'LIVE' : 'STATIC MAP';
    badge.className   = 'map-source-badge ' + (live ? 'live' : 'static');
  }
}

async function loadMapInfo() {
  try {
    const res = await fetch('/api/map/info');
    if (!res.ok) return;
    const info = await res.json();
    applyMapInfo(info);
  } catch (_) {}
}

// ---------------------------------------------------------------------------
// Lifter
// ---------------------------------------------------------------------------

function initLifter() {
  const col = document.getElementById('lifterColumn');
  if (!col) return;
  col.innerHTML = '';
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
    seg.classList.toggle('active', i <= level);
  }
  setText('lifterText', String(level));
}

// ---------------------------------------------------------------------------
// Tab switching
// ---------------------------------------------------------------------------

function initTabs() {
  document.querySelectorAll('.tab-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach(p => p.classList.add('hidden'));
      btn.classList.add('active');
      const pane = document.getElementById(`tab-${btn.dataset.tab}`);
      if (pane) pane.classList.remove('hidden');
    });
  });
}

// ---------------------------------------------------------------------------
// Waypoints — load full {name: {x, y, theta}} data
// ---------------------------------------------------------------------------

async function loadWaypoints() {
  try {
    const res  = await fetch('/api/waypoints');
    const data = await res.json();
    _waypoints = data.waypoints || {};

    const select = document.getElementById('waypointSelect');
    if (!select) return;
    const names = Object.keys(_waypoints).sort();
    select.innerHTML = '<option value="">— select waypoint —</option>';
    names.forEach(name => {
      const opt = document.createElement('option');
      opt.value       = name;
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

async function deleteWaypoint(name) {
  if (!confirm(`Delete waypoint "${name}"?`)) return;
  try {
    const res = await fetch(`/api/waypoints/${encodeURIComponent(name)}`, {
      method: 'DELETE',
    });
    if (res.ok) {
      showToast(`Waypoint "${name}" deleted`, 'success');
      await loadWaypoints();
    } else {
      const data = await res.json().catch(() => ({}));
      showToast(`Delete failed: ${data.error || res.status}`, 'error');
    }
  } catch (err) {
    showToast(`Network error: ${err.message}`, 'error');
  }
}

async function sendGoalWaypoint(name) {
  try {
    const res  = await fetch('/api/goal', {
      method:  'POST',
      headers: {'Content-Type': 'application/json'},
      body:    JSON.stringify({waypoint: name}),
    });
    const json = await res.json();
    if (json.ok) {
      showToast(name ? `Navigating to: ${name}` : 'Navigation cancelled', 'success');
    } else {
      showToast(`Error: ${json.error || 'unknown'}`, 'error');
    }
  } catch (err) {
    showToast(`Network error: ${err.message}`, 'error');
  }
}

function initNavControls() {
  const btnGo     = document.getElementById('btnGoWaypoint');
  const btnCancel = document.getElementById('btnCancelNav');

  btnGo?.addEventListener('click', () => {
    const sel = document.getElementById('waypointSelect');
    const wp  = sel ? sel.value : '';
    if (!wp) { showToast('Select a waypoint first.', 'error'); return; }
    sendGoalWaypoint(wp);
  });

  btnCancel?.addEventListener('click', () => sendGoalWaypoint(''));
}

// ---------------------------------------------------------------------------
// MAP CANVAS — coordinate transforms
// ---------------------------------------------------------------------------

/**
 * Convert ROS world coords (m) → canvas pixel coords.
 * Requires _mapRender to be set by drawMapOverlay.
 */
function worldToCanvas(wx, wy) {
  if (!_mapRender) return [0, 0];
  const {drawX, drawY, drawW, drawH, mapW, mapH} = _mapRender;
  const px = (wx - MAP_ORIGIN_X) / MAP_RESOLUTION;
  const py = mapH - (wy - MAP_ORIGIN_Y) / MAP_RESOLUTION;
  return [drawX + px * drawW / mapW, drawY + py * drawH / mapH];
}

/**
 * Convert canvas pixel coords → ROS world coords (m).
 */
function canvasToWorld(cx, cy) {
  if (!_mapRender) return [0, 0];
  const {drawX, drawY, drawW, drawH, mapW, mapH} = _mapRender;
  const px = (cx - drawX) * mapW / drawW;
  const py = (cy - drawY) * mapH / drawH;
  return [
    px * MAP_RESOLUTION + MAP_ORIGIN_X,
    (mapH - py) * MAP_RESOLUTION + MAP_ORIGIN_Y,
  ];
}

// ---------------------------------------------------------------------------
// MAP CANVAS — main draw loop (10 Hz)
// ---------------------------------------------------------------------------

function drawMapOverlay() {
  const canvas = document.getElementById('mapCanvas');
  if (!canvas) return;

  const container = canvas.parentElement;
  const cw = container.clientWidth;
  const ch = container.clientHeight;
  if (!cw || !ch) return;

  // Update canvas resolution to match CSS display size
  if (canvas.width !== cw || canvas.height !== ch) {
    canvas.width  = cw;
    canvas.height = ch;
  }

  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, cw, ch);

  // ---- Draw map image (letterboxed) ----
  if (_mapImg.complete && _mapImg.naturalWidth > 0) {
    const iw = _mapImg.naturalWidth;
    const ih = _mapImg.naturalHeight;
    const imgRatio = iw / ih;
    const canRatio = cw / ch;

    let drawW, drawH, drawX, drawY;
    if (imgRatio >= canRatio) {
      drawW = cw;
      drawH = cw / imgRatio;
      drawX = 0;
      drawY = (ch - drawH) / 2;
    } else {
      drawH = ch;
      drawW = ch * imgRatio;
      drawX = (cw - drawW) / 2;
      drawY = 0;
    }

    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(_mapImg, drawX, drawY, drawW, drawH);
    _mapRender = {drawX, drawY, drawW, drawH, mapW: iw, mapH: ih};
  } else {
    _mapRender = null;
    // Waiting placeholder
    ctx.fillStyle = '#0a0f1e';
    ctx.fillRect(0, 0, cw, ch);
    ctx.fillStyle = '#4a5f80';
    ctx.font = '14px "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('Waiting for map data…', cw / 2, ch / 2);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'alphabetic';
    return;
  }

  // ---- Draw LiDAR scan (red dots) ----
  if (_currentScan && _currentScan.ranges) {
    const {ranges, angle_min, angle_increment, range_max} = _currentScan;
    ctx.fillStyle = 'rgba(255, 55, 55, 0.65)';
    ranges.forEach((r, i) => {
      if (r <= 0 || r >= (range_max || 12)) return;
      const angle = _currentPose.theta + angle_min + i * angle_increment;
      const [cx2, cy2] = worldToCanvas(
        _currentPose.x + r * Math.cos(angle),
        _currentPose.y + r * Math.sin(angle)
      );
      ctx.beginPath();
      ctx.arc(cx2, cy2, 1.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // ---- Draw navigation path (orange) ----
  if (_currentPlan && _currentPlan.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(255, 100, 0, 0.85)';
    ctx.lineWidth   = 2;
    _currentPlan.forEach((pt, i) => {
      const [cx2, cy2] = worldToCanvas(pt.x, pt.y);
      i === 0 ? ctx.moveTo(cx2, cy2) : ctx.lineTo(cx2, cy2);
    });
    ctx.stroke();

    ctx.fillStyle = '#ff9900';
    _currentPlan.forEach((pt) => {
      const [cx2, cy2] = worldToCanvas(pt.x, pt.y);
      ctx.beginPath();
      ctx.arc(cx2, cy2, 3, 0, Math.PI * 2);
      ctx.fill();
    });
  }

  // ---- Draw waypoints ----
  drawWaypoints(ctx);

  // ---- Draw drag preview (RViz-style "click & drag pose") ----
  if (_dragWp) {
    const [c0x, c0y] = worldToCanvas(_dragWp.wx0, _dragWp.wy0);
    const [c1x, c1y] = worldToCanvas(_dragWp.wx1, _dragWp.wy1);
    const dragColor = '#cc88ff';

    ctx.save();
    ctx.shadowColor = dragColor;
    ctx.shadowBlur  = 8;

    // Origin marker
    ctx.beginPath();
    ctx.arc(c0x, c0y, 8, 0, Math.PI * 2);
    ctx.fillStyle = dragColor + '40';
    ctx.fill();
    ctx.strokeStyle = dragColor;
    ctx.lineWidth   = 2;
    ctx.stroke();

    const distPx = Math.hypot(c1x - c0x, c1y - c0y);
    if (distPx > 4) {
      const ang = Math.atan2(c1y - c0y, c1x - c0x);
      // Shaft
      ctx.beginPath();
      ctx.moveTo(c0x, c0y);
      ctx.lineTo(c1x, c1y);
      ctx.strokeStyle = dragColor;
      ctx.lineWidth   = 3;
      ctx.stroke();
      // Arrowhead
      const ahLen = 12;
      ctx.beginPath();
      ctx.moveTo(c1x, c1y);
      ctx.lineTo(c1x - ahLen * Math.cos(ang - 0.45),
                 c1y - ahLen * Math.sin(ang - 0.45));
      ctx.lineTo(c1x - ahLen * Math.cos(ang + 0.45),
                 c1y - ahLen * Math.sin(ang + 0.45));
      ctx.closePath();
      ctx.fillStyle = dragColor;
      ctx.fill();
    }
    ctx.restore();
  }

  // ---- Draw robot arrow ----
  const [rx, ry] = worldToCanvas(_currentPose.x, _currentPose.y);
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(-_currentPose.theta);
  ctx.fillStyle   = '#00ff88';
  ctx.strokeStyle = '#00aa55';
  ctx.lineWidth   = 1;
  ctx.shadowColor = '#00ff88';
  ctx.shadowBlur  = 8;
  ctx.beginPath();
  ctx.moveTo(10, 0);
  ctx.lineTo(-6, -6);
  ctx.lineTo(-4, 0);
  ctx.lineTo(-6, 6);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

setInterval(drawMapOverlay, 100);

// ---------------------------------------------------------------------------
// Waypoint rendering
// ---------------------------------------------------------------------------

function waypointColor(name) {
  if (name.startsWith('truck'))  return '#ffd166';
  if (name.startsWith('rack'))   return '#00c8ff';
  if (name.startsWith('roller')) return '#66ee88';
  return '#cc88ff';
}

function drawWaypoints(ctx) {
  Object.entries(_waypoints).forEach(([name, wp]) => {
    const [cx, cy] = worldToCanvas(wp.x, wp.y);
    const color    = waypointColor(name);
    const theta    = wp.theta || 0;

    // Direction arrow (in canvas: negate sin due to Y-flip)
    const dx =  Math.cos(theta);
    const dy = -Math.sin(theta);

    ctx.save();
    ctx.shadowColor = color;
    ctx.shadowBlur  = 6;

    // Circle fill
    ctx.beginPath();
    ctx.arc(cx, cy, 8, 0, Math.PI * 2);
    ctx.fillStyle = color + '28';
    ctx.fill();

    // Circle stroke
    ctx.strokeStyle = color;
    ctx.lineWidth   = 2;
    ctx.stroke();

    // Heading arrow
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + 14 * dx, cy + 14 * dy);
    ctx.strokeStyle = color;
    ctx.lineWidth   = 2;
    ctx.stroke();

    // Arrowhead
    const ax = cx + 14 * dx;
    const ay = cy + 14 * dy;
    const headAngle = Math.atan2(dy, dx);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(ax - 5 * Math.cos(headAngle - 0.5), ay - 5 * Math.sin(headAngle - 0.5));
    ctx.lineTo(ax - 5 * Math.cos(headAngle + 0.5), ay - 5 * Math.sin(headAngle + 0.5));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();

    ctx.restore();

    // Label background
    ctx.font = 'bold 10px "Courier New", monospace';
    const tw = ctx.measureText(name).width;
    ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
    ctx.fillRect(cx - tw / 2 - 3, cy - 23, tw + 6, 13);

    // Label text
    ctx.fillStyle     = color;
    ctx.textAlign     = 'center';
    ctx.textBaseline  = 'bottom';
    ctx.fillText(name, cx, cy - 11);
    ctx.textAlign    = 'left';
    ctx.textBaseline = 'alphabetic';
  });
}

// ---------------------------------------------------------------------------
// Map canvas — click handling
// ---------------------------------------------------------------------------

function initMapInteraction() {
  const canvas = document.getElementById('mapCanvas');
  if (!canvas) return;

  const DRAG_THRESHOLD_PX = 10;
  let downCx = 0, downCy = 0;

  const evToCanvas = (e) => {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width  / rect.width;
    const scaleY = canvas.height / rect.height;
    return [(e.clientX - rect.left) * scaleX,
            (e.clientY - rect.top)  * scaleY];
  };

  canvas.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;
    // Don't start a drag while the popup is up
    if (!document.getElementById('waypointPopup').classList.contains('hidden')) return;
    const [cx, cy] = evToCanvas(e);
    downCx = cx; downCy = cy;
    const [wx, wy] = canvasToWorld(cx, cy);
    _dragWp = {wx0: wx, wy0: wy, wx1: wx, wy1: wy};
    drawMapOverlay();
  });

  window.addEventListener('mousemove', (e) => {
    if (!_dragWp) return;
    const [cx, cy] = evToCanvas(e);
    const [wx, wy] = canvasToWorld(cx, cy);
    _dragWp.wx1 = wx;
    _dragWp.wy1 = wy;
    drawMapOverlay();
  });

  window.addEventListener('mouseup', (e) => {
    if (!_dragWp || e.button !== 0) return;
    const drag = _dragWp;
    _dragWp = null;

    const [cx, cy] = evToCanvas(e);
    const dragDistPx = Math.hypot(cx - downCx, cy - downCy);
    drawMapOverlay();

    if (_editMode) {
      // Short click in edit mode → delete the waypoint under the cursor
      // (if any).  Long drag → create a new waypoint with heading.
      if (dragDistPx < DRAG_THRESHOLD_PX) {
        for (const [name, wp] of Object.entries(_waypoints)) {
          const [wpx, wpy] = worldToCanvas(wp.x, wp.y);
          const dist = Math.hypot(cx - wpx, cy - wpy);
          if (dist < 14) {
            deleteWaypoint(name);
            return;
          }
        }
        return;   // click on empty space in edit mode: do nothing
      }
      // Map Y-axis: world Y up, canvas Y down → atan2 in world coords directly
      const theta = Math.atan2(drag.wy1 - drag.wy0, drag.wx1 - drag.wx0);
      _pendingWp = {x: drag.wx0, y: drag.wy0, theta};
      showWaypointPopup(drag.wx0, drag.wy0, theta);
    } else if (dragDistPx < DRAG_THRESHOLD_PX) {
      // Short click outside edit mode → navigate to nearby waypoint
      for (const [name, wp] of Object.entries(_waypoints)) {
        const [wpx, wpy] = worldToCanvas(wp.x, wp.y);
        const dist = Math.hypot(cx - wpx, cy - wpy);
        if (dist < 14) {
          sendGoalWaypoint(name);
          const sel = document.getElementById('waypointSelect');
          if (sel) sel.value = name;
          return;
        }
      }
    }
  });
}

// ---------------------------------------------------------------------------
// Edit mode toggle
// ---------------------------------------------------------------------------

function initEditMode() {
  const btn  = document.getElementById('btnEditWaypoints');
  const hint = document.getElementById('editHint');
  const canvas = document.getElementById('mapCanvas');
  if (!btn) return;

  btn.addEventListener('click', () => {
    _editMode = !_editMode;
    btn.classList.toggle('active', _editMode);
    hint.classList.toggle('hidden', !_editMode);
    if (canvas) canvas.classList.toggle('edit-mode', _editMode);
  });

  // ESC cancels: in-flight drag, then edit mode + popup
  document.addEventListener('keydown', (e) => {
    if (e.key !== 'Escape') return;
    if (_dragWp) {
      _dragWp = null;
      drawMapOverlay();
      return;
    }
    if (_editMode) {
      _editMode = false;
      btn.classList.remove('active');
      hint.classList.add('hidden');
      if (canvas) canvas.classList.remove('edit-mode');
      hideWaypointPopup();
    }
  });
}

// ---------------------------------------------------------------------------
// Waypoint popup
// ---------------------------------------------------------------------------

function showWaypointPopup(wx, wy, theta) {
  setText('wpCoords', `x: ${wx.toFixed(3)} m,  y: ${wy.toFixed(3)} m`);
  setText('wpHeading',
    `θ: ${theta.toFixed(3)} rad (${(theta * 180 / Math.PI).toFixed(1)}°)`);

  const nameIn = document.getElementById('wpName');
  if (nameIn) nameIn.value = '';

  document.getElementById('waypointPopup').classList.remove('hidden');
  setTimeout(() => nameIn && nameIn.focus(), 50);
}

function hideWaypointPopup() {
  document.getElementById('waypointPopup').classList.add('hidden');
  _pendingWp = null;
}

function initWaypointPopup() {
  document.getElementById('btnCancelWaypoint')?.addEventListener('click', hideWaypointPopup);

  document.getElementById('btnSaveWaypoint')?.addEventListener('click', async () => {
    const name = (document.getElementById('wpName')?.value || '').trim();
    if (!name) {
      document.getElementById('wpName')?.focus();
      return;
    }
    if (!_pendingWp) return;
    const theta = _pendingWp.theta || 0;

    try {
      const res = await fetch('/api/waypoints', {
        method:  'POST',
        headers: {'Content-Type': 'application/json'},
        body:    JSON.stringify({name, x: _pendingWp.x, y: _pendingWp.y, theta}),
      });
      const json = await res.json();
      if (json.ok) {
        hideWaypointPopup();
        await loadWaypoints();
        showToast(`Waypoint "${name}" saved`, 'success');
      } else {
        showToast(`Error: ${json.error || 'unknown'}`, 'error');
      }
    } catch (err) {
      showToast(`Network error: ${err.message}`, 'error');
    }
  });

  // Allow Enter to save in the name field
  document.getElementById('wpName')?.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') document.getElementById('btnSaveWaypoint')?.click();
  });
}

// ---------------------------------------------------------------------------
// Teleop
// ---------------------------------------------------------------------------

function _isInputFocused() {
  const tag = document.activeElement ? document.activeElement.tagName : '';
  return tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA';
}

function _computeTeleopVelocities() {
  let linear = 0, angular = 0;
  if (_keysDown.has('w') || _keysDown.has('arrowup'))    linear  += TELEOP_LINEAR;
  if (_keysDown.has('s') || _keysDown.has('arrowdown'))  linear  -= TELEOP_LINEAR;
  if (_keysDown.has('a') || _keysDown.has('arrowleft'))  angular += TELEOP_ANGULAR;
  if (_keysDown.has('d') || _keysDown.has('arrowright')) angular -= TELEOP_ANGULAR;
  return {linear, angular};
}

function _startTeleopInterval() {
  if (_teleopInterval !== null) return;
  _teleopInterval = setInterval(async () => {
    if (_keysDown.size === 0) { _stopTeleopInterval(); return; }
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
  if (_teleopInterval !== null) { clearInterval(_teleopInterval); _teleopInterval = null; }
}

function _highlightKeys() {
  const map = {w: 'keyW', a: 'keyA', s: 'keyS', d: 'keyD'};
  Object.entries(map).forEach(([k, id]) => {
    document.getElementById(id)?.classList.toggle('pressed', _keysDown.has(k));
  });
}

function initTeleop() {
  document.addEventListener('keydown', (e) => {
    if (_isInputFocused()) return;
    const key = e.key.toLowerCase();
    if (key === ' ') { e.preventDefault(); triggerEstop(); return; }
    const valid = new Set(['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright']);
    if (!valid.has(key)) return;
    e.preventDefault();
    _keysDown.add(key);
    _highlightKeys();
    _startTeleopInterval();
  });

  document.addEventListener('keyup', (e) => {
    if (_isInputFocused()) return;
    _keysDown.delete(e.key.toLowerCase());
    _highlightKeys();
    if (_keysDown.size === 0) {
      _stopTeleopInterval();
      updateTeleopBars(0, 0);
      fetch('/api/teleop/stop', {method: 'POST'}).catch(() => {});
    }
  });

  document.getElementById('btnEstop')?.addEventListener('click', triggerEstop);
}

function triggerEstop() {
  _keysDown.clear();
  _stopTeleopInterval();
  _highlightKeys();
  updateTeleopBars(0, 0);
  fetch('/api/teleop/stop', {method: 'POST'}).catch(() => {});
}

function updateTeleopBars(linear, angular) {
  const linPct = Math.min(Math.abs(linear) / TELEOP_LINEAR * 50, 50);
  const angPct = Math.min(Math.abs(angular) / TELEOP_ANGULAR * 50, 50);
  const tBarL  = document.getElementById('tBarLinear');
  const tBarA  = document.getElementById('tBarAngular');
  if (tBarL) { tBarL.style.width = linPct + '%'; tBarL.style.left = linear  >= 0 ? '50%' : (50 - linPct) + '%'; }
  if (tBarA) { tBarA.style.width = angPct + '%'; tBarA.style.left = angular >= 0 ? '50%' : (50 - angPct) + '%'; }
  setText('tVelLinear',  linear.toFixed(2));
  setText('tVelAngular', angular.toFixed(2));
}

// ---------------------------------------------------------------------------
// System mode (MAPPING ⇄ NAVIGATION) + Save Map
// ---------------------------------------------------------------------------

let _systemMode = 'NAVIGATION';

function applyModeToUI(mode) {
  _systemMode = mode;

  // Highlight the active button
  document.querySelectorAll('.mode-switch .mode-btn').forEach(btn => {
    btn.classList.toggle('active', btn.dataset.mode === mode);
  });

  // Save Map button only relevant during mapping
  const saveBtn = document.getElementById('btnSaveMap');
  if (saveBtn) saveBtn.classList.toggle('hidden', mode !== 'MAPPING');

  // Disable nav controls in mapping mode
  const navGo = document.getElementById('btnGoWaypoint');
  const navCancel = document.getElementById('btnCancelNav');
  const navSel = document.getElementById('waypointSelect');
  [navGo, navCancel, navSel].forEach(el => {
    if (!el) return;
    el.disabled = (mode !== 'NAVIGATION');
    el.title = (mode !== 'NAVIGATION')
      ? 'Switch to NAV mode to send navigation goals'
      : '';
  });
}

async function fetchCurrentMode() {
  try {
    const res  = await fetch('/api/mode');
    const data = await res.json();
    if (data && data.mode) applyModeToUI(data.mode);
  } catch (e) { /* ignore */ }
}

async function requestModeSwitch(target, options = {}) {
  try {
    const res  = await fetch('/api/mode', {
      method:  'POST',
      headers: {'Content-Type': 'application/json'},
      body:    JSON.stringify({mode: target, ...options}),
    });
    const data = await res.json();
    if (!res.ok) {
      showToast(data.error || 'Mode switch failed', 'error');
      return false;
    }
    applyModeToUI(data.mode);
    showToast(`Mode: ${data.mode}`, 'success');
    return true;
  } catch (err) {
    showToast(`Mode switch error: ${err}`, 'error');
    return false;
  }
}

function initSystemMode() {
  // Fetch the current mode from the backend on startup
  fetchCurrentMode();

  // Map / Nav toggle
  document.querySelectorAll('.mode-switch .mode-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const target = btn.dataset.mode;
      if (target === _systemMode) return;

      if (_systemMode === 'NAVIGATION' && target === 'MAPPING') {
        // Ask whether to continue existing map or start fresh
        const dlg = document.getElementById('mapResetDialog');
        if (dlg) dlg.classList.remove('hidden');
        return;
      }

      // MAPPING → NAVIGATION (auto-save happens server-side)
      requestModeSwitch(target);
    });
  });

  // Dialog buttons (NAV → MAP)
  document.getElementById('btnModeContinueMap')?.addEventListener('click', () => {
    document.getElementById('mapResetDialog')?.classList.add('hidden');
    requestModeSwitch('MAPPING', {reset_map: false});
  });
  document.getElementById('btnModeResetMap')?.addEventListener('click', () => {
    document.getElementById('mapResetDialog')?.classList.add('hidden');
    requestModeSwitch('MAPPING', {reset_map: true});
  });
  document.getElementById('btnModeCancelSwitch')?.addEventListener('click', () => {
    document.getElementById('mapResetDialog')?.classList.add('hidden');
  });

  // Save Map button
  document.getElementById('btnSaveMap')?.addEventListener('click', async () => {
    const btn = document.getElementById('btnSaveMap');
    btn.classList.add('saving');
    try {
      const res  = await fetch('/api/map/save', {method: 'POST'});
      const data = await res.json();
      if (res.ok && data.success) {
        showToast('Map saved to ~/ros2_maps/', 'success');
      } else {
        showToast(`Save failed: ${data.message || 'unknown error'}`, 'error');
      }
    } catch (err) {
      showToast(`Save error: ${err}`, 'error');
    } finally {
      btn.classList.remove('saving');
    }
  });

  // Relocalize button — re-runs SLAM global auto-localization
  document.getElementById('btnRelocalize')?.addEventListener('click', async () => {
    const btn = document.getElementById('btnRelocalize');
    btn.classList.add('saving');
    try {
      const res  = await fetch('/api/slam/relocalize', {method: 'POST'});
      const data = await res.json();
      if (res.ok && data.success) {
        showToast('Relocating… watch the laser snap to the map', 'success');
      } else {
        showToast(`Relocate failed: ${data.message || 'unknown error'}`, 'error');
      }
    } catch (err) {
      showToast(`Relocate error: ${err}`, 'error');
    } finally {
      btn.classList.remove('saving');
    }
  });

  // Fit Workspace button — auto-detect & lock the room rectangle
  document.getElementById('btnFitWorkspace')?.addEventListener('click', async () => {
    const btn = document.getElementById('btnFitWorkspace');
    btn.classList.add('saving');
    try {
      const res  = await fetch('/api/slam/fit_workspace', {method: 'POST'});
      const data = await res.json();
      if (res.ok && data.success) {
        showToast(data.message || 'Workspace locked', 'success');
      } else {
        showToast(`Fit failed: ${data.message || 'unknown error'}`, 'error');
      }
    } catch (err) {
      showToast(`Fit error: ${err}`, 'error');
    } finally {
      btn.classList.remove('saving');
    }
  });
}

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
  initEditMode();
  initMapInteraction();
  initWaypointPopup();
  initVoice();
  initSystemMode();

  // Preload map and fetch metadata immediately
  _mapImg.src = '/api/map';
  loadMapInfo();

  const form = document.getElementById('missionForm');
  if (!form) return;

  form.addEventListener('submit', async (e) => {
    e.preventDefault();

    const palletId    = document.getElementById('palletId').value.trim();
    const source      = document.getElementById('sourceSelect').value;
    const sourceLevel = parseInt(document.getElementById('sourceLevelSelect').value, 10);
    const destination = document.getElementById('destSelect').value;

    if (!palletId || !source || !sourceLevel || !destination) {
      showToast('Please fill in all fields.', 'error'); return;
    }
    if (source === destination) {
      showToast('Source and destination must be different.', 'error'); return;
    }

    const btn = form.querySelector('.btn-submit');
    btn.disabled = true;

    try {
      const res  = await fetch('/api/mission', {
        method:  'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({pallet_id: palletId, source, source_level: sourceLevel, destination}),
      });
      const json = await res.json();

      if (!res.ok || json.error) {
        showToast(`Error: ${json.error || 'Unknown error'}`, 'error'); return;
      }

      showToast(`Mission sent: ${palletId} → ${formatLocation(destination)}`, 'success');
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
// Mission history
// ---------------------------------------------------------------------------

function addHistoryItem(mission) {
  const list = document.getElementById('historyList');
  if (!list) return;
  list.querySelector('.dim')?.remove();

  const li = document.createElement('li');
  li.className = 'history-item';
  li.innerHTML =
    `<span class="hist-id">${mission.pallet_id}</span>` +
    `<span class="hist-meta"> · ${formatLocation(mission.source)} (L${mission.source_level}) → ${formatLocation(mission.destination)}</span>`;
  list.insertBefore(li, list.firstChild);
  while (list.children.length > MAX_HISTORY) list.removeChild(list.lastChild);
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
  _toastTimer = setTimeout(() => { toast.className = 'toast hidden'; }, 4000);
}

// ---------------------------------------------------------------------------
// Uptime clock
// ---------------------------------------------------------------------------

function startUptimeClock() {
  setInterval(() => {
    const s = Math.floor((Date.now() - _startTime) / 1000);
    setText('statUptime', `${pad(Math.floor(s/3600))}:${pad(Math.floor(s%3600/60))}:${pad(s%60)}`);
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

function pad(n) { return String(n).padStart(2, '0'); }

function formatLocation(loc) {
  if (!loc) return '—';
  return loc.replace('_', ' ').replace(/\b\w/g, c => c.toUpperCase());
}

// ---------------------------------------------------------------------------
// Voice tab
// ---------------------------------------------------------------------------

let _mediaRecorder  = null;
let _audioChunks    = [];
let _voiceRecording = false;

socket.on('voice_result', (data) => {
  if (data && data.word) {
    _showVoiceResult(data.word);
  }
});

async function initVoice() {
  // Load vocabulary from server
  try {
    const res  = await fetch('/api/voice/status');
    const data = await res.json();
    const statusEl = document.getElementById('voiceModelStatus');
    const vocabEl  = document.getElementById('voiceVocab');

    if (data.ready) {
      if (statusEl) statusEl.textContent = `HMM ready — ${data.vocabulary.length} words`;
      if (vocabEl) {
        vocabEl.innerHTML = data.vocabulary
          .map(w => `<span class="voice-vocab-chip">${w}</span>`)
          .join('');
      }
    } else {
      if (statusEl) statusEl.textContent = 'Models not found — train first';
    }
  } catch (_) {}

  const btn = document.getElementById('btnVoiceRecord');
  if (!btn) return;

  // Touch/pointer events for hold-to-record
  btn.addEventListener('pointerdown', (e) => { e.preventDefault(); _startRecording(); });
  btn.addEventListener('pointerup',   (e) => { e.preventDefault(); _stopRecording();  });
  btn.addEventListener('pointerleave',(e) => { if (_voiceRecording) _stopRecording(); });

  // Keyboard shortcut: hold V to record
  document.addEventListener('keydown', (e) => {
    if (e.code === 'KeyV' && !_voiceRecording && !_isInputFocused()) {
      e.preventDefault();
      _startRecording();
    }
  });
  document.addEventListener('keyup', (e) => {
    if (e.code === 'KeyV' && _voiceRecording) {
      e.preventDefault();
      _stopRecording();
    }
  });
}

async function _startRecording() {
  if (_voiceRecording) return;
  _voiceRecording = true;

  const btn = document.getElementById('btnVoiceRecord');
  _setVoiceStatus('recording', '&#9679; Recording…');
  btn?.classList.add('recording');
  btn && (btn.textContent = '⬛ Release to Send');

  _audioChunks = [];

  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: {sampleRate: 16000, channelCount: 1, echoCancellation: true},
    });

    const opts = MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
      ? {mimeType: 'audio/webm;codecs=opus'}
      : {};
    _mediaRecorder = new MediaRecorder(stream, opts);

    _mediaRecorder.ondataavailable = (e) => {
      if (e.data.size > 0) _audioChunks.push(e.data);
    };

    _mediaRecorder.onstop = async () => {
      stream.getTracks().forEach(t => t.stop());
      await _sendAudio();
    };

    _mediaRecorder.start(100); // collect in 100ms chunks
  } catch (err) {
    _setVoiceStatus('error', 'Mic error: ' + err.message);
    _voiceRecording = false;
    btn?.classList.remove('recording');
    btn && (btn.textContent = '● Hold to Record');
  }
}

function _stopRecording() {
  if (!_voiceRecording) return;
  _voiceRecording = false;

  const btn = document.getElementById('btnVoiceRecord');
  btn?.classList.remove('recording');
  btn && (btn.textContent = '● Hold to Record');
  _setVoiceStatus('processing', 'Processing…');

  if (_mediaRecorder && _mediaRecorder.state !== 'inactive') {
    _mediaRecorder.stop();
  }
}

async function _sendAudio() {
  if (_audioChunks.length === 0) {
    _setVoiceStatus('error', 'No audio captured');
    return;
  }

  const blob = new Blob(_audioChunks, {type: _mediaRecorder?.mimeType || 'audio/webm'});
  const form = new FormData();
  form.append('audio', blob, 'recording.webm');

  try {
    const res  = await fetch('/api/voice', {method: 'POST', body: form});
    const json = await res.json();

    if (res.ok && json.word) {
      _showVoiceResult(json.word);
    } else {
      _setVoiceStatus('error', 'Error: ' + (json.error || 'unknown'));
    }
  } catch (err) {
    _setVoiceStatus('error', 'Network error: ' + err.message);
  }
}

function _showVoiceResult(word) {
  setText('voiceResultWord', word);
  _setVoiceStatus('done', 'Sent: ' + word);

  // History
  const list = document.getElementById('voiceHistoryList');
  if (list) {
    list.querySelector('.dim')?.remove();
    const li = document.createElement('li');
    li.className   = 'history-item';
    li.textContent = word + '  ' + new Date().toLocaleTimeString();
    list.insertBefore(li, list.firstChild);
    while (list.children.length > MAX_HISTORY) list.removeChild(list.lastChild);
  }

  setTimeout(() => _setVoiceStatus('', 'Ready'), 3000);
}

function _setVoiceStatus(cls, text) {
  const el = document.getElementById('voiceStatus');
  if (!el) return;
  el.className  = 'voice-status' + (cls ? ' ' + cls : '');
  el.innerHTML  = text;
}
