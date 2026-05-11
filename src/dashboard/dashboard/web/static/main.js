/**
 * AMR Warehouse Robot Dashboard — main.js
 *
 * Responsibilities:
 *  - Establish Socket.IO connection with auto-reconnect
 *  - Handle robot_pose, robot_state, telemetry events
 *  - Update all UI elements reactively
 *  - Submit mission form via fetch POST /api/mission
 *  - Track session stats (missions sent, uptime, WS events)
 */

'use strict';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const MAX_LINEAR_VEL  = 0.5;   // m/s — for bar scaling
const MAX_ANGULAR_VEL = 2.0;   // rad/s
const MAX_HISTORY     = 20;

// ---------------------------------------------------------------------------
// Session counters
// ---------------------------------------------------------------------------

let _missionCount = 0;
let _eventCount   = 0;
let _startTime    = Date.now();

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
  updatePose(data);
});

socket.on('robot_state', (data) => {
  _eventCount++;
  updateState(data.state, data.mission);
});

socket.on('telemetry', (data) => {
  _eventCount++;
  updateTelemetry(data);
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
// Mission form
// ---------------------------------------------------------------------------

document.addEventListener('DOMContentLoaded', () => {
  initLifter();
  startUptimeClock();

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
