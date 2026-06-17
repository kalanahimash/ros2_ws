/* ═══════════════════════════════════════════════════════════
   Rover Dashboard — Frontend Logic
   ═══════════════════════════════════════════════════════════ */

'use strict';

// ─── State ───────────────────────────────────────────────────────────────────
const state = {
  connected:    false,
  estop:        false,
  tracking:     false,
  autonomous:   false,
  linear:       0.0,
  angular:      0.0,
  keysPressed:  new Set(),
};

// ─── Socket.IO ───────────────────────────────────────────────────────────────
const socket = io({ transports: ['websocket', 'polling'] });

socket.on('connect', () => {
  state.connected = true;
  setConnBadge(true);
  log('Connected to ROS2 bridge', 'ok');
});

socket.on('disconnect', () => {
  state.connected = false;
  setConnBadge(false);
  log('Disconnected from ROS2 bridge', 'err');
});

socket.on('status', (data) => {
  updateTelemetry(data);
});

// ─── Telemetry Update ────────────────────────────────────────────────────────
function updateTelemetry(d) {
  // Distance
  const dist = d.distance ?? 400;
  el('dist-val').textContent = dist.toFixed(1) + ' cm';
  const distPct = Math.min(100, (dist / 200) * 100);
  el('dist-bar').style.width = distPct + '%';
  el('dist-bar').style.background = dist < 30 ? '#f85149' : dist < 60 ? '#d29922' : '#58a6ff';

  // Pan / Tilt
  const pan  = d.pan_angle  ?? 90;
  const tilt = d.tilt_angle ?? 90;
  el('pan-display').value  = pan;
  el('tilt-display').value = tilt;
  el('pan-val').textContent  = pan.toFixed(0)  + '°';
  el('tilt-val').textContent = tilt.toFixed(0) + '°';

  // Battery
  const bat    = d.battery_voltage ?? 0;
  const batPct = Math.min(100, (bat / 12.6) * 100);
  el('bat-bar').style.width = batPct + '%';
  el('bat-val').textContent = bat.toFixed(1) + ' V';
  el('bat-bar').style.background = batPct < 20 ? '#f85149' : batPct < 40 ? '#d29922' : '#3fb950';

  // Watchdog
  const wdOk = d.watchdog_ok ?? true;
  el('watchdog-badge').textContent = wdOk ? 'OK' : 'FIRED';
  el('watchdog-badge').className   = 'badge ' + (wdOk ? 'badge-ok' : 'badge-error');

  // Badges
  el('track-state-badge').textContent = 'TRACKING: ' + (d.tracking_state ?? 'OFF');
  el('nav-state-badge').textContent   = 'NAV: '      + (d.nav_state      ?? 'IDLE');
  el('mode-badge').textContent        = 'MODE: '     + (d.mode           ?? 'IDLE');

  // FPS
  el('fps-badge').textContent = (d.tracking_fps ?? 0).toFixed(1) + ' FPS';

  // E-stop overlay
  const estopActive = d.estop_active ?? false;
  el('estop-overlay').classList.toggle('hidden', !estopActive);
  state.estop = estopActive;

  // Sync toggles with server state (don't fight user input)
  if (!el('tracking-toggle').matches(':focus')) {
    el('tracking-toggle').checked = d.tracking ?? false;
  }
  if (!el('auto-toggle').matches(':focus')) {
    el('auto-toggle').checked = d.autonomous ?? false;
  }
}

// ─── Connection Badge ─────────────────────────────────────────────────────────
function setConnBadge(online) {
  const b = el('conn-badge');
  b.textContent  = online ? 'Online' : 'Offline';
  b.className    = 'badge ' + (online ? 'badge-ok' : 'badge-error');
}

// ─── API Helpers ─────────────────────────────────────────────────────────────
async function apiPost(path, body = {}) {
  try {
    await fetch(path, {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify(body),
    });
  } catch (e) {
    log('API error: ' + e.message, 'err');
  }
}

// ─── Drive Commands ───────────────────────────────────────────────────────────
let driveTimer = null;

function startDriveLoop() {
  if (driveTimer) return;
  driveTimer = setInterval(sendDrive, 100);
}

function stopDriveLoop() {
  if (driveTimer) { clearInterval(driveTimer); driveTimer = null; }
}

function sendDrive() {
  if (!state.connected || state.estop) return;
  socket.emit('joystick', { linear: state.linear, angular: state.angular });
}

function computeKeyDrive() {
  const k = state.keysPressed;
  let lin = 0, ang = 0;
  if (k.has('w') || k.has('arrowup'))    lin += 0.8;
  if (k.has('s') || k.has('arrowdown'))  lin -= 0.8;
  if (k.has('a') || k.has('arrowleft'))  ang -= 0.7;
  if (k.has('d') || k.has('arrowright')) ang += 0.7;
  state.linear  = lin;
  state.angular = ang;
}

// ─── Keyboard ─────────────────────────────────────────────────────────────────
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  const key = e.key.toLowerCase();
  state.keysPressed.add(key);
  computeKeyDrive();
  startDriveLoop();

  if (key === ' ') { e.preventDefault(); state.linear = 0; state.angular = 0; }
  if (key === 'e') triggerEstop();
  if (key === 'r') resetEstop();
  if (key === 't') { el('tracking-toggle').checked = !el('tracking-toggle').checked; toggleTracking(el('tracking-toggle').checked); }
  if (key === 'n') { el('auto-toggle').checked     = !el('auto-toggle').checked;     toggleAutonomous(el('auto-toggle').checked); }
});

document.addEventListener('keyup', (e) => {
  state.keysPressed.delete(e.key.toLowerCase());
  computeKeyDrive();
  if (state.keysPressed.size === 0) {
    stopDriveLoop();
    state.linear = 0; state.angular = 0;
    sendDrive();
  }
});

// ─── Pan / Tilt ───────────────────────────────────────────────────────────────
function sendPan(val) {
  el('pan-manual-val').textContent = val + '°';
  apiPost('/api/pan', { angle: parseFloat(val) });
}
function sendTilt(val) {
  el('tilt-manual-val').textContent = val + '°';
  apiPost('/api/tilt', { angle: parseFloat(val) });
}

// ─── Toggles ─────────────────────────────────────────────────────────────────
function toggleTracking(enabled) {
  apiPost('/api/tracking', { enabled });
  log('Tracking ' + (enabled ? 'ENABLED' : 'DISABLED'), enabled ? 'ok' : 'warn');
}
function toggleAutonomous(enabled) {
  apiPost('/api/autonomous', { enabled });
  log('Autonomous ' + (enabled ? 'ENABLED' : 'DISABLED'), enabled ? 'ok' : 'warn');
}

// ─── E-Stop ───────────────────────────────────────────────────────────────────
function triggerEstop() {
  socket.emit('estop', {});
  log('⛔ EMERGENCY STOP triggered', 'err');
}
function resetEstop() {
  socket.emit('reset', {});
  log('↺ Emergency stop RESET', 'ok');
}

// ─── Joystick Canvas ─────────────────────────────────────────────────────────
(function initJoystick() {
  const canvas = el('joystick-canvas');
  const ctx    = canvas.getContext('2d');
  const cx = canvas.width  / 2;
  const cy = canvas.height / 2;
  const radius = 80;
  let thumb = { x: cx, y: cy };
  let active = false;

  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Background circle
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(48,54,61,0.8)';
    ctx.fill();
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth   = 2;
    ctx.stroke();

    // Cross hairs
    ctx.strokeStyle = '#3d4451';
    ctx.lineWidth   = 1;
    ctx.beginPath(); ctx.moveTo(cx - radius, cy); ctx.lineTo(cx + radius, cy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, cy - radius); ctx.lineTo(cx, cy + radius); ctx.stroke();

    // Thumb
    ctx.beginPath();
    ctx.arc(thumb.x, thumb.y, 22, 0, Math.PI * 2);
    ctx.fillStyle = active ? '#58a6ff' : '#388bfd';
    ctx.fill();
  }

  function updateThumb(clientX, clientY) {
    const rect  = canvas.getBoundingClientRect();
    const dx    = clientX - rect.left - cx;
    const dy    = clientY - rect.top  - cy;
    const dist  = Math.hypot(dx, dy);
    const clamp = Math.min(dist, radius - 22);
    const angle = Math.atan2(dy, dx);
    thumb.x = cx + clamp * Math.cos(angle);
    thumb.y = cy + clamp * Math.sin(angle);

    // Map to linear/angular: dy → linear (up = forward), dx → angular (right = +)
    const nx = (thumb.x - cx) / (radius - 22);
    const ny = (thumb.y - cy) / (radius - 22);
    state.linear  = -ny;
    state.angular =  nx;
    draw();
    startDriveLoop();
  }

  function release() {
    active    = false;
    thumb.x   = cx; thumb.y = cy;
    state.linear  = 0; state.angular = 0;
    stopDriveLoop();
    sendDrive();
    draw();
  }

  canvas.addEventListener('mousedown',  (e) => { active = true;  updateThumb(e.clientX, e.clientY); });
  canvas.addEventListener('mousemove',  (e) => { if (active) updateThumb(e.clientX, e.clientY); });
  canvas.addEventListener('mouseup',    release);
  canvas.addEventListener('mouseleave', release);

  canvas.addEventListener('touchstart',  (e) => { e.preventDefault(); active = true;  updateThumb(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
  canvas.addEventListener('touchmove',   (e) => { e.preventDefault(); if (active) updateThumb(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
  canvas.addEventListener('touchend',    (e) => { e.preventDefault(); release(); }, { passive: false });

  draw();
})();

// ─── Log ──────────────────────────────────────────────────────────────────────
function log(msg, level = '') {
  const area  = el('log-area');
  const entry = document.createElement('div');
  entry.className   = 'log-entry ' + level;
  const ts   = new Date().toTimeString().slice(0, 8);
  entry.textContent = `[${ts}] ${msg}`;
  area.appendChild(entry);
  area.scrollTop = area.scrollHeight;
  // Keep last 200 entries
  while (area.children.length > 200) area.removeChild(area.firstChild);
}

// ─── Utility ──────────────────────────────────────────────────────────────────
function el(id) { return document.getElementById(id); }

// ─── Init log ────────────────────────────────────────────────────────────────
log('Dashboard loaded — connecting...', '');
