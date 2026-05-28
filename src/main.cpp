// esp32-wifi-radar — WiFi Motion/Presence Detector for LilyGo T-Display (ESP32, 135x240)
// Libraries: TFT_eSPI by Bodmer, WebServer (built-in ESP32 Arduino core)
// In TFT_eSPI/User_Setup_Select.h: uncomment Setup25_TTGO_T_Display.h
//
// Approach: RSSI-based motion sensing
//   - Connects to your home WiFi router
//   - Samples RSSI every 100ms, keeps a 50-sample rolling buffer
//   - Calculates rolling mean + stddev; deviation > SENSITIVITY * stddev = motion event
//   - Display: top = live RSSI oscilloscope waveform, bottom = motion status + stats
//   - Serial: logs every motion event with millis() timestamp
//   - Web dashboard: served at http://<device-ip>/  — live RSSI chart, event log, stats

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ============================================================
//  USER CONFIG — fill these in before flashing
// ============================================================
#define WIFI_SSID       "swyft 2.4"
#define WIFI_PASSWORD   "Corona33!"

// Sensitivity: how many standard deviations from the rolling mean
// triggers a motion event. Lower = more sensitive (more false positives).
// Higher = less sensitive (may miss subtle movement).
//   1.5 = very sensitive (twitchy, good for detecting distant/subtle motion)
//   2.0 = balanced default (recommended starting point)
//   2.5 = less sensitive (only strong disturbances)
//   3.0 = high threshold (only dramatic movement near the router)
#define SENSITIVITY     2.0f

// ============================================================
//  Display / hardware
// ============================================================
#define W   240
#define H   135
#define BL_PIN  4

// ============================================================
//  Motion detection config
// ============================================================
#define SAMPLE_INTERVAL_MS   100    // sample RSSI every 100ms
#define BUFFER_SIZE          50     // rolling window of samples
#define MOTION_HOLD_MS       3000   // keep "MOTION" state visible for 3s after last trigger
#define MIN_STDDEV           0.5f   // ignore micro-noise below this stddev (flat signal)

// ============================================================
//  Event log config
// ============================================================
#define EVENT_LOG_SIZE  50          // keep last 50 motion events

// ============================================================
//  Display layout
// ============================================================
#define GRAPH_Y      0           // oscilloscope graph starts at top
#define GRAPH_H      72          // top 72 rows = waveform area
#define DIVIDER_Y    73
#define STATUS_Y     78          // bottom area starts here
#define GRAPH_W      W           // full width

// ============================================================
//  Colors
// ============================================================
#define COL_BG          TFT_BLACK
#define COL_GRID        0x1082    // very dark grey
#define COL_WAVE        0x07E0    // bright green (RSSI line)
#define COL_WAVE_OLD    0x0320    // dim green (older samples)
#define COL_CLEAR       0x07E0    // green text for CLEAR
#define COL_MOTION      TFT_RED
#define COL_TEXT        TFT_WHITE
#define COL_LABEL       0xAD75    // muted grey-blue for labels
#define COL_DIVIDER     0x2945
#define COL_MEAN        0x04BF    // cyan tint for mean line
#define COL_BAR_BG      0x2104    // dark bar background
#define COL_BAR_FG      TFT_GREEN

// ============================================================
//  Web dashboard HTML (PROGMEM)
// ============================================================
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 WiFi Radar</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg: #0a0a0f;
    --panel: #111118;
    --border: #1e1e2e;
    --text: #e0e0f0;
    --muted: #555580;
    --green: #00e676;
    --red: #ff1744;
    --cyan: #00e5ff;
    --yellow: #ffd740;
    --orange: #ff6d00;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif;
    min-height: 100vh;
    padding: 16px;
  }
  h1 {
    font-size: 1.1rem;
    font-weight: 600;
    letter-spacing: 0.1em;
    color: var(--cyan);
    text-transform: uppercase;
    margin-bottom: 16px;
    display: flex;
    align-items: center;
    gap: 10px;
  }
  h1 span.dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--green);
    display: inline-block;
    box-shadow: 0 0 8px var(--green);
    animation: blink 1.4s ease-in-out infinite;
  }
  @keyframes blink {
    0%, 100% { opacity: 1; }
    50%       { opacity: 0.3; }
  }

  /* Tab bar */
  .tab-bar {
    display: flex;
    gap: 4px;
    margin-bottom: 16px;
    border-bottom: 1px solid var(--border);
  }
  .tab-btn {
    background: none;
    border: none;
    border-bottom: 2px solid transparent;
    color: var(--muted);
    cursor: pointer;
    font-size: 0.82rem;
    font-weight: 600;
    letter-spacing: 0.08em;
    padding: 8px 18px;
    text-transform: uppercase;
    transition: color 0.15s, border-color 0.15s;
    margin-bottom: -1px;
  }
  .tab-btn.active {
    color: var(--cyan);
    border-bottom-color: var(--cyan);
  }
  .tab-pane { display: none; }
  .tab-pane.active { display: block; }

  /* Status badge */
  #status-badge {
    display: inline-block;
    padding: 8px 28px;
    border-radius: 6px;
    font-size: 2rem;
    font-weight: 800;
    letter-spacing: 0.15em;
    text-align: center;
    margin-bottom: 16px;
    transition: background 0.2s, color 0.2s, box-shadow 0.2s;
  }
  #status-badge.clear {
    background: #002200;
    color: var(--green);
    box-shadow: 0 0 18px #00e67640;
  }
  #status-badge.motion {
    background: #220000;
    color: var(--red);
    box-shadow: 0 0 24px #ff174480;
    animation: flash 0.5s ease-in-out infinite alternate;
  }
  @keyframes flash {
    from { box-shadow: 0 0 12px #ff174440; }
    to   { box-shadow: 0 0 32px #ff1744cc; }
  }

  /* Stats bar */
  .stats-bar {
    display: flex;
    gap: 12px;
    flex-wrap: wrap;
    margin-bottom: 16px;
  }
  .stat-card {
    flex: 1;
    min-width: 110px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 10px 14px;
  }
  .stat-label {
    font-size: 0.68rem;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--muted);
    margin-bottom: 4px;
  }
  .stat-value {
    font-size: 1.35rem;
    font-weight: 700;
    color: var(--text);
  }
  .stat-value.rssi-value { color: var(--green); }
  .stat-value.events-value { color: var(--cyan); }

  /* Chart */
  .chart-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
    margin-bottom: 16px;
    height: 220px;
    position: relative;
  }
  .chart-panel canvas { width: 100% !important; height: 100% !important; }

  /* Event log */
  .log-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 12px;
  }
  .log-title {
    font-size: 0.72rem;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    color: var(--muted);
    margin-bottom: 10px;
  }
  #event-log {
    max-height: 220px;
    overflow-y: auto;
    font-family: 'Courier New', monospace;
    font-size: 0.78rem;
  }
  #event-log::-webkit-scrollbar { width: 4px; }
  #event-log::-webkit-scrollbar-track { background: transparent; }
  #event-log::-webkit-scrollbar-thumb { background: var(--border); border-radius: 2px; }
  .log-row {
    display: flex;
    justify-content: space-between;
    padding: 4px 6px;
    border-radius: 4px;
    margin-bottom: 2px;
    animation: fadein 0.3s ease;
  }
  @keyframes fadein { from { opacity: 0; transform: translateY(-4px); } to { opacity: 1; } }
  .log-row:nth-child(odd) { background: #0d0d18; }
  .log-num  { color: var(--muted); min-width: 40px; }
  .log-time { color: var(--cyan); flex: 1; padding: 0 8px; }
  .log-ms   { color: var(--yellow); text-align: right; }
  .no-events { color: var(--muted); font-style: italic; padding: 8px 6px; }

  /* Connection indicator */
  #conn-indicator {
    position: fixed;
    top: 12px; right: 16px;
    font-size: 0.7rem;
    color: var(--muted);
    letter-spacing: 0.05em;
  }
  #conn-indicator.ok   { color: var(--green); }
  #conn-indicator.err  { color: var(--red); }

  /* ---- Map tab styles ---- */
  .map-toolbar {
    display: flex;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
    margin-bottom: 10px;
  }
  .map-btn {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    cursor: pointer;
    font-size: 0.78rem;
    font-weight: 600;
    letter-spacing: 0.06em;
    padding: 7px 14px;
    text-transform: uppercase;
    transition: background 0.15s, border-color 0.15s, color 0.15s;
    white-space: nowrap;
  }
  .map-btn:hover { background: #1a1a28; border-color: var(--cyan); color: var(--cyan); }
  .map-btn.active { background: #001a22; border-color: var(--cyan); color: var(--cyan); }
  .map-btn.danger:hover { border-color: var(--red); color: var(--red); }
  #room-label-input {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    font-size: 0.78rem;
    padding: 7px 10px;
    outline: none;
    width: 140px;
  }
  #room-label-input:focus { border-color: var(--cyan); }
  #room-label-input::placeholder { color: var(--muted); }
  .map-canvas-wrap {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    overflow: hidden;
    position: relative;
  }
  #floorCanvas {
    display: block;
    width: 100%;
    height: 400px;
    cursor: crosshair;
  }
  .map-hint {
    font-size: 0.7rem;
    color: var(--muted);
    margin-top: 8px;
    letter-spacing: 0.04em;
  }
</style>
</head>
<body>
<div id="conn-indicator">●&nbsp;CONNECTING</div>

<h1><span class="dot"></span>ESP32 WiFi Radar</h1>

<!-- Tab bar -->
<div class="tab-bar">
  <button class="tab-btn active" onclick="switchTab('live')">Live</button>
  <button class="tab-btn" onclick="switchTab('map')">Map</button>
</div>

<!-- Live tab -->
<div id="tab-live" class="tab-pane active">
  <div id="status-badge" class="clear">CLEAR</div>

  <div class="stats-bar">
    <div class="stat-card">
      <div class="stat-label">RSSI</div>
      <div class="stat-value rssi-value" id="s-rssi">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Mean</div>
      <div class="stat-value" id="s-mean">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Std Dev</div>
      <div class="stat-value" id="s-stddev">—</div>
    </div>
    <div class="stat-card">
      <div class="stat-label">Events</div>
      <div class="stat-value events-value" id="s-events">—</div>
    </div>
  </div>

  <div class="chart-panel">
    <canvas id="rssiChart"></canvas>
  </div>

  <div class="log-panel">
    <div class="log-title">Motion Event Log (newest first)</div>
    <div id="event-log"><div class="no-events">No events yet</div></div>
  </div>
</div>

<!-- Map tab -->
<div id="tab-map" class="tab-pane">
  <div class="map-toolbar">
    <button class="map-btn active" id="btn-draw" onclick="setMode('draw')">Draw Room</button>
    <button class="map-btn" id="btn-sensor" onclick="setMode('sensor')">Place Sensor</button>
    <input id="room-label-input" type="text" placeholder="Room label…" maxlength="24">
    <button class="map-btn danger" onclick="clearMap()">Clear Map</button>
  </div>
  <div class="map-canvas-wrap">
    <canvas id="floorCanvas"></canvas>
  </div>
  <div class="map-hint">
    Draw Room: click &amp; drag to draw. Double-click a room to rename it. Place Sensor: click to drop the sensor pin (single-sensor — no triangulation, just presence pulse at pin location).
  </div>
</div>

<script>
// ================================================================
// TAB SWITCHING
// ================================================================
function switchTab(name) {
  document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('tab-' + name).classList.add('active');
  document.querySelectorAll('.tab-btn').forEach(b => {
    if (b.textContent.toLowerCase() === name) b.classList.add('active');
  });
  if (name === 'map') { resizeCanvas(); redrawMap(); }
}

// ================================================================
// CHART SETUP (Live tab)
// ================================================================
const MAX_POINTS = 120;
const labels = [];
const data = [];
for (let i = 0; i < MAX_POINTS; i++) { labels.push(''); data.push(null); }

const ctx = document.getElementById('rssiChart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels,
    datasets: [{
      label: 'RSSI (dBm)',
      data,
      borderColor: '#00e676',
      borderWidth: 1.5,
      pointRadius: 0,
      tension: 0.3,
      fill: true,
      backgroundColor: 'rgba(0,230,118,0.06)',
    }]
  },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: { display: false },
      tooltip: { enabled: false },
    },
    scales: {
      x: { display: false },
      y: {
        min: -100,
        max: -30,
        ticks: {
          color: '#555580',
          font: { size: 10 },
          callback: v => v + ' dBm',
          stepSize: 10,
        },
        grid: { color: '#1e1e2e' },
        border: { color: '#1e1e2e' },
      }
    }
  }
});

// ================================================================
// HELPERS
// ================================================================
function fmtMs(ms) {
  const s  = Math.floor(ms / 1000);
  const h  = Math.floor(s / 3600);
  const m  = Math.floor((s % 3600) / 60);
  const sc = s % 60;
  return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sc).padStart(2,'0')}`;
}

const connEl = document.getElementById('conn-indicator');
function setConn(ok) {
  connEl.textContent = ok ? '● LIVE' : '● OFFLINE';
  connEl.className   = ok ? 'ok' : 'err';
}

// ================================================================
// POLL /status EVERY 500ms
// ================================================================
let lastStddev = 0;
async function pollStatus() {
  try {
    const r = await fetch('/status');
    if (!r.ok) throw new Error();
    const d = await r.json();
    setConn(true);
    lastStddev = d.stddev || 0;

    // Badge
    const badge = document.getElementById('status-badge');
    if (d.motion) {
      badge.textContent = 'MOTION';
      badge.className   = 'motion';
      triggerMapMotion(d.stddev || 1);
    } else {
      badge.textContent = 'CLEAR';
      badge.className   = 'clear';
    }

    // Stats
    document.getElementById('s-rssi').textContent   = d.rssi + ' dBm';
    document.getElementById('s-mean').textContent   = d.mean.toFixed(1) + ' dBm';
    document.getElementById('s-stddev').textContent = d.stddev.toFixed(2);
    document.getElementById('s-events').textContent = d.event_count;

    // Chart
    data.shift(); data.push(d.rssi);
    labels.shift(); labels.push('');
    chart.update('none');
  } catch(e) {
    setConn(false);
  }
}

// ================================================================
// POLL /events EVERY 2s
// ================================================================
let lastEventCount = -1;
async function pollEvents() {
  try {
    const r = await fetch('/events');
    if (!r.ok) throw new Error();
    const events = await r.json();
    if (events.length === lastEventCount) return;
    lastEventCount = events.length;

    const log = document.getElementById('event-log');
    if (events.length === 0) {
      log.innerHTML = '<div class="no-events">No events yet</div>';
      return;
    }
    log.innerHTML = events.map(e => `
      <div class="log-row">
        <span class="log-num">#${e.num}</span>
        <span class="log-time">${e.timestamp}</span>
        <span class="log-ms">${fmtMs(e.millis_since_boot)}</span>
      </div>`).join('');
  } catch(e) { /* status poller handles offline indicator */ }
}

// Kick off polls
pollStatus();
pollEvents();
setInterval(pollStatus, 500);
setInterval(pollEvents, 2000);

// ================================================================
// FLOOR PLAN MAP
// ================================================================
const GRID = 20; // grid snap size in logical px

// State
let mapMode = 'draw'; // 'draw' | 'sensor'
let rooms = [];       // [{x,y,w,h,label}]  logical coords
let sensor = null;    // {x, y} logical coords — null = not placed
let pulses = [];      // [{x,y,r,maxR,alpha,born,stddev}]
let heatDots = [];    // [{x,y,born}]

// Drawing state
let drawing = false;
let drawStart = null;
let drawCurrent = null;

// Load from localStorage
function loadState() {
  try {
    const s = localStorage.getItem('esp32map');
    if (s) {
      const obj = JSON.parse(s);
      rooms  = obj.rooms  || [];
      sensor = obj.sensor || null;
    }
  } catch(e) {}
}

function saveState() {
  try {
    localStorage.setItem('esp32map', JSON.stringify({ rooms, sensor }));
  } catch(e) {}
}

loadState();

// Canvas setup
const fc = document.getElementById('floorCanvas');
const fctx = fc.getContext('2d');

function resizeCanvas() {
  const wrap = fc.parentElement;
  fc.width  = wrap.clientWidth;
  fc.height = 400;
}

window.addEventListener('resize', () => { resizeCanvas(); redrawMap(); });
resizeCanvas();

// Snap to grid
function snap(v) { return Math.round(v / GRID) * GRID; }

// Convert canvas client coords -> logical canvas coords
function canvasXY(e) {
  const rect = fc.getBoundingClientRect();
  const scaleX = fc.width  / rect.width;
  const scaleY = fc.height / rect.height;
  const cx = (e.clientX - rect.left) * scaleX;
  const cy = (e.clientY - rect.top)  * scaleY;
  return { x: snap(cx), y: snap(cy) };
}

function setMode(m) {
  mapMode = m;
  document.getElementById('btn-draw').classList.toggle('active',   m === 'draw');
  document.getElementById('btn-sensor').classList.toggle('active', m === 'sensor');
  fc.style.cursor = (m === 'sensor') ? 'cell' : 'crosshair';
}

function clearMap() {
  rooms   = [];
  sensor  = null;
  pulses  = [];
  heatDots = [];
  saveState();
  redrawMap();
}

// ---- Mouse events ----
fc.addEventListener('mousedown', e => {
  if (e.button !== 0) return;
  const pos = canvasXY(e);

  if (mapMode === 'sensor') {
    sensor = { x: pos.x, y: pos.y };
    saveState();
    redrawMap();
    return;
  }

  // draw mode
  drawing    = true;
  drawStart  = pos;
  drawCurrent = pos;
});

fc.addEventListener('mousemove', e => {
  if (!drawing) return;
  drawCurrent = canvasXY(e);
  redrawMap();
});

fc.addEventListener('mouseup', e => {
  if (!drawing) return;
  drawing = false;
  const pos = canvasXY(e);
  const x = Math.min(drawStart.x, pos.x);
  const y = Math.min(drawStart.y, pos.y);
  const w = Math.abs(pos.x - drawStart.x);
  const h = Math.abs(pos.y - drawStart.y);
  if (w >= GRID && h >= GRID) {
    const label = document.getElementById('room-label-input').value.trim() || '';
    rooms.push({ x, y, w, h, label });
    saveState();
  }
  drawStart   = null;
  drawCurrent = null;
  redrawMap();
});

fc.addEventListener('mouseleave', e => {
  if (drawing) {
    drawing = false;
    drawStart = null;
    drawCurrent = null;
    redrawMap();
  }
});

// Touch support (basic)
fc.addEventListener('touchstart', e => {
  e.preventDefault();
  const t = e.touches[0];
  fc.dispatchEvent(new MouseEvent('mousedown', { clientX: t.clientX, clientY: t.clientY, button: 0 }));
}, { passive: false });
fc.addEventListener('touchmove', e => {
  e.preventDefault();
  const t = e.touches[0];
  fc.dispatchEvent(new MouseEvent('mousemove', { clientX: t.clientX, clientY: t.clientY }));
}, { passive: false });
fc.addEventListener('touchend', e => {
  e.preventDefault();
  const t = e.changedTouches[0];
  fc.dispatchEvent(new MouseEvent('mouseup', { clientX: t.clientX, clientY: t.clientY, button: 0 }));
}, { passive: false });

// Double-click a room → rename it
fc.addEventListener('dblclick', e => {
  const pos = canvasXY(e);
  for (let i = rooms.length - 1; i >= 0; i--) {
    const r = rooms[i];
    if (pos.x >= r.x && pos.x <= r.x + r.w &&
        pos.y >= r.y && pos.y <= r.y + r.h) {
      const nl = prompt('Room name:', r.label || '');
      if (nl !== null) { r.label = nl.trim(); saveState(); redrawMap(); }
      return;
    }
  }
});

// ---- Motion pulse trigger (called by pollStatus) ----
function triggerMapMotion(stddev) {
  if (!sensor) return;
  // Pulse radius scales with stddev: 40–120px
  const maxR = Math.min(120, Math.max(40, stddev * 25));
  pulses.push({ x: sensor.x, y: sensor.y, r: 0, maxR, alpha: 1.0, born: Date.now(), stddev });

  // Heat dot — keep last 200
  heatDots.push({ x: sensor.x, y: sensor.y, born: Date.now() });
  if (heatDots.length > 200) heatDots.shift();
}

// ---- Draw everything ----
function redrawMap() {
  const W = fc.width;
  const H = fc.height;
  const now = Date.now();

  fctx.clearRect(0, 0, W, H);

  // Background
  fctx.fillStyle = '#0a0a0f';
  fctx.fillRect(0, 0, W, H);

  // Grid
  fctx.strokeStyle = '#1e1e2e';
  fctx.lineWidth   = 0.5;
  for (let x = 0; x < W; x += GRID) {
    fctx.beginPath(); fctx.moveTo(x, 0); fctx.lineTo(x, H); fctx.stroke();
  }
  for (let y = 0; y < H; y += GRID) {
    fctx.beginPath(); fctx.moveTo(0, y); fctx.lineTo(W, y); fctx.stroke();
  }

  // Heat dots (fade over 30 s)
  for (const dot of heatDots) {
    const age = (now - dot.born) / 30000;
    if (age >= 1) continue;
    const alpha = (1 - age) * 0.35;
    fctx.beginPath();
    fctx.arc(dot.x, dot.y, 18, 0, Math.PI * 2);
    fctx.fillStyle = `rgba(255,23,68,${alpha})`;
    fctx.fill();
  }

  // Rooms
  for (const room of rooms) {
    fctx.strokeStyle = '#00e5ff';
    fctx.lineWidth   = 1.5;
    fctx.strokeRect(room.x + 0.5, room.y + 0.5, room.w, room.h);
    fctx.fillStyle = 'rgba(0,229,255,0.04)';
    fctx.fillRect(room.x, room.y, room.w, room.h);

    if (room.label) {
      fctx.fillStyle = '#00e5ff';
      fctx.font      = '11px "Segoe UI", system-ui, sans-serif';
      fctx.textAlign = 'center';
      fctx.textBaseline = 'middle';
      fctx.fillText(room.label, room.x + room.w / 2, room.y + room.h / 2);
    }
  }

  // Active draw preview
  if (drawing && drawStart && drawCurrent) {
    const px = Math.min(drawStart.x, drawCurrent.x);
    const py = Math.min(drawStart.y, drawCurrent.y);
    const pw = Math.abs(drawCurrent.x - drawStart.x);
    const ph = Math.abs(drawCurrent.y - drawStart.y);
    fctx.strokeStyle = 'rgba(0,229,255,0.5)';
    fctx.lineWidth   = 1;
    fctx.setLineDash([4, 4]);
    fctx.strokeRect(px + 0.5, py + 0.5, pw, ph);
    fctx.setLineDash([]);
  }

  // Pulses (sonar rings, fade over 2 s)
  for (let i = pulses.length - 1; i >= 0; i--) {
    const p = pulses[i];
    const age = (now - p.born) / 2000;
    if (age >= 1) { pulses.splice(i, 1); continue; }
    const r     = p.maxR * age;
    const alpha = 1 - age;
    // Pulse color: low stddev = orange, high = red
    const red   = 255;
    const green = Math.round(Math.max(0, 100 - p.stddev * 15));
    fctx.beginPath();
    fctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    fctx.strokeStyle = `rgba(${red},${green},68,${alpha * 0.9})`;
    fctx.lineWidth   = 2;
    fctx.stroke();
    // Inner glow ring
    if (r > 8) {
      fctx.beginPath();
      fctx.arc(p.x, p.y, r * 0.6, 0, Math.PI * 2);
      fctx.strokeStyle = `rgba(${red},${green},68,${alpha * 0.35})`;
      fctx.lineWidth = 1;
      fctx.stroke();
    }
  }

  // Sensor pin
  if (sensor) {
    // Outer glow
    const grd = fctx.createRadialGradient(sensor.x, sensor.y, 2, sensor.x, sensor.y, 14);
    grd.addColorStop(0, 'rgba(255,23,68,0.45)');
    grd.addColorStop(1, 'rgba(255,23,68,0)');
    fctx.beginPath();
    fctx.arc(sensor.x, sensor.y, 14, 0, Math.PI * 2);
    fctx.fillStyle = grd;
    fctx.fill();
    // Pin circle
    fctx.beginPath();
    fctx.arc(sensor.x, sensor.y, 6, 0, Math.PI * 2);
    fctx.fillStyle = '#ff1744';
    fctx.fill();
    fctx.strokeStyle = '#fff';
    fctx.lineWidth = 1.5;
    fctx.stroke();
    // Label
    fctx.fillStyle = '#ff1744';
    fctx.font = 'bold 11px "Segoe UI", sans-serif';
    fctx.textAlign = 'center';
    fctx.textBaseline = 'bottom';
    fctx.fillText('ESP32', sensor.x, sensor.y - 8);
  } else {
    // Placeholder hint
    fctx.fillStyle = '#555580';
    fctx.font = '13px "Segoe UI", sans-serif';
    fctx.textAlign = 'center';
    fctx.textBaseline = 'middle';
    fctx.fillText('Click "Place Sensor" then click the map to drop the sensor pin', W / 2, H / 2);
  }
}

// Animation loop for pulses + heat fade
function mapAnimLoop() {
  requestAnimationFrame(mapAnimLoop);
  // Only redraw when the map tab is visible and there's something animating
  const mapVisible = document.getElementById('tab-map').classList.contains('active');
  if (mapVisible && (pulses.length > 0 || heatDots.length > 0)) {
    redrawMap();
  }
}
mapAnimLoop();

// Initial draw
redrawMap();
</script>
</body>
</html>
)rawhtml";

// ============================================================
//  Globals
// ============================================================
TFT_eSPI    tft;
TFT_eSprite graphSprite(&tft);
TFT_eSprite statusSprite(&tft);

WebServer   server(80);

int32_t  rssiBuffer[BUFFER_SIZE];
int      bufHead      = 0;       // next write index (circular)
int      bufCount     = 0;       // samples filled so far
float    rollingMean  = 0.0f;
float    rollingStddev = 0.0f;

uint32_t motionCount   = 0;
uint32_t lastMotionMs  = 0;      // millis() of last detected motion event
bool     inMotion      = false;
char     lastMotionStr[32] = "None";

uint32_t lastSampleMs  = 0;
uint32_t lastDrawMs    = 0;

// ============================================================
//  Event log (circular, last EVENT_LOG_SIZE motion events)
// ============================================================
struct MotionEvent {
  uint32_t millisSinceBoot;
  char     timestamp[16];   // HH:MM:SS
  uint32_t eventNum;
};
MotionEvent eventLog[EVENT_LOG_SIZE];
int eventLogHead  = 0;
int eventLogCount = 0;

void logMotionEvent(uint32_t nowMs, const char* ts, uint32_t num) {
  eventLog[eventLogHead].millisSinceBoot = nowMs;
  strncpy(eventLog[eventLogHead].timestamp, ts, sizeof(eventLog[0].timestamp) - 1);
  eventLog[eventLogHead].eventNum = num;
  eventLogHead = (eventLogHead + 1) % EVENT_LOG_SIZE;
  if (eventLogCount < EVENT_LOG_SIZE) eventLogCount++;
}

// ============================================================
//  Rolling statistics
// ============================================================
void computeStats() {
  if (bufCount == 0) { rollingMean = 0; rollingStddev = 0; return; }

  float sum = 0;
  int   count = min(bufCount, BUFFER_SIZE);
  for (int i = 0; i < count; i++) sum += rssiBuffer[i];
  rollingMean = sum / count;

  float varSum = 0;
  for (int i = 0; i < count; i++) {
    float d = rssiBuffer[i] - rollingMean;
    varSum += d * d;
  }
  rollingStddev = sqrtf(varSum / count);
}

// ============================================================
//  WiFi connect (blocks until connected)
// ============================================================
void connectWiFi() {
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(2);
  tft.setCursor(8, 50);
  tft.print("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    tft.print(".");
    if (millis() - start > 20000) {
      tft.fillScreen(COL_BG);
      tft.setCursor(8, 50);
      tft.setTextColor(TFT_RED, COL_BG);
      tft.print("WiFi FAILED. Check creds.");
      while (1) delay(5000);
    }
  }

  // Show IP on TFT briefly
  String ip = WiFi.localIP().toString();
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextFont(2);
  tft.setCursor(8, 40);
  tft.print("Connected!");
  tft.setCursor(8, 62);
  tft.setTextColor(COL_CLEAR, COL_BG);
  tft.print("http://");
  tft.print(ip);
  delay(3000);  // show IP for 3 seconds before switching to radar view

  Serial.printf("[BOOT] Connected to %s  IP: %s\n",
                WIFI_SSID, ip.c_str());
  Serial.printf("[BOOT] Dashboard: http://%s/\n", ip.c_str());
}

// ============================================================
//  HTTP handlers
// ============================================================
void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleStatus() {
  // Get current RSSI
  int32_t currentRssi = 0;
  if (bufCount > 0) {
    currentRssi = rssiBuffer[(bufHead + BUFFER_SIZE - 1) % BUFFER_SIZE];
  }

  char json[256];
  snprintf(json, sizeof(json),
    "{\"rssi\":%d,\"mean\":%.2f,\"stddev\":%.2f,\"motion\":%s,\"event_count\":%lu,\"last_event_ms\":%lu}",
    (int)currentRssi,
    rollingMean,
    rollingStddev,
    inMotion ? "true" : "false",
    motionCount,
    lastMotionMs
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleEvents() {
  // Build JSON array of last EVENT_LOG_SIZE events, newest first
  String json = "[";
  bool first = true;

  // Walk from newest to oldest
  for (int i = 0; i < eventLogCount; i++) {
    // Index of i-th newest: (eventLogHead - 1 - i + EVENT_LOG_SIZE) % EVENT_LOG_SIZE
    int idx = (eventLogHead - 1 - i + EVENT_LOG_SIZE) % EVENT_LOG_SIZE;
    if (!first) json += ",";
    first = false;

    char entry[128];
    snprintf(entry, sizeof(entry),
      "{\"num\":%lu,\"timestamp\":\"%s\",\"millis_since_boot\":%lu}",
      eventLog[idx].eventNum,
      eventLog[idx].timestamp,
      eventLog[idx].millisSinceBoot
    );
    json += entry;
  }
  json += "]";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ============================================================
//  Sample RSSI and check for motion
// ============================================================
void takeSample() {
  int32_t rssi = WiFi.RSSI();

  // Store in circular buffer
  rssiBuffer[bufHead] = rssi;
  bufHead = (bufHead + 1) % BUFFER_SIZE;
  if (bufCount < BUFFER_SIZE) bufCount++;

  // Need at least 10 samples before declaring motion
  if (bufCount < 10) return;

  computeStats();

  // Only fire if there's meaningful variance (not a perfectly flat signal)
  if (rollingStddev < MIN_STDDEV) return;

  float deviation = fabsf((float)rssi - rollingMean);
  if (deviation > SENSITIVITY * rollingStddev) {
    uint32_t now = millis();

    // Debounce: don't log a new event if we just logged one < 1s ago
    if (now - lastMotionMs > 1000) {
      motionCount++;
      lastMotionMs = now;

      // Format timestamp as H:M:S since boot
      uint32_t secs   = now / 1000;
      uint32_t mins   = secs / 60;
      uint32_t hours  = mins / 60;
      snprintf(lastMotionStr, sizeof(lastMotionStr),
               "%02lu:%02lu:%02lu", hours, mins % 60, secs % 60);

      logMotionEvent(now, lastMotionStr, motionCount);

      Serial.printf("[MOTION] Event #%lu at %s  RSSI=%d  mean=%.1f  stddev=%.2f  dev=%.1f\n",
                    motionCount, lastMotionStr, rssi,
                    rollingMean, rollingStddev, deviation);
    }
    inMotion = true;
  }

  // Clear motion state after hold period
  if (inMotion && (millis() - lastMotionMs > MOTION_HOLD_MS)) {
    inMotion = false;
  }
}

// ============================================================
//  Draw oscilloscope waveform (top half)
// ============================================================
void drawGraph() {
  graphSprite.fillSprite(COL_BG);

  // Grid lines
  for (int y = 0; y < GRAPH_H; y += GRAPH_H / 4) {
    graphSprite.drawFastHLine(0, y, GRAPH_W, COL_GRID);
  }
  graphSprite.drawFastHLine(0, GRAPH_H - 1, GRAPH_W, COL_GRID);

  // Draw the mean line
  if (bufCount >= 10) {
    // Map mean RSSI to Y pixel
    int meanY = map((int)rollingMean, -100, -30, GRAPH_H - 2, 2);
    meanY = constrain(meanY, 2, GRAPH_H - 2);
    graphSprite.drawFastHLine(0, meanY, GRAPH_W, COL_MEAN);
  }

  // Draw RSSI samples as a waveform — oldest on left, newest on right
  // We have up to BUFFER_SIZE samples. Map them across GRAPH_W pixels.
  int count = min(bufCount, BUFFER_SIZE);
  if (count < 2) { graphSprite.pushSprite(0, GRAPH_Y); return; }

  // Walk from oldest to newest
  // oldest sample index: if buffer is full, it's bufHead (next write = oldest)
  //                      if not full, it's 0
  int startIdx = (bufCount >= BUFFER_SIZE) ? bufHead : 0;

  int prevX = -1, prevY = -1;
  for (int i = 0; i < count; i++) {
    int idx  = (startIdx + i) % BUFFER_SIZE;
    int32_t rssi = rssiBuffer[idx];

    int px = map(i, 0, count - 1, 0, GRAPH_W - 1);
    int py = map((int)rssi, -100, -30, GRAPH_H - 2, 2);
    py = constrain(py, 2, GRAPH_H - 2);

    // Older samples dimmer, newer brighter
    uint16_t col = (i > count - 10) ? COL_WAVE : COL_WAVE_OLD;

    if (prevX >= 0) {
      graphSprite.drawLine(prevX, prevY, px, py, col);
    }
    prevX = px;
    prevY = py;
  }

  // Label: current RSSI in top-left
  graphSprite.setTextFont(1);
  graphSprite.setTextColor(COL_TEXT, COL_BG);
  graphSprite.setCursor(2, 2);
  if (bufCount > 0) {
    int32_t latest = rssiBuffer[(bufHead + BUFFER_SIZE - 1) % BUFFER_SIZE];
    graphSprite.printf("%d dBm", latest);
  }

  // Label: "RSSI" header on right
  graphSprite.setTextColor(COL_LABEL, COL_BG);
  graphSprite.setCursor(GRAPH_W - 36, 2);
  graphSprite.print("RSSI");

  graphSprite.pushSprite(0, GRAPH_Y);
}

// ============================================================
//  Draw status panel (bottom half)
// ============================================================
void drawStatus() {
  statusSprite.fillSprite(COL_BG);

  int panelW = GRAPH_W;

  // --- Motion status badge ---
  if (inMotion) {
    statusSprite.fillRoundRect(0, 0, 90, 22, 4, COL_MOTION);
    statusSprite.setTextFont(4);
    statusSprite.setTextColor(TFT_WHITE, COL_MOTION);
    statusSprite.setCursor(5, 3);
    statusSprite.print("MOTION");
  } else {
    statusSprite.fillRoundRect(0, 0, 68, 22, 4, 0x0340);
    statusSprite.setTextFont(4);
    statusSprite.setTextColor(COL_CLEAR, 0x0340);
    statusSprite.setCursor(5, 3);
    statusSprite.print("CLEAR");
  }

  // --- Motion confidence bar (right of badge) ---
  // Shows how close current RSSI is to triggering threshold
  float confidence = 0;
  if (bufCount >= 10 && rollingStddev >= MIN_STDDEV) {
    int32_t latest = rssiBuffer[(bufHead + BUFFER_SIZE - 1) % BUFFER_SIZE];
    float dev = fabsf((float)latest - rollingMean);
    confidence = dev / (SENSITIVITY * rollingStddev);  // 1.0 = at threshold
    confidence = constrain(confidence, 0.0f, 1.0f);
  }
  int barX   = 96;
  int barW   = panelW - barX - 2;
  int fillW  = (int)(barW * confidence);
  uint16_t barCol = (confidence > 0.85f) ? TFT_ORANGE : (confidence > 0.6f ? TFT_YELLOW : COL_BAR_FG);
  statusSprite.fillRoundRect(barX, 5, barW, 12, 2, COL_BAR_BG);
  if (fillW > 0) statusSprite.fillRoundRect(barX, 5, fillW, 12, 2, barCol);
  statusSprite.setTextFont(1);
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(barX, 20);
  statusSprite.print("sensitivity");

  // --- Stats row ---
  statusSprite.setTextFont(1);

  // Motion count
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 28);
  statusSprite.print("Events:");
  statusSprite.setTextColor(COL_TEXT, COL_BG);
  statusSprite.printf(" %lu", motionCount);

  // Last event timestamp
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 40);
  statusSprite.print("Last:   ");
  statusSprite.setTextColor(COL_TEXT, COL_BG);
  statusSprite.print(lastMotionStr);

  // Mean / stddev
  statusSprite.setTextColor(COL_LABEL, COL_BG);
  statusSprite.setCursor(0, 52);
  if (bufCount >= 2) {
    statusSprite.setTextColor(COL_LABEL, COL_BG);
    statusSprite.printf("mean %.1f  sd %.2f", rollingMean, rollingStddev);
  } else {
    statusSprite.setTextColor(COL_LABEL, COL_BG);
    statusSprite.print("Warming up...");
  }

  statusSprite.pushSprite(0, STATUS_Y);
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);

  tft.init();
  tft.setRotation(1);  // landscape, USB on left
  tft.fillScreen(COL_BG);

  // Sprites
  graphSprite.createSprite(GRAPH_W, GRAPH_H);
  graphSprite.setTextFont(1);

  int statusH = H - STATUS_Y;
  statusSprite.createSprite(GRAPH_W, statusH);
  statusSprite.setTextFont(1);

  connectWiFi();

  // Start web server
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/events", HTTP_GET, handleEvents);
  server.begin();
  Serial.println("[BOOT] HTTP server started on port 80");

  tft.fillScreen(COL_BG);
  tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);

  Serial.printf("[BOOT] WiFi Motion Radar running. SSID=%s  Sensitivity=%.1f\n",
                WIFI_SSID, SENSITIVITY);
  Serial.println("[BOOT] Serial format: [MOTION] Event #N at HH:MM:SS  RSSI=X  mean=M  stddev=S  dev=D");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  uint32_t now = millis();

  // Handle incoming HTTP requests (non-blocking)
  server.handleClient();

  // Sample RSSI on interval
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    if (WiFi.status() == WL_CONNECTED) {
      takeSample();
    } else {
      // Reconnect if dropped
      Serial.println("[WARN] WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }
  }

  // Redraw display every ~100ms (synced to sample rate)
  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);
    drawGraph();
    drawStatus();
  }
}
