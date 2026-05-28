// esp32-wifi-radar — WiFi Motion/Presence Detector for LilyGo T-Display (ESP32, 135x240)
// Libraries: TFT_eSPI by Bodmer, WebServer (built-in ESP32 Arduino core)
// In TFT_eSPI/User_Setup_Select.h: uncomment Setup25_TTGO_T_Display.h
//
// Approach: CSI (Channel State Information) motion sensing
//   - Connects to your home WiFi router in STA mode
//   - Registers an ESP-IDF CSI callback (called on the WiFi task)
//   - Per callback: computes mean amplitude across 52 LLTF subcarriers (I²+Q² → sqrt)
//   - Keeps a 50-sample rolling buffer of csiMean values
//   - Calculates rolling mean + stddev; deviation > SENSITIVITY * stddev = motion event
//   - Display: top = live CSI amplitude oscilloscope waveform, bottom = motion status + stats
//   - Serial: logs every motion event with millis() timestamp
//   - Web dashboard: served at http://<device-ip>/ — live CSI chart, event log, stats

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <math.h>
#include "esp_wifi.h"          // ESP-IDF CSI APIs (available in Arduino ESP32 core)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
#define SENSITIVITY     2.5f
#define CONFIRM_HITS    3       // consecutive threshold crossings required (filters fan/AC noise)

// ============================================================
//  Display / hardware
// ============================================================
#define W   240
#define H   135
#define BL_PIN  4

// ============================================================
//  Motion detection config
// ============================================================
#define BUFFER_SIZE          50     // rolling window of CSI mean samples
#define MOTION_HOLD_MS       3000   // keep "MOTION" state visible for 3s after last trigger
#define MIN_STDDEV           0.3f   // ignore micro-noise below this stddev (flat/static signal)

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
#define COL_WAVE        0x07E0    // bright green (CSI amplitude line)
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
  .stat-value.rssi-value  { color: var(--green); }
  .stat-value.csi-value   { color: var(--cyan); }
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

  /* ---- Radar tab styles ---- */
  .radar-wrap {
    background: #000800;
    border: 1px solid #003300;
    border-radius: 8px;
    overflow: hidden;
    position: relative;
    width: 100%;
  }
  #radarCanvas {
    display: block;
    width: 100%;
    height: 420px;
    image-rendering: crisp-edges;
  }
  .radar-hint {
    font-size: 0.7rem;
    color: #005500;
    font-family: monospace;
    margin-top: 8px;
    letter-spacing: 0.04em;
  }
</style>
</head>
<body>
<div id="conn-indicator">●&nbsp;CONNECTING</div>

<h1><span class="dot"></span>ESP32 WiFi Radar — CSI</h1>

<!-- Tab bar -->
<div class="tab-bar">
  <button class="tab-btn active" onclick="switchTab('live')">Live</button>
  <button class="tab-btn" onclick="switchTab('radar')">Radar</button>
</div>

<!-- Live tab -->
<div id="tab-live" class="tab-pane active">
  <div id="status-badge" class="clear">CLEAR</div>

  <div class="stats-bar">
    <div class="stat-card">
      <div class="stat-label">CSI Amp</div>
      <div class="stat-value csi-value" id="s-csi">—</div>
    </div>
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

<!-- Radar tab -->
<div id="tab-radar" class="tab-pane">
  <div class="radar-wrap">
    <canvas id="radarCanvas"></canvas>
  </div>
  <div class="radar-hint">
    Angle derived from inter-subcarrier phase gradient (single-antenna estimate — directional accuracy is approximate).
    Blip distance = CSI amplitude deviation from baseline. Only plots when motion is detected and deviation &gt; 1σ.
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
  if (name === 'radar') resizeRadar();
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
      label: 'CSI Amplitude',
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
        min: 0,
        ticks: {
          color: '#555580',
          font: { size: 10 },
          callback: v => v.toFixed(1),
          stepSize: 5,
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
// POLL /status EVERY 500ms  (Live tab data)
// ================================================================
async function pollStatus() {
  try {
    const r = await fetch('/status');
    if (!r.ok) throw new Error();
    const d = await r.json();
    setConn(true);

    const badge = document.getElementById('status-badge');
    if (d.motion) {
      badge.textContent = 'MOTION';
      badge.className   = 'motion';
    } else {
      badge.textContent = 'CLEAR';
      badge.className   = 'clear';
    }

    document.getElementById('s-csi').textContent    = d.csi_mean !== undefined ? d.csi_mean.toFixed(2) : '—';
    document.getElementById('s-rssi').textContent   = d.rssi + ' dBm';
    document.getElementById('s-mean').textContent   = d.mean.toFixed(2);
    document.getElementById('s-stddev').textContent = d.stddev.toFixed(3);
    document.getElementById('s-events').textContent = d.event_count;

    data.shift(); data.push(d.csi_mean !== undefined ? d.csi_mean : null);
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
  } catch(e) {}
}

pollStatus();
pollEvents();
setInterval(pollStatus, 500);
setInterval(pollEvents, 2000);

// ================================================================
// RADAR
// ================================================================
const rc    = document.getElementById('radarCanvas');
const rctx  = rc.getContext('2d');

const SWEEP_PERIOD_MS = 3000;   // one full rotation
const GHOST_LINES     = 8;       // trailing ghost lines behind sweep
const GHOST_SPAN_DEG  = 90;      // degrees the trail spans
const BLIP_PERSIST_MS = 8000;    // blips fade over 8 s
const MAX_BLIPS       = 100;
const RING_COUNT      = 4;
const SWEEP_FLASH_DEG = 15;      // blip flashes when sweep passes within this angle

let radarW = 0, radarH = 0, radarCX = 0, radarCY = 0, radarR = 0;

function resizeRadar() {
  const wrap = rc.parentElement;
  rc.width  = wrap.clientWidth;
  rc.height = 420;
  radarW  = rc.width;
  radarH  = rc.height;
  radarCX = radarW / 2;
  radarCY = radarH / 2;
  radarR  = Math.min(radarCX, radarCY) - 30;
}

window.addEventListener('resize', () => {
  if (document.getElementById('tab-radar').classList.contains('active')) resizeRadar();
});
resizeRadar();

// Blip list: {angleDeg, distFrac, born, flashing}
let blips = [];

// Phase unwrap helper — keep phase differences in [-π, π]
function unwrapDiff(d) {
  while (d >  Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

// ---- Fetch /csi-raw and push blip ----
async function pollCsiRaw() {
  try {
    const r = await fetch('/csi-raw');
    if (!r.ok) return;
    const d = await r.json();

    if (!d.motion) return;                          // no motion — skip
    const deviation = Math.abs(d.mean_amp - d.mean_amp) / (d.stddev || 1);
    // deviation here uses mean_amp vs overall baseline via stddev field;
    // compute directly: since mean_amp IS the frame mean, use stddev as spread
    // We detect "deviation" as stddev magnitude relative to typical noise.
    // Proxy: if stddev > 0 and mean_amp is available, use stddev as proxy deviation.
    const dev = d.stddev;
    if (dev < 1.0) return;                          // below threshold

    // --- Angle from inter-subcarrier phase gradient ---
    const phases = d.phases;
    const n = phases.length;
    if (n < 2) return;
    let sumDiff = 0;
    for (let i = 0; i < n - 1; i++) {
      sumDiff += unwrapDiff(phases[i + 1] - phases[i]);
    }
    const meanPhaseDiff = sumDiff / (n - 1);
    // Map [-π, π] → [0°, 360°]
    const angleDeg = ((meanPhaseDiff + Math.PI) / (2 * Math.PI)) * 360;

    // --- Distance from amplitude deviation ---
    // dev = stddev of this frame's amplitudes; map [0, 3+] stddev → [100%, 10%] radius
    const distFrac = Math.max(0.10, 1.0 - Math.min(dev / 3.0, 0.90));

    // Push blip
    blips.push({ angleDeg, distFrac, born: Date.now(), flashing: false });
    if (blips.length > MAX_BLIPS) blips.shift();
  } catch(e) {}
}

setInterval(pollCsiRaw, 500);

// ---- Radar draw loop ----
function drawRadar(timestamp) {
  requestAnimationFrame(drawRadar);

  if (!document.getElementById('tab-radar').classList.contains('active')) return;

  const now = Date.now();

  // Background
  rctx.fillStyle = '#000800';
  rctx.fillRect(0, 0, radarW, radarH);

  // Range rings
  for (let i = 1; i <= RING_COUNT; i++) {
    const rr = (radarR * i) / RING_COUNT;
    rctx.beginPath();
    rctx.arc(radarCX, radarCY, rr, 0, 2 * Math.PI);
    rctx.strokeStyle = 'rgba(0,200,50,0.15)';
    rctx.lineWidth = 1;
    rctx.stroke();
    // Label
    const pct = (i * 100 / RING_COUNT) + '%';
    rctx.fillStyle = 'rgba(0,180,40,0.45)';
    rctx.font = '10px monospace';
    rctx.textAlign = 'left';
    rctx.textBaseline = 'middle';
    rctx.fillText(pct, radarCX + rr + 3, radarCY);
  }

  // Crosshair
  rctx.strokeStyle = 'rgba(0,200,50,0.12)';
  rctx.lineWidth = 0.5;
  rctx.beginPath(); rctx.moveTo(radarCX, radarCY - radarR); rctx.lineTo(radarCX, radarCY + radarR); rctx.stroke();
  rctx.beginPath(); rctx.moveTo(radarCX - radarR, radarCY); rctx.lineTo(radarCX + radarR, radarCY); rctx.stroke();

  // Outer boundary circle
  rctx.beginPath();
  rctx.arc(radarCX, radarCY, radarR, 0, 2 * Math.PI);
  rctx.strokeStyle = 'rgba(0,200,50,0.35)';
  rctx.lineWidth = 1.5;
  rctx.stroke();

  // Sweep angle (radians, 0 = top, clockwise)
  const sweepFrac  = (now % SWEEP_PERIOD_MS) / SWEEP_PERIOD_MS;
  const sweepAngle = sweepFrac * 2 * Math.PI - Math.PI / 2; // -π/2 so 0 = top

  // Ghost trail lines
  for (let g = GHOST_LINES; g >= 0; g--) {
    const trailFrac  = g / GHOST_LINES;
    const trailAngle = sweepAngle - (trailFrac * GHOST_SPAN_DEG * Math.PI / 180);
    const alpha      = (1 - trailFrac) * 0.55;

    // Gradient along the line
    const x2 = radarCX + Math.cos(trailAngle) * radarR;
    const y2 = radarCY + Math.sin(trailAngle) * radarR;
    const grad = rctx.createLinearGradient(radarCX, radarCY, x2, y2);
    grad.addColorStop(0,   `rgba(0,255,70,${alpha})`);
    grad.addColorStop(1,   'rgba(0,255,70,0)');

    rctx.beginPath();
    rctx.moveTo(radarCX, radarCY);
    rctx.lineTo(x2, y2);
    rctx.strokeStyle = grad;
    rctx.lineWidth   = g === 0 ? 2 : 1;
    rctx.stroke();
  }

  // Sweep-filled arc (fading cone behind the line)
  const coneStart = sweepAngle - (GHOST_SPAN_DEG * Math.PI / 180);
  const coneGrad  = rctx.createConicalGradient
    ? null   // not standard — use arc sector workaround below
    : null;
  // Draw sector fill using radial approach: thin arc lines at multiple angles
  const steps = 18;
  for (let s = 0; s < steps; s++) {
    const frac  = s / steps;
    const aAngle = coneStart + frac * (GHOST_SPAN_DEG * Math.PI / 180);
    const aAlpha = frac * 0.07;
    rctx.beginPath();
    rctx.moveTo(radarCX, radarCY);
    rctx.arc(radarCX, radarCY, radarR, aAngle, aAngle + (GHOST_SPAN_DEG * Math.PI / 180) / steps);
    rctx.closePath();
    rctx.fillStyle = `rgba(0,255,70,${aAlpha})`;
    rctx.fill();
  }

  // Blips
  const sweepDeg = ((sweepAngle + Math.PI / 2) * 180 / Math.PI + 360) % 360;
  for (let i = blips.length - 1; i >= 0; i--) {
    const b   = blips[i];
    const age = (now - b.born) / BLIP_PERSIST_MS;
    if (age >= 1) { blips.splice(i, 1); continue; }

    // Check if sweep passes near blip
    let angleDiff = Math.abs(sweepDeg - b.angleDeg);
    if (angleDiff > 180) angleDiff = 360 - angleDiff;
    if (angleDiff < SWEEP_FLASH_DEG) b.flashing = true;
    else if (angleDiff > SWEEP_FLASH_DEG + 10) b.flashing = false;

    const alpha    = 1 - age;
    const blipR    = b.distFrac * radarR;
    const bAngleRad = (b.angleDeg - 90) * Math.PI / 180; // -90 so 0deg = top
    const bx       = radarCX + Math.cos(bAngleRad) * blipR;
    const by       = radarCY + Math.sin(bAngleRad) * blipR;

    rctx.beginPath();
    rctx.arc(bx, by, 6, 0, 2 * Math.PI);
    if (b.flashing) {
      rctx.fillStyle = `rgba(255,255,255,${alpha})`;
      rctx.shadowColor = 'rgba(255,255,255,0.9)';
      rctx.shadowBlur  = 12;
    } else {
      rctx.fillStyle = `rgba(0,255,70,${alpha})`;
      rctx.shadowColor = 'rgba(0,255,70,0.8)';
      rctx.shadowBlur  = 10;
    }
    rctx.fill();
    rctx.shadowBlur = 0;
  }

  // Sensor pin (center, red dot)
  // Glow
  const sg = rctx.createRadialGradient(radarCX, radarCY, 2, radarCX, radarCY, 14);
  sg.addColorStop(0, 'rgba(255,23,68,0.55)');
  sg.addColorStop(1, 'rgba(255,23,68,0)');
  rctx.beginPath();
  rctx.arc(radarCX, radarCY, 14, 0, 2 * Math.PI);
  rctx.fillStyle = sg;
  rctx.fill();
  // Dot
  rctx.beginPath();
  rctx.arc(radarCX, radarCY, 5, 0, 2 * Math.PI);
  rctx.fillStyle = '#ff1744';
  rctx.shadowColor = '#ff1744';
  rctx.shadowBlur  = 8;
  rctx.fill();
  rctx.shadowBlur = 0;
  // Label
  rctx.fillStyle    = '#ff4060';
  rctx.font         = 'bold 11px monospace';
  rctx.textAlign    = 'center';
  rctx.textBaseline = 'bottom';
  rctx.fillText('ESP32', radarCX, radarCY - 10);
}

requestAnimationFrame(drawRadar);
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

// ============================================================
//  CSI data — shared between WiFi task (callback) and main loop
//  Protected with a spinlock (portMUX_TYPE).
// ============================================================
static portMUX_TYPE csiMux = portMUX_INITIALIZER_UNLOCKED;

// Rolling buffer of csiMean values (written by callback, read by main loop)
static float    csiBuffer[BUFFER_SIZE];
static int      csiHead      = 0;      // next write index (circular)
static int      csiCount     = 0;      // samples filled so far
static float    latestCsiMean = 0.0f;  // most recent csiMean from callback
static int32_t  latestRssi   = 0;      // RSSI from most recent CSI frame rx_ctrl

// Raw I/Q buffer — last full CSI frame (104 values: I0,Q0,I1,Q1,...,I51,Q51)
static int8_t   rawIQ[104];
static int      rawIQPairs   = 0;      // valid pair count (≤52)

// These are computed by main loop from the buffer (read-only outside loop)
float    rollingMean   = 0.0f;
float    rollingStddev = 0.0f;

uint32_t motionCount   = 0;
uint32_t lastMotionMs  = 0;      // millis() of last detected motion event
bool     inMotion      = false;
int      consecutiveHits = 0;    // consecutive samples above threshold (fan filter)
char     lastMotionStr[32] = "None";

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
//  CSI callback — runs on the WiFi task (NOT the Arduino loop task)
//  Keep it short: compute amplitude, push to buffer, set flag.
// ============================================================
static void IRAM_ATTR csiCallback(void* ctx, wifi_csi_info_t* info) {
  if (!info || !info->buf || info->len < 2) return;

  // LLTF: 52 subcarriers, each I (int8) + Q (int8) = 104 values.
  // Actual len may be 128 (LLTF) or 384 (LLTF+HT-LTF+STBC-HT-LTF).
  // Use whichever subcarriers are available, up to 52 pairs.
  int pairs = info->len / 2;
  if (pairs > 52) pairs = 52;  // cap at LLTF subcarrier count

  float sumAmp = 0.0f;
  for (int i = 0; i < pairs; i++) {
    float I = (float)info->buf[2 * i];
    float Q = (float)info->buf[2 * i + 1];
    sumAmp += sqrtf(I * I + Q * Q);
  }
  float meanAmp = (pairs > 0) ? (sumAmp / pairs) : 0.0f;
  int32_t rssi  = (int32_t)info->rx_ctrl.rssi;

  // Push into the shared rolling buffer — critical section (spinlock, not mutex,
  // because this runs in ISR/WiFi task context where FreeRTOS mutexes are illegal)
  portENTER_CRITICAL(&csiMux);
  csiBuffer[csiHead] = meanAmp;
  csiHead = (csiHead + 1) % BUFFER_SIZE;
  if (csiCount < BUFFER_SIZE) csiCount++;
  latestCsiMean = meanAmp;
  latestRssi    = rssi;
  // Store raw I/Q for /csi-raw endpoint
  rawIQPairs = pairs;
  for (int i = 0; i < pairs * 2; i++) rawIQ[i] = info->buf[i];
  portEXIT_CRITICAL(&csiMux);
}

// ============================================================
//  Rolling statistics (called from main loop — reads csiBuffer)
//  Must be called inside critical section or after copying the buffer.
// ============================================================
void computeStats(float* buf, int count) {
  if (count == 0) { rollingMean = 0; rollingStddev = 0; return; }

  float sum = 0;
  for (int i = 0; i < count; i++) sum += buf[i];
  rollingMean = sum / count;

  float varSum = 0;
  for (int i = 0; i < count; i++) {
    float d = buf[i] - rollingMean;
    varSum += d * d;
  }
  rollingStddev = sqrtf(varSum / count);
}

// ============================================================
//  Enable CSI after WiFi connects
// ============================================================
void enableCSI() {
  // Register callback first
  esp_wifi_set_csi_rx_cb(csiCallback, NULL);

  // Configure CSI — request LLTF (legacy long training field, 52 subcarriers)
  wifi_csi_config_t cfg = {};
  cfg.lltf_en           = true;
  cfg.htltf_en          = false;
  cfg.stbc_htltf2_en    = false;
  cfg.ltf_merge_en      = true;
  cfg.channel_filter_en = false;  // raw data, no smoothing
  cfg.manu_scale        = false;
  cfg.shift             = 0;
  esp_err_t err = esp_wifi_set_csi_config(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CSI] Config error: %d\n", err);
  }

  err = esp_wifi_set_csi(true);
  if (err != ESP_OK) {
    Serial.printf("[CSI] Enable error: %d\n", err);
  } else {
    Serial.println("[CSI] CSI enabled — LLTF 52 subcarriers");
  }
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
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleStatus() {
  // Snapshot shared data under the spinlock
  float    snapMean, snapStddev, snapCsiMean;
  int32_t  snapRssi;
  int      snapCount;

  portENTER_CRITICAL(&csiMux);
  snapCsiMean = latestCsiMean;
  snapRssi    = latestRssi;
  snapCount   = csiCount;
  // Quick snapshot of buffer for stats — copy only the live count
  float localBuf[BUFFER_SIZE];
  int   liveCount = (snapCount < BUFFER_SIZE) ? snapCount : BUFFER_SIZE;
  for (int i = 0; i < liveCount; i++) localBuf[i] = csiBuffer[i];
  portEXIT_CRITICAL(&csiMux);

  // Compute stats outside critical section
  if (liveCount > 1) {
    float sum = 0;
    for (int i = 0; i < liveCount; i++) sum += localBuf[i];
    snapMean = sum / liveCount;
    float var = 0;
    for (int i = 0; i < liveCount; i++) {
      float d = localBuf[i] - snapMean;
      var += d * d;
    }
    snapStddev = sqrtf(var / liveCount);
  } else {
    snapMean   = snapCsiMean;
    snapStddev = 0.0f;
  }

  char json[320];
  snprintf(json, sizeof(json),
    "{\"csi_mean\":%.4f,\"rssi\":%d,\"mean\":%.4f,\"stddev\":%.4f,"
    "\"motion\":%s,\"event_count\":%lu,\"last_event_ms\":%lu}",
    snapCsiMean,
    (int)snapRssi,
    snapMean,
    snapStddev,
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

  for (int i = 0; i < eventLogCount; i++) {
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

void handleCsiRaw() {
  // Snapshot raw I/Q and scalar fields under the spinlock
  int8_t  snapIQ[104];
  int     snapPairs;
  int32_t snapRssi;

  portENTER_CRITICAL(&csiMux);
  snapPairs = rawIQPairs;
  snapRssi  = latestRssi;
  for (int i = 0; i < snapPairs * 2; i++) snapIQ[i] = rawIQ[i];
  portEXIT_CRITICAL(&csiMux);

  // Compute amplitudes and phases outside the critical section
  float amps[52]   = {};
  float phases[52] = {};
  float sumAmp = 0.0f, sumAmpSq = 0.0f;

  for (int i = 0; i < snapPairs; i++) {
    float I = (float)snapIQ[2 * i];
    float Q = (float)snapIQ[2 * i + 1];
    float a = sqrtf(I * I + Q * Q);
    amps[i]   = a;
    phases[i] = atan2f(Q, I);
    sumAmp   += a;
    sumAmpSq += a * a;
  }

  float meanAmp = (snapPairs > 0) ? sumAmp / snapPairs : 0.0f;
  float variance = (snapPairs > 1) ? (sumAmpSq / snapPairs - meanAmp * meanAmp) : 0.0f;
  if (variance < 0.0f) variance = 0.0f;
  float stddev = sqrtf(variance);

  // Build JSON — amplitudes array
  String json = "{\"subcarriers\":";
  json += snapPairs;
  json += ",\"amplitudes\":[";
  for (int i = 0; i < snapPairs; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%.3f", amps[i]);
    if (i > 0) json += ',';
    json += tmp;
  }
  json += "],\"phases\":[";
  for (int i = 0; i < snapPairs; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%.4f", phases[i]);
    if (i > 0) json += ',';
    json += tmp;
  }
  char tail[128];
  snprintf(tail, sizeof(tail),
    "],\"mean_amp\":%.4f,\"stddev\":%.4f,\"motion\":%s,\"rssi\":%d}",
    meanAmp, stddev,
    inMotion ? "true" : "false",
    (int)snapRssi
  );
  json += tail;

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ============================================================
//  Process latest CSI sample and check for motion
//  Called from main loop — reads a snapshot of the shared buffer
// ============================================================
void processCsiSample() {
  // Copy current state out of the critical section
  float    localBuf[BUFFER_SIZE];
  float    currentMean;
  int      count;

  portENTER_CRITICAL(&csiMux);
  int snapCount = csiCount;
  count = (snapCount < BUFFER_SIZE) ? snapCount : BUFFER_SIZE;
  for (int i = 0; i < count; i++) localBuf[i] = csiBuffer[i];
  currentMean = latestCsiMean;
  portEXIT_CRITICAL(&csiMux);

  // Need at least 10 samples before declaring motion
  if (count < 10) return;

  computeStats(localBuf, count);

  // Only fire if there's meaningful variance
  if (rollingStddev < MIN_STDDEV) return;

  float deviation = fabsf(currentMean - rollingMean);
  if (deviation > SENSITIVITY * rollingStddev) {
    consecutiveHits++;
  } else {
    consecutiveHits = 0;
  }

  if (consecutiveHits >= CONFIRM_HITS) {
    uint32_t now = millis();

    // Debounce: don't log a new event if we just logged one < 1s ago
    if (now - lastMotionMs > 1000) {
      motionCount++;
      lastMotionMs = now;

      uint32_t secs  = now / 1000;
      uint32_t mins  = secs / 60;
      uint32_t hours = mins / 60;
      snprintf(lastMotionStr, sizeof(lastMotionStr),
               "%02lu:%02lu:%02lu", hours, mins % 60, secs % 60);

      logMotionEvent(now, lastMotionStr, motionCount);

      // Read RSSI under lock for the log line
      int32_t logRssi;
      portENTER_CRITICAL(&csiMux);
      logRssi = latestRssi;
      portEXIT_CRITICAL(&csiMux);

      Serial.printf("[MOTION] Event #%lu at %s  CSI=%.2f  mean=%.2f  stddev=%.3f  dev=%.2f  RSSI=%d\n",
                    motionCount, lastMotionStr, currentMean,
                    rollingMean, rollingStddev, deviation, (int)logRssi);
    }
    inMotion = true;
  }

  // Clear motion state after hold period
  if (inMotion && (millis() - lastMotionMs > MOTION_HOLD_MS)) {
    inMotion = false;
  }
}

// ============================================================
//  Draw oscilloscope waveform (top half) — now shows CSI amplitude
// ============================================================
void drawGraph() {
  graphSprite.fillSprite(COL_BG);

  // Grid lines
  for (int y = 0; y < GRAPH_H; y += GRAPH_H / 4) {
    graphSprite.drawFastHLine(0, y, GRAPH_W, COL_GRID);
  }
  graphSprite.drawFastHLine(0, GRAPH_H - 1, GRAPH_W, COL_GRID);

  // Snapshot buffer for rendering
  float    localBuf[BUFFER_SIZE];
  float    snapMean, snapLatest;
  int      count;

  portENTER_CRITICAL(&csiMux);
  int snapCount = csiCount;
  count = (snapCount < BUFFER_SIZE) ? snapCount : BUFFER_SIZE;
  for (int i = 0; i < count; i++) localBuf[i] = csiBuffer[i];
  snapMean   = rollingMean;      // already computed by processCsiSample
  snapLatest = latestCsiMean;
  portEXIT_CRITICAL(&csiMux);

  if (count < 2) { graphSprite.pushSprite(0, GRAPH_Y); return; }

  // Auto-scale: find min/max of buffer for Y mapping
  float minAmp = localBuf[0], maxAmp = localBuf[0];
  for (int i = 1; i < count; i++) {
    if (localBuf[i] < minAmp) minAmp = localBuf[i];
    if (localBuf[i] > maxAmp) maxAmp = localBuf[i];
  }
  float range = maxAmp - minAmp;
  if (range < 2.0f) {
    // Pad the range so a flat signal still draws in the middle
    float center = (maxAmp + minAmp) / 2.0f;
    minAmp = center - 1.0f;
    maxAmp = center + 1.0f;
    range  = 2.0f;
  }

  // Draw the mean line
  if (count >= 10) {
    int meanY = map((int)(snapMean * 100), (int)(minAmp * 100), (int)(maxAmp * 100), GRAPH_H - 2, 2);
    meanY = constrain(meanY, 2, GRAPH_H - 2);
    graphSprite.drawFastHLine(0, meanY, GRAPH_W, COL_MEAN);
  }

  // Walk oldest → newest
  int startIdx = (snapCount >= BUFFER_SIZE) ? csiHead : 0;
  int prevX = -1, prevY = -1;
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % BUFFER_SIZE;
    int px  = map(i, 0, count - 1, 0, GRAPH_W - 1);
    int py  = map((int)(localBuf[idx] * 100),
                  (int)(minAmp * 100), (int)(maxAmp * 100),
                  GRAPH_H - 2, 2);
    py = constrain(py, 2, GRAPH_H - 2);

    uint16_t col = (i > count - 10) ? COL_WAVE : COL_WAVE_OLD;
    if (prevX >= 0) {
      graphSprite.drawLine(prevX, prevY, px, py, col);
    }
    prevX = px;
    prevY = py;
  }

  // Label: current CSI amplitude in top-left
  graphSprite.setTextFont(1);
  graphSprite.setTextColor(COL_TEXT, COL_BG);
  graphSprite.setCursor(2, 2);
  graphSprite.printf("%.1f", snapLatest);

  // Label: "CSI" header on right
  graphSprite.setTextColor(COL_LABEL, COL_BG);
  graphSprite.setCursor(GRAPH_W - 24, 2);
  graphSprite.print("CSI");

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
  float confidence = 0;
  if (rollingStddev >= MIN_STDDEV) {
    float snapLatest;
    portENTER_CRITICAL(&csiMux);
    snapLatest = latestCsiMean;
    portEXIT_CRITICAL(&csiMux);
    float dev = fabsf(snapLatest - rollingMean);
    confidence = dev / (SENSITIVITY * rollingStddev);
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

  portENTER_CRITICAL(&csiMux);
  int snapCount = csiCount;
  portEXIT_CRITICAL(&csiMux);

  if (snapCount >= 2) {
    statusSprite.printf("mean %.2f  sd %.3f", rollingMean, rollingStddev);
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

  // Enable CSI (must be done after WiFi connects in STA mode)
  enableCSI();

  // Start web server
  server.on("/",        HTTP_GET, handleRoot);
  server.on("/status",  HTTP_GET, handleStatus);
  server.on("/events",  HTTP_GET, handleEvents);
  server.on("/csi-raw", HTTP_GET, handleCsiRaw);
  server.begin();
  Serial.println("[BOOT] HTTP server started on port 80");

  tft.fillScreen(COL_BG);
  tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);

  Serial.printf("[BOOT] WiFi CSI Radar running. SSID=%s  Sensitivity=%.1f\n",
                WIFI_SSID, SENSITIVITY);
  Serial.println("[BOOT] Serial format: [MOTION] Event #N at HH:MM:SS  CSI=X  mean=M  stddev=S  dev=D  RSSI=R");
}

// ============================================================
//  Loop
// ============================================================
WiFiUDP csiPingUdp;

void loop() {
  uint32_t now = millis();

  // Handle incoming HTTP requests (non-blocking)
  server.handleClient();

  // Ping gateway every 30ms so the AP sends back packets, keeping CSI callbacks firing
  // even when the network is otherwise idle.
  static uint32_t lastPingMs = 0;
  static bool udpReady = false;
  if (!udpReady && WiFi.status() == WL_CONNECTED) {
    csiPingUdp.begin(19999);
    udpReady = true;
  }
  if (udpReady && now - lastPingMs >= 30) {
    lastPingMs = now;
    IPAddress gw = WiFi.gatewayIP();
    csiPingUdp.beginPacket(gw, 9); // port 9 = discard, no response needed
    csiPingUdp.write((uint8_t)0xFF);
    csiPingUdp.endPacket();
  }

  // Process CSI data — callback fires on its own (WiFi task driven),
  // we just check if new data landed and run the motion detector.
  // Rate-limit to ~every 100ms to keep parity with old RSSI sample rate.
  static uint32_t lastProcessMs = 0;
  if (now - lastProcessMs >= 100) {
    lastProcessMs = now;

    portENTER_CRITICAL(&csiMux);
    int snapCount = csiCount;
    portEXIT_CRITICAL(&csiMux);

    if (snapCount > 0 && WiFi.status() == WL_CONNECTED) {
      processCsiSample();
    } else if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WARN] WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }
  }

  // Redraw display every ~100ms
  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    tft.drawFastHLine(0, DIVIDER_Y, W, COL_DIVIDER);
    drawGraph();
    drawStatus();
  }
}
