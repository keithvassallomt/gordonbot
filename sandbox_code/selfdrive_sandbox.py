#!/usr/bin/env python3
"""Serve a lightweight browser controller for GordonBot."""

from __future__ import annotations

import os
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

# ---------------------------------------------------------------------------
# Configuration (mirrors dashboard defaults)
# ---------------------------------------------------------------------------
COMMAND_HZ = 20.0
DEADMAN_MS = 500
MAX_SPEED = 1.0
BOOST_MULTIPLIER = 1.5
KEY_ACCEL_PER_S = 1.2
KEY_DECEL_PER_S = 2.0
KEY_TURN_ACCEL_PER_S = 1.2
KEY_TURN_DECEL_PER_S = 2.0
SENSOR_POLL_MS = 500

CONTROL_WS_PATH = "/ws/control"
DEFAULT_HOST = os.environ.get("GORDONBOT_HOST", "127.0.0.1:8000")
DEFAULT_WS_URL = os.environ.get("GORDONBOT_CONTROL_WS")
if not DEFAULT_WS_URL:
    if DEFAULT_HOST.startswith(("ws://", "wss://")):
        DEFAULT_WS_URL = f"{DEFAULT_HOST}{CONTROL_WS_PATH}"
    elif DEFAULT_HOST.startswith(("http://", "https://")):
        scheme = "wss" if DEFAULT_HOST.startswith("https:") else "ws"
        DEFAULT_WS_URL = f"{scheme}://{urlsplit(DEFAULT_HOST).netloc}{CONTROL_WS_PATH}"
    else:
        DEFAULT_WS_URL = f"ws://{DEFAULT_HOST}{CONTROL_WS_PATH}"

DEFAULT_API_BASE = os.environ.get("GORDONBOT_API")
if not DEFAULT_API_BASE:
    if DEFAULT_HOST.startswith(("http://", "https://")):
        DEFAULT_API_BASE = DEFAULT_HOST
    else:
        DEFAULT_API_BASE = f"http://{DEFAULT_HOST}"

BROWSER_PORT = int(os.environ.get("GORDONBOT_WEB_PORT", "8765"))

PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>GordonBot Manual Drive</title>
  <style>
    :root {{ color-scheme: dark light; font-family: system-ui, -apple-system, BlinkMacSystemFont, \"Segoe UI\", sans-serif; }}
    body {{ margin: 0; padding: 1.5rem; }}
    main {{ width: 100%; max-width: 100%; display: grid; grid-template-columns: 1fr 1fr; gap: 1.5rem; }}
    header {{ grid-column: 1 / -1; }}
    fieldset {{ border: 1px solid #8884; border-radius: 0.75rem; padding: 1rem 1.25rem; }}
    legend {{ padding: 0 0.5rem; font-weight: 600; }}
    label {{ display: flex; flex-direction: column; gap: 0.25rem; font-size: 0.95rem; }}
    input[type=\"text\"] {{ font: inherit; padding: 0.45rem 0.65rem; border-radius: 0.5rem; border: 1px solid #8886; }}
    button {{ font: inherit; padding: 0.55rem 0.85rem; border-radius: 0.5rem; border: none; color: white; background: #2563eb; cursor: pointer; }}
    button[disabled] {{ opacity: 0.6; cursor: not-allowed; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 0.9rem; }}
    td {{ padding: 0.25rem 0; }}
    pre {{ background: #1112; padding: 0.75rem; border-radius: 0.5rem; font-size: 0.9rem; min-height: 4.5rem; }}
    .connection-section {{ grid-column: 1 / -1; }}
    .full-width {{ grid-column: 1 / -1; }}
    @media (max-width: 1024px) {{ main {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
  <main>
    <header>
      <h1>GordonBot Manual Drive</h1>
      <p>Connect, use WASD / Arrows for manual drive, or run the automated moves.</p>
    </header>
    <fieldset class="connection-section">
      <legend>Connection</legend>
      <label>
        Control WebSocket URL
        <input id=\"ws-url\" type=\"text\" value=\"{ws_url}\" autocomplete=\"off\" spellcheck=\"false\" />
      </label>
      <label>
        Sensors API Base URL
        <input id=\"api-url\" type=\"text\" value=\"{api_base}\" autocomplete=\"off\" spellcheck=\"false\" />
      </label>
      <div style=\"display:flex; gap:0.6rem; align-items:center; flex-wrap:wrap;\">
        <button id=\"connect-btn\">Connect</button>
        <button id=\"square-btn\" disabled>Square 20cm</button>
        <button id=\"straight-btn\" disabled>Straight 0.5m</button>
        <button id=\"circle-btn\" disabled>Circle r20cm</button>
        <button id=\"figure-eight-btn\" disabled>Figure-8 r20cm</button>
        <button id=\"turn-left-btn\" disabled>Turn Left 90°</button>
        <button id=\"turn-right-btn\" disabled>Turn Right 90°</button>
        <button id=\"zero-btn\" disabled>Zero Telemetry</button>
        <button id=\"test-power-btn\" disabled>Test Power</button>
        <label style=\"display:flex; align-items:center; gap:0.4rem; margin-left:0.5rem;\">
          <input type=\"checkbox\" id=\"stepped-mode\" />
          <span>Stepped</span>
        </label>
        <button id=\"step-next-btn\" disabled style=\"display:none;\">Next Step</button>
        <div id=\"power-test-controls\" style=\"display:none; gap:0.6rem; align-items:center;\">
          <label style=\"display:flex; align-items:center; gap:0.4rem;\">
            How many degrees turned?
            <input type=\"number\" id=\"power-test-degrees\" min=\"0\" max=\"360\" step=\"1\" placeholder=\"0-360\" style=\"width:80px;\" disabled />
          </label>
          <button id=\"power-test-submit\" disabled>Submit</button>
        </div>
        <span id=\"status\">Disconnected</span>
      </div>
    </fieldset>
    <fieldset>
      <legend>Live Telemetry</legend>
      <table>
        <tr><td>Status:</td><td id=\"tele-status\">idle</td></tr>
        <tr><td>Left cmd:</td><td id=\"tele-left\">0.00</td></tr>
        <tr><td>Right cmd:</td><td id=\"tele-right\">0.00</td></tr>
        <tr><td>Boost:</td><td id=\"tele-boost\">off</td></tr>
        <tr><td>Dead-man:</td><td id=\"tele-deadman\">inactive</td></tr>
        <tr><td>Left dist (m):</td><td id=\"tele-dist-left\">n/a</td></tr>
        <tr><td>Right dist (m):</td><td id=\"tele-dist-right\">n/a</td></tr>
        <tr><td>Avg dist (m):</td><td id=\"tele-dist-avg\">n/a</td></tr>
        <tr><td>Yaw (°):</td><td id=\"tele-yaw\">n/a</td></tr>
      </table>
    </fieldset>
    <fieldset>
      <legend>Activity Log</legend>
      <div id=\"log-container\" style=\"height: 400px; overflow-y: auto; background: #1112; padding: 0.75rem; border-radius: 0.5rem; font-family: monospace; font-size: 0.85rem; line-height: 1.4;\"></div>
    </fieldset>
    <fieldset class=\"full-width\">
      <legend>Keyboard Shortcuts</legend>
      <pre>W / ↑  Forward
S / ↓  Reverse
A / ←  Pivot Left
D / →  Pivot Right
Shift  Boost
Space  Stop
Q or Esc  Disconnect</pre>
    </fieldset>
  </main>
  <script>
    (() => {{
      const COMMAND_HZ = {command_hz};
      const DEADMAN_MS = {deadman_ms};
      const MAX_SPEED = {max_speed};
      const BOOST_MULTIPLIER = {boost};
      const KEY_ACCEL_PER_S = {key_accel};
      const KEY_DECEL_PER_S = {key_decel};
      const KEY_TURN_ACCEL_PER_S = {key_turn_accel};
      const KEY_TURN_DECEL_PER_S = {key_turn_decel};
      const SENSOR_POLL_MS = {poll_ms};

      // Calibration data from power test (deg/sec at each power level)
      const TURN_CALIBRATION = [
        {{ power: 0.5, degPerSec: 1.1 }},
        {{ power: 0.6, degPerSec: 3.6 }},
        {{ power: 0.7, degPerSec: 31.3 }},
        {{ power: 0.8, degPerSec: 81.6 }},
        {{ power: 0.9, degPerSec: 149.5 }},
        {{ power: 1.0, degPerSec: 221.4 }}
      ];

      const statusEl = document.getElementById('status');
      const teleStatus = document.getElementById('tele-status');
      const teleLeft = document.getElementById('tele-left');
      const teleRight = document.getElementById('tele-right');
      const teleBoost = document.getElementById('tele-boost');
      const teleDeadman = document.getElementById('tele-deadman');
      const teleDistLeft = document.getElementById('tele-dist-left');
      const teleDistRight = document.getElementById('tele-dist-right');
      const teleDistAvg = document.getElementById('tele-dist-avg');
      const teleYaw = document.getElementById('tele-yaw');

      const connectBtn = document.getElementById('connect-btn');
      const squareBtn = document.getElementById('square-btn');
      const straightBtn = document.getElementById('straight-btn');
      const circleBtn = document.getElementById('circle-btn');
      const figureEightBtn = document.getElementById('figure-eight-btn');
      const turnLeftBtn = document.getElementById('turn-left-btn');
      const turnRightBtn = document.getElementById('turn-right-btn');
      const zeroBtn = document.getElementById('zero-btn');
      const testPowerBtn = document.getElementById('test-power-btn');
      const steppedModeCheckbox = document.getElementById('stepped-mode');
      const stepNextBtn = document.getElementById('step-next-btn');
      const powerTestControls = document.getElementById('power-test-controls');
      const powerTestDegreesInput = document.getElementById('power-test-degrees');
      const powerTestSubmitBtn = document.getElementById('power-test-submit');
      const wsInput = document.getElementById('ws-url');
      const apiInput = document.getElementById('api-url');
      const logContainer = document.getElementById('log-container');

      const held = new Set();
      const state = {{
        ws: null,
        timer: null,
        sensorTimer: null,
        vec: {{ x: 0, y: 0 }},
        target: {{ x: 0, y: 0 }},
        boost: false,
        lastInput: performance.now(),
        lastTick: performance.now(),
        autoActive: false,
        autoAbort: false,
        lastLeft: 0,
        lastRight: 0,
        distLeft: null,
        distRight: null,
        distAvg: null,
        yaw: null,
        // Zero offsets for telemetry
        zeroDistLeft: 0,
        zeroDistRight: 0,
        zeroDistAvg: 0,
        zeroYaw: 0,
        // Stepped mode control
        stepResolve: null,
        // Power test control
        powerTestResolve: null,
      }};

      const clamp = (n, lo = -1, hi = 1) => Math.max(lo, Math.min(hi, n));
      const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
      const vecToTank = (v) => ({{ left: clamp(v.y + v.x), right: clamp(v.y - v.x) }});
      const normalizeHeading = (deg) => ((deg % 360) + 360) % 360;
      const normalizeAngleDiff = (deg) => ((deg + 180) % 360 + 360) % 360 - 180;

      function setStatus(msg) {{ statusEl.textContent = msg; }}

      // Stepped mode: wait for user to click "Next Step"
      function waitForStep() {{
        if (!steppedModeCheckbox.checked) return Promise.resolve();
        return new Promise((resolve) => {{
          state.stepResolve = resolve;
          stepNextBtn.disabled = false;
          stepNextBtn.style.display = 'inline-block';
          logWarn('Waiting for step confirmation...');
        }});
      }}

      stepNextBtn.addEventListener('click', () => {{
        if (state.stepResolve) {{
          state.stepResolve();
          state.stepResolve = null;
          stepNextBtn.disabled = true;
          stepNextBtn.style.display = 'none';
          log('Step confirmed, continuing...');
        }}
      }});

      // Logging functions with timestamps and auto-scroll
      function log(message, type = 'info') {{
        const timestamp = new Date().toLocaleTimeString('en-US', {{ hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' }});
        const entry = document.createElement('div');
        entry.style.marginBottom = '0.25rem';

        let color = '#888';
        let prefix = '[INFO]';
        if (type === 'success') {{ color = '#4ade80'; prefix = '[✓]'; }}
        else if (type === 'error') {{ color = '#f87171'; prefix = '[✗]'; }}
        else if (type === 'warn') {{ color = '#fbbf24'; prefix = '[!]'; }}
        else if (type === 'action') {{ color = '#60a5fa'; prefix = '[→]'; }}

        entry.innerHTML = `<span style="color: #666;">${{timestamp}}</span> <span style="color: ${{color}}; font-weight: 600;">${{prefix}}</span> ${{message}}`;
        logContainer.appendChild(entry);

        // Auto-scroll to bottom
        logContainer.scrollTop = logContainer.scrollHeight;

        // Limit to last 100 entries
        while (logContainer.children.length > 100) {{
          logContainer.removeChild(logContainer.firstChild);
        }}
      }}

      function logSuccess(msg) {{ log(msg, 'success'); }}
      function logError(msg) {{ log(msg, 'error'); }}
      function logWarn(msg) {{ log(msg, 'warn'); }}
      function logAction(msg) {{ log(msg, 'action'); }}

      function setDriveVector(x, y, boost = false) {{
        state.target.x = clamp(x);
        state.target.y = clamp(y);
        state.boost = boost;
        state.lastInput = performance.now();
      }}

      function stopDrive() {{ setDriveVector(0, 0, false); }}

      function updateTargetsFromKeys() {{
        if (state.autoActive) return;
        const forward = held.has('forward');
        const backward = held.has('backward');
        const left = held.has('left');
        const right = held.has('right');
        state.target.y = forward && !backward ? 1 : backward && !forward ? -1 : 0;
        state.target.x = right && !left ? 1 : left && !right ? -1 : 0;
        state.boost = held.has('boost');
        state.lastInput = performance.now();
      }}

      function updateTelemetry(left, right, boost, inactive) {{
        teleStatus.textContent = state.ws ? 'connected' : 'disconnected';
        teleLeft.textContent = left.toFixed(2);
        teleRight.textContent = right.toFixed(2);
        teleBoost.textContent = boost ? 'ON' : 'off';
        teleDeadman.textContent = inactive ? 'tripped' : 'clear';
        // Apply zero offsets to telemetry display
        teleDistLeft.textContent = state.distLeft != null ? (state.distLeft - state.zeroDistLeft).toFixed(3) : 'n/a';
        teleDistRight.textContent = state.distRight != null ? (state.distRight - state.zeroDistRight).toFixed(3) : 'n/a';
        teleDistAvg.textContent = state.distAvg != null ? (state.distAvg - state.zeroDistAvg).toFixed(3) : 'n/a';
        const displayYaw = state.yaw != null ? normalizeHeading(state.yaw - state.zeroYaw) : null;
        teleYaw.textContent = displayYaw != null ? displayYaw.toFixed(1) : 'n/a';
      }}

      const keyMap = new Map([
        ['KeyW', 'forward'], ['ArrowUp', 'forward'],
        ['KeyS', 'backward'], ['ArrowDown', 'backward'],
        ['KeyA', 'left'], ['ArrowLeft', 'left'],
        ['KeyD', 'right'], ['ArrowRight', 'right'],
        ['ShiftLeft', 'boost'], ['ShiftRight', 'boost'],
        ['Space', 'stop'], ['Digit0', 'stop'], ['Numpad0', 'stop'],
        ['KeyQ', 'disconnect'], ['Escape', 'disconnect'],
      ]);

      window.addEventListener('keydown', (e) => {{
        if (e.repeat) return;
        if (e.target instanceof HTMLInputElement) return;
        const action = keyMap.get(e.code) || keyMap.get(e.key);
        if (!action) return;
        e.preventDefault();
        if (action === 'stop') {{
          held.clear();
          stopDrive();
        }} else if (action === 'disconnect') {{
          requestDisconnect();
        }} else {{
          held.add(action);
          if (action === 'boost') state.boost = true;
          updateTargetsFromKeys();
        }}
      }});

      window.addEventListener('keyup', (e) => {{
        const action = keyMap.get(e.code) || keyMap.get(e.key);
        if (!action) return;
        e.preventDefault();
        if (action === 'boost') held.delete('boost');
        else if (action !== 'disconnect' && action !== 'stop') held.delete(action);
        updateTargetsFromKeys();
      }});

      function resolveWsUrl() {{
        const raw = wsInput.value.trim();
        if (!raw) return '';
        if (raw.startsWith('ws://') || raw.startsWith('wss://')) return raw;
        const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        if (raw.startsWith('/')) return `${{scheme}}//${{window.location.host}}${{raw}}`;
        return `${{scheme}}//${{raw}}`;
      }}

      function currentApiBase() {{
        let base = apiInput.value.trim();
        if (!base) {{
          const wsUrl = resolveWsUrl();
          try {{
            if (wsUrl) {{
              const u = new URL(wsUrl);
              const proto = u.protocol === 'wss:' ? 'https:' : 'http:';
              base = `${{proto}}//${{u.host}}`;
            }}
          }} catch (err) {{
            console.warn('Unable to derive API base', err);
          }}
        }}
        if (base.endsWith('/')) base = base.slice(0, -1);
        return base;
      }}

      async function pollSensorsOnce() {{
        const base = currentApiBase();
        if (!base) return;
        try {{
          const res = await fetch(`${{base}}/api/sensors/status`, {{ cache: 'no-store' }});
          if (!res.ok) throw new Error(res.statusText);
          const data = await res.json();
          const enc = data?.encoders || {{}};
          const left = enc.left?.distance_m ?? null;
          const right = enc.right?.distance_m ?? null;
          let avg = null;
          if (left != null && right != null) avg = (left + right) / 2;
          else if (left != null) avg = left;
          else if (right != null) avg = right;
          const yaw = data?.bno055?.euler?.yaw ?? null;
          state.distLeft = typeof left === 'number' ? left : null;
          state.distRight = typeof right === 'number' ? right : null;
          state.distAvg = typeof avg === 'number' ? avg : null;
          state.yaw = typeof yaw === 'number' ? normalizeHeading(yaw) : null;
          updateTelemetry(state.lastLeft, state.lastRight, state.boost, false);
        }} catch (err) {{
          console.warn('Sensor poll failed', err);
          if (state.autoActive) {{
            logError(`Sensor poll failed: ${{err.message}}`);
          }}
        }}
      }}

      function startSensorTimer() {{
        if (state.sensorTimer) return;
        pollSensorsOnce();
        state.sensorTimer = setInterval(pollSensorsOnce, SENSOR_POLL_MS);
      }}

      function stopSensorTimer() {{
        if (!state.sensorTimer) return;
        clearInterval(state.sensorTimer);
        state.sensorTimer = null;
      }}

      function sendDriveCommand(left, right) {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;

        // Check WebSocket buffer state to prevent overflow
        // bufferedAmount returns bytes queued but not yet sent
        if (state.ws.bufferedAmount > 1024) {{
          console.warn('WebSocket send buffer high:', state.ws.bufferedAmount, 'bytes');
          // Still try to send, but this indicates network/backend slowness
        }}

        try {{
          state.ws.send(JSON.stringify({{ type: 'drive', payload: {{ left, right, ts: Date.now() }} }}));
        }} catch (err) {{
          console.error('send error', err);
        }}
      }}

      function tick() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
        const now = performance.now();
        const dt = Math.min(0.1, Math.max(0, (now - state.lastTick) / 1000));
        state.lastTick = now;

        const approach = (cur, tgt, accel, decel) => {{
          const rate = Math.abs(tgt) > Math.abs(cur) ? accel : decel;
          const maxDelta = rate * dt;
          const delta = clamp(tgt - cur, -maxDelta, maxDelta);
          const next = clamp(cur + delta);
          return Math.abs(next - tgt) < 1e-4 ? tgt : next;
        }};

        state.vec.x = approach(state.vec.x, state.target.x, KEY_TURN_ACCEL_PER_S, KEY_TURN_DECEL_PER_S);
        state.vec.y = approach(state.vec.y, state.target.y, KEY_ACCEL_PER_S, KEY_DECEL_PER_S);

        if (Math.hypot(state.vec.x, state.vec.y) > 1e-3 || Math.hypot(state.target.x, state.target.y) > 1e-3) {{
          state.lastInput = now;
        }}

        const inactive = (now - state.lastInput) > DEADMAN_MS;
        const tank = vecToTank(state.vec);
        let left = tank.left * MAX_SPEED * (state.boost ? BOOST_MULTIPLIER : 1);
        let right = tank.right * MAX_SPEED * (state.boost ? BOOST_MULTIPLIER : 1);

        if (inactive) {{ left = 0; right = 0; }}

        left = clamp(left);
        right = clamp(right);
        state.lastLeft = left;
        state.lastRight = right;
        updateTelemetry(left, right, state.boost, inactive);
        sendDriveCommand(left, right);
      }}

      function startCommandTimer() {{
        if (state.timer) return;
        state.lastTick = performance.now();
        state.timer = setInterval(tick, 1000 / COMMAND_HZ);
      }}

      function stopCommandTimer() {{
        if (state.timer) {{
          clearInterval(state.timer);
          state.timer = null;
        }}
      }}

      function cleanupSocket() {{
        stopCommandTimer();
        stopSensorTimer();
        if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
          try {{ state.ws.close(); }} catch (err) {{}}
        }}
        state.ws = null;
        squareBtn.disabled = true;
        straightBtn.disabled = true;
        circleBtn.disabled = true;
        figureEightBtn.disabled = true;
        turnLeftBtn.disabled = true;
        turnRightBtn.disabled = true;
        zeroBtn.disabled = true;
        testPowerBtn.disabled = true;
        squareBtn.textContent = 'Square 20cm';
        straightBtn.textContent = 'Straight 0.5m';
        circleBtn.textContent = 'Circle r20cm';
        figureEightBtn.textContent = 'Figure-8 r20cm';
        turnLeftBtn.textContent = 'Turn Left 90°';
        turnRightBtn.textContent = 'Turn Right 90°';
        testPowerBtn.textContent = 'Test Power';
        state.autoActive = false;
        state.autoAbort = false;
        state.lastLeft = 0;
        state.lastRight = 0;
        state.distLeft = state.distRight = state.distAvg = state.yaw = null;
        state.zeroDistLeft = state.zeroDistRight = state.zeroDistAvg = state.zeroYaw = 0;
        held.clear();
        stopDrive();
        updateTelemetry(0, 0, false, false);
      }}

      function connect() {{
        const url = resolveWsUrl();
        if (!url) {{ setStatus('Enter WebSocket URL'); return; }}
        try {{
          const ws = new WebSocket(url);
          state.ws = ws;
          setStatus('Connecting...');
          ws.addEventListener('open', () => {{
            setStatus('Connected');
            logSuccess('WebSocket connected');
            state.lastTick = performance.now();
            state.lastInput = performance.now();
            startCommandTimer();
            startSensorTimer();
            stopDrive();
            squareBtn.disabled = false;
            straightBtn.disabled = false;
            circleBtn.disabled = false;
            figureEightBtn.disabled = false;
            turnLeftBtn.disabled = false;
            turnRightBtn.disabled = false;
            zeroBtn.disabled = false;
            testPowerBtn.disabled = false;
            connectBtn.textContent = 'Disconnect';
          }});
          ws.addEventListener('close', () => {{
            setStatus('Disconnected');
            logWarn('WebSocket disconnected');
            cleanupSocket();
          }});
          ws.addEventListener('error', (err) => {{
            console.error('WebSocket error', err);
            logError('WebSocket connection error');
            setStatus('Error');
            cleanupSocket();
          }});
        }} catch (err) {{
          console.error('WS connect failed', err);
          setStatus('Failed to connect');
          cleanupSocket();
        }}
      }}

      function requestDisconnect() {{
        if (!state.ws) return;
        state.autoAbort = true;
        stopDrive();
        setStatus('Disconnecting...');
        try {{ state.ws.close(); }} catch (err) {{}}
      }}

      connectBtn.addEventListener('click', () => {{
        if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
          requestDisconnect();
        }} else {{
          connect();
        }}
      }});

      async function driveDistanceMeters(distance, apiBase, desiredHeading) {{
        const forward = distance >= 0 ? 1 : -1;
        const target = Math.abs(distance);
        const cruise = 0.7;
        let startLeft = state.distLeft;
        let startRight = state.distRight;
        let startAvg = state.distAvg;
        const yawTarget = typeof desiredHeading === 'number' ? normalizeHeading(desiredHeading) : null;

        const direction = forward > 0 ? 'forward' : 'backward';
        if (yawTarget != null) {{
          logAction(`Driving ${{direction}} ${{(target * 1000).toFixed(0)}}mm at heading ${{yawTarget.toFixed(1)}}°`);
        }} else {{
          logAction(`Driving ${{direction}} ${{(target * 1000).toFixed(0)}}mm (no heading hold)`);
        }}

        setDriveVector(0, forward * cruise, false);
        let lastLogDistance = 0;

        while (!state.autoAbort) {{
          await sleep(120);
          await pollSensorsOnce();
          if (state.autoAbort) break;

          if (yawTarget != null && state.yaw != null) {{
            const yawErr = normalizeAngleDiff(yawTarget - state.yaw);
            const correction = clamp(yawErr / 45, -0.45, 0.45);
            if (Math.abs(correction) > 0.05) setDriveVector(correction, forward * cruise, false);
          }}

          let travelled = null;
          if (state.distAvg != null && startAvg != null) travelled = Math.abs(state.distAvg - startAvg);
          else if (state.distLeft != null && startLeft != null && state.distRight != null && startRight != null)
            travelled = (Math.abs(state.distLeft - startLeft) + Math.abs(state.distRight - startRight)) / 2;
          else if (state.distLeft != null && startLeft != null) travelled = Math.abs(state.distLeft - startLeft);
          else if (state.distRight != null && startRight != null) travelled = Math.abs(state.distRight - startRight);

          // Log progress every 50mm
          if (travelled != null && travelled - lastLogDistance >= 0.05) {{
            log(`Progress: ${{(travelled * 1000).toFixed(0)}}/${{(target * 1000).toFixed(0)}}mm`);
            lastLogDistance = travelled;
          }}

          if (travelled != null && travelled >= target) break;
        }}

        stopDrive();

        let finalDist = null;
        if (state.distAvg != null && startAvg != null) finalDist = Math.abs(state.distAvg - startAvg);
        else if (state.distLeft != null && startLeft != null && state.distRight != null && startRight != null)
          finalDist = (Math.abs(state.distLeft - startLeft) + Math.abs(state.distRight - startRight)) / 2;

        if (state.autoAbort) {{
          logWarn(`Drive aborted at ${{finalDist != null ? (finalDist * 1000).toFixed(0) : '?'}}mm`);
        }} else {{
          logSuccess(`Drive complete: ${{finalDist != null ? (finalDist * 1000).toFixed(0) : '?'}}mm`);
        }}

        await sleep(250);
        return !state.autoAbort;
      }}

      async function turnToHeading(targetHeading, apiBase) {{
        const target = normalizeHeading(targetHeading);
        let yaw = state.yaw;
        if (yaw == null) {{
          await pollSensorsOnce();
          yaw = state.yaw;
        }}
        if (yaw == null) {{
          logWarn('IMU unavailable, performing blind turn');
          // Fallback: blind turn at fixed speed/time if IMU unavailable
          setDriveVector(0.55, 0, false);
          await sleep(800);
          stopDrive();
          await sleep(200);
          return !state.autoAbort;
        }}

        const startYaw = normalizeHeading(yaw);
        logAction(`Turning from ${{startYaw.toFixed(1)}}° to ${{target.toFixed(1)}}° (Δ${{normalizeAngleDiff(target - startYaw).toFixed(1)}}°)`);

        // Stop the background tick() timer to prevent command conflicts
        const wasRunning = state.timer != null;
        stopCommandTimer();

        let diff = normalizeAngleDiff(target - yaw);
        let guard = 0;
        let iterations = 0;
        let prevDiffSign = Math.sign(diff) || 1;
        let overshootPenalty = 1;

        // Threshold for completion: 5° for precise turns
        while (!state.autoAbort && Math.abs(diff) > 5) {{
          iterations++;

          // Direction logic adjusted for hardware where RIGHT turn increases yaw, LEFT turn decreases yaw
          // Positive diff means yaw needs to increase → turn RIGHT (dir = -1)
          // Negative diff means yaw needs to decrease → turn LEFT (dir = 1)
          const dir = diff > 0 ? -1 : 1;

          // Use calibration data to calculate power and duration
          const errorDeg = Math.abs(diff);

          // Choose power and duration parameters based on current error
          let power, degPerSec, fudge, minMs, maxMs;
          if (errorDeg > 60) {{
            power = 0.8;
            degPerSec = 81.6;
            fudge = 0.55;
            minMs = 160;
            maxMs = 900;
          }} else if (errorDeg > 30) {{
            power = 0.78;
            degPerSec = 31.3 + ((power - 0.7) / 0.1) * (81.6 - 31.3);
            fudge = 0.6;
            minMs = 140;
            maxMs = 750;
          }} else if (errorDeg > 10) {{
            power = 0.75;
            degPerSec = 31.3 + ((power - 0.7) / 0.1) * (81.6 - 31.3);
            fudge = 0.7;
            minMs = 130;
            maxMs = 520;
          }} else {{
            power = 0.7;
            degPerSec = 31.3;
            fudge = 0.8;
            minMs = 120;
            maxMs = 320;
          }}

          // Calculate duration based on measured speed and error, with overshoot adjustment
          const predictedMs = (errorDeg / degPerSec) * 1000;
          const duration = Math.max(
            minMs,
            Math.min(maxMs, predictedMs * fudge * overshootPenalty),
          );

          // Tank turn: pivot in place with counter-rotating wheels
          // Left turn (dir=1): left backward, right forward
          // Right turn (dir=-1): left forward, right backward
          const leftCmd = -dir * power;
          const rightCmd = dir * power;

          // Log iteration details: show how many degrees off from target (with sign)
          const degreesOff = normalizeAngleDiff(yaw - target);
          const turnDir = dir > 0 ? 'LEFT' : 'RIGHT';
          log(`Iteration ${{iterations}}: yaw ${{yaw.toFixed(1)}}° | error (${{degreesOff >= 0 ? '+' : ''}}${{degreesOff.toFixed(1)}}° off) | commanding: ${{turnDir}} turn at power ${{power.toFixed(1)}} for ${{duration.toFixed(0)}}ms (L:${{leftCmd.toFixed(2)}}, R:${{rightCmd.toFixed(2)}})`);

          // Wait for step confirmation if in stepped mode
          await waitForStep();

          // Send raw tank commands for pivot turn continuously for calculated duration
          const turnStart = performance.now();
          while (performance.now() - turnStart < duration) {{
            sendDriveCommand(leftCmd, rightCmd);
            state.lastInput = performance.now();
            await sleep(50);  // Send command every 50ms to keep dead-man timer happy
          }}

          // Stop motors and allow brief settling before polling
          sendDriveCommand(0, 0);
          await sleep(50);
          sendDriveCommand(0, 0);

          // Retry sensor polling with backoff (up to ~400ms total wait)
          let retries = 0;
          let backoff = 50;
          let waited = 0;
          const maxBackoffWindow = 400;
          while (waited <= maxBackoffWindow) {{
            await pollSensorsOnce();
            yaw = state.yaw;
            if (yaw != null) break;
            retries++;
            if (waited >= maxBackoffWindow) break;
            const delay = Math.min(backoff, maxBackoffWindow - waited);
            logWarn(`IMU read failed, retry ${{retries}} (waiting ${{delay}}ms)...`);
            await sleep(delay);
            waited += delay;
            backoff = Math.min(Math.round(backoff * 1.6), maxBackoffWindow);
          }}

          if (yaw == null) {{
            logError('IMU unavailable after retries, aborting turn');
            break;
          }}

          const previousDiff = diff;
          diff = normalizeAngleDiff(target - yaw);
          const newDiffSign = Math.sign(diff) || prevDiffSign;
          if (newDiffSign !== prevDiffSign && Math.abs(diff) < Math.abs(previousDiff)) {{
            // We crossed past the target — damp the next pulse significantly
            overshootPenalty = 0.55;
          }} else if (Math.abs(diff) > Math.abs(previousDiff) + 2) {{
            // Diverging; give the next pulse a bit more punch (but stay safe)
            overshootPenalty = 0.9;
          }} else {{
            overshootPenalty = 1;
          }}
          prevDiffSign = newDiffSign;

          // Safety: if we've rotated way more than intended (>1.8x desired rotation), stop immediately
          const actualRotationSoFar = Math.abs(normalizeAngleDiff(yaw - startYaw));
          const desiredRotation = Math.abs(normalizeAngleDiff(target - startYaw));
          if (actualRotationSoFar > desiredRotation * 1.8) {{
            logWarn(`Massive overshoot detected (rotated ${{actualRotationSoFar.toFixed(1)}}° vs target ${{desiredRotation.toFixed(1)}}°), stopping turn`);
            break;
          }}

          if (++guard > 120) break;
        }}

        // Send explicit stop command and ensure it's processed
        sendDriveCommand(0, 0);
        await sleep(50);  // Brief delay to ensure stop command is sent
        sendDriveCommand(0, 0);  // Send again for safety
        stopDrive();

        // Restart the background timer if it was running
        if (wasRunning) {{
          startCommandTimer();
        }}

        // Get final heading from last sensor reading (don't fake it with target!)
        const finalYaw = state.yaw != null ? normalizeHeading(state.yaw) : null;

        // Calculate error: how much we rotated vs how much we wanted to rotate
        let finalError = null;
        if (finalYaw != null) {{
          const actualRotation = Math.abs(normalizeAngleDiff(finalYaw - startYaw));
          const desiredRotation = Math.abs(normalizeAngleDiff(target - startYaw));
          finalError = Math.abs(desiredRotation - actualRotation);
        }}

        if (state.autoAbort) {{
          const yawStr = finalYaw != null ? `${{finalYaw.toFixed(1)}}°` : 'unknown';
          const errStr = finalError != null ? `${{finalError.toFixed(1)}}°` : 'unknown';
          logWarn(`Turn aborted at ${{yawStr}} (error: ${{errStr}})`);
        }} else if (finalError != null && finalError <= 5) {{
          logSuccess(`Turn complete: ${{finalYaw.toFixed(1)}}° (${{iterations}} iterations, error: ${{finalError.toFixed(1)}}°)`);
        }} else if (finalError != null) {{
          logWarn(`Turn ended with error ${{finalError.toFixed(1)}}° at ${{finalYaw.toFixed(1)}}°`);
        }} else {{
          logError(`Turn ended but IMU reading unavailable (${{iterations}} iterations)`);
        }}

        await sleep(150);
        return !state.autoAbort;
      }}

      async function runSquare() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{ setStatus('Connect first'); return; }}
        if (state.autoActive) return;
        state.autoActive = true;
        state.autoAbort = false;
        squareBtn.textContent = 'Abort Square';
        straightBtn.disabled = true;
        circleBtn.disabled = true;
        figureEightBtn.disabled = true;
        turnLeftBtn.disabled = true;
        turnRightBtn.disabled = true;
        held.clear();
        stopDrive();
        setStatus('Running square (20cm sides)');

        log('═══════════════════════════════════');
        logAction('Starting square pattern (20cm sides)');

        try {{
          // Poll sensors to get initial heading
          await pollSensorsOnce();
          await sleep(200);

          // Get starting heading or default to 0
          let heading = state.yaw;
          if (heading == null) {{
            logWarn('No IMU heading available, using 0° as reference');
            heading = 0;
          }} else {{
            log(`Initial heading: ${{heading.toFixed(1)}}°`);
          }}
          heading = normalizeHeading(heading);

          const base = currentApiBase();
          const sideLength = 0.20; // 20cm sides

          // Drive all four sides of the square
          for (let i = 0; i < 4 && !state.autoAbort; i += 1) {{
            log(`─────── Side ${{i + 1}}/4 ───────`);
            setStatus(`Square: side ${{i + 1}}/4 (heading ${{heading.toFixed(1)}}°)`);

            // Drive forward for one side, maintaining heading
            const ok = await driveDistanceMeters(sideLength, base, heading);
            if (!ok || state.autoAbort) break;

            // Pause briefly after completing the side
            await sleep(300);
            if (state.autoAbort) break;

            // Turn 90° left for next side (add 90° to heading)
            heading = normalizeHeading(heading + 90);
            setStatus(`Square: turning to ${{heading.toFixed(1)}}°`);

            const turnResult = await runTurnLeft90({{
              nested: true,
              messagePrefix: 'Square',
              actionLabel: 'Square: turning 90° left',
              targetHeading: heading,
            }});
            if (state.autoAbort) break;
            if (!turnResult?.success) break;

            if (turnResult.finalYaw != null) {{
              heading = normalizeHeading(turnResult.finalYaw);
            }}

            // Pause briefly after turning before next side
            await sleep(300);
          }}

          // Final status
          if (state.autoAbort) {{
            logError('Square pattern aborted by user');
            setStatus('Square aborted');
          }} else {{
            logSuccess('Square pattern complete!');
            setStatus('Square complete (20cm sides)');
          }}
          log('═══════════════════════════════════');
        }} catch (err) {{
          console.error('Square run failed', err);
          logError(`Square error: ${{err.message}}`);
          setStatus('Square error: ' + err.message);
        }} finally {{
          state.autoActive = false;
          state.autoAbort = false;
          squareBtn.textContent = 'Square 20cm';
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            straightBtn.disabled = false;
            circleBtn.disabled = false;
            figureEightBtn.disabled = false;
            turnLeftBtn.disabled = false;
            turnRightBtn.disabled = false;
          }} else {{
            straightBtn.disabled = true;
            circleBtn.disabled = true;
            figureEightBtn.disabled = true;
            turnLeftBtn.disabled = true;
            turnRightBtn.disabled = true;
          }}
          stopDrive();
        }}
      }}

      async function runStraight() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{ setStatus('Connect first'); return; }}
        if (state.autoActive) return;
        state.autoActive = true;
        state.autoAbort = false;
        straightBtn.textContent = 'Abort Straight';
        squareBtn.disabled = true;
        circleBtn.disabled = true;
        figureEightBtn.disabled = true;
        turnLeftBtn.disabled = true;
        turnRightBtn.disabled = true;
        held.clear();
        stopDrive();
        setStatus('Driving straight');

        log('═══════════════════════════════════');
        logAction('Starting straight line drive (0.5m)');

        try {{
          await pollSensorsOnce();
          const base = currentApiBase();
          const heading = state.yaw ?? 0;

          if (state.yaw == null) {{
            logWarn('No IMU heading, driving without heading correction');
          }} else {{
            log(`Locked heading: ${{heading.toFixed(1)}}°`);
          }}

          const ok = await driveDistanceMeters(0.5, base, heading);

          if (state.autoAbort) {{
            logError('Straight line aborted by user');
            setStatus('Straight aborted');
          }} else if (ok) {{
            logSuccess('Straight line complete!');
            setStatus('Straight complete');
          }} else {{
            logWarn('Straight line incomplete');
            setStatus('Straight incomplete');
          }}
          log('═══════════════════════════════════');
        }} catch (err) {{
          console.error('Straight run failed', err);
          logError(`Straight line error: ${{err.message}}`);
          setStatus('Straight error');
        }} finally {{
          state.autoActive = false;
          state.autoAbort = false;
          straightBtn.textContent = 'Straight 0.5m';
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            squareBtn.disabled = false;
            circleBtn.disabled = false;
            figureEightBtn.disabled = false;
            turnLeftBtn.disabled = false;
            turnRightBtn.disabled = false;
          }} else {{
            squareBtn.disabled = true;
            circleBtn.disabled = true;
            figureEightBtn.disabled = true;
            turnLeftBtn.disabled = true;
            turnRightBtn.disabled = true;
          }}
          stopDrive();
        }}
      }}

      async function driveCircle(direction, radius, startYaw) {{
        // Helper function to drive a single circle (left or right)
        // direction: 'left' (negative turn) or 'right' (positive turn)
        // radius: in metres
        // startYaw: starting heading
        // Returns: {{ success: boolean, travelled: number }}

        const trackWidth = 0.13;      // approximate wheel track (m)
        const circumference = 2 * Math.PI * radius;
        const forwardThrottle = 0.6;
        const wheelRatio = (radius + trackWidth / 2) / Math.max(0.05, radius - trackWidth / 2);
        const baseTurnMagnitude = forwardThrottle * (wheelRatio - 1) / (wheelRatio + 1);
        const baseTurn = direction === 'left'
          ? -clamp(baseTurnMagnitude, 0.05, 0.35)  // negative → left arc
          : clamp(baseTurnMagnitude, 0.05, 0.35);   // positive → right arc

        // Snapshot encoder distances for travelled computation
        const startLeft = state.distLeft;
        const startRight = state.distRight;
        const startAvg = state.distAvg;

        let travelled = 0;
        let lastLogDistance = 0;
        setDriveVector(baseTurn, forwardThrottle, false);

        while (!state.autoAbort) {{
          await sleep(120);
          await pollSensorsOnce();
          if (state.autoAbort) break;

          if (state.distAvg != null && startAvg != null) travelled = Math.abs(state.distAvg - startAvg);
          else if (state.distLeft != null && startLeft != null && state.distRight != null && startRight != null)
            travelled = (Math.abs(state.distLeft - startLeft) + Math.abs(state.distRight - startRight)) / 2;
          else if (state.distLeft != null && startLeft != null) travelled = Math.abs(state.distLeft - startLeft);
          else if (state.distRight != null && startRight != null) travelled = Math.abs(state.distRight - startRight);

          if (travelled != null && Number.isFinite(travelled)) {{
            const progress = Math.min(travelled / circumference, 1);

            if (state.yaw != null) {{
              const yawDelta = direction === 'left' ? -progress * 360 : progress * 360;
              const desiredYaw = normalizeHeading(startYaw + yawDelta);
              const yawErr = normalizeAngleDiff(desiredYaw - state.yaw);
              const correction = clamp(yawErr / 45, -0.3, 0.3);
              const turnCmd = clamp(baseTurn + correction, -0.8, 0.8);
              setDriveVector(turnCmd, forwardThrottle, false);
            }} else {{
              setDriveVector(baseTurn, forwardThrottle, false);
            }}

            if (travelled - lastLogDistance >= 0.1) {{
              log(`${{direction === 'left' ? 'Left' : 'Right'}} circle progress: ${{(travelled * 1000).toFixed(0)}}/${{(circumference * 1000).toFixed(0)}}mm`);
              lastLogDistance = travelled;
            }}

            if (travelled >= circumference) break;
          }}
        }}

        stopDrive();
        return {{ success: !state.autoAbort, travelled }};
      }}

      async function runCircle20cm() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{ setStatus('Connect first'); return; }}
        if (state.autoActive) return;
        state.autoActive = true;
        state.autoAbort = false;
        circleBtn.textContent = 'Abort Circle';
        squareBtn.disabled = true;
        straightBtn.disabled = true;
        figureEightBtn.disabled = true;
        turnLeftBtn.disabled = true;
        turnRightBtn.disabled = true;
        held.clear();
        stopDrive();
        setStatus('Driving circle (r=20cm)');

        log('═══════════════════════════════════');
        logAction('Starting circle drive (radius 20cm)');

        try {{
          await pollSensorsOnce();
          await sleep(200);

          let heading = state.yaw;
          if (heading == null) {{
            logWarn('No IMU heading available, using 0° as reference for circle');
            heading = 0;
          }} else {{
            log(`Circle starting heading: ${{heading.toFixed(1)}}°`);
          }}
          const startYaw = normalizeHeading(heading);

          const result = await driveCircle('left', 0.20, startYaw);

          await sleep(150);

          if (state.autoAbort) {{
            logWarn('Circle drive aborted by user');
            setStatus('Circle aborted');
          }} else if (result.success) {{
            logSuccess('Circle drive complete!');
            setStatus('Circle complete (r=20cm)');
          }} else {{
            logWarn('Circle drive incomplete');
            setStatus('Circle incomplete');
          }}
          log('═══════════════════════════════════');
        }} catch (err) {{
          console.error('Circle drive failed', err);
          logError(`Circle drive error: ${{err.message}}`);
          setStatus('Circle error');
        }} finally {{
          state.autoActive = false;
          state.autoAbort = false;
          circleBtn.textContent = 'Circle r20cm';
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            squareBtn.disabled = false;
            straightBtn.disabled = false;
            figureEightBtn.disabled = false;
            turnLeftBtn.disabled = false;
            turnRightBtn.disabled = false;
            circleBtn.disabled = false;
          }} else {{
            squareBtn.disabled = true;
            straightBtn.disabled = true;
            figureEightBtn.disabled = true;
            turnLeftBtn.disabled = true;
            turnRightBtn.disabled = true;
            circleBtn.disabled = true;
          }}
          stopDrive();
        }}
      }}

      async function runFigureEight() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{ setStatus('Connect first'); return; }}
        if (state.autoActive) return;
        state.autoActive = true;
        state.autoAbort = false;
        figureEightBtn.textContent = 'Abort Figure-8';
        squareBtn.disabled = true;
        straightBtn.disabled = true;
        circleBtn.disabled = true;
        turnLeftBtn.disabled = true;
        turnRightBtn.disabled = true;
        held.clear();
        stopDrive();
        setStatus('Driving figure-8 (r=20cm)');

        log('═══════════════════════════════════');
        logAction('Starting figure-8 pattern (radius 20cm each loop)');

        try {{
          await pollSensorsOnce();
          await sleep(200);

          let heading = state.yaw;
          if (heading == null) {{
            logWarn('No IMU heading available, using 0° as reference for figure-8');
            heading = 0;
          }} else {{
            log(`Figure-8 starting heading: ${{heading.toFixed(1)}}°`);
          }}
          const startYaw = normalizeHeading(heading);

          // First loop: left circle
          log('─────── Loop 1/2: Left Circle ───────');
          const leftResult = await driveCircle('left', 0.20, startYaw);
          if (state.autoAbort || !leftResult.success) {{
            throw new Error('Left circle incomplete');
          }}

          await sleep(300);
          if (state.autoAbort) throw new Error('Aborted between loops');

          // Update heading after first circle
          await pollSensorsOnce();
          let midHeading = state.yaw ?? startYaw;
          log(`Completed left circle, current heading: ${{midHeading.toFixed(1)}}°`);

          // Second loop: right circle
          log('─────── Loop 2/2: Right Circle ───────');
          const rightResult = await driveCircle('right', 0.20, normalizeHeading(midHeading));
          if (state.autoAbort || !rightResult.success) {{
            throw new Error('Right circle incomplete');
          }}

          await sleep(150);

          if (state.autoAbort) {{
            logWarn('Figure-8 drive aborted by user');
            setStatus('Figure-8 aborted');
          }} else {{
            logSuccess('Figure-8 pattern complete!');
            setStatus('Figure-8 complete (r=20cm)');
          }}
          log('═══════════════════════════════════');
        }} catch (err) {{
          console.error('Figure-8 drive failed', err);
          logError(`Figure-8 drive error: ${{err.message}}`);
          setStatus('Figure-8 error');
        }} finally {{
          state.autoActive = false;
          state.autoAbort = false;
          figureEightBtn.textContent = 'Figure-8 r20cm';
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            squareBtn.disabled = false;
            straightBtn.disabled = false;
            circleBtn.disabled = false;
            turnLeftBtn.disabled = false;
            turnRightBtn.disabled = false;
            figureEightBtn.disabled = false;
          }} else {{
            squareBtn.disabled = true;
            straightBtn.disabled = true;
            circleBtn.disabled = true;
            turnLeftBtn.disabled = true;
            turnRightBtn.disabled = true;
            figureEightBtn.disabled = true;
          }}
          stopDrive();
        }}
      }}

      async function executeNinetyTurn(direction, options = {{}}) {{
        const {{
          nested = false,
          messagePrefix = '',
          updateStatus = !nested,
          logDivider = !nested,
          actionLabel,
          targetHeading: targetOverride,
          emitLogs = true,
        }} = options;
        const prefix = messagePrefix ? `${{messagePrefix}} ` : '';
        const isLeft = direction === 'left';
        const button = isLeft ? turnLeftBtn : turnRightBtn;
        const otherButton = isLeft ? turnRightBtn : turnLeftBtn;
        const delta = isLeft ? -90 : 90;
        const dirLabel = isLeft ? 'left' : 'right';
        const dirTitle = isLeft ? 'Left' : 'Right';
        const defaultStatus = `Turning ${{dirLabel}} 90°`;

        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{
          if (updateStatus) setStatus('Connect first');
          return {{ success: false, reason: 'no-connection' }};
        }}
        if (state.autoActive && !nested) return {{ success: false, reason: 'busy' }};
        if (state.autoAbort) return {{ success: false, reason: 'auto-abort' }};

        if (!nested) {{
          state.autoActive = true;
          state.autoAbort = false;
          button.textContent = 'Abort Turn';
          squareBtn.disabled = true;
          straightBtn.disabled = true;
          circleBtn.disabled = true;
          figureEightBtn.disabled = true;
          otherButton.disabled = true;
          held.clear();
          stopDrive();
          if (updateStatus) setStatus(defaultStatus);
        }} else {{
          held.clear();
          stopDrive();
        }}

        if (logDivider) log('═══════════════════════════════════');
        if (actionLabel != null) logAction(actionLabel);
        else if (emitLogs) logAction(`${{prefix}}Starting 90° ${{dirLabel}} turn`);

        let ok = false;
        let currentHeading = null;
        let targetHeading = null;
        let finalYaw = null;
        let actualFinalError = null;
        let errorObj = null;

        try {{
          await pollSensorsOnce();
          await sleep(100);

          const base = currentApiBase();
          currentHeading = state.yaw;

          if (currentHeading == null) {{
            if (emitLogs) logWarn(`${{prefix}}No IMU heading available, using 0° as reference`);
            currentHeading = 0;
          }} else if (emitLogs) {{
            log(`${{prefix}}Current heading: ${{currentHeading.toFixed(1)}}°`);
          }}

          targetHeading = targetOverride != null
            ? normalizeHeading(targetOverride)
            : normalizeHeading(currentHeading + delta);
          if (emitLogs) {{
            log(`${{prefix}}Target heading: ${{targetHeading.toFixed(1)}}°`);
          }}

          ok = await turnToHeading(targetHeading, base);

          if (state.yaw != null) {{
            finalYaw = normalizeHeading(state.yaw);
            const actualRotation = Math.abs(normalizeAngleDiff(finalYaw - currentHeading));
            const desiredRotation = Math.abs(normalizeAngleDiff(targetHeading - currentHeading));
            actualFinalError = Math.abs(desiredRotation - actualRotation);
          }}

          if (state.autoAbort) {{
            if (emitLogs) logError(`${{prefix}}${{dirTitle}} turn aborted by user`);
            if (updateStatus) setStatus('Turn aborted');
          }} else if (!ok) {{
            if (emitLogs) logError(`${{prefix}}${{dirTitle}} turn failed (IMU or timeout issue)`);
            if (updateStatus) setStatus('Turn failed');
          }} else if (actualFinalError != null && actualFinalError <= 5) {{
            if (emitLogs) logSuccess(`${{prefix}}${{dirTitle}} turn 90° complete! (final error: ${{actualFinalError.toFixed(1)}}°)`);
            if (updateStatus) setStatus('Turn complete');
          }} else if (actualFinalError != null) {{
            if (emitLogs) logWarn(`${{prefix}}${{dirTitle}} turn ended with large error: ${{actualFinalError.toFixed(1)}}°`);
            if (updateStatus) setStatus(`Turn incomplete (error: ${{actualFinalError.toFixed(1)}}°)`);
          }} else {{
            if (emitLogs) logError(`${{prefix}}${{dirTitle}} turn ended but IMU unavailable`);
            if (updateStatus) setStatus('Turn failed - no IMU');
          }}
        }} catch (err) {{
          errorObj = err;
          console.error(`${{dirLabel}} turn failed`, err);
          if (emitLogs) logError(`${{prefix}}${{dirLabel}} turn error: ${{err.message}}`);
          if (updateStatus) setStatus('Turn error');
        }} finally {{
          if (!nested) {{
            state.autoActive = false;
            state.autoAbort = false;
            button.textContent = isLeft ? 'Turn Left 90°' : 'Turn Right 90°';
            if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
              squareBtn.disabled = false;
              straightBtn.disabled = false;
              circleBtn.disabled = false;
              figureEightBtn.disabled = false;
              otherButton.disabled = false;
            }} else {{
              squareBtn.disabled = true;
              straightBtn.disabled = true;
              circleBtn.disabled = true;
              figureEightBtn.disabled = true;
              otherButton.disabled = true;
            }}
          }}
          stepNextBtn.disabled = true;
          stepNextBtn.style.display = 'none';
          if (state.stepResolve) {{
            state.stepResolve();
            state.stepResolve = null;
          }}
          stopDrive();
        }}

        return {{
          success: !state.autoAbort && ok && !errorObj,
          ok,
          autoAbort: state.autoAbort,
          actualFinalError,
          finalYaw,
          targetHeading,
          startHeading: currentHeading,
          error: errorObj,
        }};
      }}

      async function runTurnLeft90(options = {{}}) {{
        return executeNinetyTurn('left', options);
      }}

      async function runTurnRight90(options = {{}}) {{
        return executeNinetyTurn('right', options);
      }}

      squareBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Square abort requested'); return; }}
        runSquare();
      }});

      straightBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Straight abort requested'); return; }}
        runStraight();
      }});

      circleBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Circle abort requested'); return; }}
        runCircle20cm();
      }});

      figureEightBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Figure-8 abort requested'); return; }}
        runFigureEight();
      }});

      turnLeftBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Turn abort requested'); return; }}
        runTurnLeft90();
      }});

      turnRightBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Turn abort requested'); return; }}
        runTurnRight90();
      }});

      zeroBtn.addEventListener('click', async () => {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
        if (state.autoActive) {{
          setStatus('Cannot zero during automated move');
          logWarn('Cannot zero telemetry during automated move');
          return;
        }}

        logAction('Zeroing telemetry...');

        // Poll sensors to get current values
        await pollSensorsOnce();

        // Set current values as zero offsets
        state.zeroDistLeft = state.distLeft ?? 0;
        state.zeroDistRight = state.distRight ?? 0;
        state.zeroDistAvg = state.distAvg ?? 0;
        state.zeroYaw = state.yaw ?? 0;

        // Update display immediately
        updateTelemetry(state.lastLeft, state.lastRight, state.boost, false);
        setStatus('Telemetry zeroed');

        logSuccess(`Telemetry zeroed at heading ${{state.zeroYaw.toFixed(1)}}°, distance ${{(state.zeroDistAvg * 1000).toFixed(0)}}mm`);

        // Reset status after brief delay
        setTimeout(() => {{
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            setStatus('Connected');
          }}
        }}, 1500);
      }});

      function requestDisconnect() {{
        if (!state.ws) return;
        state.autoAbort = true;
        stopDrive();
        setStatus('Disconnecting...');
        try {{ state.ws.close(); }} catch (err) {{}}
      }}

      connectBtn.addEventListener('click', () => {{
        if (state.ws && state.ws.readyState === WebSocket.OPEN) requestDisconnect();
        else connect();
      }});

      // Power test functionality
      async function runPowerTest() {{
        if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {{ setStatus('Connect first'); return; }}
        if (state.autoActive) return;
        state.autoActive = true;
        state.autoAbort = false;
        testPowerBtn.textContent = 'Abort Test';
        squareBtn.disabled = true;
        straightBtn.disabled = true;
        circleBtn.disabled = true;
        figureEightBtn.disabled = true;
        turnRightBtn.disabled = true;
        zeroBtn.disabled = true;
        held.clear();
        stopDrive();
        setStatus('Power test running');

        log('═══════════════════════════════════');
        logAction('Starting power calibration test');
        log('Testing power levels from 0.5 to 1.0 (10 runs each)');

        const powerLevels = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0];
        const allResults = [];  // Store all runs for each power level

        try {{
          for (let i = 0; i < powerLevels.length && !state.autoAbort; i++) {{
            const power = powerLevels[i];
            log(`─────── Test ${{i + 1}}/6: Power ${{power.toFixed(1)}} ───────`);

            const runs = [];

            // Run 10 times for this power level
            for (let run = 0; run < 10 && !state.autoAbort; run++) {{
              setStatus(`Testing power ${{power.toFixed(1)}} (run ${{run + 1}}/10)`);

              // Measure starting yaw with retry logic
              let startYaw = null;
              for (let attempt = 0; attempt < 3; attempt++) {{
                await pollSensorsOnce();
                startYaw = state.yaw;
                if (startYaw != null) break;
                logWarn(`IMU read attempt ${{attempt + 1}} failed, retrying...`);
                await sleep(100);
              }}
              if (startYaw == null) {{
                logError('IMU unavailable after 3 attempts, skipping this run');
                continue;
              }}
              log(`Start yaw: ${{startYaw.toFixed(1)}}°`);

              // Send turn command continuously for 500ms (to keep dead-man timer alive)
              const startTime = performance.now();
              const testDuration = 500;  // 500ms

              while (performance.now() - startTime < testDuration) {{
                sendDriveCommand(power, -power);
                state.lastInput = performance.now();
                await sleep(50);  // Send command every 50ms to keep dead-man timer happy
              }}

              // Stop motors
              sendDriveCommand(0, 0);
              await sleep(400);  // Increased settling time for robot to fully stop
              sendDriveCommand(0, 0);
              stopDrive();

              // Wait for IMU to settle before reading
              await sleep(200);

              // Measure ending yaw with retry logic
              let endYaw = null;
              for (let attempt = 0; attempt < 3; attempt++) {{
                await pollSensorsOnce();
                endYaw = state.yaw;
                if (endYaw != null) break;
                logWarn(`IMU read attempt ${{attempt + 1}} failed, retrying...`);
                await sleep(100);
              }}
              if (endYaw == null) {{
                logError('IMU unavailable after turn (3 attempts), skipping this run');
                continue;
              }}
              log(`End yaw: ${{endYaw.toFixed(1)}}°`);

              // Calculate degrees changed (ignore sign - just absolute difference)
              const degreesChanged = Math.abs(normalizeAngleDiff(endYaw - startYaw));

              // Ignore measurements lower than 0.5 degrees
              if (degreesChanged >= 0.5) {{
                runs.push(degreesChanged);
                log(`Run ${{run + 1}}: ${{degreesChanged.toFixed(1)}}°`);
              }} else {{
                log(`Run ${{run + 1}}: ${{degreesChanged.toFixed(1)}}° (ignored - too low)`);
              }}

              // Pause before next run
              if (run < 9) {{
                await sleep(1200);
              }}
            }}

            allResults.push({{ power, runs }});

            // Pause before next power level
            if (i < powerLevels.length - 1) {{
              await sleep(1500);
            }}
          }}

          // Display summary
          if (state.autoAbort) {{
            logError('Power test aborted by user');
            setStatus('Test aborted');
          }} else {{
            log('═══════════════════════════════════');
            logSuccess('Power test complete! Averaging results...');
            log('');

            // Calculate averages with outlier removal
            const finalResults = [];
            for (const {{ power, runs }} of allResults) {{
              if (runs.length === 0) {{
                finalResults.push({{ power, avgDegrees: 'N/A', validRuns: 0 }});
                continue;
              }}

              // Remove outliers using IQR method
              const sorted = [...runs].sort((a, b) => a - b);
              const q1 = sorted[Math.floor(sorted.length * 0.25)];
              const q3 = sorted[Math.floor(sorted.length * 0.75)];
              const iqr = q3 - q1;
              const lowerBound = q1 - 1.5 * iqr;
              const upperBound = q3 + 1.5 * iqr;

              const filtered = runs.filter(v => v >= lowerBound && v <= upperBound);
              const avgDegrees = filtered.length > 0
                ? filtered.reduce((sum, v) => sum + v, 0) / filtered.length
                : 'N/A';

              finalResults.push({{ power, avgDegrees, validRuns: filtered.length }});
            }}

            log('Power | Avg Degrees | Deg/sec | Valid Runs');
            log('------|-------------|---------|----------');
            for (const {{ power, avgDegrees, validRuns }} of finalResults) {{
              const padded = power.toFixed(1).padStart(5);
              if (avgDegrees === 'N/A') {{
                log(`${{padded}} |         N/A |     N/A |     ${{validRuns}}/10`);
              }} else {{
                const degStr = avgDegrees.toFixed(1).padStart(11);
                const degPerSec = (avgDegrees / 0.5).toFixed(1).padStart(9);
                log(`${{padded}} | ${{degStr}} | ${{degPerSec}} |     ${{validRuns}}/10`);
              }}
            }}
            log('');

            // Find thresholds
            const withMovement = finalResults.filter(r => r.avgDegrees !== 'N/A' && r.avgDegrees > 0);
            if (withMovement.length > 0) {{
              const firstMovement = withMovement[0];
              logSuccess(`Minimum power for movement: ${{firstMovement.power.toFixed(1)}} (${{firstMovement.avgDegrees.toFixed(1)}}° avg)`);
            }}

            log('═══════════════════════════════════');
            setStatus('Test complete');
          }}
        }} catch (err) {{
          console.error('Power test failed', err);
          logError(`Power test error: ${{err.message}}`);
          setStatus('Test error');
        }} finally {{
          state.autoActive = false;
          state.autoAbort = false;
          testPowerBtn.textContent = 'Test Power';
          powerTestControls.style.display = 'none';
          powerTestDegreesInput.disabled = true;
          powerTestSubmitBtn.disabled = true;
          if (state.ws && state.ws.readyState === WebSocket.OPEN) {{
            squareBtn.disabled = false;
            straightBtn.disabled = false;
            circleBtn.disabled = false;
            figureEightBtn.disabled = false;
            turnRightBtn.disabled = false;
            zeroBtn.disabled = false;
          }}
          stopDrive();
        }}
      }}

      testPowerBtn.addEventListener('click', () => {{
        if (state.autoActive) {{ state.autoAbort = true; setStatus('Test abort requested'); return; }}
        runPowerTest();
      }});

      powerTestSubmitBtn.addEventListener('click', () => {{
        if (state.powerTestResolve) {{
          const degrees = parseInt(powerTestDegreesInput.value, 10);
          if (isNaN(degrees) || degrees < 0 || degrees > 360) {{
            logError('Please enter a valid number between 0 and 360');
            return;
          }}
          state.powerTestResolve(degrees);
          state.powerTestResolve = null;
        }}
      }});

      // Allow Enter key to submit
      powerTestDegreesInput.addEventListener('keypress', (e) => {{
        if (e.key === 'Enter') {{
          powerTestSubmitBtn.click();
        }}
      }});

      updateTelemetry(0, 0, false, false);
      setStatus('Disconnected');

      // Startup log
      log('GordonBot Manual Drive Interface initialized');
      log('Connect to robot to begin');
    }})();
  </script>
</body>
</html>
"""


def _render_page() -> str:
    return PAGE_TEMPLATE.format(
        ws_url=DEFAULT_WS_URL,
        api_base=DEFAULT_API_BASE,
        command_hz=COMMAND_HZ,
        deadman_ms=DEADMAN_MS,
        max_speed=MAX_SPEED,
        boost=BOOST_MULTIPLIER,
        key_accel=KEY_ACCEL_PER_S,
        key_decel=KEY_DECEL_PER_S,
        key_turn_accel=KEY_TURN_ACCEL_PER_S,
        key_turn_decel=KEY_TURN_DECEL_PER_S,
        poll_ms=SENSOR_POLL_MS,
    )


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: D401
        if self.path in {"/", "/index.html"}:
            body = _render_page().encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            # Cache-busting headers - force browser to always reload
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404, "Not Found")

    def log_message(self, format: str, *args) -> None:  # noqa: D401
        return


def _guess_host_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
    except Exception:
        return "localhost"


def browser_drive(port: int = BROWSER_PORT) -> None:
    server = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    host_ip = _guess_host_ip()
    print(
        "Serving browser controls."
        f"\n  Local access:  http://localhost:{port}"
        f"\n  LAN access:    http://{host_ip}:{port}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
    finally:
        server.server_close()


def main() -> None:
    browser_drive()


if __name__ == "__main__":
    main()
