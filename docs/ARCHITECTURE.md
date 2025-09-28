# GordonBot Architecture

This document provides a thorough, high‑level map of the GordonBot project to accelerate onboarding and maintenance.

## Overview

- Backend: FastAPI app serving REST APIs, WebSocket control, and a WHEP proxy for WebRTC playback via MediaMTX. Optional RTSP publishers (raw and annotated) produced by the Pi camera pipeline.
- Frontend: Vite + React TypeScript dashboard for camera view, battery and system diagnostics, and drive control (WebSocket).
- Media: MediaMTX (external) provides RTSP ingest and WHEP signaling endpoints.

## Tech Stack

- Python: FastAPI, Uvicorn, Pydantic v2, OpenCV (headless), Pillow, gpiozero, psutil, dotenv
- Node/JS: React 19 + Vite, Tailwind CSS 4
- Streaming: ffmpeg (invoked from Python) to push RTSP; WebRTC WHEP via MediaMTX

## Directory Structure

```
gordonbot-backend/
  app/
    core/                 # Config and settings
      config.py           # Pydantic model reading env vars
    routers/              # REST endpoints
      ping.py             # GET /api/ping → "pong"
      battery.py          # GET /api/battery/status → BatteryData
      diagnostics.py      # GET /api/diag/system → system metrics
      video.py            # GET /api/video/snapshot.jpg; POST/OPTIONS /api/video/whep/{stream}
    services/             # Internal services
      camera.py           # Camera capture, encoding, detection, RTSP publishers
      battery.py          # Battery stat reader
      diagnostics.py      # System metrics
      tof.py              # VL53L1X time-of-flight distance sensor
    sockets/              # WebSockets
      control.py          # /ws/control — tank drive commands
    main.py               # FastAPI app composition & startup hooks
  requirements.txt
  .env                    # Backend runtime configuration (gitignored)

gordonbot-dashboard/
  src/components/
    CameraPanel.tsx       # MJPEG snapshot & WebRTC (WHEP) player
    ControlPanel.tsx      # Drive joystick + WASD → WebSocket
    DiagnosticsPanel.tsx  # System metrics display
    TopBar.tsx, ui/*, hooks/*, types.ts, config.ts
  package.json, vite.config.ts, tsconfig*.json
  .env                    # Frontend runtime config (gitignored)

scripts/
  dev.sh                  # TUI supervisor to run backend + frontend, optional MediaMTX
  bootstrap.sh            # Create venv, install deps for backend & dashboard

codex.yaml                # Dev tasks (setup, dev, backend/frontend helpers)
docs/ARCHITECTURE.md      # This file
.codex/scan.json          # Machine‑readable index (generated)
```

## Runtime: Ports and Processes

- Backend (Uvicorn): `:8000`
- Frontend (Vite dev): `:5173`
- MediaMTX HTTP (WHEP): `:8889` (default)
- MediaMTX RTSP: `:8554` (default)

`scripts/dev.sh` supervises backend + frontend with a simple terminal UI. It can optionally start MediaMTX if present; otherwise it just connects to whatever you already run.

## Backend Composition

Entrypoint: `gordonbot-backend/app/main.py`

- Creates the FastAPI root app and a mounted sub‑app `api` under `settings.api_prefix` (default `/api`).
- Adds CORS with either `CORS_ALLOW_ORIGIN_REGEX` or explicit list `CORS_ORIGINS`.
- Includes routers:
  - `ping` → healthcheck
  - `battery` → latest battery telemetry
  - `diagnostics` → system metrics
  - `video` → snapshot and WHEP proxy
- Includes WebSocket router `control` at `settings.control_ws_path` (default `/ws/control`) on the root app (not under `/api`).
- Startup hook optionally starts RTSP publishers to MediaMTX (raw and annotated) if corresponding env vars are set.

### REST API Endpoints (mounted under `/api`)

- `GET /api/ping` → plain `"pong"`.
- `GET /api/battery/status` → `BatteryData`:
  - Fields: `percent, charging, vbusVoltage, vbusCurrent, vbusPower, batteryVoltage, batteryCurrent, remainingCapacity, avgTimeToFullMin, cells`.
- `GET /api/diag/system` → JSON with CPU, memory, temps, uptime, throttling, etc.
- `GET /api/video/snapshot.jpg` → current JPEG frame from the camera.
- `POST /api/video/whep/{stream}` → Proxies `application/sdp` offer to MediaMTX, returns `application/sdp` answer. CORS preflight supported via `OPTIONS`.
- `GET /api/sensors/status` → Consolidated sensor snapshot (encoders, ToF, BNO055). Optional fields if hardware absent. When a VL53L1X is present the backend streams live distance readings via the ToF service.

### WebSocket: `/ws/control`

- Purpose: send normalized tank drive commands at client cadence and receive acks.
- Messages:
  - Heartbeat: `{ "type": "ping", "ts": <number> }` → `{ "type": "pong", "ts": <same> }`
  - Drive command: `{ "type": "drive", "payload": { "left": -1..1, "right": -1..1, "ts": <number> } }`
    - Server clamps to [-1, 1], applies a cubic response curve for finer low‑end control, actuates motors (gpiozero if available; dev mode just logs), and replies:
      `{ "type": "ack", "ts": <same>, "echo": { "left": <clamped>, "right": <clamped> } }`
  - Errors: `{ "type": "error", "message": "..." }`
- Safety: dead‑man watchdog (~0.4s). If no commands arrive, motors are stopped.

### Camera and Streaming

Defined in `gordonbot-backend/app/services/camera.py`.

- Capture: Provides `capture_jpeg()` and frame sources.
- RTSP Publishers: Two independent publishers push H.264 to MediaMTX via `ffmpeg`:
  - Raw stream → `CAMERA_RTSP_URL` (e.g., `rtsp://127.0.0.1:8554/gordon`).
  - Annotated stream → `CAMERA_RTSP_ANNOT_URL` (e.g., `rtsp://127.0.0.1:8554/gordon-annot`).
- Annotation/Detection (optional):
  - Motion detection fallback, or ONNX object detection (e.g., YOLOv5/8) using OpenCV DNN.
  - Configurable via env: `DETECT_ENABLED`, `DETECT_ONNX_PATH`, `DETECT_LABELS`, `DETECT_CONF_THRESHOLD`, `DETECT_NMS_THRESHOLD`, `DETECT_INPUT_SIZE`, `DETECT_INTERVAL`, `ANNOT_FPS`, `ANNOT_MIN_AREA`.

### Configuration and Env Vars (backend)

Defined in `gordonbot-backend/app/core/config.py` and `gordonbot-backend/.env`.

- API + CORS
  - `api_prefix` → `/api` (constant in code; override requires code change)
  - `CORS_ORIGINS` → comma/space list (default includes `http://localhost:5173`)
  - `CORS_ALLOW_ORIGIN_REGEX` → optional regex (used instead of the list)
- Control WebSocket
  - `control_ws_path` → `/ws/control` (constant in code; change via code)
- Media (RTSP/WHEP)
  - `CAMERA_RTSP_URL` → raw RTSP target
  - `CAMERA_RTSP_ANNOT_URL` → annotated RTSP target
  - `CAMERA_BITRATE` → H.264 bitrate (bps)
  - `MEDIAMTX_WHEP_BASE` → base HTTP endpoint (e.g., `http://127.0.0.1:8889/whep`)
  - `MEDIAMTX_WHEP_STYLE` → `suffix` (default) or `prefix`; influences target URL formed for MediaMTX
- Detection
  - `DETECT_ENABLED`, `DETECT_ONNX_PATH`, `DETECT_LABELS`, `DETECT_CONF_THRESHOLD`, `DETECT_NMS_THRESHOLD`, `DETECT_INPUT_SIZE`, `DETECT_INTERVAL`
- Speech capture / ASR
  - `SPEECH_BACKEND` (default `auto`: prefer `whisper-api`, fall back to `whispercpp`, then `faster-whisper`)
  - `SPEECH_MODEL`, `SPEECH_DEVICE`, `SPEECH_COMPUTE_TYPE`
  - `SPEECH_SAVE_RECORDINGS` + `SPEECH_RECORDING_DIR`
  - Whisper API (`SPEECH_BACKEND=auto` or `whisper-api`): `SPEECH_API_KEY`, `SPEECH_API_MODEL`, `SPEECH_API_BASE`, `SPEECH_API_TIMEOUT`, `SPEECH_API_ORG`
- Speech synthesis
  - `TTS_BACKEND` (default `auto`: prefer OpenAI `gpt-4o-mini-tts`, fall back to `espeak-ng`)
  - OpenAI settings: `TTS_API_BASE`, `TTS_API_TIMEOUT`, `TTS_OPENAI_MODEL`, `TTS_OPENAI_VOICE` (uses `SPEECH_API_KEY`)
  - eSpeak NG settings: `ESPEAK_NG_BIN`, `ESPEAK_VOICE`, `ESPEAK_RATE`, `ESPEAK_PITCH`, `ESPEAK_AMPLITUDE`
  - SoX filtering/output: `TTS_USE_FILTER`, `SOX_BIN`, `TTS_SOX_DESTINATION`, `TTS_SOX_EFFECTS`
- Voice responses (LLM fallback)
  - `VOICE_REPLY_MODEL`, `VOICE_REPLY_SYSTEM_PROMPT`
- Annotation
  - `ANNOT_FPS`, `ANNOT_MIN_AREA`
- Verbose logging
  - `VERBOSE=true|false`

## Frontend Composition

- Vite + React TS app under `gordonbot-dashboard/`.
- Env (`gordonbot-dashboard/.env`): `VITE_API_BASE` → backend base URL, e.g., `http://127.0.0.1:8000`.
- Key modules:
  - `components/config.ts`: constants including `VIDEO_WHEP_BASE = "/api/video/whep/"`, streams `gordon` and `gordon-annot`.
- `CameraPanel.tsx`: snapshot endpoint and WHEP WebRTC playback via backend proxy.
- `SensorsPanel.tsx`: polls `/api/sensors/status` and displays encoders, ToF, and BNO055 telemetry. The speed gauge now shows both average wheel speed and the closest obstacle distance reported by the VL53L1X.
  - `ControlPanel.tsx`: joystick + keyboard → WebSocket `/ws/control` client.
  - `DiagnosticsPanel.tsx`: queries `GET /api/diag/system` and displays metrics.

### Encoders

- Hardware: Pololu micro metal gearmotor with integrated 2‑channel Hall encoder (12 CPR on motor shaft) with ~100:1 gearbox → ~1204 counts per output shaft revolution.
- Wiring (first encoder): Channel A → GPIO 17, Channel B → GPIO 27.
- Backend: `services/encoder.py` uses `gpiozero.RotaryEncoder` if available; exposed via `GET /api/sensors/status` in `encoders.left` with fields `ticks`, `distance_m`, and `rpm`.
- Calibration: `ENCODER_COUNTS_PER_REV_OUTPUT` (default `1204`) and `WHEEL_DIAMETER_M` (default `0.03`) in backend `.env`.

## Dev, Build, and Run

Bootstrap once:

```bash
bash scripts/bootstrap.sh
```

Run both (TUI supervisor):

```bash
bash scripts/dev.sh
```

Individual services:

```bash
# Backend
cd gordonbot-backend
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

# Frontend
cd gordonbot-dashboard
npm run dev:host
```

## Security & Privacy Notes

- No credentials are required by default. If your MediaMTX/RTSP/WHEP endpoints require auth, embed user:pass in RTSP URLs or configure MediaMTX auth and extend the proxy headers accordingly.
- CORS is permissive by default for common local dev hosts; for production, lock down `CORS_ALLOW_ORIGIN_REGEX` or provide an explicit origins list.

## Operational Notes

- MediaMTX URL style:
  - `MEDIAMTX_WHEP_STYLE=suffix` yields `/api/video/whep/{stream}` → MediaMTX `/{stream}/whep`.
  - `prefix` yields `/whep/{stream}`.
- If ffmpeg is missing, RTSP publishers won’t start; the app continues to serve APIs.
- On dev machines without GPIO, the motor controller runs in log‑only mode.

## Quick Reference

- REST base: `http://127.0.0.1:8000/api`
- WebSocket: `ws://127.0.0.1:8000/ws/control`
- Snapshot: `GET /api/video/snapshot.jpg`
- WHEP proxy: `POST /api/video/whep/{gordon|gordon-annot}` (Content‑Type: `application/sdp`)
