# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GordonBot is a Raspberry Pi-based robot with:
- **Backend**: FastAPI (Python) serving REST APIs, WebSocket control, and WHEP proxy for WebRTC streaming
- **Frontend**: Vite + React TypeScript dashboard for camera, navigation, diagnostics, and drive control
- **Media**: MediaMTX (external) for RTSP ingest and WHEP signaling

## Common Commands

### Setup and Development
```bash
# Run Gordon monitor (supervises backend + frontend; optional MediaMTX)
go run ./gordonmon

# Backend only (port 8000)
cd gordonbot-backend
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

# Frontend only (port 5173)
cd gordonbot-dashboard
npm run dev:host
```

### Frontend Tasks
```bash
cd gordonbot-dashboard

npm run dev          # Dev server on localhost only
npm run dev:host     # Dev server accessible on network
npm run build        # Production build
npm run typecheck    # TypeScript check without emitting
npm run lint         # ESLint
npm run lint:css     # Stylelint for CSS
npm run check        # Run typecheck + lint + lint:css
npm run format       # Prettier format
```

### Backend Tasks
```bash
cd gordonbot-backend

# Install/update dependencies
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

# Run with auto-reload
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

## Architecture

### Backend Structure (`gordonbot-backend/app/`)

- **`main.py`**: FastAPI app entrypoint; mounts routers and WebSockets; startup hook for RTSP publishers
- **`core/config.py`**: Pydantic settings model loading from `.env`
- **`routers/`**: REST endpoints under `/api`
  - `ping.py`: Health check
  - `battery.py`: Battery telemetry
  - `diagnostics.py`: System metrics (CPU, memory, temps)
  - `video.py`: Snapshot JPEG + WHEP proxy
  - `sensors.py`: Consolidated sensor status (encoders, ToF, BNO055)
- **`services/`**: Internal services
  - `camera.py`: Camera capture, H.264 encoding, RTSP publishing, optional ONNX detection
  - `battery.py`: Battery stat reader
  - `encoder.py`: Quadrature encoder (lgpio/RPi.GPIO)
  - `tof.py`: VL53L1X time-of-flight distance sensor
  - `bno055.py`: BNO055 IMU orientation quaternions
  - `drive_state.py`: Motor control coordination
  - `transcription.py`: Whisper ASR (API, whispercpp, faster-whisper)
  - `text_to_speech.py`: TTS (OpenAI or espeak-ng)
  - `voice_interaction.py`: LLM fallback for voice replies
- **`sockets/`**: WebSocket endpoints on root app
  - `control.py`: `/ws/control` — Tank drive commands with dead-man watchdog
  - `orientation.py`: `/ws/orientation` — BNO055 orientation stream + calibration control

### Frontend Structure (`gordonbot-dashboard/src/`)

- **`components/`**:
  - `CameraPanel.tsx`: MJPEG snapshot and WebRTC WHEP player
  - `ControlPanel.tsx`: Drive joystick + WASD keyboard → WebSocket
  - `NavigationPanel.tsx`: Speed, distance, compass + collapsible encoder/BNO055 sections
  - `OrientationPanel.tsx`: Three.js cube driven by orientation WebSocket; calibration UI
  - `DiagnosticsPanel.tsx`: System metrics display
  - `TopBar.tsx`, `ui/`, `hooks/`, `types.ts`, `config.ts`

### Configuration

Backend config is in [gordonbot-backend/.env](gordonbot-backend/.env) (gitignored). Key settings:

- **API/CORS**: `CORS_ORIGINS` (comma-separated), `CORS_ALLOW_ORIGIN_REGEX`
- **Media**: `CAMERA_RTSP_URL`, `CAMERA_RTSP_ANNOT_URL`, `CAMERA_BITRATE`, `MEDIAMTX_WHEP_BASE`, `MEDIAMTX_WHEP_STYLE`
- **Detection**: `DETECT_ENABLED`, `DETECT_ONNX_PATH`, `DETECT_LABELS`, `DETECT_CONF_THRESHOLD`, `DETECT_NMS_THRESHOLD`, `DETECT_INPUT_SIZE`, `DETECT_INTERVAL`
- **Encoders**: `ENCODER_COUNTS_PER_REV_OUTPUT`, `WHEEL_DIAMETER_M`, encoder GPIO pins
- **Speech/TTS**: `SPEECH_BACKEND`, `SPEECH_API_KEY`, `TTS_BACKEND`, `TTS_OPENAI_MODEL`
- **Logging**: `VERBOSE=true|false`

Frontend config is in [gordonbot-dashboard/.env](gordonbot-dashboard/.env): `VITE_API_BASE` (e.g., `http://127.0.0.1:8000`)

## Key Patterns

### WebSocket: `/ws/control`
- **Drive commands**: `{ "type": "drive", "payload": { "left": -1..1, "right": -1..1, "ts": <ms> } }`
- **Ack**: `{ "type": "ack", "ts": <same>, "echo": { "left": <clamped>, "right": <clamped> } }`
- **Heartbeat**: `{ "type": "ping", "ts": <ms> }` → `{ "type": "pong", "ts": <same> }`
- **Dead-man watchdog**: ~0.4s timeout stops motors if no commands received

### WebSocket: `/ws/orientation`
- **Outgoing**: `{ "type": "orientation", "ts": <ms>, "qw": <float>, "qx": <float>, "qy": <float>, "qz": <float>, "euler": { roll, pitch, yaw }, "calib": { sys, gyro, accel, mag }, "stale": <bool> }`
- **Incoming**: `{ "action": "start_calibration" }` — launches guided figure-eight IMU calibration
- **Ack**: `{ "type": "calibration", "ok": <bool> }`
- **Progress**: `{ "type": "calibration_run", "status": "started" }`, `{ "type": "calibration_complete", "ok": <bool> }`

### RTSP + WHEP Streaming
- Backend pushes H.264 to MediaMTX via `ffmpeg` at startup (raw and optional annotated streams)
- Frontend fetches WebRTC via `POST /api/video/whep/{stream}` (SDP offer → SDP answer)
- Configurable via `MEDIAMTX_WHEP_STYLE`: `suffix` (default: `/{stream}/whep`) or `prefix` (`/whep/{stream}`)

### Encoders and Odometry
- Hardware: Pololu micro metal gearmotor with 12 CPR Hall encoder + ~100:1 gearbox → ~1204 counts/rev
- Backend: Quadrature decoder in `services/encoder.py` (lgpio preferred, RPi.GPIO fallback)
- Right encoder: GPIO 12 (A), GPIO 26 (B); Left encoder: GPIO 22 (A), GPIO 27 (B)
- Calibration: `ENCODER_COUNTS_PER_REV_OUTPUT`, `WHEEL_DIAMETER_M`, optional per-direction scale factors

### Detection (Optional)
- `services/camera.py` supports OpenCV DNN with ONNX models (YOLOv5/8) or Hailo AI accelerator
- Annotations drawn on secondary stream; configurable interval, confidence/NMS thresholds
- Fallback to motion detection if no ONNX model specified

## Hardware Notes

- **Raspberry Pi**: Likely Pi 4/5 (lgpio for GPIO access)
- **Camera**: Picamera2 (libcamera backend)
- **IMU**: Adafruit BNO055 (I²C)
- **Distance sensor**: VL53L1X (I²C time-of-flight)
- **Motors**: gpiozero-controlled; dead-man safety watchdog
- **Battery**: Custom reader in `services/battery.py`

## Development Notes

- **No tests**: No test suite is present; test manually via dashboard and API endpoints
- **Logging**: Set `VERBOSE=true` for detailed logs; default is concise
- **GPIO dev mode**: On non-Pi systems, motor control logs commands without actuating hardware
- **MediaMTX**: Start separately or let Gordonmon (`go run ./gordonmon`) handle it
- **Ports**: Backend `:8000`, Frontend `:5173`, MediaMTX HTTP `:8889`, MediaMTX RTSP `:8554`

## Reference

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for comprehensive details.

Also see [sandbox_code/FEATURES.md](sandbox_code/FEATURES.md) for full list of available features.
