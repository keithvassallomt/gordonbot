# GordonBot Features Reference

**Project Structure:**
- **gordonbot-backend**: FastAPI Python backend (REST APIs, WebSockets, hardware control)
- **gordonbot-dashboard**: Vite + React TypeScript frontend (web dashboard)
- **gordonmon**: Go-based TUI supervisor (Bubble Tea) for managing all services
- **sandbox_code**: Testing ground for experimental code before production implementation

---

## Table of Contents

1. [Motor Control & Drive System](#1-motor-control--drive-system)
2. [Camera & Video Streaming](#2-camera--video-streaming)
3. [Object Detection](#3-object-detection)
4. [Navigation & Sensors](#4-navigation--sensors)
5. [IMU & Orientation](#5-imu--orientation)
6. [Voice Interaction System](#6-voice-interaction-system)
7. [Battery Monitoring](#7-battery-monitoring)
8. [System Diagnostics](#8-system-diagnostics)
9. [Development Tools](#9-development-tools)

---

## 1. Motor Control & Drive System

### Overview
Tank-drive robot with two independently controlled DC motors, dead-man safety watchdog, and cubic response curve for fine control.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/sockets/control.py` - WebSocket endpoint and motor control
  - `gordonbot-backend/app/services/drive_state.py` - Shared drive state coordination
- **Frontend:**
  - `gordonbot-dashboard/src/components/ControlPanel.tsx` - Joystick UI
  - `gordonbot-dashboard/src/components/JoystickPad.tsx` - Touch/mouse joystick widget

### How It Works
1. **Frontend** sends drive commands via WebSocket `/ws/control`:
   ```json
   {"type": "drive", "payload": {"left": -1.0, "right": 1.0, "ts": 1234567890}}
   ```
2. **Backend** receives commands, clamps values to [-1, 1], applies cubic response curve for finer low-speed control
3. **Hardware control** via gpiozero (Motor objects on GPIO pins: Left=5/6, Right=19/13)
4. **Dead-man watchdog**: ~0.4s timeout; motors stop if no commands received
5. **Dev mode**: If GPIO unavailable, logs commands without actuating hardware

### Key Features
- Normalized inputs: -1.0 (full reverse) to +1.0 (full forward)
- Cubic response curve: `y = x³` for smoother low-speed control
- WebSocket heartbeat: `{"type": "ping"}` → `{"type": "pong"}`
- Acknowledgments: Backend echoes clamped values in `{"type": "ack", "echo": {...}}`
- Keyboard control: WASD keys mapped to drive commands in frontend

### GPIO Pins
- Left motor: Forward=5, Backward=6 (PWM)
- Right motor: Forward=19, Backward=13 (PWM)

---

## 2. Camera & Video Streaming

### Overview
Dual-stream camera system: raw H.264 and optional annotated stream with object detection overlays, streamed via RTSP to MediaMTX and played back in browser via WebRTC WHEP.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/camera.py` - Picamera2 capture, JPEG encoding, RTSP publishing
  - `gordonbot-backend/app/routers/video.py` - Snapshot endpoint + WHEP proxy
- **Frontend:**
  - `gordonbot-dashboard/src/components/CameraPanel.tsx` - MJPEG snapshot viewer + WebRTC player
- **External:**
  - MediaMTX (separate process) - RTSP server and WHEP signaling

### How It Works
1. **Picamera2** captures frames at configured resolution (default 1920x1080 main + 480x270 lores)
2. **Raw stream**: H.264 encoding via `ffmpeg` → RTSP push to `CAMERA_RTSP_URL` (e.g., `rtsp://127.0.0.1:8554/gordon`)
3. **Annotated stream** (optional): OpenCV overlays from detection → H.264 → `CAMERA_RTSP_ANNOT_URL` (e.g., `rtsp://127.0.0.1:8554/gordon-annot`)
4. **MediaMTX** ingests RTSP and provides WHEP endpoints
5. **Frontend** requests WebRTC via `POST /api/video/whep/{stream}` (backend proxies SDP offer/answer to MediaMTX)
6. **Snapshot**: `GET /api/video/snapshot.jpg` returns latest JPEG frame

### Key Features
- Dual-resolution capture: main (high-res for viewing) + lores (low-res for detection)
- RTSP publishing via ffmpeg subprocess
- WebRTC playback in browser (no plugins required)
- Configurable bitrate, FPS, resolution
- Lazy camera initialization (opens on first use)
- Thread-safe frame buffering

### Configuration
- `CAMERA_RTSP_URL` - Raw stream target
- `CAMERA_RTSP_ANNOT_URL` - Annotated stream target (optional)
- `CAMERA_BITRATE` - H.264 bitrate (default 2Mbps)
- `MEDIAMTX_WHEP_BASE` - WHEP signaling endpoint (default `http://127.0.0.1:8889/whep`)
- `MEDIAMTX_WHEP_STYLE` - URL style: `suffix` (default) or `prefix`

---

## 3. Object Detection

### Overview
Pluggable detection backends (CPU OpenCV DNN, Hailo AI accelerator) for annotating camera streams with bounding boxes and labels. Supports ONNX models (YOLOv5/8) or Hailo HEF files.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/detectors/__init__.py` - Detector factory
  - `gordonbot-backend/app/services/detectors/base.py` - Abstract detector interface
  - `gordonbot-backend/app/services/detectors/cpu.py` - OpenCV DNN backend (ONNX)
  - `gordonbot-backend/app/services/detectors/hailo.py` - Hailo accelerator backend
  - `gordonbot-backend/app/services/detectors/postprocess.py` - NMS and box filtering
  - `gordonbot-backend/app/services/camera.py` - Annotation rendering loop

### How It Works
1. **Detection backend** selected at startup based on config (`DETECT_BACKEND`: `cpu` or `hailo`)
2. **Input frames** from lores stream downscaled/preprocessed to model input size (e.g., 640x640)
3. **Inference** runs every N frames (`DETECT_INTERVAL`) to reduce CPU load
4. **Post-processing**: NMS (non-max suppression), confidence thresholding, class filtering
5. **Annotation**: Bounding boxes + labels drawn on frames in annotated stream
6. **Fallback**: Motion detection if no ONNX/Hailo model configured

### Key Features
- **CPU backend**: Uses OpenCV DNN module with ONNX models
- **Hailo backend**: Uses Hailo AI accelerator for faster inference (requires HEF model + pyhailort)
- Configurable confidence/NMS thresholds
- COCO80 labels built-in or custom labels from file
- Detection interval skipping for performance
- Thread-safe detector creation and usage

### Configuration
- `DETECT_ENABLED` - Enable/disable detection
- `DETECT_BACKEND` - `cpu` (OpenCV DNN) or `hailo` (Hailo AI)
- `DETECT_ONNX_PATH` - Path to ONNX model (for CPU backend)
- `DETECT_HAILO_HEF_PATH` - Path to Hailo HEF file
- `DETECT_HAILO_POSTPROCESS` - Custom post-processing function (`module:function`)
- `DETECT_LABELS` - `coco` or path to labels.txt
- `DETECT_CONF_THRESHOLD` - Confidence threshold (default 0.4)
- `DETECT_NMS_THRESHOLD` - NMS IoU threshold (default 0.45)
- `DETECT_INPUT_SIZE` - Model input size (default 640)
- `DETECT_INTERVAL` - Run detection every N frames (default 2)

---

## 4. Navigation & Sensors

### Overview
Comprehensive sensor suite for navigation: quadrature encoders for odometry, VL53L1X time-of-flight distance sensor for obstacle detection.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/encoder.py` - Quadrature encoder decoder (lgpio/RPi.GPIO)
  - `gordonbot-backend/app/services/tof.py` - VL53L1X distance sensor (I²C)
  - `gordonbot-backend/app/services/sensors.py` - Unified sensor data aggregation
  - `gordonbot-backend/app/routers/sensors.py` - `GET /api/sensors/status` endpoint
- **Frontend:**
  - `gordonbot-dashboard/src/components/NavigationPanel.tsx` - Speed, distance, compass display
  - `gordonbot-dashboard/src/components/hooks/useSensors.ts` - Polling hook

### Encoders (Odometry)

#### Hardware
- Pololu micro metal gearmotor with integrated 2-channel Hall encoder
- 12 CPR (counts per revolution) on motor shaft
- ~100:1 gearbox → ~1204 counts per output shaft revolution
- Right encoder: GPIO 12 (A), GPIO 26 (B)
- Left encoder: GPIO 22 (A), GPIO 27 (B)

#### How It Works
1. **Interrupt-driven quadrature decoding** via lgpio (preferred) or RPi.GPIO (fallback)
2. **Edge detection** on both A and B channels → increment/decrement counters
3. **Metrics calculation**:
   - Distance (m/mm) from counts and wheel diameter
   - Speed (mm/s) from delta distance / delta time
   - RPM from counts and time window
4. **Calibration** via env vars: counts per rev, wheel diameter, per-direction scale factors

#### Configuration
- `ENCODER_COUNTS_PER_REV_OUTPUT` - Counts per output shaft revolution (default 1204)
- `WHEEL_DIAMETER_M` - Wheel diameter in meters (default 0.03)
- `ENCODER_RIGHT_PA`, `ENCODER_RIGHT_PB` - Right encoder GPIO pins
- `ENCODER_LEFT_PA`, `ENCODER_LEFT_PB` - Left encoder GPIO pins
- Optional scale factors: `ENCODER_RIGHT_FWD_SCALE`, `ENCODER_RIGHT_REV_SCALE`, etc.

### Time-of-Flight Distance Sensor (VL53L1X)

#### How It Works
1. **I²C communication** via Adafruit CircuitPython library
2. **Distance ranging** (up to ~4m) with configurable timing budget
3. **Continuous polling** in background thread
4. **Data** exposed via `GET /api/sensors/status` → `{"tof": {"distance_mm": 1234}}`

#### Configuration
- Auto-detected on I²C bus (default address 0x29)
- Optional: `TOF_TIMING_BUDGET` for accuracy/speed trade-off

---

## 5. IMU & Orientation

### Overview
BNO055 9-DOF IMU providing orientation quaternions, Euler angles, and calibration status. Supports scripted figure-eight calibration routine via WebSocket commands.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/bno055.py` - IMU reader + calibration state machine
  - `gordonbot-backend/app/sockets/orientation.py` - WebSocket `/ws/orientation` endpoint
- **Frontend:**
  - `gordonbot-dashboard/src/components/OrientationPanel.tsx` - 3D cube visualization (Three.js)
  - `gordonbot-dashboard/src/components/hooks/useOrientation.ts` - WebSocket client

### How It Works
1. **BNO055** connected via I²C, provides quaternions (qw, qx, qy, qz) and Euler angles (roll, pitch, yaw)
2. **Calibration status** (sys, gyro, accel, mag) polled every frame (0-3 per sensor)
3. **Backend** streams orientation frames at ~20Hz:
   ```json
   {"type": "orientation", "ts": 1234567890, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 90.0},
    "calib": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3}, "stale": false}
   ```
4. **Frontend** receives frames, updates Three.js cube rotation in real-time
5. **Calibration routine**:
   - Client sends `{"action": "start_calibration"}`
   - Backend executes scripted figure-eight drive pattern at low speed
   - Progress events: `{"type": "calibration_run", "status": "started"}`, `{"type": "calibration_complete", "ok": true}`

### Key Features
- Real-time 3D orientation visualization (Three.js cube)
- Derived heading/pitch/roll display
- Calibration status indicators (color-coded 0-3 levels)
- One-click recenter (resets cube to default orientation)
- Scripted calibration drive (automated figure-eight pattern)
- Stale data detection (warns if IMU stops updating)

### WebSocket Protocol
- **Server → Client**: Orientation frames, calibration events, errors
- **Client → Server**: `{"action": "start_calibration"}` to trigger calibration
- **Acks**: `{"type": "calibration", "ok": true/false, "message": "..."}`

---

## 6. Voice Interaction System

### Overview
Complete voice pipeline: wake-word detection (Porcupine), speech capture (VAD), transcription (Whisper), intent routing, and text-to-speech responses (OpenAI or espeak-ng).

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/wake_word.py` - Porcupine wake-word listener
  - `gordonbot-backend/app/services/voice_interaction.py` - Orchestration controller
  - `gordonbot-backend/app/services/transcription.py` - Multi-backend Whisper transcription
  - `gordonbot-backend/app/services/text_to_speech.py` - TTS with OpenAI/espeak fallbacks
  - `gordonbot-backend/app/services/voice_router.py` - Intent matching and command dispatch
  - `gordonbot-backend/app/services/audio_playback.py` - Audio cue player (wake/ack sounds)
  - `gordonbot-backend/app/routers/wakeword.py` - Wake-word status API
  - `gordonbot-backend/app/main.py` - Service initialization on startup

### How It Works

#### 1. Wake-Word Detection (Porcupine)
- Background thread listens to microphone via `PvRecorder`
- Picovoice Porcupine engine detects custom wake-word (`.ppn` file)
- On detection: callback triggers voice interaction controller
- Optional wake-word audio cue (e.g., beep sound via ffplay)

#### 2. Speech Capture (WebRTC VAD)
- Wake-word service pauses during capture to avoid feedback
- `SpeechRecorder` uses WebRTC Voice Activity Detection (VAD)
- Pre-roll buffer captures audio before speech starts
- Stops after configurable silence duration (e.g., 1.5s)
- Optional: saves recordings to disk for debugging

#### 3. Transcription (Whisper)
Three backends with automatic fallback:
- **Whisper API** (cloud): OpenAI-compatible API (default)
- **whisper.cpp** (local): C++ implementation (fallback #1)
- **faster-whisper** (local): CTranslate2-based (fallback #2)

Returns transcript text + confidence score.

#### 4. Intent Routing
- Pattern matching on transcript (keyword-based rules)
- Built-in intents: "cpu temperature" → reports system temp
- Fallback: offload to ChatGPT/LLM for general queries
- Returns spoken response text

#### 5. Text-to-Speech
- **OpenAI TTS** (cloud): `gpt-4o-mini-tts` model (default)
- **espeak-ng** (local): Linux TTS engine (fallback)
- Optional SoX filtering for audio effects
- Playback via SoX or direct espeak output

#### 6. Acknowledgment Cue
- Optional "listening" audio cue after speech capture
- Provides feedback that transcription is in progress

### Key Features
- Fully offline-capable (whisper.cpp + espeak-ng)
- Cloud-first with local fallbacks
- Configurable sensitivity, VAD parameters, silence thresholds
- Automatic wake-word service pause/resume during interaction
- Recording archival for training/debugging
- Extensible intent routing (add new commands via `voice_router.py`)

### Configuration

#### Wake-Word
- `WAKEWORD_ENABLED` - Enable wake-word detection
- `PICOVOICE_ACCESS_KEY` - Porcupine API key
- `WAKEWORD_KEYWORD_PATH` - Path to `.ppn` wake-word model
- `WAKEWORD_SENSITIVITY` - Detection sensitivity 0.0-1.0 (default 0.6)
- `WAKEWORD_AUDIO_DEVICE_INDEX` - Microphone device index

#### Speech Capture
- `SPEECH_VAD_AGGRESSIVENESS` - VAD mode 0-3 (default 2)
- `SPEECH_VAD_SILENCE_MS` - Silence duration to end capture (default 1500ms)
- `SPEECH_VAD_MAX_MS` - Max recording length (default 10000ms)
- `SPEECH_PRE_ROLL_MS` - Pre-speech buffer (default 300ms)
- `SPEECH_SAVE_RECORDINGS` - Archive recordings to disk
- `SPEECH_RECORDING_DIR` - Recording save directory

#### Transcription
- `SPEECH_BACKEND` - `auto`, `whisper-api`, `whispercpp`, `faster-whisper`
- `SPEECH_API_KEY` - OpenAI API key
- `SPEECH_API_MODEL` - Whisper model (default `whisper-1`)
- `SPEECH_API_BASE` - API base URL (default OpenAI)
- `SPEECH_API_TIMEOUT` - Request timeout (default 30s)
- `WHISPERCPP_BIN` - Path to whisper.cpp binary
- `WHISPERCPP_MODEL` - Path to `.bin` model file
- `SPEECH_MODEL` - faster-whisper model name (e.g., `base.en`)
- `SPEECH_DEVICE` - `cpu` or `cuda`
- `SPEECH_COMPUTE_TYPE` - `int8`, `float16`, etc.

#### Text-to-Speech
- `TTS_BACKEND` - `auto`, `openai`, `espeakng`, `none`
- `TTS_OPENAI_MODEL` - OpenAI TTS model (default `gpt-4o-mini-tts`)
- `TTS_OPENAI_VOICE` - Voice name (default `onyx`)
- `ESPEAK_NG_BIN` - espeak-ng binary path
- `ESPEAK_VOICE` - Voice (default `en-US`)
- `ESPEAK_RATE`, `ESPEAK_PITCH`, `ESPEAK_AMPLITUDE` - Voice parameters
- `TTS_USE_FILTER` - Enable SoX filtering
- `SOX_BIN` - SoX binary path
- `TTS_SOX_DESTINATION` - Output device (default `-d`)
- `TTS_SOX_EFFECTS` - Effect chain (e.g., `reverb 50`)

#### Audio Cues
- `WAKEWORD_AUDIO_ENABLED` - Play wake-word cue
- `WAKEWORD_AUDIO_PATH` - Path to wake sound file
- `WAKEWORD_AUDIO_COMMAND` - Playback command template (e.g., `ffplay -nodisp -autoexit -loglevel quiet {path}`)
- `ACK_AUDIO_ENABLED` - Play acknowledgment cue
- `ACK_AUDIO_PATH`, `ACK_AUDIO_COMMAND` - Ack sound settings

#### Voice Replies (LLM Fallback)
- `VOICE_REPLY_MODEL` - LLM model (default `gpt-4o-mini`)
- `VOICE_REPLY_SYSTEM_PROMPT` - System prompt for LLM

### Status API
- `GET /api/wakeword/status` → `{"enabled": true, "detections": 42, "last_detected_at": "...", "recent": [...]}`

---

## 7. Battery Monitoring

### Overview
Real-time battery telemetry: voltage, current, power, state of charge, charging status, and multi-cell voltage breakdowns.

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/battery.py` - Battery data reader
  - `gordonbot-backend/app/routers/battery.py` - `GET /api/battery/status` endpoint
- **Frontend:**
  - `gordonbot-dashboard/src/components/BatteryPanel.tsx` - Battery status display

### How It Works
1. **Battery management IC** (custom implementation, details in `battery.py`)
2. **Polling**: Frontend requests battery data every 2-5 seconds
3. **Data fields**:
   - `percent` - State of charge (0-100%)
   - `charging` - Boolean charging status
   - `vbusVoltage`, `vbusCurrent`, `vbusPower` - USB/charger input
   - `batteryVoltage`, `batteryCurrent` - Battery output
   - `remainingCapacity` - mAh remaining
   - `avgTimeToFullMin` - Estimated charge time
   - `cells` - Per-cell voltages (array)

### Key Features
- Real-time voltage/current/power monitoring
- Per-cell voltage display for multi-cell packs
- Charging state detection
- Estimated time to full charge
- Color-coded status indicators (low/medium/high)

---

## 8. System Diagnostics

### Overview
System health metrics: CPU usage, memory, temperature, uptime, throttling status (Raspberry Pi).

### Files Involved
- **Backend:**
  - `gordonbot-backend/app/services/diagnostics.py` - System metrics collector
  - `gordonbot-backend/app/routers/diagnostics.py` - `GET /api/diag/system` endpoint
- **Frontend:**
  - `gordonbot-dashboard/src/components/DiagnosticsPanel.tsx` - Metrics display

### How It Works
1. **psutil** library reads CPU/memory/disk stats
2. **Raspberry Pi specific**: Reads throttle status from `vcgencmd`, CPU temp from `/sys/class/thermal`
3. **Polling**: Frontend refreshes every 2-5 seconds

### Metrics Provided
- CPU usage (%) and per-core breakdown
- Memory: used/available (MB/GB), usage %
- Disk: used/total (GB), usage %
- CPU temperature (°C)
- Uptime (formatted string)
- Throttling status (undervoltage, frequency capping, thermal throttling)

---

## 9. Development Tools

### Supervisor: gordonmon (Go TUI)

#### Overview
Bubble Tea-based terminal UI for managing all GordonBot services.

#### Files
- `gordonmon/main.go` - Main TUI application
- `gordonmon/README.md` - Setup and usage

#### Features
- Starts/stops/restarts backend, frontend, and MediaMTX
- Sticky header: service status, IP address, derived URLs, spinner
- Scrollable log pane (mouse wheel, PgUp/PgDn, Home/End)
- Sticky footer: hotkeys (Q=quit, R=reload all, B=restart backend, F=restart frontend, P=toggle pager mode)
- Automatic port cleanup
- Optional MediaMTX auto-start if not running

#### Usage
```bash
cd gordonmon
go run .
# or
go build -o gordonmon && ./gordonmon
```

### Supervisor: dev.sh (Bash TUI)

#### Overview
Alternative shell-based supervisor (simpler, no Go required).

#### Files
- `scripts/dev.sh` - Bash TUI script
- `scripts/bootstrap.sh` - One-time setup (venv, deps)

#### Features
- Runs backend + frontend with simple log streaming
- Interactive controls: R=reload, Q=quit
- Port cleanup on exit
- Optional MediaMTX startup

#### Usage
```bash
bash scripts/dev.sh
```

### Bootstrap Script

#### File
- `scripts/bootstrap.sh`

#### What It Does
1. Creates Python venv for backend
2. Installs backend dependencies (`requirements.txt`)
3. Installs frontend dependencies (`npm ci`)
4. Prints next steps

#### Usage
```bash
bash scripts/bootstrap.sh
```

---

## Feature Matrix

| Feature | Backend Files | Frontend Files | Hardware | Config Vars |
|---------|--------------|----------------|----------|-------------|
| **Motor Control** | `sockets/control.py`, `services/drive_state.py` | `ControlPanel.tsx`, `JoystickPad.tsx` | GPIO 5,6,19,13 | - |
| **Camera/Video** | `services/camera.py`, `routers/video.py` | `CameraPanel.tsx` | Picamera2 | `CAMERA_RTSP_URL`, `CAMERA_BITRATE` |
| **Detection** | `services/detectors/*.py` | - | Optional Hailo | `DETECT_ENABLED`, `DETECT_ONNX_PATH` |
| **Encoders** | `services/encoder.py`, `routers/sensors.py` | `NavigationPanel.tsx`, `hooks/useSensors.ts` | GPIO 12,26,22,27 | `ENCODER_COUNTS_PER_REV_OUTPUT` |
| **ToF Sensor** | `services/tof.py`, `routers/sensors.py` | `NavigationPanel.tsx` | VL53L1X (I²C) | - |
| **IMU** | `services/bno055.py`, `sockets/orientation.py` | `OrientationPanel.tsx`, `hooks/useOrientation.ts` | BNO055 (I²C) | - |
| **Wake-Word** | `services/wake_word.py`, `routers/wakeword.py` | - | Microphone | `WAKEWORD_ENABLED`, `PICOVOICE_ACCESS_KEY` |
| **Transcription** | `services/transcription.py` | - | - | `SPEECH_BACKEND`, `SPEECH_API_KEY` |
| **TTS** | `services/text_to_speech.py` | - | Speaker | `TTS_BACKEND`, `TTS_OPENAI_MODEL` |
| **Voice Router** | `services/voice_router.py`, `services/voice_interaction.py` | - | - | `VOICE_REPLY_MODEL` |
| **Battery** | `services/battery.py`, `routers/battery.py` | `BatteryPanel.tsx` | Custom BMS | - |
| **Diagnostics** | `services/diagnostics.py`, `routers/diagnostics.py` | `DiagnosticsPanel.tsx` | - | - |

---

## Quick Reference: Common Workflows

### Adding a New Voice Command
1. Edit `gordonbot-backend/app/services/voice_router.py`
2. Add new `IntentRule` to `RULES` list with keywords and action function
3. Implement action function (return spoken response string)
4. Test via wake-word or transcription API

### Adding a New Sensor
1. Create service in `gordonbot-backend/app/services/` (e.g., `new_sensor.py`)
2. Add sensor data fields to `gordonbot-backend/app/schemas.py` (e.g., `SensorsStatus` model)
3. Update `gordonbot-backend/app/services/sensors.py` to poll new sensor
4. Add display component in `gordonbot-dashboard/src/components/` (e.g., `NewSensorPanel.tsx`)
5. Hook into `NavigationPanel.tsx` or create new panel in `GordonBotDashboard.tsx`

### Adding a New REST Endpoint
1. Create router in `gordonbot-backend/app/routers/` (e.g., `new_feature.py`)
2. Define endpoint with `@router.get()` or `@router.post()`
3. Include router in `gordonbot-backend/app/main.py`: `api.include_router(new_feature_router.router)`
4. Add frontend API call in dashboard (fetch or hook)

### Adding a New WebSocket Endpoint
1. Create socket module in `gordonbot-backend/app/sockets/` (e.g., `new_stream.py`)
2. Define WebSocket route with `@router.websocket()`
3. Include router in `gordonbot-backend/app/main.py`: `app.include_router(new_stream_socket.router)`
4. Add frontend hook in `gordonbot-dashboard/src/components/hooks/` (e.g., `useNewStream.ts`)
5. Use hook in component to receive/send messages

### Switching Detection Backend
1. Set `DETECT_BACKEND=hailo` in `gordonbot-backend/.env`
2. Provide `DETECT_HAILO_HEF_PATH=/path/to/model.hef`
3. Optional: Custom post-processing via `DETECT_HAILO_POSTPROCESS=module:function`
4. Restart backend

### Testing Without Hardware
- Motors: Automatically falls back to log-only mode if GPIO unavailable
- Encoders: Gracefully degrades; sensor API returns null values
- Camera: Requires Picamera2; will fail without it (use dev machine for frontend-only testing)
- IMU/ToF: Returns null/error if I²C devices missing

---

## Architecture Patterns

### Backend Service Pattern
- **Service modules** in `services/` provide core functionality (stateful, singleton or factory)
- **Router modules** in `routers/` expose HTTP/REST APIs (stateless handlers)
- **Socket modules** in `sockets/` expose WebSocket endpoints (stateful connections)
- **Schemas** in `schemas.py` define Pydantic models for validation and serialization

### Frontend Component Pattern
- **Panel components** (`*Panel.tsx`) are self-contained UI sections (one per feature)
- **Hooks** (`hooks/use*.ts`) encapsulate data fetching, WebSocket connections, state management
- **UI primitives** (`ui/*.tsx`) are reusable shadcn/ui components (buttons, cards, badges)
- **Config** (`config.ts`) centralizes API base URLs, WebSocket paths, stream names
- **Types** (`types.ts`) defines TypeScript interfaces matching backend schemas

### Configuration Pattern
- **Backend**: Pydantic `Settings` model in `core/config.py` reads from `.env` file
- **Frontend**: Vite env vars in `.env` (prefixed with `VITE_`) injected at build time
- **Runtime**: Both use `python-dotenv` / Vite's built-in env loading

### Error Handling
- **Backend**: Exceptions logged, HTTP errors returned (400/500), WebSocket errors sent as `{"type": "error", "message": "..."}`
- **Frontend**: Try/catch around API calls, WebSocket reconnect logic, toast notifications for errors

---

## Testing & Debugging

### Backend Logs
- Set `VERBOSE=true` in `.env` for detailed logs
- Logs include timestamps, logger names, and structured messages
- Motor commands, sensor reads, transcription results all logged

### Frontend Console
- WebSocket messages logged to browser console
- API fetch errors caught and logged
- React dev tools for component inspection

### Common Issues
- **Motors not working**: Check GPIO permissions, verify gpiozero pin factory (lgpio preferred)
- **Camera not starting**: Ensure `python3-picamera2` installed via apt (not pip), check venv sys.path
- **Encoders not counting**: Verify GPIO pin numbers, check pullup/pulldown resistors, test with `lgpio` directly
- **Wake-word not detecting**: Check microphone permissions, verify Picovoice access key, test VAD sensitivity
- **WebRTC not playing**: Ensure MediaMTX running, check CORS settings, verify WHEP proxy URL format

### Performance Tuning
- **Detection**: Increase `DETECT_INTERVAL` to skip more frames (lower CPU load)
- **Camera**: Reduce resolution or bitrate if streaming lags
- **Encoders**: Tune interrupt priorities if counts drop at high speeds
- **Voice**: Use whisper.cpp `tiny` model for faster transcription (lower accuracy)

---

## Future Enhancement Ideas (Sandbox Testing)

1. **Autonomous Navigation**: SLAM, path planning, obstacle avoidance
2. **Computer Vision**: Object tracking, face recognition, gesture control
3. **Multi-Robot Coordination**: Fleet management, task allocation
4. **Advanced Voice**: Continuous conversation mode, emotion detection, multi-language
5. **Teleoperation**: Remote control over internet (WebRTC data channels)
6. **Machine Learning**: On-device training, reinforcement learning for driving
7. **Map Building**: Occupancy grids from ToF + encoders, loop closure
8. **Advanced Sensors**: LiDAR, depth camera, GPS, magnetometer fusion

---

**Last Updated**: 2025-10-01
**Maintainer**: Keith (sandbox testing before production implementation)
