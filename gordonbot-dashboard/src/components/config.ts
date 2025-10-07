// src/components/config.ts

// REST endpoints (served by the robot)
/**
 * Base URL for REST API calls.
 * Empty string means same-origin as dashboard.
 */
// During dev, backend is on 127.0.0.1:8000 and frontend on 127.0.0.1:5173
// Point API_BASE to the backend to avoid 404s from the Vite dev server.
// Override via Vite env: VITE_API_BASE="http://192.168.x.x:8000" for LAN testing.
type ImportMetaWithOptionalEnv = ImportMeta & {
  readonly env?: {
    readonly VITE_API_BASE?: string
    readonly VITE_DASHBOARD_DEBUG?: string
  }
}

const envApiBase = (import.meta as ImportMetaWithOptionalEnv).env?.VITE_API_BASE
export const API_BASE = envApiBase && envApiBase.length > 0 ? envApiBase : "http://192.168.96.187:8000"

const envDebugMode = (import.meta as ImportMetaWithOptionalEnv).env?.VITE_DASHBOARD_DEBUG
export const DEBUG_MODE = envDebugMode?.toLowerCase() === "true"
/**
 * REST endpoint path for battery telemetry data.
 */
export const BATTERY_ENDPOINT = "/api/battery/status"
/**
 * REST endpoint path for ping checks.
 */
export const PING_ENDPOINT = "/api/ping"
/**
 * MJPEG video stream path for robot camera.
 */
export const VIDEO_MJPEG_ENDPOINT = "/api/video/stream.mjpg"
/**
 * Snapshot path for robot camera.
 */
export const VIDEO_SNAPSHOT_ENDPOINT = "/api/video/snapshot.jpg"
/** Video status metadata */
export const VIDEO_STATUS_ENDPOINT = "/api/video/status"
export const VIDEO_RAW_START_ENDPOINT = "/api/video/raw/start"
export const VIDEO_RAW_STOP_ENDPOINT = "/api/video/raw/stop"

/** Sensors status endpoint */
export const SENSORS_ENDPOINT = "/api/sensors/status"

// MediaMTX WebRTC (WHEP) playback endpoint (proxied by backend)
/**
 * Path on the backend that proxies WHEP to MediaMTX, avoiding CORS.
 * Combine with `API_BASE` when fetching.
 */
/** Base path for WHEP proxy on backend. */
export const VIDEO_WHEP_BASE = "/api/video/whep/"
/** Stream name for raw camera feed. Must match MediaMTX stream. */
export const VIDEO_WHEP_STREAM_RAW = "gordon"
/** Stream name for annotated feed (overlays). Must match MediaMTX stream. */
export const VIDEO_WHEP_STREAM_ANNOT = "gordon-annot"

// Control transport (WebSocket path mounted on the robot)
/**
 * WebSocket endpoint path for sending drive control commands.
 */
export const CONTROL_WS_PATH = "/ws/control"
/**
 * WebSocket endpoint path for BNO055 orientation stream.
 */
export const ORIENTATION_WS_PATH = "/ws/orientation"
/**
 * WebSocket endpoint path for LIDAR scan stream.
 */
export const LIDAR_WS_PATH = "/ws/lidar"

// Drive loop tuning
/**
 * Dead-man timeout in milliseconds.
 * If no input within this time, outputs are zeroed for safety.
 */
export const DEADMAN_MS = 300
/**
 * Frequency (Hz) at which drive commands are sent.
 */
export const COMMAND_HZ = 20
/**
 * Forward/backward speed (W/S keys) - 80% normal, 100% with boost.
 */
export const FORWARD_SPEED = 0.8
/**
 * Turn speed (A/D keys) - always 100% for responsive turning.
 */
export const TURN_SPEED = 1.0
/**
 * Multiplier applied to forward speed when boost (Shift) is active (0.8 × 1.25 = 1.0).
 */
export const BOOST_MULTIPLIER = 1.25

// Creep mode speeds (for precise mapping)
/**
 * Forward/backward speed in creep mode (W/S keys) - 70% for precision.
 */
export const CREEP_FORWARD_SPEED = 0.7
/**
 * Turn speed in creep mode (A/D keys) - 80% for controlled turning.
 */
export const CREEP_TURN_SPEED = 0.8

// Keyboard drive ramp (smooth acceleration/deceleration)
/**
 * Keyboard acceleration rate (units per second to approach target).
 * 1.2 -> ~0.83s from 0 to 1.0 (slower, gentler start)
 */
export const KEY_ACCEL_PER_S = 1.2
/**
 * Keyboard deceleration rate (units per second back toward 0/target).
 * Lower than before to avoid abrupt stops.
 */
export const KEY_DECEL_PER_S = 2.0

/**
 * Keyboard turn acceleration rate (A/D keys), units per second.
 * Matches forward accel by default; tune separately if needed.
 */
export const KEY_TURN_ACCEL_PER_S = 1.2
/**
 * Keyboard turn deceleration rate (A/D keys), units per second.
 */
export const KEY_TURN_DECEL_PER_S = 2.0

// Joystick smoothing (apply same defaults; tune independently if needed)
/**
 * Joystick forward/back acceleration (units per second).
 */
export const JOY_ACCEL_PER_S = 1.2
/**
 * Joystick forward/back deceleration (units per second).
 */
export const JOY_DECEL_PER_S = 2.0
/**
 * Joystick turn acceleration (units per second).
 */
export const JOY_TURN_ACCEL_PER_S = 1.2
/**
 * Joystick turn deceleration (units per second).
 */
export const JOY_TURN_DECEL_PER_S = 2.0
