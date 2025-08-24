// src/components/config.ts

// REST endpoints (served by the robot)
/**
 * Base URL for REST API calls.
 * Empty string means same-origin as dashboard.
 */
export const API_BASE = "" // same-origin by default
/**
 * REST endpoint path for battery telemetry data.
 */
export const BATTERY_ENDPOINT = "/api/battery/status"
/**
 * REST endpoint path for ping checks.
 */
export const PING_ENDPOINT = "/api/ping"

// Control transport (WebSocket path mounted on the robot)
/**
 * WebSocket endpoint path for sending drive control commands.
 */
export const CONTROL_WS_PATH = "/ws/control"

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
 * Maximum normalized speed value (1.0 = 100%).
 */
export const MAX_SPEED = 1.0
/**
 * Multiplier applied when boost (Shift) is active.
 */
export const BOOST_MULTIPLIER = 1.5