// src/components/types.ts

// Transport / control
/**
 * Transport connection state.
 * - "disconnected": No connection established.
 * - "connecting": Attempting to connect.
 * - "connected": WebSocket established.
 */
export type TransportStatus = "disconnected" | "connecting" | "connected"

/**
 * Drive command for tank treads.
 * @property left - Left tread power in [-1, 1].
 * @property right - Right tread power in [-1, 1].
 * @property ts - Timestamp (ms since epoch).
 */
export type ControlCommand = {
  left: number
  right: number
  ts: number
}

/**
 * Transport API interface for sending control commands.
 * @property status - Current connection status.
 * @property connect - Establish the connection.
 * @property disconnect - Close the connection.
 * @property send - Send a {@link ControlCommand}.
 */
export interface ControlTransport {
  status: TransportStatus
  connect: () => void
  disconnect: () => void
  send: (cmd: ControlCommand) => void
}

// Shared vector type
/**
 * 2D vector for joystick and map offsets.
 * @property x - Horizontal component, typically in [-1, 1].
 * @property y - Vertical component, typically in [-1, 1].
 */
export type Vec2 = { x: number; y: number }

// Battery domain
/**
 * Battery telemetry data from the robot.
 * @property percent - Battery charge percent (0..100).
 * @property charging - Whether currently charging.
 * @property vbusVoltage - USB VBUS voltage (V).
 * @property vbusCurrent - USB VBUS current (A).
 * @property vbusPower - USB VBUS power (W).
 * @property batteryVoltage - Pack voltage (V).
 * @property batteryCurrent - Pack current (A, negative = discharging).
 * @property remainingCapacity - Remaining capacity (unit depends on API).
 * @property avgTimeToFullMin - Estimated time to full charge (minutes).
 * @property cells - Per-cell voltages [c1, c2, c3, c4] (V).
 */
export type BatteryData = {
  percent: number // 0..100
  charging?: "idle" | "charging" | "fast_charging" | "discharging"
  vbusVoltage?: number // V
  vbusCurrent?: number // A
  vbusPower?: number // W
  batteryVoltage?: number // V
  batteryCurrent?: number // A (negative = discharging)
  remainingCapacity?: number // Wh or Ah depending on your API — label accordingly
  avgTimeToFullMin?: number // minutes
  cells?: number[] // [c1, c2, c3, c4] in V
}

// Diagnostics domain
/**
 * System diagnostics data from the robot.
 * @property cpu_load - CPU load percentage (0..100).
 * @property cpu_temperature - CPU temperature in Celsius (if available).
 */
export type DiagnosticsData = {
  cpu_load: number // 0..100
  cpu_temperature?: number // Celsius
} 