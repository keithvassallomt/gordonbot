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

// Sensors domain
export type EncoderData = {
  connected?: boolean
  ticks?: number
  distance_m?: number
  distance_mm?: number
  rpm?: number
  speed_mm_s?: number
}

export type MotorEncoders = {
  left?: EncoderData
  right?: EncoderData
}

export type Vector3 = { x?: number; y?: number; z?: number }
export type Quaternion = { w?: number; x?: number; y?: number; z?: number }
export type EulerDeg = { roll?: number; pitch?: number; yaw?: number }

export type BNO055Data = {
  euler?: EulerDeg
  quat?: Quaternion
  ang_vel_rad_s?: Vector3
  accel_m_s2?: Vector3
  mag_uT?: Vector3
  lin_accel_m_s2?: Vector3
  gravity_m_s2?: Vector3
  temp_c?: number
  calibration?: {
    sys?: number
    gyro?: number
    accel?: number
    mag?: number
  }
}

export type ToFData = { distance_mm?: number }

export type SensorsStatus = {
  ts: number
  encoders?: MotorEncoders
  tof?: ToFData
  bno055?: BNO055Data
}

// SLAM domain
export type SlamMapOrigin = {
  x: number // meters
  y: number // meters
  theta: number // radians
}

export type SlamMapMessage = {
  type: "map"
  ts: number // milliseconds
  width: number // cells
  height: number // cells
  resolution: number // meters per cell
  origin: SlamMapOrigin
  data: number[] // occupancy grid: -1=unknown, 0=free, 100=occupied
}

export type SlamPoseMessage = {
  type: "pose"
  ts: number // milliseconds
  x: number // meters
  y: number // meters
  theta: number // radians
  frame_id: string
}

export type SlamMessage = SlamMapMessage | SlamPoseMessage
