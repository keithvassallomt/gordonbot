import { useCallback, useEffect, useRef, useState } from "react"

import { API_BASE, ORIENTATION_WS_PATH } from "@/components/config"
import type { TransportStatus } from "@/components/types"

export type OrientationFrameMessage = {
  type: "orientation"
  ts: number
  qw?: number
  qx?: number
  qy?: number
  qz?: number
  euler?: {
    roll?: number
    pitch?: number
    yaw?: number
  }
  calib?: {
    sys?: number
    gyro?: number
    accel?: number
    mag?: number
  }
  stale?: boolean
}

export type OrientationNotification = {
  id: number
  kind: "error" | "success" | "info"
  message: string
}

type OrientationAckMessage = {
  type: "calibration"
  ok: boolean
  message?: string
}

type ErrorMessage = {
  type: "error"
  message?: string
}

type PongMessage = {
  type: "pong"
  ts?: number
}

type CalibrationRunMessage = {
  type: "calibration_run"
  status: string
}

type CalibrationCompleteMessage = {
  type: "calibration_complete"
  ok: boolean
}

type SocketMessage =
  | OrientationFrameMessage
  | OrientationNotification
  | OrientationAckMessage
  | ErrorMessage
  | PongMessage
  | CalibrationRunMessage
  | CalibrationCompleteMessage

const RECONNECT_INITIAL_MS = 1000
const RECONNECT_MAX_MS = 10000

function resolveWsUrl(path: string) {
  try {
    if (/^https?:\/\//i.test(API_BASE)) {
      const api = new URL(API_BASE)
      const scheme = api.protocol === "https:" ? "wss" : "ws"
      return `${scheme}://${api.host}${path}`
    }
  } catch (err) {
    console.debug("Failed to parse API_BASE for orientation socket", err)
  }

  const { protocol, host } = window.location
  const scheme = protocol === "https:" ? "wss" : "ws"
  return `${scheme}://${host}${path}`
}

export function useOrientation() {
  const [frame, setFrame] = useState<OrientationFrameMessage | null>(null)
  const [status, setStatus] = useState<TransportStatus>("disconnected")
  const [notification, setNotification] = useState<OrientationNotification | null>(null)
  const [isCalibrating, setIsCalibrating] = useState(false)
  const [isReconnecting, setIsReconnecting] = useState(false)
  const [isInitialDelay, setIsInitialDelay] = useState(true)

  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimerRef = useRef<number | null>(null)
  const reconnectDelayRef = useRef(RECONNECT_INITIAL_MS)
  const stopRef = useRef(false)
  const hasEverConnectedRef = useRef(false)

  const clearReconnectTimer = useCallback(() => {
    if (reconnectTimerRef.current !== null) {
      window.clearTimeout(reconnectTimerRef.current)
      reconnectTimerRef.current = null
    }
    setIsReconnecting(false)
  }, [])

  const handleMessage = useCallback((raw: MessageEvent) => {
    let data: SocketMessage | null = null
    try {
      data = JSON.parse(raw.data)
    } catch (err) {
      console.debug("Orientation socket received non-JSON payload", err)
      return
    }

    if (!data || typeof data !== "object") return

    if ((data as OrientationFrameMessage).type === "orientation") {
      setFrame(data as OrientationFrameMessage)
      return
    }

    if ((data as OrientationAckMessage).type === "calibration") {
      const msg = data as OrientationAckMessage
      if (msg.ok) {
        setIsCalibrating(true)
        setNotification({ id: Date.now(), kind: "info", message: "Calibration routine started" })
      } else {
        setIsCalibrating(false)
        setNotification({
          id: Date.now(),
          kind: "error",
          message: msg.message ?? "Calibration failed",
        })
      }
      return
    }

    if ((data as CalibrationRunMessage).type === "calibration_run") {
      setIsCalibrating(true)
      return
    }

    if ((data as CalibrationCompleteMessage).type === "calibration_complete") {
      const msg = data as CalibrationCompleteMessage
      setIsCalibrating(false)
      setNotification({
        id: Date.now(),
        kind: msg.ok ? "success" : "error",
        message: msg.ok ? "Calibration drive complete" : "Calibration drive cancelled",
      })
      return
    }

    if ((data as ErrorMessage).type === "error") {
      const msg = data as ErrorMessage
      setNotification({ id: Date.now(), kind: "error", message: msg.message ?? "Orientation sensor unavailable" })
      return
    }

    // ignore pong / unknown messages
  }, [])

  const connect = useCallback(() => {
    if (stopRef.current) return
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return

    clearReconnectTimer()

    // Close any existing WebSocket that's still connecting or closing
    if (wsRef.current) {
      try {
        wsRef.current.close()
      } catch (err) {
        console.debug("Failed to close pending WebSocket", err)
      }
      wsRef.current = null
    }

    const url = resolveWsUrl(ORIENTATION_WS_PATH)
    const socket = new WebSocket(url)
    wsRef.current = socket
    setStatus("connecting")

    socket.onopen = () => {
      setStatus("connected")
      reconnectDelayRef.current = RECONNECT_INITIAL_MS
      hasEverConnectedRef.current = true
      setIsReconnecting(false)
    }

    socket.onclose = (event) => {
      if (stopRef.current) {
        return
      }
      if (wsRef.current === socket) {
        wsRef.current = null
      }
      setStatus("disconnected")
      setIsCalibrating(false)
      if (hasEverConnectedRef.current) {
        setNotification(
          event.wasClean
            ? null
            : {
                id: Date.now(),
                kind: "error",
                message: "Orientation stream disconnected",
              },
        )
      }
      if (!stopRef.current && reconnectTimerRef.current === null) {
        const delay = reconnectDelayRef.current
        reconnectTimerRef.current = window.setTimeout(() => {
          reconnectTimerRef.current = null
          connect()
        }, delay)
        setIsReconnecting(true)
        reconnectDelayRef.current = Math.min(reconnectDelayRef.current * 1.5, RECONNECT_MAX_MS)
      }
    }

    socket.onerror = () => {
      if (hasEverConnectedRef.current) {
        setNotification({ id: Date.now(), kind: "error", message: "Orientation stream error" })
      }
      setIsCalibrating(false)
    }

    socket.onmessage = handleMessage
  }, [clearReconnectTimer, handleMessage])

  const disconnect = useCallback(() => {
    stopRef.current = true
    clearReconnectTimer()
    if (wsRef.current) {
      try {
        wsRef.current.close()
      } catch (err) {
        console.debug("Failed to close orientation socket", err)
      }
      wsRef.current = null
    }
  }, [clearReconnectTimer])

  useEffect(() => {
    stopRef.current = false

    // Delay connection by 5 seconds to avoid race conditions on startup
    const connectionTimer = window.setTimeout(() => {
      setIsInitialDelay(false)
      connect()
    }, 5000)

    return () => {
      window.clearTimeout(connectionTimer)
      disconnect()
    }
  }, [connect, disconnect])

  const startCalibration = useCallback(() => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      setNotification({ id: Date.now(), kind: "error", message: "Orientation stream not connected" })
      return
    }
    ws.send(JSON.stringify({ action: "start_calibration" }))
    setNotification({ id: Date.now(), kind: "info", message: "Calibration requested" })
  }, [])

  const abortCalibration = useCallback(() => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return
    }
    ws.send(JSON.stringify({ action: "abort_calibration" }))
    setNotification({ id: Date.now(), kind: "info", message: "Aborting calibration..." })
  }, [])

  const acknowledgeNotification = useCallback(() => {
    setNotification(null)
  }, [])

  return {
    frame,
    status,
    startCalibration,
    abortCalibration,
    notification,
    acknowledgeNotification,
    reconnecting: isReconnecting,
    calibrating: isCalibrating,
    initializing: isInitialDelay,
  }
}
