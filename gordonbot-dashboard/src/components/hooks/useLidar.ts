import { useCallback, useEffect, useRef, useState } from "react"

import { API_BASE, LIDAR_WS_PATH } from "@/components/config"
import type { TransportStatus } from "@/components/types"

export type LidarPoint = {
  angle: number  // degrees (0-360)
  distance_mm: number  // millimeters
  quality: number  // 0-255
}

export type LidarScanMessage = {
  type: "scan"
  ts: number
  points: LidarPoint[]
  scan_rate_hz: number
}

type ErrorMessage = {
  type: "error"
  message?: string
}

type PongMessage = {
  type: "pong"
  ts?: number
}

type SocketMessage = LidarScanMessage | ErrorMessage | PongMessage

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
    console.debug("Failed to parse API_BASE for LIDAR socket", err)
  }

  const { protocol, host } = window.location
  const scheme = protocol === "https:" ? "wss" : "ws"
  return `${scheme}://${host}${path}`
}

export function useLidar() {
  const [scan, setScan] = useState<LidarScanMessage | null>(null)
  const [status, setStatus] = useState<TransportStatus>("disconnected")
  const [errorMessage, setErrorMessage] = useState<string | null>(null)
  const [isReconnecting, setIsReconnecting] = useState(false)

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
      console.debug("LIDAR socket received non-JSON payload", err)
      return
    }

    if (!data || typeof data !== "object") return

    if ((data as LidarScanMessage).type === "scan") {
      setScan(data as LidarScanMessage)
      setErrorMessage(null)
      return
    }

    if ((data as ErrorMessage).type === "error") {
      const msg = data as ErrorMessage
      setErrorMessage(msg.message ?? "LIDAR error")
      return
    }

    // ignore pong / unknown messages
  }, [])

  const connect = useCallback(() => {
    if (stopRef.current) return
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return

    clearReconnectTimer()
    const url = resolveWsUrl(LIDAR_WS_PATH)
    const socket = new WebSocket(url)
    wsRef.current = socket
    setStatus("connecting")

    socket.onopen = () => {
      setStatus("connected")
      reconnectDelayRef.current = RECONNECT_INITIAL_MS
      hasEverConnectedRef.current = true
      setIsReconnecting(false)
      setErrorMessage(null)
    }

    socket.onclose = (event) => {
      if (stopRef.current) {
        return
      }
      if (wsRef.current === socket) {
        wsRef.current = null
      }
      setStatus("disconnected")
      if (hasEverConnectedRef.current && !event.wasClean) {
        setErrorMessage("LIDAR stream disconnected")
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
        setErrorMessage("LIDAR stream error")
      }
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
        console.debug("Failed to close LIDAR socket", err)
      }
      wsRef.current = null
    }
  }, [clearReconnectTimer])

  useEffect(() => {
    stopRef.current = false
    connect()
    return () => {
      disconnect()
    }
  }, [connect, disconnect])

  const clearError = useCallback(() => {
    setErrorMessage(null)
  }, [])

  return {
    scan,
    status,
    errorMessage,
    clearError,
    reconnecting: isReconnecting,
  }
}
