import { useCallback, useEffect, useRef, useState } from "react"

import { API_BASE } from "@/components/config"
import type { TransportStatus } from "@/components/types"

const SCAN_QUALITY_WS_PATH = "/ws/scan-quality"
const RECONNECT_INITIAL_MS = 1000
const RECONNECT_MAX_MS = 10000

export type QualityLevel = "excellent" | "good" | "fair" | "poor" | "critical"

export interface ScanQualityLatest {
  quality_level: QualityLevel
  quality_score: number
  point_count: number
  scan_density: number
  angular_coverage: number
  max_gap_size: number
  valid_range_ratio: number
  mean_range: number
  scan_rate: number
  issues: string[]
  warnings: string[]
}

export interface ScanQualityAverages {
  quality_score: number
  point_count: number
}

export interface ScanQualityConfig {
  min_point_count: number
  min_scan_density: number
  min_angular_coverage: number
  max_allowed_gap: number
  expected_scan_rate: number
}

export interface MapQuality {
  explored_ratio: number
  occupied_ratio: number
  entropy: number
  noise_score: number
  wall_sharpness: number
  feature_density: number
  corner_count: number
  edge_count: number
  quality_level: QualityLevel
  quality_score: number
  issues: string[]
  warnings: string[]
  has_walls: boolean
  is_empty: boolean
}

export interface ScanQualityMessage {
  type: "scan_quality"
  status: "ok" | "no_data"
  timestamp: number
  latest?: ScanQualityLatest
  averages?: ScanQualityAverages
  config?: ScanQualityConfig
  map_quality?: MapQuality
}

type PongMessage = {
  type: "pong"
  ts?: number
}

type SocketMessage = ScanQualityMessage | PongMessage

function resolveWsUrl(path: string) {
  try {
    if (/^https?:\/\//i.test(API_BASE)) {
      const api = new URL(API_BASE)
      const scheme = api.protocol === "https:" ? "wss" : "ws"
      return `${scheme}://${api.host}${path}`
    }
  } catch (err) {
    console.debug("Failed to parse API_BASE for scan quality socket", err)
  }

  const { protocol, host } = window.location
  const scheme = protocol === "https:" ? "wss" : "ws"
  return `${scheme}://${host}${path}`
}

export function useScanQuality() {
  const [quality, setQuality] = useState<ScanQualityMessage | null>(null)
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
      console.debug("Scan quality socket received non-JSON payload", err)
      return
    }

    if (!data || typeof data !== "object") return

    if ((data as ScanQualityMessage).type === "scan_quality") {
      setQuality(data as ScanQualityMessage)
      setErrorMessage(null)
      return
    }

    // ignore pong / unknown messages
  }, [])

  const connect = useCallback(() => {
    if (stopRef.current) return
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return

    clearReconnectTimer()
    const url = resolveWsUrl(SCAN_QUALITY_WS_PATH)
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
        setErrorMessage("Scan quality stream disconnected")
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
        setErrorMessage("Scan quality stream error")
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
        console.debug("Failed to close scan quality socket", err)
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
    quality,
    status,
    errorMessage,
    clearError,
    reconnecting: isReconnecting,
  }
}
