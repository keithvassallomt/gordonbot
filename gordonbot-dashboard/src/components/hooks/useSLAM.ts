import { useCallback, useEffect, useRef, useState } from "react"

import { API_BASE } from "@/components/config"
import type { SlamMapMessage, SlamPoseMessage, TransportStatus } from "@/components/types"

const SLAM_WS_PATH = "/ws/slam"
const RECONNECT_INITIAL_MS = 1000
const RECONNECT_MAX_MS = 10000

type PongMessage = {
  type: "pong"
  ts?: number
}

type SocketMessage = SlamMapMessage | SlamPoseMessage | PongMessage

function resolveWsUrl(path: string) {
  try {
    if (/^https?:\/\//i.test(API_BASE)) {
      const api = new URL(API_BASE)
      const scheme = api.protocol === "https:" ? "wss" : "ws"
      return `${scheme}://${api.host}${path}`
    }
  } catch (err) {
    console.debug("Failed to parse API_BASE for SLAM socket", err)
  }

  const { protocol, host } = window.location
  const scheme = protocol === "https:" ? "wss" : "ws"
  return `${scheme}://${host}${path}`
}

export function useSLAM() {
  const [map, setMap] = useState<SlamMapMessage | null>(null)
  const [pose, setPose] = useState<SlamPoseMessage | null>(null)
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
      console.debug("SLAM socket received non-JSON payload", err)
      return
    }

    if (!data || typeof data !== "object") return

    if ((data as SlamMapMessage).type === "map") {
      setMap(data as SlamMapMessage)
      setErrorMessage(null)
      return
    }

    if ((data as SlamPoseMessage).type === "pose") {
      setPose(data as SlamPoseMessage)
      setErrorMessage(null)
      return
    }

    // ignore pong / unknown messages
  }, [])

  const connect = useCallback(() => {
    if (stopRef.current) return
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return

    clearReconnectTimer()
    const url = resolveWsUrl(SLAM_WS_PATH)
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
        setErrorMessage("SLAM stream disconnected")
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
        setErrorMessage("SLAM stream error")
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
        console.debug("Failed to close SLAM socket", err)
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

  const clearMapData = useCallback(() => {
    setMap(null)
    setPose(null)
  }, [])

  return {
    map,
    pose,
    status,
    errorMessage,
    clearError,
    clearMapData,
    reconnecting: isReconnecting,
  }
}
