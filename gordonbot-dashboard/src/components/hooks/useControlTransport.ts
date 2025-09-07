import { useCallback, useRef, useState } from "react"
import { API_BASE, CONTROL_WS_PATH } from "../config"
import type { TransportStatus, ControlCommand } from "../types"

/**
 * React hook that manages a WebSocket connection for GordonBot drive commands.
 *
 * @param path - WebSocket path (default: CONTROL_WS_PATH = "/ws/control").
 * @returns Transport API with:
 * - `status`: Current {@link TransportStatus} ("disconnected" | "connecting" | "connected").
 * - `connect()`: Attempt to open the WebSocket connection.
 * - `disconnect()`: Close the connection and reset state.
 * - `send(cmd)`: Send a {@link ControlCommand} as JSON `{ type: "drive", payload: cmd }`.
 *
 * @remarks
 * - Resolves the full URL based on `window.location` (ws://host/path or wss:// for HTTPS).
 * - Fails gracefully in dev if no backend is present (no-op on send).
 *
 * @example
 * ```tsx
 * const transport = useControlTransport();
 * useEffect(() => { transport.connect(); }, []);
 *
 * transport.send({ left: 0.5, right: 0.5, ts: Date.now() });
 * ```
 */
export function useControlTransport(path = CONTROL_WS_PATH) {
  const [status, setStatus] = useState<TransportStatus>("disconnected")
  const wsRef = useRef<WebSocket | null>(null)

  const resolveUrl = () => {
    try {
      // If API_BASE is absolute (http/https), derive ws(s) URL from it to avoid relying on dev proxy.
      if (/^https?:\/\//i.test(API_BASE)) {
        const api = new URL(API_BASE)
        const wsScheme = api.protocol === "https:" ? "wss" : "ws"
        return `${wsScheme}://${api.host}${path}`
      }
    } catch {
      // fall back to window.location below
    }
    // Fallback: same-origin as the current page (useful when frontend is served by backend in prod)
    const { host, protocol } = window.location
    const scheme = protocol === "https:" ? "wss" : "ws"
    return `${scheme}://${host}${path}`
  }

  /**
   * Open the WebSocket connection to the robot.
   * Updates status to "connecting" then "connected" on success.
   * On error or close, sets status to "disconnected".
   */
  const connect = useCallback(() => {
    try {
      setStatus("connecting")
      const ws = new WebSocket(resolveUrl())
      wsRef.current = ws
      ws.onopen = () => setStatus("connected")
      ws.onclose = () => setStatus("disconnected")
      ws.onerror = () => setStatus("disconnected")
    } catch (err) {
      console.warn("WS connect failed (stub mode)", err)
      setStatus("disconnected")
    }
  }, [path])

  /**
   * Close the WebSocket connection and reset state.
   */
  const disconnect = useCallback(() => {
    const ws = wsRef.current
    if (!ws) {
      setStatus("disconnected")
      return
    }

    // In React 18 StrictMode, effects mount/unmount twice in dev. Closing a CONNECTING socket
    // triggers a console error ("closed before connection is established"). To avoid that noise,
    // if still CONNECTING, close immediately after it opens instead of during handshake.
    if (ws.readyState === WebSocket.CONNECTING) {
      const handleOpen = () => {
        try { ws.close() } catch {}
      }
      ws.addEventListener("open", handleOpen, { once: true })
    } else {
      try { ws.close() } catch {}
    }

    wsRef.current = null
    setStatus("disconnected")
  }, [])

  /**
   * Send a control command over the WebSocket if open.
   * No-op if the socket is not connected.
   */
  const send = useCallback((cmd: ControlCommand) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ type: "drive", payload: cmd }))
    } else {
      // no-op in dev without backend
    }
  }, [])

  return { status, connect, disconnect, send }
}
