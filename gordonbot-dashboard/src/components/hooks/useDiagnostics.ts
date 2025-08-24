import { useEffect, useState } from "react"
import { PING_ENDPOINT } from "../config"

/**
 * React hook that measures and reports UI frames per second (FPS).
 *
 * @returns The latest FPS value, updated once per second.
 * @remarks
 * Uses `requestAnimationFrame` internally and cancels on unmount.
 *
 * @example
 * const fps = useFps();
 * console.log(`Current FPS: ${fps}`);
 */
export function useFps() {
  const [fps, setFps] = useState(0)
  useEffect(() => {
    let frame = 0
    let last = performance.now()
    let rafId = 0
    const tick = () => {
      const now = performance.now()
      frame++
      if (now - last >= 1000) {
        setFps(frame)
        frame = 0
        last = now
      }
      rafId = requestAnimationFrame(tick)
    }
    rafId = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(rafId)
  }, [])
  return fps
}
 
/**
 * React hook that pings a URL periodically to measure latency and availability.
 *
 * @param intervalMs - Polling interval in milliseconds (default: 10000).
 * @param url - URL to ping (default: {@link PING_ENDPOINT}).
 * @returns Object with:
 * - `latency`: Measured round-trip time in ms, or null if failed.
 * - `ok`: True if last ping returned HTTP OK, false/null otherwise.
 *
 * @remarks
 * Starts immediately on mount and repeats on the given interval.
 * Cleans up the interval on unmount.
 *
 * @example
 * const { latency, ok } = usePing(5000);
 * console.log(ok ? `Ping ${latency} ms` : "Ping failed");
 */
export function usePing(intervalMs = 10000, url = PING_ENDPOINT) {
  const [latency, setLatency] = useState<number | null>(null)
  const [ok, setOk] = useState<boolean | null>(null)

  useEffect(() => {
    let mounted = true
    const pingOnce = async () => {
      const t0 = performance.now()
      try {
        const res = await fetch(url, { cache: "no-store" })
        const t1 = performance.now()
        if (!mounted) return
        setLatency(t1 - t0)
        setOk(res.ok)
      } catch {
        if (!mounted) return
        setLatency(null)
        setOk(false)
      }
    }
    pingOnce()
    const id = setInterval(pingOnce, intervalMs)
    return () => {
      mounted = false
      clearInterval(id)
    }
  }, [intervalMs, url])

  return { latency, ok }
}