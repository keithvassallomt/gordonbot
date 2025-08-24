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

/**
 * System diagnostics hook.
 *
 * Polls `/api/diag/system` and returns raw fields plus derived helpers
 * for easy rendering (temps, load averages, per-CPU load, throttling flags).
 *
 * @param intervalMs - Poll interval in ms (default: 5000)
 * @param url - Endpoint URL (default: "/api/diag/system")
 * @returns Object with:
 * - `raw`: the raw response from the backend
 * - `cpuTempC`, `rp1TempC`: numbers or null
 * - `cpuUtilPercent`: number | null
 * - `load1`, `load5`, `load15`: numbers | null
 * - `load1PerCpu`, `load5PerCpu`, `load15PerCpu`: numbers | null
 * - `memory`, `swap`, `diskRoot`: the corresponding objects from backend (or null)
 * - `uptimeSeconds`, `bootTime`: numbers | null
 * - `throttling`: flags object or null
 * - `error`: string | null
 */
export function useSystemDiagnostics(intervalMs = 5000, url = "/api/diag/system") {
  type LoadObj = { ["1m"]: number; ["5m"]: number; ["15m"]: number }
  type MemObj = { total: number; available?: number; used: number; percent: number; free?: number }
  type DiskObj = { total: number; used: number; free: number; percent: number }
  type Throttle = {
    raw: number
    under_voltage_now?: boolean
    freq_capped_now?: boolean
    throttled_now?: boolean
    soft_temp_limit_now?: boolean
    under_voltage_occured?: boolean
    freq_capped_occured?: boolean
    throttled_occured?: boolean
    soft_temp_limit_occured?: boolean
  }

  interface RawDiag {
    cpu_util_percent?: number
    load_average?: Partial<LoadObj> | null
    load_per_cpu?: Partial<LoadObj> | null
    cpu_temperature?: number | null
    rp1_temperature?: number | null
    memory?: Partial<MemObj> | null
    swap?: Partial<MemObj> | null
    disk_root?: Partial<DiskObj> | null
    uptime_seconds?: number | null
    boot_time?: number | null
    throttling?: Throttle | null
  }

  const [raw, setRaw] = useState<RawDiag | null>(null)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let mounted = true
    const fetchOnce = async () => {
      try {
        setError(null)
        const res = await fetch(url, { cache: "no-store" })
        if (!res.ok) throw new Error(String(res.status))
        const json: RawDiag = await res.json()
        if (!mounted) return
        setRaw(json)
      } catch (e) {
        if (!mounted) return
        setError(e instanceof Error ? e.message : "failed")
      }
    }
    fetchOnce()
    const id = setInterval(fetchOnce, intervalMs)
    return () => {
      mounted = false
      clearInterval(id)
    }
  }, [intervalMs, url])

  const cpuTempC = raw?.cpu_temperature ?? null
  const rp1TempC = raw?.rp1_temperature ?? null
  const cpuUtilPercent = raw?.cpu_util_percent ?? null
  const load1 = raw?.load_average?.["1m"] ?? null
  const load5 = raw?.load_average?.["5m"] ?? null
  const load15 = raw?.load_average?.["15m"] ?? null
  const load1PerCpu = raw?.load_per_cpu?.["1m"] ?? null
  const load5PerCpu = raw?.load_per_cpu?.["5m"] ?? null
  const load15PerCpu = raw?.load_per_cpu?.["15m"] ?? null

  return {
    raw,
    error,
    cpuTempC,
    rp1TempC,
    cpuUtilPercent,
    load1,
    load5,
    load15,
    load1PerCpu,
    load5PerCpu,
    load15PerCpu,
    memory: raw?.memory ?? null,
    swap: raw?.swap ?? null,
    diskRoot: raw?.disk_root ?? null,
    uptimeSeconds: raw?.uptime_seconds ?? null,
    bootTime: raw?.boot_time ?? null,
    throttling: raw?.throttling ?? null,
  }
}