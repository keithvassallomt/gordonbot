import { useCallback, useEffect, useState } from "react"

import type { BatteryData } from "../types"
import { API_BASE, BATTERY_ENDPOINT } from "../config"

/**
 * React hook to fetch and poll GordonBot's battery status from the REST API.
 *
 * @param pollMs - Polling interval in milliseconds (default: 10000).
 * @returns An object containing:
 * - `data`: Latest {@link BatteryData} or null if unavailable.
 * - `error`: Error message if last fetch failed, otherwise null.
 * - `refresh`: Manually trigger a one-shot refresh.
 *
 * @remarks
 * Fetches from {@link API_BASE}{@link BATTERY_ENDPOINT}. Uses `setInterval` to refresh on a fixed schedule.
 * Clears the interval on unmount.
 *
 * @example
 * ```tsx
 * const { data, error, refresh } = useBattery(5000);
 * if (data) {
 *   console.log(`Battery at ${data.percent}%`);
 * }
 * ```
 */
export function useBattery(pollMs = 10000) {
  const [data, setData] = useState<BatteryData | null>(null)
  const [error, setError] = useState<string | null>(null)

  /**
   * Fetch the battery data once from the API and update state.
   * Sets `error` if request fails or returns non-OK.
   */
  const fetchBattery = useCallback(async () => {
    try {
      setError(null)
      const res = await fetch(API_BASE + BATTERY_ENDPOINT, { cache: "no-store" })
      if (!res.ok) throw new Error(`${res.status}`)
      const json: BatteryData = await res.json()
      setData(json)
    } catch (err: unknown) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message || "failed")
    }
  }, [])

  useEffect(() => {
    fetchBattery()
    const id = setInterval(fetchBattery, pollMs)
    return () => clearInterval(id)
  }, [fetchBattery, pollMs])

  return { data, error, refresh: fetchBattery }
}