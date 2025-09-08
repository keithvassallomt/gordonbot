import { useCallback, useEffect, useState } from "react"

import type { SensorsStatus } from "../types"
import { API_BASE, SENSORS_ENDPOINT } from "../config"

/**
 * Fetch and poll consolidated sensor readings from the robot.
 *
 * @param pollMs - Polling interval in milliseconds (default: 1000ms).
 */
export function useSensors(pollMs = 1000) {
  const [data, setData] = useState<SensorsStatus | null>(null)
  const [error, setError] = useState<string | null>(null)

  const fetchSensors = useCallback(async () => {
    try {
      setError(null)
      const res = await fetch(API_BASE + SENSORS_ENDPOINT, { cache: "no-store" })
      if (!res.ok) throw new Error(`${res.status}`)
      const json: SensorsStatus = await res.json()
      setData(json)
    } catch (err: unknown) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message || "failed")
    }
  }, [])

  useEffect(() => {
    fetchSensors()
    const id = setInterval(fetchSensors, pollMs)
    return () => clearInterval(id)
  }, [fetchSensors, pollMs])

  return { data, error, refresh: fetchSensors }
}

