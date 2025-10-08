import { useEffect, useState, useCallback } from "react"
import { API_BASE } from "@/components/config"

export function useSavedMap() {
  const [hasSavedMap, setHasSavedMap] = useState(false)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const checkSavedMap = useCallback(async () => {
    try {
      setLoading(true)
      const response = await fetch(`${API_BASE}/api/slam/status`)
      if (!response.ok) {
        throw new Error("Failed to fetch SLAM status")
      }
      const data = await response.json()
      setHasSavedMap(data.saved_map_exists || false)
      setError(null)
    } catch (err) {
      console.error("Error checking saved map:", err)
      setError(err instanceof Error ? err.message : "Unknown error")
      setHasSavedMap(false)
    } finally {
      setLoading(false)
    }
  }, [])

  useEffect(() => {
    checkSavedMap()
  }, [checkSavedMap])

  return {
    hasSavedMap,
    loading,
    error,
    refresh: checkSavedMap,
  }
}
