import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import MapCanvas from "./MapCanvas"
import { API_BASE } from "@/components/config"
import { useSlamMode } from "./contexts/SlamModeContext"

interface GoToToast {
  text: string
  tone: "info" | "error"
}

interface GoToState {
  state: "idle" | "running"
  target?: { x?: number; y?: number }
  distance_remaining?: number
  elapsed_s?: number
  reason?: string
  success?: boolean
  tof_distance_mm?: number
}

const GO_TO_STATUS_POLL_MS = 1000

export default function SlamMapPanel() {
  const { speedMode, setSpeedMode } = useSlamMode()
  const [isClearing, setIsClearing] = useState(false)
  const [isGoToMode, setIsGoToMode] = useState(false)
  const [goToLoading, setGoToLoading] = useState(false)
  const [goToStatus, setGoToStatus] = useState<GoToState | null>(null)
  const [toast, setToast] = useState<GoToToast | null>(null)
  const statusTimerRef = useRef<number | null>(null)

  const fetchGoToStatus = useCallback(async (quiet = false) => {
    try {
      const response = await fetch(`${API_BASE}/api/slam/goto/status`)
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`)
      }
      const payload = (await response.json()) as GoToState
      setGoToStatus(payload)
      if (!quiet && payload.success === false && payload.reason) {
        setToast({ text: `Go-to ended: ${payload.reason}`, tone: "error" })
      }
    } catch (error) {
      if (!quiet) {
        setToast({
          text: `Go-to status unavailable: ${error instanceof Error ? error.message : "Unknown error"}`,
          tone: "error",
        })
      }
    }
  }, [])

  useEffect(() => {
    fetchGoToStatus(true)
    statusTimerRef.current = window.setInterval(() => fetchGoToStatus(true), GO_TO_STATUS_POLL_MS)
    return () => {
      if (statusTimerRef.current !== null) {
        window.clearInterval(statusTimerRef.current)
      }
    }
  }, [fetchGoToStatus])

  useEffect(() => {
    if (!toast) return
    const id = window.setTimeout(() => setToast(null), 5000)
    return () => window.clearTimeout(id)
  }, [toast])

  const handleClearMap = useCallback(async () => {
    if (!confirm("Clear the SLAM map?")) {
      return
    }
    setIsClearing(true)
    try {
      const response = await fetch(`${API_BASE}/api/slam/clear`, { method: "POST" })
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`)
      }
      setToast({ text: "Map clear initiated", tone: "info" })
    } catch (error) {
      setToast({
        text: `Failed to clear map: ${error instanceof Error ? error.message : "Unknown error"}`,
        tone: "error",
      })
    } finally {
      setIsClearing(false)
    }
  }, [])

  const handleGoToSelect = useCallback(
    async ({ x, y }: { x: number; y: number }) => {
      if (goToLoading) {
        return
      }
      setGoToLoading(true)
      try {
        const response = await fetch(`${API_BASE}/api/slam/goto`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ x, y }),
        })
        const payload = await response.json().catch(() => null)
        if (!response.ok) {
          const detail = payload && (payload.detail ?? payload.message)
          throw new Error(detail || `${response.status} ${response.statusText}`)
        }
        setGoToStatus(payload as GoToState)
        setToast({ text: "Go-to command accepted", tone: "info" })
      } catch (error) {
        setToast({
          text: `Failed to start go-to: ${error instanceof Error ? error.message : "Unknown error"}`,
          tone: "error",
        })
      } finally {
        setGoToLoading(false)
        setIsGoToMode(false)
      }
    },
    [goToLoading],
  )

  const handleCancelGoTo = useCallback(async () => {
    try {
      const response = await fetch(`${API_BASE}/api/slam/goto/cancel`, { method: "POST" })
      const payload = await response.json().catch(() => null)
      if (!response.ok) {
        const detail = payload && (payload.detail ?? payload.message)
        throw new Error(detail || `${response.status} ${response.statusText}`)
      }
      setGoToStatus(payload as GoToState)
      setToast({ text: "Go-to cancelled", tone: "info" })
    } catch (error) {
      setToast({
        text: `Failed to cancel go-to: ${error instanceof Error ? error.message : "Unknown error"}`,
        tone: "error",
      })
    }
  }, [])

  const distanceRemaining = useMemo(() => {
    if (typeof goToStatus?.distance_remaining === "number" && Number.isFinite(goToStatus.distance_remaining)) {
      return goToStatus.distance_remaining
    }
    return null
  }, [goToStatus])

  const elapsedSeconds = useMemo(() => {
    if (typeof goToStatus?.elapsed_s === "number" && Number.isFinite(goToStatus.elapsed_s)) {
      return goToStatus.elapsed_s
    }
    return null
  }, [goToStatus])

  const targetLabel = useMemo(() => {
    if (goToStatus?.target && Number.isFinite(goToStatus.target.x ?? NaN) && Number.isFinite(goToStatus.target.y ?? NaN)) {
      return `${(goToStatus.target.x ?? 0).toFixed(2)}, ${(goToStatus.target.y ?? 0).toFixed(2)}`
    }
    return null
  }, [goToStatus])

  return (
    <Card className="w-full">
      <CardHeader className="flex flex-col gap-2 sm:flex-row sm:items-center sm:justify-between">
        <CardTitle className="text-base">SLAM Map</CardTitle>
        <div className="flex flex-wrap gap-2">
          <Button
            size="sm"
            variant={speedMode === "creep" ? "default" : "outline"}
            onClick={() => setSpeedMode(speedMode === "creep" ? "normal" : "creep")}
          >
            {speedMode === "creep" ? "Creep Enabled" : "Enable Creep"}
          </Button>
          <Button
            size="sm"
            variant={isGoToMode ? "default" : "outline"}
            onClick={() => setIsGoToMode((prev) => !prev)}
            disabled={goToLoading || (goToStatus?.state === "running")}
          >
            {isGoToMode ? "Cancel pick" : "Go to point"}
          </Button>
          {goToStatus?.state === "running" && (
            <Button size="sm" variant="destructive" onClick={handleCancelGoTo}>
              Cancel go-to
            </Button>
          )}
          <Button size="sm" variant="secondary" onClick={handleClearMap} disabled={isClearing}>
            {isClearing ? "Clearing..." : "Clear map"}
          </Button>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="h-[560px] w-full">
          <MapCanvas gotoMode={isGoToMode} onSelectPoint={handleGoToSelect} />
        </div>
        {goToStatus && (
          <div className="flex flex-wrap items-center gap-3 text-xs">
            <Badge variant={goToStatus.state === "running" ? "default" : "secondary"}>
              {goToStatus.state === "running" ? "Go-to running" : "Idle"}
            </Badge>
            {targetLabel && <span>Target: {targetLabel}</span>}
            {distanceRemaining !== null && <span>Remaining: {distanceRemaining.toFixed(2)} m</span>}
            {elapsedSeconds !== null && <span>Elapsed: {elapsedSeconds.toFixed(1)} s</span>}
            {goToStatus.reason && goToStatus.state !== "running" && (
              <span>Reason: {goToStatus.reason}</span>
            )}
            {typeof goToStatus.tof_distance_mm === "number" && (
              <span>ToF: {goToStatus.tof_distance_mm} mm</span>
            )}
          </div>
        )}
        {toast && (
          <div className={`rounded-md border px-3 py-2 text-xs ${toast.tone === "error" ? "border-red-500 text-red-600" : "border-emerald-500 text-emerald-600"}`}>
            {toast.text}
          </div>
        )}
      </CardContent>
    </Card>
  )
}
