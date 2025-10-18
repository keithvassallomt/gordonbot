import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible"
import { ChevronDown } from "lucide-react"
import MapCanvas from "./MapCanvas"
import ScanQualityPanel from "./ScanQualityPanel"
import { API_BASE } from "@/components/config"
import { useSlamMode } from "./contexts/SlamModeContext"
import type { SensorsStatus } from "./types"

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
const SENSORS_POLL_MS = 200 // 5Hz polling for movement detection
const MOVEMENT_THRESHOLD_MM_S = 10 // 10mm/s = 1cm/s minimum to show movement

export default function SlamMapPanel() {
  const { speedMode, setSpeedMode } = useSlamMode()
  const [isClearing, setIsClearing] = useState(false)
  const [isGoToMode, setIsGoToMode] = useState(false)
  const [goToLoading, setGoToLoading] = useState(false)
  const [goToStatus, setGoToStatus] = useState<GoToState | null>(null)
  const [toast, setToast] = useState<GoToToast | null>(null)
  const [qualityOpen, setQualityOpen] = useState(false)
  const [sensorsData, setSensorsData] = useState<SensorsStatus | null>(null)
  const statusTimerRef = useRef<number | null>(null)
  const sensorsTimerRef = useRef<number | null>(null)

  const fetchSensorsData = useCallback(async () => {
    try {
      const response = await fetch(`${API_BASE}/api/sensors`)
      if (response.ok) {
        const data = (await response.json()) as SensorsStatus
        setSensorsData(data)
      }
    } catch {
      // Silently fail - movement detection is non-critical
    }
  }, [])

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
    fetchSensorsData()
    sensorsTimerRef.current = window.setInterval(fetchSensorsData, SENSORS_POLL_MS)
    return () => {
      if (sensorsTimerRef.current !== null) {
        window.clearInterval(sensorsTimerRef.current)
      }
    }
  }, [fetchSensorsData])

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

  const isMoving = useMemo(() => {
    if (!sensorsData?.encoders) return false
    const leftSpeed = Math.abs(sensorsData.encoders.left?.speed_mm_s ?? 0)
    const rightSpeed = Math.abs(sensorsData.encoders.right?.speed_mm_s ?? 0)
    const maxSpeed = Math.max(leftSpeed, rightSpeed)
    return maxSpeed >= MOVEMENT_THRESHOLD_MM_S
  }, [sensorsData])

  return (
    <Card className="w-full">
      <CardHeader className="flex flex-col gap-2 sm:flex-row sm:items-center sm:justify-between">
        <div className="flex items-center gap-2">
          <CardTitle className="text-base">SLAM Map</CardTitle>
          {isMoving && (
            <Badge variant="default" className="bg-green-500 hover:bg-green-600">
              Moving
            </Badge>
          )}
        </div>
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

        {/* Scan Quality Section (Collapsible) */}
        <Collapsible open={qualityOpen} onOpenChange={setQualityOpen}>
          <CollapsibleTrigger className="flex w-full items-center justify-between rounded-lg border bg-muted/30 px-4 py-2 text-sm font-medium hover:bg-muted/50">
            <span>Scan Quality</span>
            <ChevronDown className={`h-4 w-4 transition-transform ${qualityOpen ? "rotate-180" : ""}`} />
          </CollapsibleTrigger>
          <CollapsibleContent className="mt-3">
            <ScanQualityPanel />
          </CollapsibleContent>
        </Collapsible>

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
