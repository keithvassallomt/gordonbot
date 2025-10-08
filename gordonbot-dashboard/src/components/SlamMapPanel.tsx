import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Map as MapIcon, Save, Maximize2, Minimize2, RotateCcw, Crosshair } from "lucide-react"
import MapCanvas from "./MapCanvas"
import { useSlamMode } from "./contexts/SlamModeContext"
import { useState, useEffect, useCallback, useRef } from "react"
import { API_BASE } from "./config"
import { useSavedMap } from "./hooks/useSavedMap"

type GoToToast = { text: string; tone: "info" | "error" }

type GoToState = {
  state: "idle" | "running"
  target?: { x: number; y: number }
  tolerance?: number
  distance_remaining?: number
  initial_distance?: number
  started_at?: number
  elapsed_s?: number
  success?: boolean
  reason?: string
  tof_distance_mm?: number
  finished_at?: number
  message?: string
  detail?: string
}

/**
 * SLAM Map panel with mode toggles and expand/collapse functionality.
 *
 * Displays the SLAM map canvas and provides controls for:
 * - SLAM mode: Create Map / Localisation
 * - Speed mode: Normal / Creep (only visible in Create Map mode)
 * - Panel expansion: Collapsed (default if saved map exists) / Expanded
 * - Map view toggle: Raw / Post-processed (only in expanded mode with saved map)
 */
export default function SlamMapPanel() {
  const { slamMode, setSlamMode, speedMode, setSpeedMode } = useSlamMode()
  const [isSaving, setIsSaving] = useState(false)
  const [isClearing, setIsClearing] = useState(false)
  const { hasSavedMap, loading, refresh: refreshSavedMap } = useSavedMap()
  const [isExpanded, setIsExpanded] = useState(false)
  const [showProcessedMap, setShowProcessedMap] = useState(false)
  const [isGoToMode, setIsGoToMode] = useState(false)
  const [goToLoading, setGoToLoading] = useState(false)
  const [goToToast, setGoToToast] = useState<GoToToast | null>(null)
  const [goToStatus, setGoToStatus] = useState<GoToState | null>(null)
  const statusTimerRef = useRef<number | null>(null)
  const lastStatusStateRef = useRef<GoToState["state"] | null>(null)
  const userExpandedRef = useRef(false)
  const prevHasSavedMapRef = useRef<boolean | null>(null)

  const clearStatusTimer = useCallback(() => {
    if (statusTimerRef.current !== null) {
      window.clearInterval(statusTimerRef.current)
      statusTimerRef.current = null
    }
  }, [])

  const fetchGoToStatus = useCallback(async (suppressToast = false) => {
    try {
      const response = await fetch(`${API_BASE}/api/slam/goto/status`)
      if (!response.ok) {
        throw new Error(`${response.status} ${response.statusText}`)
      }
      const payload = (await response.json()) as GoToState
      setGoToStatus(payload)

      const prevState = lastStatusStateRef.current
      lastStatusStateRef.current = payload.state

      if (prevState === "running" && payload.state !== "running" && !suppressToast) {
        if (payload.success === true) {
          const target = payload.target
          const targetText =
            target && Number.isFinite(target.x) && Number.isFinite(target.y)
              ? `(${target.x.toFixed(2)}, ${target.y.toFixed(2)})`
              : ""
          const dist =
            typeof payload.distance_remaining === "number" && Number.isFinite(payload.distance_remaining)
              ? payload.distance_remaining.toFixed(2)
              : null
          setGoToToast({
            text: `Go-to complete${targetText ? ` at ${targetText}` : ""}${dist !== null ? `, residual ${dist} m` : ""}`,
            tone: "info",
          })
        } else if (payload.success === false) {
          const reason = payload.reason ? payload.reason.replace(/_/g, " ") : "stopped"
          setGoToToast({
            text: `Go-to ended: ${reason}`,
            tone: "error",
          })
        }
      }
    } catch (error) {
      console.error("Failed to fetch go-to status:", error)
      if (!suppressToast) {
        setGoToToast({
          text: `Failed to read go-to status: ${error instanceof Error ? error.message : "Unknown error"}`,
          tone: "error",
        })
      }
    }
  }, [])

  // Force expanded mode if no saved map exists
  useEffect(() => {
    if (loading) {
      return
    }

    if (prevHasSavedMapRef.current !== hasSavedMap) {
      prevHasSavedMapRef.current = hasSavedMap
      userExpandedRef.current = false
    }

    if (!userExpandedRef.current) {
      setIsExpanded(!hasSavedMap)
    }
  }, [hasSavedMap, loading])

  useEffect(() => {
    if (!goToToast) {
      return
    }
    const timer = window.setTimeout(() => setGoToToast(null), 6000)
    return () => window.clearTimeout(timer)
  }, [goToToast])

  useEffect(() => {
    fetchGoToStatus(true)
    return () => {
      clearStatusTimer()
    }
  }, [clearStatusTimer, fetchGoToStatus])

  useEffect(() => {
    if (goToStatus?.state === "running") {
      if (statusTimerRef.current === null) {
        statusTimerRef.current = window.setInterval(() => {
          fetchGoToStatus(true)
        }, 1000)
      }
    } else {
      clearStatusTimer()
    }
  }, [goToStatus?.state, clearStatusTimer, fetchGoToStatus])

  useEffect(() => {
    if (slamMode !== "localisation") {
      setIsGoToMode(false)
    }
  }, [slamMode])

  useEffect(() => {
    if (!hasSavedMap) {
      setIsGoToMode(false)
    }
  }, [hasSavedMap])

  useEffect(() => {
    if (goToStatus?.state === "running" && isGoToMode) {
      setIsGoToMode(false)
    }
  }, [goToStatus?.state, isGoToMode])

  const handleClearMap = useCallback(async () => {
    if (!confirm("Clear the SLAM map? This will restart the mapping process.")) {
      return
    }

    setIsClearing(true)
    try {
      const response = await fetch(`${API_BASE}/api/slam/clear`, {
        method: "POST",
      })

      if (!response.ok) {
        throw new Error(`Failed to clear map: ${response.statusText}`)
      }

      // Refresh saved map status after clearing
      await refreshSavedMap()
      setSlamMode("create")
      // Wait a moment for the container to restart
      await new Promise(resolve => setTimeout(resolve, 2000))
    } catch (error) {
      console.error("Error clearing map:", error)
      alert("Failed to clear map. Check console for details.")
    } finally {
      setIsClearing(false)
    }
  }, [refreshSavedMap, setSlamMode])

  const handleSaveMap = async () => {
    setIsSaving(true)
    try {
      const response = await fetch(`${API_BASE}/api/slam/save`, {
        method: "POST",
      })
      if (!response.ok) {
        throw new Error("Failed to save map")
      }
      // Refresh saved map status after saving
      await refreshSavedMap()
      // Switch to post-processed view in localisation mode
      if (slamMode === "localisation") {
        setShowProcessedMap(true)
      }
    } catch (error) {
      console.error("Error saving map:", error)
    } finally {
      setIsSaving(false)
    }
  }

  const handleGoToSelect = useCallback(
    async ({ x, y }: { x: number; y: number }) => {
      if (!isGoToMode || goToLoading) {
        return
      }

      setGoToLoading(true)
      try {
        const response = await fetch(`${API_BASE}/api/slam/goto`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ x, y, tolerance: 0.1 }),
        })

        let payload: (GoToState & { message?: string }) | null = null
        try {
          payload = (await response.json()) as GoToState & { message?: string }
        } catch {
          payload = null
        }

        if (!response.ok) {
          const detail =
            (payload && (payload.detail ?? payload.message)) ||
            `${response.status} ${response.statusText}`
          throw new Error(detail)
        }

        const message = payload?.message ?? "Go-to routine started"

        const distance =
          payload && typeof payload.distance_remaining === "number"
            ? payload.distance_remaining
            : null

        const targetX = payload?.target?.x ?? x
        const targetY = payload?.target?.y ?? y

        const format = (value: number) =>
          Number.isFinite(value) ? value.toFixed(2) : value.toString()

        const distanceText = distance !== null ? `, distance ${format(distance)} m` : ""

        setGoToToast({
          text: `${message} target (${format(targetX)}, ${format(targetY)})${distanceText}`,
          tone: "info",
        })
        setGoToStatus(payload)
        if (payload) {
          lastStatusStateRef.current = payload.state
        }
        if (payload?.state === "running") {
          fetchGoToStatus(true)
        }
      } catch (error) {
        const message = error instanceof Error ? error.message : "Unknown error"
        setGoToToast({
          text: `Failed to start go-to: ${message}`,
          tone: "error",
        })
        console.error("Failed to start go-to:", error)
      } finally {
        setGoToLoading(false)
        setIsGoToMode(false)
      }
    },
    [fetchGoToStatus, goToLoading, isGoToMode],
  )

  const handleCancelGoTo = useCallback(async () => {
    try {
      const response = await fetch(`${API_BASE}/api/slam/goto/cancel`, {
        method: "POST",
      })
      let payload: (GoToState & { message?: string }) | null = null
      try {
        payload = (await response.json()) as GoToState & { message?: string }
      } catch {
        payload = null
      }
      if (!response.ok) {
        const detail =
          (payload && (payload.detail ?? payload.message)) || `${response.status} ${response.statusText}`
        throw new Error(detail)
      }
      if (payload) {
        setGoToStatus(payload)
        lastStatusStateRef.current = payload.state
      }
      setGoToToast({
        text: payload?.message ?? "Go-to cancelled",
        tone: "info",
      })
      fetchGoToStatus(true)
    } catch (error) {
      setGoToToast({
        text: `Failed to cancel go-to: ${error instanceof Error ? error.message : "Unknown error"}`,
        tone: "error",
      })
      console.error("Failed to cancel go-to:", error)
    }
  }, [fetchGoToStatus])

  // When switching to localisation with a saved map, default to post-processed view
  useEffect(() => {
    if (slamMode === "localisation" && hasSavedMap) {
      setShowProcessedMap(true)
    } else {
      setShowProcessedMap(false)
    }
  }, [slamMode, hasSavedMap])

  const goToRunning = goToStatus?.state === "running"
  const distanceRemaining =
    typeof goToStatus?.distance_remaining === "number" && Number.isFinite(goToStatus.distance_remaining)
      ? goToStatus.distance_remaining
      : null
  const elapsedSeconds =
    typeof goToStatus?.elapsed_s === "number" && Number.isFinite(goToStatus.elapsed_s)
      ? goToStatus.elapsed_s
      : null
  const targetLabel =
    goToStatus?.target && Number.isFinite(goToStatus.target.x) && Number.isFinite(goToStatus.target.y)
      ? `(${goToStatus.target.x.toFixed(2)}, ${goToStatus.target.y.toFixed(2)})`
      : null

  return (
    <Card
      className={`transition-all duration-300 ease-in-out ${
        isExpanded
          ? "fixed inset-x-4 top-4 bottom-4 z-50 h-auto"
          : "h-[400px] sm:h-[480px] lg:h-[560px]"
      }`}
    >
      {/* Collapsed Mode Header - only visible when NOT expanded */}
      {!isExpanded && (
        <CardHeader>
          <div className="flex items-center justify-between">
            <CardTitle className="flex items-center gap-2 text-base">
              <MapIcon className="h-4 w-4" /> SLAM Map
            </CardTitle>
          </div>
        </CardHeader>
      )}

      {/* Expanded Mode Header - only visible when expanded */}
      {isExpanded && (
        <CardHeader>
          <div className="flex items-center justify-between">
            <CardTitle className="flex items-center gap-2 text-base">
              <MapIcon className="h-4 w-4" /> SLAM Map
            </CardTitle>
            <div className="flex items-center gap-2">
              {/* Raw/Post-processed toggle - only available in localisation mode with saved map */}
              {slamMode === "localisation" && hasSavedMap && (
                <div className="flex items-center gap-1 rounded-md border p-1">
                  <Button
                    size="sm"
                    variant={!showProcessedMap ? "default" : "ghost"}
                    onClick={() => setShowProcessedMap(false)}
                    className="h-7 px-2 text-xs"
                  >
                    Raw
                  </Button>
                  <Button
                    size="sm"
                    variant={showProcessedMap ? "default" : "ghost"}
                    onClick={() => setShowProcessedMap(true)}
                    className="h-7 px-2 text-xs"
                  >
                    Post-processed
                  </Button>
                </div>
              )}

              {/* SLAM Mode Toggle */}
              <div className="flex items-center gap-1 rounded-md border p-1">
                <Button
                  size="sm"
                  variant={slamMode === "create" ? "default" : "ghost"}
                  onClick={() => setSlamMode("create")}
                  className="h-7 px-2 text-xs"
                >
                  Create Map
                </Button>
                <Button
                  size="sm"
                  variant={slamMode === "localisation" ? "default" : "ghost"}
                  onClick={() => setSlamMode("localisation")}
                  className="h-7 px-2 text-xs"
                  disabled={!hasSavedMap}
                >
                  Localisation
                </Button>
              </div>

              {/* Speed Mode Toggle - always visible in expanded mode */}
              <div className="flex items-center gap-1 rounded-md border p-1">
                <Button
                  size="sm"
                  variant={speedMode === "normal" ? "default" : "ghost"}
                  onClick={() => setSpeedMode("normal")}
                  className="h-7 px-2 text-xs"
                >
                  Normal
                </Button>
                <Button
                  size="sm"
                  variant={speedMode === "creep" ? "default" : "ghost"}
                  onClick={() => setSpeedMode("creep")}
                  className="h-7 px-2 text-xs"
                >
                  Creep
                </Button>
              </div>

              {/* Save Map Button - only in Create Map mode */}
              {slamMode === "create" && (
                <Button
                  size="sm"
                  onClick={handleSaveMap}
                  disabled={isSaving}
                  className="h-7 px-2 text-xs"
                >
                  <Save className="h-3 w-3 mr-1" />
                  {isSaving ? "Saving..." : "Save Map"}
                </Button>
              )}

              {/* Clear Map Button */}
              <Button
                size="sm"
                variant="destructive"
                onClick={handleClearMap}
                disabled={isClearing}
                className="h-7 px-2 text-xs"
              >
                <RotateCcw className={`h-3 w-3 mr-1 ${isClearing ? "animate-spin" : ""}`} />
                {isClearing ? "Clearing..." : "Clear Map"}
              </Button>
            </div>
          </div>
        </CardHeader>
      )}

      <CardContent className="h-full relative">
        {goToToast && (
          <div className="pointer-events-none absolute left-1/2 top-4 z-20 -translate-x-1/2">
            <Badge
              variant={goToToast.tone === "error" ? "destructive" : "default"}
              className="px-3 py-1 text-xs"
            >
              {goToToast.text}
            </Badge>
          </div>
        )}

        {slamMode === "localisation" && hasSavedMap && (
          <div className="absolute top-4 right-4 z-20 flex flex-col items-end gap-2">
            {goToRunning && (
              <div className="w-full max-w-xs rounded-md border bg-background/95 p-3 text-xs shadow-lg backdrop-blur">
                <div className="mb-1 font-medium text-muted-foreground">Go-to in progress</div>
                {targetLabel && <div>Target: {targetLabel} m</div>}
                {distanceRemaining !== null && (
                  <div>Distance: {distanceRemaining.toFixed(2)} m</div>
                )}
                {elapsedSeconds !== null && <div>Elapsed: {elapsedSeconds.toFixed(1)} s</div>}
                <div className="mt-2 flex items-center justify-between gap-2">
                  {goToStatus?.tof_distance_mm !== undefined && goToStatus.tof_distance_mm !== null ? (
                    <div className="text-muted-foreground">ToF: {goToStatus.tof_distance_mm} mm</div>
                  ) : (
                    <div className="text-muted-foreground">Monitoring sensors…</div>
                  )}
                  <Button
                    size="sm"
                    variant="destructive"
                    onClick={handleCancelGoTo}
                    className="h-7 px-2 text-xs"
                  >
                    Cancel
                  </Button>
                </div>
              </div>
            )}
            <Button
              size="sm"
              variant={isGoToMode ? "default" : "secondary"}
              onClick={() => {
                if (goToLoading || goToRunning) {
                  return
                }
                setIsGoToMode(prev => !prev)
              }}
              disabled={goToLoading || goToRunning}
              className="h-8"
            >
              <Crosshair className="mr-1 h-3 w-3" />
              {goToLoading ? "Sending..." : isGoToMode ? "Pick target" : "Go to point"}
            </Button>
          </div>
        )}

        <MapCanvas
          showProcessed={showProcessedMap && slamMode === "localisation" && hasSavedMap}
          gotoMode={isGoToMode && slamMode === "localisation" && hasSavedMap}
          onSelectPoint={handleGoToSelect}
        />

        {/* Expand/Collapse Button - bottom right corner */}
        {hasSavedMap && (
          <Button
            size="sm"
            variant="secondary"
            onClick={() => {
              userExpandedRef.current = true
              setIsExpanded(!isExpanded)
            }}
            className="absolute bottom-4 right-4 z-10 h-8 w-8 p-0"
            title={isExpanded ? "Collapse" : "Expand"}
          >
            {isExpanded ? <Minimize2 className="h-4 w-4" /> : <Maximize2 className="h-4 w-4" />}
          </Button>
        )}
      </CardContent>
    </Card>
  )
}
