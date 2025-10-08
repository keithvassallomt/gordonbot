import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Map as MapIcon, Save, Maximize2, Minimize2, RotateCcw } from "lucide-react"
import MapCanvas from "./MapCanvas"
import { useSlamMode } from "./contexts/SlamModeContext"
import { useState, useEffect, useCallback, useRef } from "react"
import { API_BASE } from "./config"
import { useSavedMap } from "./hooks/useSavedMap"

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
  const userExpandedRef = useRef(false)
  const prevHasSavedMapRef = useRef<boolean | null>(null)

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
      // Wait a moment for the container to restart
      await new Promise(resolve => setTimeout(resolve, 2000))
    } catch (error) {
      console.error("Error clearing map:", error)
      alert("Failed to clear map. Check console for details.")
    } finally {
      setIsClearing(false)
    }
  }, [refreshSavedMap])

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

  // When switching to localisation with a saved map, default to post-processed view
  useEffect(() => {
    if (slamMode === "localisation" && hasSavedMap) {
      setShowProcessedMap(true)
    } else {
      setShowProcessedMap(false)
    }
  }, [slamMode, hasSavedMap])

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
        <MapCanvas showProcessed={showProcessedMap && slamMode === "localisation" && hasSavedMap} />

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
