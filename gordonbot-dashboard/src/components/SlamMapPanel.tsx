import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Map as MapIcon } from "lucide-react"
import MapCanvas from "./MapCanvas"
import { useSlamMode } from "./contexts/SlamModeContext"

/**
 * SLAM Map panel with mode toggles.
 *
 * Displays the SLAM map canvas and provides controls for:
 * - SLAM mode: Create Map / Localisation
 * - Speed mode: Normal / Creep (only visible in Create Map mode)
 */
export default function SlamMapPanel() {
  const { slamMode, setSlamMode, speedMode, setSpeedMode } = useSlamMode()

  return (
    <Card className="h-[400px] sm:h-[480px] lg:h-[560px]">
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="flex items-center gap-2 text-base">
            <MapIcon className="h-4 w-4" /> SLAM Map
          </CardTitle>
          <div className="flex items-center gap-2">
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
              >
                Localisation
              </Button>
            </div>

            {/* Speed Mode Toggle - only visible in Create Map mode */}
            {slamMode === "create" && (
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
            )}
          </div>
        </div>
        {slamMode === "localisation" && (
          <div className="mt-2">
            <Badge variant="secondary" className="text-xs">
              Localisation mode (not yet implemented)
            </Badge>
          </div>
        )}
      </CardHeader>
      <CardContent className="h-full">
        <MapCanvas />
      </CardContent>
    </Card>
  )
}
