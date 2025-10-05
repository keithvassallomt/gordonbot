import { useEffect, useRef } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Radar, AlertCircle } from "lucide-react"

import { useLidar } from "./hooks/useLidar"

/**
 * LidarPanel - 2D polar visualization of LIDAR point cloud data
 *
 * Displays LIDAR scan data on a canvas with:
 * - Robot at center
 * - Points drawn as dots in polar coordinates
 * - Color-coded by distance (close = red, far = green)
 * - Real-time updates via WebSocket
 */
export default function LidarPanel() {
  const { scan, status, errorMessage, reconnecting } = useLidar()
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext("2d")
    if (!ctx) return

    // Set canvas size to match display size
    const rect = canvas.getBoundingClientRect()
    const dpr = window.devicePixelRatio || 1
    canvas.width = rect.width * dpr
    canvas.height = rect.height * dpr
    ctx.scale(dpr, dpr)

    // Clear canvas
    ctx.fillStyle = "#000000"
    ctx.fillRect(0, 0, rect.width, rect.height)

    // Drawing parameters
    const centerX = rect.width / 2
    const centerY = rect.height / 2
    const maxRadius = Math.min(centerX, centerY) * 0.9
    const maxDistance = 12000 // 12m in mm (RPLIDAR C1 max range)

    // Draw range circles (1m, 3m, 6m, 9m, 12m)
    ctx.strokeStyle = "#333333"
    ctx.lineWidth = 1
    for (const distM of [1, 3, 6, 9, 12]) {
      const r = (distM / 12) * maxRadius
      ctx.beginPath()
      ctx.arc(centerX, centerY, r, 0, Math.PI * 2)
      ctx.stroke()
    }

    // Draw cardinal directions
    ctx.strokeStyle = "#444444"
    ctx.lineWidth = 1
    ctx.setLineDash([5, 5])

    // Forward (0°)
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX, centerY - maxRadius)
    ctx.stroke()

    // Right (90°)
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX + maxRadius, centerY)
    ctx.stroke()

    // Back (180°)
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX, centerY + maxRadius)
    ctx.stroke()

    // Left (270°)
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX - maxRadius, centerY)
    ctx.stroke()

    ctx.setLineDash([])

    // Draw labels
    ctx.fillStyle = "#666666"
    ctx.font = "12px sans-serif"
    ctx.textAlign = "center"
    ctx.textBaseline = "middle"

    ctx.fillText("0°", centerX, centerY - maxRadius - 15)
    ctx.fillText("90°", centerX + maxRadius + 20, centerY)
    ctx.fillText("180°", centerX, centerY + maxRadius + 15)
    ctx.fillText("270°", centerX - maxRadius - 20, centerY)

    // Draw distance labels
    ctx.fillStyle = "#555555"
    ctx.font = "10px sans-serif"
    for (const distM of [3, 6, 9, 12]) {
      const r = (distM / 12) * maxRadius
      ctx.fillText(`${distM}m`, centerX + 5, centerY - r)
    }

    // Draw LIDAR points if available
    if (scan && scan.points && scan.points.length > 0) {
      for (const point of scan.points) {
        const { angle, distance_mm, quality } = point

        // Skip invalid points
        if (distance_mm <= 0 || distance_mm > maxDistance) continue
        if (quality < 10) continue  // Filter low-quality points

        // Convert to canvas coordinates
        // LIDAR angle: 0° = forward, increases clockwise
        // Canvas: 0° = right, increases clockwise, so rotate by -90°
        const angleRad = ((angle - 90) * Math.PI) / 180
        const r = (distance_mm / maxDistance) * maxRadius

        const x = centerX + r * Math.cos(angleRad)
        const y = centerY + r * Math.sin(angleRad)

        // Color by distance (close = red, far = green)
        const distNorm = Math.min(distance_mm / maxDistance, 1.0)
        const red = Math.floor((1 - distNorm) * 255)
        const green = Math.floor(distNorm * 255)
        const blue = 0

        // Draw point
        ctx.fillStyle = `rgb(${red}, ${green}, ${blue})`
        ctx.beginPath()
        ctx.arc(x, y, 2, 0, Math.PI * 2)
        ctx.fill()
      }
    }

    // Draw robot at center
    ctx.fillStyle = "#00aaff"
    ctx.beginPath()
    ctx.arc(centerX, centerY, 5, 0, Math.PI * 2)
    ctx.fill()

    // Draw robot direction indicator (forward)
    ctx.strokeStyle = "#00aaff"
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX, centerY - 15)
    ctx.stroke()

  }, [scan])

  // Render status/error messages
  const renderStatus = () => {
    if (errorMessage) {
      return (
        <div className="absolute inset-0 flex items-center justify-center bg-black/50">
          <div className="flex items-center gap-2 text-red-400">
            <AlertCircle className="h-5 w-5" />
            <span>{errorMessage}</span>
          </div>
        </div>
      )
    }

    if (status === "connecting" || reconnecting) {
      return (
        <div className="absolute inset-0 flex items-center justify-center bg-black/50">
          <div className="flex items-center gap-2 text-yellow-400">
            <Radar className="h-5 w-5 animate-pulse" />
            <span>{reconnecting ? "Reconnecting..." : "Connecting..."}</span>
          </div>
        </div>
      )
    }

    if (status === "disconnected") {
      return (
        <div className="absolute inset-0 flex items-center justify-center bg-black/50">
          <div className="flex items-center gap-2 text-gray-400">
            <AlertCircle className="h-5 w-5" />
            <span>LIDAR disconnected</span>
          </div>
        </div>
      )
    }

    if (!scan || !scan.points || scan.points.length === 0) {
      return (
        <div className="absolute inset-0 flex items-center justify-center bg-black/50">
          <div className="flex items-center gap-2 text-gray-400">
            <Radar className="h-5 w-5 animate-spin" />
            <span>Waiting for scan data...</span>
          </div>
        </div>
      )
    }

    return null
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between text-base">
          <span className="flex items-center gap-2">
            <Radar className="h-4 w-4" />
            LIDAR Point Cloud
          </span>
          {scan && (
            <span className="text-sm font-normal text-muted-foreground">
              {scan.points?.length || 0} points @ {scan.scan_rate_hz.toFixed(1)} Hz
            </span>
          )}
        </CardTitle>
      </CardHeader>
      <CardContent className="relative p-0">
        <div className="relative aspect-square w-full">
          <canvas
            ref={canvasRef}
            className="h-full w-full"
            style={{ imageRendering: "crisp-edges" }}
          />
          {renderStatus()}
        </div>
      </CardContent>
    </Card>
  )
}
