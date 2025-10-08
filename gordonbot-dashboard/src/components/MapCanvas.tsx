import React, { useCallback, useEffect, useRef, useState } from "react"
import { Badge } from "@/components/ui/badge"
import { Crosshair, Move, ZoomIn, ZoomOut } from "lucide-react"
import { useSLAM } from "@/components/hooks/useSLAM"
import type { SlamMapMessage } from "@/components/types"
import { API_BASE } from "@/components/config"

interface MapCanvasProps {
  showProcessed?: boolean
  gotoMode?: boolean
  onSelectPoint?: (point: { x: number; y: number }) => void
}

/**
 * 2D vector used for map panning offsets.
 */
type Vec2 = { x: number; y: number }
/**
 * Clamp a numeric value to the provided range.
 * @param n - Number to clamp.
 * @param min - Minimum value (default -1).
 * @param max - Maximum value (default 1).
 * @returns The clamped number.
 */
function clamp(n: number, min = -1, max = 1) {
  return Math.max(min, Math.min(max, n))
}

const METERS_TO_PX = 100

/**
 * Map canvas placeholder with pan & zoom.
 *
 * Renders a grid and a simple robot pose marker. Supports smooth panning
 * via pointer drag and zooming via mouse wheel, with device‑pixel‑ratio aware
 * rendering and responsive resizing.
 *
 * @param showProcessed - If true, displays the post-processed saved map instead of raw live map
 *
 * @returns JSX wrapper with a canvas and on‑screen usage badges.
 *
 * @remarks
 * - Wheel zoom is exponential for smoothness and clamped to [0.25, 4].
 * - Panning updates an (x,y) offset in CSS pixels.
 * - Resizes to its parent and scales for high‑DPI displays.
 * - Replace the placeholder draw routine with your SLAM/occupancy map later.
 *
 * @example
 * ```tsx
 * <MapCanvas showProcessed={false} />
 * ```
 */
export default function MapCanvas({ showProcessed = false, gotoMode = false, onSelectPoint }: MapCanvasProps) {
  const containerRef = useRef<HTMLDivElement | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const [scale, setScale] = useState(1)
  const [offset, setOffset] = useState<Vec2>({ x: 0, y: 0 })
  const hasUserAdjustedRef = useRef(false)
  const dragRef = useRef<{ dragging: boolean; x: number; y: number }>({
    dragging: false,
    x: 0,
    y: 0,
  })

  // SLAM data hook
  const { map, pose, status } = useSLAM()

  // Processed map occupancy grid state
  const [processedMapData, setProcessedMapData] = useState<SlamMapMessage | null>(null)
  const [processedMapLoading, setProcessedMapLoading] = useState(false)

  // Load processed map grid when requested
  useEffect(() => {
    if (!showProcessed) {
      return
    }

    let cancelled = false

    const loadProcessedMap = async () => {
      setProcessedMapLoading(true)
      try {
        const response = await fetch(`${API_BASE}/api/slam/map/processed-grid?t=${Date.now()}`)
        if (!response.ok) {
          throw new Error(`Failed to load processed map: ${response.statusText}`)
        }
        const data = await response.json()
        if (!cancelled) {
          setProcessedMapData(data as SlamMapMessage)
        }
      } catch (error) {
        console.error("Failed to load processed map grid:", error)
        if (!cancelled) {
          setProcessedMapData(null)
        }
      } finally {
        if (!cancelled) {
          setProcessedMapLoading(false)
        }
      }
    }

    // Avoid refetching if we already have data
    if (!processedMapData) {
      loadProcessedMap()
    } else {
      setProcessedMapLoading(false)
    }

    return () => {
      cancelled = true
    }
  }, [showProcessed, processedMapData])

  /**
   * Draw the SLAM occupancy grid map.
   */
  const drawSlamMap = useCallback(
    (
      ctx: CanvasRenderingContext2D,
      mapData: SlamMapMessage,
      toCanvasX: (x: number) => number,
      toCanvasY: (y: number) => number,
    ) => {
      const { width, height, resolution, origin, data } = mapData
      const cellSize = resolution * METERS_TO_PX
      const half = cellSize / 2

      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const idx = y * width + x
          const value = data[idx]

          let color: string
          if (value < 0) {
            color = "#808080"
          } else if (value === 0) {
            color = "#ffffff"
          } else {
            const intensity = Math.floor(255 * (1 - value / 100))
            color = `rgb(${intensity},${intensity},${intensity})`
          }

          ctx.fillStyle = color
          const worldX = origin.x + x * resolution
          const worldY = origin.y + y * resolution
          const px = toCanvasX(worldX)
          const py = toCanvasY(worldY)
          ctx.fillRect(px - half, py - half, cellSize, cellSize)
        }
      }
    },
    [],
  )

  /**
   * Draw the current frame: grid, SLAM map, and robot pose.
   * Applies current pan (offset) and zoom (scale) while keeping the robot centred.
   */
  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    const dpr = window.devicePixelRatio || 1
    const width = canvas.width / dpr
    const height = canvas.height / dpr

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    ctx.clearRect(0, 0, width, height)

    const robotWorldX = pose?.x ?? 0
    const robotWorldY = pose?.y ?? 0
    const robotTheta = pose?.theta ?? 0

    const toCanvasX = (x: number) => (x - robotWorldX) * METERS_TO_PX
    const toCanvasY = (y: number) => -(y - robotWorldY) * METERS_TO_PX

    ctx.save()
    ctx.translate(width / 2 + offset.x, height / 2 + offset.y)
    ctx.scale(scale, scale)

    // Grid
    const grid = 100 // px per meter at scale 1
    ctx.strokeStyle =
      getComputedStyle(document.documentElement).getPropertyValue("--muted-foreground") || "#ccc"
    ctx.globalAlpha = 0.2
    ctx.lineWidth = 1 / scale
    for (let x = -2000; x <= 2000; x += grid) {
      ctx.beginPath()
      ctx.moveTo(x, -2000)
      ctx.lineTo(x, 2000)
      ctx.stroke()
    }
    for (let y = -2000; y <= 2000; y += grid) {
      ctx.beginPath()
      ctx.moveTo(-2000, y)
      ctx.lineTo(2000, y)
      ctx.stroke()
    }
    ctx.globalAlpha = 1

    const mapToDraw = showProcessed && processedMapData ? processedMapData : map

    if (mapToDraw) {
      drawSlamMap(ctx, mapToDraw, toCanvasX, toCanvasY)
    }

    // Robot pose at origin
    ctx.save()
    ctx.rotate(robotTheta)
    ctx.fillStyle = "#10b981" // emerald-ish
    ctx.beginPath()
    ctx.moveTo(20, 0)      // front
    ctx.lineTo(-10, 12)    // back left
    ctx.lineTo(-10, -12)   // back right
    ctx.closePath()
    ctx.fill()

    ctx.strokeStyle = "#10b981"
    ctx.lineWidth = 3 / scale
    ctx.beginPath()
    ctx.moveTo(20, 0)
    ctx.lineTo(35, 0)
    ctx.stroke()
    ctx.restore()

    ctx.restore()
  }, [scale, offset, map, pose, drawSlamMap, showProcessed, processedMapData])

  useEffect(() => {
    const canvas = canvasRef.current!
    const resize = () => {
      const parent = canvas.parentElement!
      const dpr = window.devicePixelRatio || 1
      canvas.width = parent.clientWidth * dpr
      canvas.height = parent.clientHeight * dpr
      canvas.style.width = "100%"
      canvas.style.height = "100%"
      const ctx = canvas.getContext("2d")!
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      draw()
    }
    resize()
    window.addEventListener("resize", resize)
    return () => window.removeEventListener("resize", resize)
  }, [draw])

  useEffect(() => {
    draw()
  }, [draw])

  useEffect(() => {
    if (!map) {
      return
    }
    if (hasUserAdjustedRef.current) {
      return
    }
    setOffset({ x: 0, y: 0 })
  }, [map])

  useEffect(() => {
    if (!gotoMode) {
      dragRef.current.dragging = false
    }
  }, [gotoMode])

  const handleGoToSelection = useCallback(
    (event: React.PointerEvent<HTMLCanvasElement>) => {
      if (!gotoMode || !onSelectPoint) {
        return
      }
      if (!pose) {
        console.warn("Go-to selection ignored: SLAM pose unavailable")
        return
      }
      const canvas = canvasRef.current
      if (!canvas) {
        return
      }
      if (event.button !== 0) {
        return
      }

      event.preventDefault()
      event.stopPropagation()

      const rect = canvas.getBoundingClientRect()
      const px = event.clientX - rect.left
      const py = event.clientY - rect.top

      const relX = (px - rect.width / 2 - offset.x) / scale
      const relY = (py - rect.height / 2 - offset.y) / scale

      const worldX = relX / METERS_TO_PX + pose.x
      const worldY = -relY / METERS_TO_PX + pose.y

      onSelectPoint({ x: worldX, y: worldY })
    },
    [gotoMode, onSelectPoint, offset.x, offset.y, pose, scale],
  )


  /**
   * Mouse‑wheel zoom handler. Uses an exponential factor and clamps the scale.
   * Bound via a non-passive listener so the page does not scroll.
   */
  const handleWheel = useCallback((event: WheelEvent) => {
    const delta = -event.deltaY
    const factor = Math.exp(delta * 0.001)
    hasUserAdjustedRef.current = true
    setScale((s) => clamp(s * factor, 0.25, 4))
  }, [])

  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    const listener = (event: WheelEvent) => {
      event.preventDefault()
      event.stopPropagation()
      handleWheel(event)
    }

   container.addEventListener("wheel", listener, { passive: false })
    return () => {
      container.removeEventListener("wheel", listener)
    }
  }, [handleWheel])

  /**
   * Begin panning: remember the initial pointer position in screen pixels.
   */
  const onPointerDown = (e: React.PointerEvent) => {
    if (gotoMode) {
      handleGoToSelection(e as React.PointerEvent<HTMLCanvasElement>)
      dragRef.current.dragging = false
      return
    }
    hasUserAdjustedRef.current = true
    dragRef.current = { dragging: true, x: e.clientX, y: e.clientY }
  }
  /**
   * Continue panning while dragging: accumulate delta into the offset.
   */
  const onPointerMove = (e: React.PointerEvent) => {
    if (gotoMode) {
      return
    }
    if (!dragRef.current.dragging) return
    const dx = e.clientX - dragRef.current.x
    const dy = e.clientY - dragRef.current.y
    dragRef.current.x = e.clientX
    dragRef.current.y = e.clientY
    setOffset((o) => ({ x: o.x + dx, y: o.y + dy }))
  }
  /**
   * End panning.
   */
  const onPointerUp = () => {
    dragRef.current.dragging = false
  }

  return (
    <div ref={containerRef} className="relative h-full w-full overflow-hidden rounded-xl border">
      <canvas
        ref={canvasRef}
        className={`h-full w-full touch-none ${gotoMode ? "cursor-crosshair" : "cursor-grab"}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
      />
      <div className="pointer-events-none absolute left-2 top-2 flex items-center gap-2 text-xs opacity-80">
        <Badge variant="secondary" className="flex items-center gap-1">
          <Move className="h-3 w-3" /> Drag
        </Badge>
        <Badge variant="secondary" className="flex items-center gap-1">
          <ZoomIn className="h-3 w-3" />/ <ZoomOut className="h-3 w-3" /> Wheel
        </Badge>
        <Badge
          variant={status === "connected" ? "default" : "destructive"}
          className="flex items-center gap-1"
        >
          <div className={`h-2 w-2 rounded-full ${status === "connected" ? "bg-green-500" : "bg-red-500"}`} />
          SLAM {status}
        </Badge>
        {gotoMode && (
          <Badge variant="default" className="flex items-center gap-1">
            <Crosshair className="h-3 w-3" />
            Click map to set destination
          </Badge>
        )}
        {processedMapLoading && (
          <Badge variant="secondary" className="flex items-center gap-1">
            Loading processed map...
          </Badge>
        )}
      </div>
    </div>
  )
}
