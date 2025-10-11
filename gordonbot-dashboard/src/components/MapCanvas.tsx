import React, { useCallback, useEffect, useRef, useState } from "react"
import { Badge } from "@/components/ui/badge"
import { Crosshair, Move, ZoomIn, ZoomOut } from "lucide-react"
import { useSLAM } from "@/components/hooks/useSLAM"
import type { SlamMapMessage } from "@/components/types"

interface MapCanvasProps {
  gotoMode?: boolean
  onSelectPoint?: (point: { x: number; y: number }) => void
}

type Vec2 = { x: number; y: number }

const METERS_TO_PX = 100

function clamp(n: number, min = -1, max = 1) {
  return Math.max(min, Math.min(max, n))
}

export default function MapCanvas({ gotoMode = false, onSelectPoint }: MapCanvasProps) {
  const containerRef = useRef<HTMLDivElement | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const [scale, setScale] = useState(1)
  const [offset, setOffset] = useState<Vec2>({ x: 0, y: 0 })
  const hasUserAdjustedRef = useRef(false)
  const dragRef = useRef<{ dragging: boolean; x: number; y: number }>({ dragging: false, x: 0, y: 0 })

  const { map, pose, status } = useSLAM()

  const drawSlamMap = useCallback(
    (ctx: CanvasRenderingContext2D, mapData: SlamMapMessage, toCanvasX: (x: number) => number, toCanvasY: (y: number) => number) => {
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

    const grid = 100
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue("--muted-foreground") || "#ccc"
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

    if (map) {
      drawSlamMap(ctx, map, toCanvasX, toCanvasY)
    }

    ctx.save()
    ctx.rotate(-robotTheta)  // Negate because IMU is mounted upside down (rotation direction inverted)
    ctx.fillStyle = "#10b981"
    ctx.beginPath()
    ctx.moveTo(20, 0)
    ctx.lineTo(-10, 12)
    ctx.lineTo(-10, -12)
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
  }, [scale, offset, map, pose, drawSlamMap])

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

  const onPointerDown = (e: React.PointerEvent) => {
    if (gotoMode) {
      handleGoToSelection(e as React.PointerEvent<HTMLCanvasElement>)
      dragRef.current.dragging = false
      return
    }
    hasUserAdjustedRef.current = true
    dragRef.current = { dragging: true, x: e.clientX, y: e.clientY }
  }

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
            Click to target
          </Badge>
        )}
      </div>
    </div>
  )
}
