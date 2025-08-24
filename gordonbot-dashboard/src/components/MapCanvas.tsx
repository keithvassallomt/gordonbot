import React, { useCallback, useEffect, useRef, useState } from "react"
import { Badge } from "@/components/ui/badge"
import { Move, ZoomIn, ZoomOut } from "lucide-react"

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

/**
 * Map canvas placeholder with pan & zoom.
 *
 * Renders a grid and a simple robot pose marker. Supports smooth panning
 * via pointer drag and zooming via mouse wheel, with device‑pixel‑ratio aware
 * rendering and responsive resizing.
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
 * <MapCanvas />
 * ```
 */
export default function MapCanvas() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const [scale, setScale] = useState(1)
  const [offset, setOffset] = useState<Vec2>({ x: 0, y: 0 })
  const dragRef = useRef<{ dragging: boolean; x: number; y: number }>({
    dragging: false,
    x: 0,
    y: 0,
  })

  /**
   * Draw the current frame: grid, placeholder map block, and robot pose.
   * Applies current pan (offset) and zoom (scale).
   */
  // draw simple grid + robot pose placeholder
  const draw = useCallback(() => {
    const canvas = canvasRef.current!
    const ctx = canvas.getContext("2d")!
    const { width, height } = canvas
    ctx.clearRect(0, 0, width, height)

    ctx.save()
    ctx.translate(width / 2 + offset.x, height / 2 + offset.y)
    ctx.scale(scale, scale)

    // Grid
    const grid = 40 // px per cell at scale 1
    ctx.strokeStyle =
      getComputedStyle(document.documentElement).getPropertyValue("--muted-foreground") || "#ccc"
    ctx.globalAlpha = 0.4
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

    // Map placeholder block
    ctx.fillStyle = "#8883"
    ctx.fillRect(-300, -200, 600, 400)

    // Robot pose
    ctx.fillStyle = "#10b981" // emerald-ish
    ctx.beginPath()
    ctx.moveTo(0, -20)
    ctx.lineTo(12, 20)
    ctx.lineTo(-12, 20)
    ctx.closePath()
    ctx.fill()

    ctx.restore()
  }, [scale, offset])

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

  /**
   * Mouse‑wheel zoom handler. Uses an exponential factor and clamps the scale.
   */
  const onWheel = (e: React.WheelEvent) => {
    e.preventDefault()
    const delta = -e.deltaY
    const factor = Math.exp(delta * 0.001)
    setScale((s) => clamp(s * factor, 0.25, 4))
  }

  /**
   * Begin panning: remember the initial pointer position in screen pixels.
   */
  const onPointerDown = (e: React.PointerEvent) => {
    dragRef.current = { dragging: true, x: e.clientX, y: e.clientY }
  }
  /**
   * Continue panning while dragging: accumulate delta into the offset.
   */
  const onPointerMove = (e: React.PointerEvent) => {
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
    <div className="relative h-full w-full overflow-hidden rounded-xl border">
      <canvas
        ref={canvasRef}
        className="h-full w-full touch-none"
        onWheel={onWheel}
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
      </div>
    </div>
  )
}