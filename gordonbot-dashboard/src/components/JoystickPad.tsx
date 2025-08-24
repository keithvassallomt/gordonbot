import React, { useRef, useState } from "react"
import { Joystick } from "lucide-react"
import type { Vec2 } from "@/components/types"
import { clamp } from "@/components/hooks/useControls"


/**
 * On-screen joystick pad component.
 *
 * Renders a circular touch/mouse control that translates pointer movement
 * into a normalized vector ({@link Vec2}).
 *
 * @param onChange - Callback fired with the current joystick vector on move.
 * @returns JSX circular joystick UI with inner handle.
 *
 * @remarks
 * - Pointer locked to element until release.
 * - Output vector is normalized to radius 1 (clamped).
 * - Updates live on pointer move; resets to (0,0) on release.
 * - Fully responsive, styled with Tailwind and shadcn/ui conventions.
 *
 * @example
 * ```tsx
 * <JoystickPad onChange={(vec) => console.log(vec)} />
 * ```
 */
export default function JoystickPad({ onChange }: { onChange: (v: Vec2) => void }) {
  const ref = useRef<HTMLDivElement | null>(null)
  const [pos, setPos] = useState<Vec2>({ x: 0, y: 0 })
  const [active, setActive] = useState(false)

  /**
   * Convert pointer coordinates to normalized joystick vector.
   * Updates internal state and fires onChange callback.
   */
  const handle = (clientX: number, clientY: number) => {
    const el = ref.current!
    const rect = el.getBoundingClientRect()
    const cx = rect.left + rect.width / 2
    const cy = rect.top + rect.height / 2
    const dx = clientX - cx
    const dy = clientY - cy
    const r = Math.min(rect.width, rect.height) / 2
    let x = clamp(dx / r)
    let y = clamp(-dy / r)
    const mag = Math.hypot(x, y)
    if (mag > 1) { x /= mag; y /= mag }
    setPos({ x, y })
    onChange({ x, y })
  }

  const onPointerDown = (e: React.PointerEvent) => {
    ;(e.target as Element).setPointerCapture(e.pointerId)
    setActive(true)
    handle(e.clientX, e.clientY)
  }
  const onPointerMove = (e: React.PointerEvent) => { if (active) handle(e.clientX, e.clientY) }
  /**
   * Reset joystick to neutral (0,0) and fire callback.
   */
  const end = () => { setActive(false); setPos({ x: 0, y: 0 }); onChange({ x: 0, y: 0 }) }

  const dotStyle = { transform: `translate(${pos.x * 40}%, ${-pos.y * 40}%)` } as React.CSSProperties

  return (
    <div
      ref={ref}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={end}
      onPointerCancel={end}
      className="relative mx-auto aspect-square w-56 select-none rounded-full border bg-gradient-to-b from-muted to-background shadow-inner"
    >
      <div className="absolute inset-3 rounded-full border-2 border-dashed opacity-50" />
      <div className="absolute left-1/2 top-1/2 h-1 w-1 -translate-x-1/2 -translate-y-1/2 rounded-full bg-primary/60" />
      <div
        className="pointer-events-none absolute left-1/2 top-1/2 flex h-12 w-12 -translate-x-1/2 -translate-y-1/2 items-center justify-center rounded-full bg-primary/20 backdrop-blur transition-transform"
        style={dotStyle}
      >
        <Joystick className="h-6 w-6" />
      </div>
    </div>
  )
}