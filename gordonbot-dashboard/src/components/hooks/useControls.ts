import { useEffect, useRef, useState } from "react"
import { KEY_ACCEL_PER_S, KEY_DECEL_PER_S, KEY_TURN_ACCEL_PER_S, KEY_TURN_DECEL_PER_S } from "@/components/config"
import type { Vec2 } from "../types"

/**
 * Clamp a number to a range.
 *
 * @param n - Input value.
 * @param min - Minimum value (default -1).
 * @param max - Maximum value (default 1).
 * @returns The clamped number.
 * @example
 * clamp(2, -1, 1) // => 1
 */
export function clamp(n: number, min = -1, max = 1) {
  return Math.max(min, Math.min(max, n))
}

/**
 * Convert a joystick vector to tank tread commands.
 *
 * @param v - Normalized joystick vector: x (-1..1 turn), y (-1..1 forward).
 * @returns Object with `left` and `right` speeds in range [-1, 1].
 * @remarks Used for tank-style drive: left = y + x, right = y - x.
 * @example
 * vecToTank({ x: 1, y: 0 }) // => { left: 1, right: -1 }
 */
export function vecToTank(v: Vec2): { left: number; right: number } {
  // v.x: turn (-1..1), v.y: forward (+1 up)
  // left = y + x, right = y - x; then clamp to [-1, 1]
  const left = clamp(v.y + v.x)
  const right = clamp(v.y - v.x)
  return { left, right }
}

/**
 * React hook to capture keyboard input for robot driving.
 *
 * @returns Current drive vector ({@link Vec2}) and boost flag.
 * @remarks
 * - WASD / Arrow keys map to x/y vector.
 * - Shift enables boost mode.
 * - Automatically adds/removes event listeners on mount/unmount.
 * @example
 * const { vec, boost } = useKeyboardDrive();
 * console.log(vec, boost);
 */
export function useKeyboardDrive() {
  const [vec, setVec] = useState<Vec2>({ x: 0, y: 0 })
  const [boost, setBoost] = useState(false)
  // Target vector updated by key events; output vec approaches target smoothly
  const targetRef = useRef<Vec2>({ x: 0, y: 0 })

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      switch (e.key) {
        case "Shift": setBoost(true); break
        case "ArrowUp": case "w": case "W": targetRef.current = { ...targetRef.current, y: 1 }; break
        case "ArrowDown": case "s": case "S": targetRef.current = { ...targetRef.current, y: -1 }; break
        case "ArrowLeft": case "a": case "A": targetRef.current = { ...targetRef.current, x: -1 }; break
        case "ArrowRight": case "d": case "D": targetRef.current = { ...targetRef.current, x: 1 }; break
      }
    }
    const up = (e: KeyboardEvent) => {
      switch (e.key) {
        case "Shift": setBoost(false); break
        case "ArrowUp": case "w": case "W": targetRef.current = { ...targetRef.current, y: 0 }; break
        case "ArrowDown": case "s": case "S": targetRef.current = { ...targetRef.current, y: 0 }; break
        case "ArrowLeft": case "a": case "A": targetRef.current = { ...targetRef.current, x: 0 }; break
        case "ArrowRight": case "d": case "D": targetRef.current = { ...targetRef.current, x: 0 }; break
      }
    }
    window.addEventListener("keydown", down)
    window.addEventListener("keyup", up)
    return () => {
      window.removeEventListener("keydown", down)
      window.removeEventListener("keyup", up)
    }
  }, [])

  // Smoothly approach target at fixed accel/decel rates
  useEffect(() => {
    let raf = 0
    let last = performance.now()

    const step = (now: number) => {
      const dt = Math.max(0, Math.min(0.05, (now - last) / 1000)) // clamp dt to 50ms
      last = now
      setVec((v) => {
        const tx = clamp(targetRef.current.x)
        const ty = clamp(targetRef.current.y)

        const approach = (cur: number, tgt: number, accelPerS: number, decelPerS: number) => {
          const rate = Math.abs(tgt) > Math.abs(cur) ? accelPerS : decelPerS
          const maxDelta = rate * dt
          const delta = clamp(tgt - cur, -maxDelta, maxDelta)
          const next = clamp(cur + delta)
          return Math.abs(next - tgt) < 1e-4 ? tgt : next
        }

        const nx = approach(v.x, tx, KEY_TURN_ACCEL_PER_S, KEY_TURN_DECEL_PER_S)
        const ny = approach(v.y, ty, KEY_ACCEL_PER_S, KEY_DECEL_PER_S)
        return (nx === v.x && ny === v.y) ? v : { x: nx, y: ny }
      })
      raf = requestAnimationFrame(step)
    }
    raf = requestAnimationFrame(step)
    return () => cancelAnimationFrame(raf)
  }, [])

  return { vec, boost }
}

/**
 * Smooth an input vector toward a target vector using per-axis accel/decel.
 *
 * @param source - The raw input vector to follow.
 * @param params - Optional per-axis acceleration/deceleration rates (units/s).
 * @returns Smoothed output vector.
 */
export function useSmoothedVec(
  source: Vec2,
  params?: { accelX?: number; decelX?: number; accelY?: number; decelY?: number }
) {
  const { accelX = KEY_ACCEL_PER_S, decelX = KEY_DECEL_PER_S, accelY = KEY_ACCEL_PER_S, decelY = KEY_DECEL_PER_S } = params || {}
  const [vec, setVec] = useState<Vec2>(source)
  const targetRef = useRef<Vec2>(source)

  // Track latest source
  useEffect(() => { targetRef.current = source }, [source.x, source.y])

  useEffect(() => {
    let raf = 0
    let last = performance.now()
    const step = (now: number) => {
      const dt = Math.max(0, Math.min(0.05, (now - last) / 1000))
      last = now
      setVec((v) => {
        const tx = clamp(targetRef.current.x)
        const ty = clamp(targetRef.current.y)
        const approach = (cur: number, tgt: number, acc: number, dec: number) => {
          const rate = Math.abs(tgt) > Math.abs(cur) ? acc : dec
          const maxDelta = rate * dt
          const delta = clamp(tgt - cur, -maxDelta, maxDelta)
          const next = clamp(cur + delta)
          return Math.abs(next - tgt) < 1e-4 ? tgt : next
        }
        const nx = approach(v.x, tx, accelX, decelX)
        const ny = approach(v.y, ty, accelY, decelY)
        return (nx === v.x && ny === v.y) ? v : { x: nx, y: ny }
      })
      raf = requestAnimationFrame(step)
    }
    raf = requestAnimationFrame(step)
    return () => cancelAnimationFrame(raf)
  }, [accelX, decelX, accelY, decelY])

  return vec
}
