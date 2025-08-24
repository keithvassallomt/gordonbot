import { useEffect, useState } from "react"
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

  useEffect(() => {
    const down = (e: KeyboardEvent) => {
      switch (e.key) {
        case "Shift": setBoost(true); break
        case "ArrowUp": case "w": case "W": setVec(v => ({ ...v, y: 1 })); break
        case "ArrowDown": case "s": case "S": setVec(v => ({ ...v, y: -1 })); break
        case "ArrowLeft": case "a": case "A": setVec(v => ({ ...v, x: -1 })); break
        case "ArrowRight": case "d": case "D": setVec(v => ({ ...v, x: 1 })); break
      }
    }
    const up = (e: KeyboardEvent) => {
      switch (e.key) {
        case "Shift": setBoost(false); break
        case "ArrowUp": case "w": case "W": setVec(v => ({ ...v, y: 0 })); break
        case "ArrowDown": case "s": case "S": setVec(v => ({ ...v, y: 0 })); break
        case "ArrowLeft": case "a": case "A": setVec(v => ({ ...v, x: 0 })); break
        case "ArrowRight": case "d": case "D": setVec(v => ({ ...v, x: 0 })); break
      }
    }
    window.addEventListener("keydown", down)
    window.addEventListener("keyup", up)
    return () => {
      window.removeEventListener("keydown", down)
      window.removeEventListener("keyup", up)
    }
  }, [])

  return { vec, boost }
}