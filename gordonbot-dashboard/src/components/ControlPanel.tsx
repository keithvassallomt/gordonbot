import { useEffect, useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Separator } from "@/components/ui/separator"
import { Gamepad2 } from "lucide-react"
import JoystickPad from "@/components/JoystickPad"
import KV from "@/components/KV"

import { DEADMAN_MS, COMMAND_HZ, MAX_SPEED, BOOST_MULTIPLIER, JOY_ACCEL_PER_S, JOY_DECEL_PER_S, JOY_TURN_ACCEL_PER_S, JOY_TURN_DECEL_PER_S } from "@/components/config"
import type { ControlTransport, ControlCommand, Vec2 } from "@/components/types"
import { clamp, useKeyboardDrive, useSmoothedVec, vecToTank } from "@/components/hooks/useControls"

/**
 * Drive control panel (tank mode).
 *
 * Renders an on-screen joystick, listens to keyboard (WASD/Arrows + Shift),
 * merges both inputs, applies dead‑man timeout and rate limiting, and sends
 * normalized tank commands over the provided transport.
 *
 * @param transport - {@link ControlTransport} with status and connect/disconnect/send methods.
 * @returns JSX card with joystick, status readouts, and connect controls.
 *
 * @remarks
 * - Vector math: {@link vecToTank} converts (x,y) → (left,right) in [-1,1].
 * - Safety: dead‑man timer ({@link DEADMAN_MS}) zeros outputs when idle.
 * - Rate: command loop runs at {@link COMMAND_HZ} Hz using setTimeout.
 * - Boost: holding Shift increases output by {@link BOOST_MULTIPLIER}× (clamped).
 * - Fully responsive layout; keyboard shortcuts are displayed below the controls.
 *
 * @example
 * ```tsx
 * <ControlPanel transport={useControlTransport()} />
 * ```
 */
export default function ControlPanel({ transport }: { transport: ControlTransport }) {
  const [joyVec, setJoyVec] = useState<Vec2>({ x: 0, y: 0 })
  const joySmoothed = useSmoothedVec(joyVec, {
    accelX: JOY_TURN_ACCEL_PER_S,
    decelX: JOY_TURN_DECEL_PER_S,
    accelY: JOY_ACCEL_PER_S,
    decelY: JOY_DECEL_PER_S,
  })
  const { vec: keyVec, boost } = useKeyboardDrive()
  const [lastCmd, setLastCmd] = useState<ControlCommand | null>(null)
  const [enabled] = useState(true)

  /**
   * Combined joystick + keyboard vector, clamped to [-1, 1] per axis.
   */
  const merged: Vec2 = useMemo(() => ({
    x: clamp(joySmoothed.x + keyVec.x, -1, 1),
    y: clamp(joySmoothed.y + keyVec.y, -1, 1),
  }), [joySmoothed, keyVec])

  /**
   * Update the last-input timestamp whenever the merged vector changes.
   * Used by the dead‑man check in the command loop.
   */
  const lastInputRef = useRef<number>(performance.now())
  useEffect(() => { lastInputRef.current = performance.now() }, [merged.x, merged.y])

  /**
   * Command loop: computes tank outputs and sends them at a fixed rate.
   * Applies dead‑man, boost multiplier, and clamping before sending.
   * Cleans up the timer on unmount.
   */
  useEffect(() => {
    let id: number
    const tick = () => {
      const now = performance.now()
      // Treat a held non-zero command as active input to avoid tripping
      // the client dead-man while a key is held or joystick is steady.
      if (Math.hypot(merged.x, merged.y) > 1e-3) {
        lastInputRef.current = now
      }
      const inactive = now - lastInputRef.current > DEADMAN_MS
      const base = vecToTank(merged)
      const m = (boost ? BOOST_MULTIPLIER : 1) * MAX_SPEED
      const left = enabled && !inactive ? clamp(base.left * m, -1, 1) : 0
      const right = enabled && !inactive ? clamp(base.right * m, -1, 1) : 0
      const cmd: ControlCommand = { left, right, ts: Date.now() }
      transport.send(cmd)
      setLastCmd(cmd)
      id = window.setTimeout(tick, 1000 / COMMAND_HZ)
    }
    tick()
    return () => { clearTimeout(id) }
  }, [merged, boost, enabled, transport])

  /**
   * Immediately zero the on‑screen joystick vector (keyboard state may still apply).
   */
  const stopAll = () => { setJoyVec({ x: 0, y: 0 }) }

  return (
    <Card>
      <CardHeader className="space-y-1">
        <CardTitle className="flex items-center gap-2 text-base"><Gamepad2 className="h-4 w-4"/> Drive</CardTitle>
        <p className="text-xs text-muted-foreground">Tank controls. On-screen joystick + WASD/Arrows. Shift = boost.</p>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex flex-col items-center gap-3 sm:flex-row sm:items-start">
          <JoystickPad onChange={setJoyVec} />
          <div className="grid w-full grid-cols-2 gap-3 sm:ml-4 sm:max-w-xs">
            <KV label="Left" value={lastCmd ? lastCmd.left.toFixed(2) : "0.00"} />
            <KV label="Right" value={lastCmd ? lastCmd.right.toFixed(2) : "0.00"} />
            <div className="col-span-2 flex items-center justify-between rounded-md border p-2">
              <div className="text-muted-foreground">Boost</div>
              <Badge variant={boost ? "default" : "secondary"}>{boost ? "ON" : "OFF"}</Badge>
            </div>
            <div className="col-span-2 flex gap-2">
              <Button className="flex-1" variant="outline" onClick={stopAll}>Stop</Button>
              {transport.status !== "connected" ? (
                <Button className="flex-1" onClick={transport.connect}>Connect</Button>
              ) : (
                <Button className="flex-1" variant="secondary" onClick={transport.disconnect}>Disconnect</Button>
              )}
            </div>
          </div>
        </div>
        <Separator />
        <div className="grid grid-cols-3 gap-2 text-center text-xs text-muted-foreground">
          <div>W/↑ Forward</div>
          <div>S/↓ Back</div>
          <div>A/← Turn L</div>
          <div>D/→ Turn R</div>
          <div className="col-span-3">Shift = Boost • Dead-man {DEADMAN_MS}ms</div>
        </div>
      </CardContent>
    </Card>
  )
}
