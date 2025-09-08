import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Separator } from "@/components/ui/separator"
import { Activity, Gauge, Radar, Thermometer, Plug } from "lucide-react"
import { Button } from "@/components/ui/button"

import { useSensors } from "@/components/hooks/useSensors"
import type { Vector3, EulerDeg } from "@/components/types"

function fmt(n?: number, unit?: string, p = 2) {
  if (n == null || isNaN(n)) return "—"
  return unit ? `${n.toFixed(p)} ${unit}` : n.toFixed(p)
}

function V3({ v, unit, p = 2 }: { v?: Vector3; unit?: string; p?: number }) {
  return <span className="font-mono">[{fmt(v?.x, unit, p)}, {fmt(v?.y, unit, p)}, {fmt(v?.z, unit, p)}]</span>
}

function Euler({ e }: { e?: EulerDeg }) {
  return <span className="font-mono">R:{fmt(e?.roll, "°", 1)} P:{fmt(e?.pitch, "°", 1)} Y:{fmt(e?.yaw, "°", 1)}</span>
}

export default function SensorsPanel() {
  const { data, error } = useSensors(1000)
  const ts = data?.ts ? new Date(data.ts).toLocaleTimeString() : "—"
  const encConnected = Boolean(data?.encoders?.left?.connected || data?.encoders?.right?.connected)
  const speedLeft = data?.encoders?.left?.speed_mm_s
  const speedRight = data?.encoders?.right?.speed_mm_s
  const speeds = [speedLeft, speedRight].filter((v): v is number => typeof v === "number")
  const speedAvg = speeds.length ? speeds.reduce((a,b)=>a+b,0)/speeds.length : undefined
  const maxSpeed = 500 // mm/s, adjust if you know your top speed

  // Average distance travelled (mm) from both encoders (signed)
  const distLeftMm = (() => {
    const mm = data?.encoders?.left?.distance_mm
    if (typeof mm === 'number') return mm
    const m = data?.encoders?.left?.distance_m
    return typeof m === 'number' ? m * 1000.0 : undefined
  })()
  const distRightMm = (() => {
    const mm = data?.encoders?.right?.distance_mm
    if (typeof mm === 'number') return mm
    const m = data?.encoders?.right?.distance_m
    return typeof m === 'number' ? m * 1000.0 : undefined
  })()
  const dists = [distLeftMm, distRightMm].filter((v): v is number => typeof v === 'number')
  const distAvgMm = dists.length ? dists.reduce((a,b)=>a+b,0)/dists.length : undefined

  function SpeedGauge({ value, max, distanceMm }: { value?: number; max: number; distanceMm?: number }) {
    const v = Math.max(0, Math.min(max, value ?? 0))
    const pct = v / max
    const angle = Math.round(180 * pct) // 0..180deg
    const gradient = `conic-gradient(var(--primary) 0deg ${angle}deg, var(--muted) ${angle}deg 180deg, transparent 180deg 360deg)`
    return (
      <div className="flex items-center gap-3">
        <div className="relative h-16 w-32 overflow-hidden">
          <div className="absolute inset-0 rounded-b-full" style={{ background: gradient }} />
          <div className="absolute inset-0 rounded-b-full border" />
        </div>
        <div className="min-w-[80px] text-right">
          <div className="font-mono text-xl">{fmt(value, "mm/s", 0)}</div>
          <div className="text-xs text-muted-foreground">max {max} mm/s</div>
          <div className="text-xs text-muted-foreground">{distanceMm != null ? `avg dist ${distanceMm.toFixed(0)} mm` : 'avg dist —'}</div>
        </div>
      </div>
    )
  }

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <Activity className="h-4 w-4"/> Sensors
        </CardTitle>
        {error ? (
          <Badge variant="destructive">Error</Badge>
        ) : (
          <Badge variant="secondary">{ts}</Badge>
        )}
      </CardHeader>
      <CardContent className="space-y-3 text-sm">
        {/* Speed gauge */}
        <div className="flex items-center justify-between rounded-md border p-3">
          <div className="flex items-center gap-2 text-muted-foreground"><Gauge className="h-4 w-4"/> Speed</div>
          <SpeedGauge value={speedAvg} max={maxSpeed} distanceMm={distAvgMm} />
        </div>

        {/* Encoders */}
        <div className="flex items-center justify-between">
          <div className="text-muted-foreground">Encoders</div>
          <Button size="sm" variant={encConnected ? "secondary" : "outline"} className={!encConnected ? "opacity-75" : ""} disabled>
            <Plug className="mr-1 h-3.5 w-3.5" /> {encConnected ? "Connected" : "No hardware"}
          </Button>
        </div>
        <div className="grid grid-cols-2 gap-3">
          <div className="rounded-md border p-2">
            <div className="mb-1 flex items-center gap-2 text-muted-foreground"><Gauge className="h-3.5 w-3.5"/> Left Encoder</div>
            <div className="grid grid-cols-3 gap-2 font-mono text-xs">
              <div>ticks</div><div className="col-span-2">{data?.encoders?.left?.ticks ?? "—"}</div>
              <div>dist</div><div className="col-span-2">{data?.encoders?.left?.distance_mm != null ? `${data.encoders.left.distance_mm.toFixed(0)} mm` : fmt(data?.encoders?.left?.distance_m, "m", 3)}</div>
              <div>rpm</div><div className="col-span-2">{fmt(data?.encoders?.left?.rpm)}</div>
              <div>speed</div><div className="col-span-2">{data?.encoders?.left?.speed_mm_s != null ? `${data.encoders.left.speed_mm_s.toFixed(0)} mm/s` : "—"}</div>
            </div>
          </div>
          <div className="rounded-md border p-2">
            <div className="mb-1 flex items-center gap-2 text-muted-foreground"><Gauge className="h-3.5 w-3.5"/> Right Encoder</div>
            <div className="grid grid-cols-3 gap-2 font-mono text-xs">
              <div>ticks</div><div className="col-span-2">{data?.encoders?.right?.ticks ?? "—"}</div>
              <div>dist</div><div className="col-span-2">{data?.encoders?.right?.distance_mm != null ? `${data.encoders.right.distance_mm.toFixed(0)} mm` : fmt(data?.encoders?.right?.distance_m, "m", 3)}</div>
              <div>rpm</div><div className="col-span-2">{fmt(data?.encoders?.right?.rpm)}</div>
              <div>speed</div><div className="col-span-2">{data?.encoders?.right?.speed_mm_s != null ? `${data.encoders.right.speed_mm_s.toFixed(0)} mm/s` : "—"}</div>
            </div>
          </div>
        </div>

        <Separator />

        {/* ToF */}
        <div className="flex items-center justify-between rounded-md border p-2">
          <div className="flex items-center gap-2 text-muted-foreground"><Radar className="h-4 w-4"/> ToF Distance</div>
          <div className="font-mono">{data?.tof?.distance_mm != null ? `${data.tof.distance_mm} mm` : "—"}</div>
        </div>

        {/* BNO055 */}
        <div className="grid gap-2 rounded-md border p-2">
          <div className="mb-1 flex items-center gap-2 text-muted-foreground"><Thermometer className="h-4 w-4"/> BNO055</div>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div className="flex items-center justify-between rounded-md border p-2"><span>Euler</span><Euler e={data?.bno055?.euler} /></div>
            <div className="flex items-center justify-between rounded-md border p-2"><span>Ang vel</span><V3 v={data?.bno055?.ang_vel_rad_s} unit="rad/s" /></div>
            <div className="flex items-center justify-between rounded-md border p-2"><span>Accel</span><V3 v={data?.bno055?.accel_m_s2} unit="m/s²" /></div>
            <div className="flex items-center justify-between rounded-md border p-2"><span>Lin accel</span><V3 v={data?.bno055?.lin_accel_m_s2} unit="m/s²" /></div>
            <div className="flex items-center justify-between rounded-md border p-2"><span>Gravity</span><V3 v={data?.bno055?.gravity_m_s2} unit="m/s²" /></div>
            <div className="flex items-center justify-between rounded-md border p-2"><span>Mag</span><V3 v={data?.bno055?.mag_uT} unit="µT" /></div>
          </div>
          <div className="flex items-center justify-between text-xs text-muted-foreground">
            <span>Quat</span>
            <span className="font-mono">[{fmt(data?.bno055?.quat?.w)}, {fmt(data?.bno055?.quat?.x)}, {fmt(data?.bno055?.quat?.y)}, {fmt(data?.bno055?.quat?.z)}]</span>
          </div>
          <div className="flex items-center justify-between text-xs text-muted-foreground">
            <span>Temp</span>
            <span className="font-mono">{fmt(data?.bno055?.temp_c, "°C", 1)}</span>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
