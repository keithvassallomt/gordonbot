import {
  Activity,
  ChevronDown,
  Compass as CompassIcon,
  Gauge,
  Plug,
  Radar,
  Thermometer,
} from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible"
import { useSensors } from "@/components/hooks/useSensors"
import type { EncoderData, EulerDeg, SensorsStatus, Vector3 } from "@/components/types"

function fmt(n?: number, unit?: string, p = 2) {
  if (n == null || Number.isNaN(n)) return "—"
  return unit ? `${n.toFixed(p)} ${unit}` : n.toFixed(p)
}

function V3({ v, unit, p = 2 }: { v?: Vector3; unit?: string; p?: number }) {
  return <span className="font-mono">[{fmt(v?.x, unit, p)}, {fmt(v?.y, unit, p)}, {fmt(v?.z, unit, p)}]</span>
}

function Euler({ e }: { e?: EulerDeg }) {
  return <span className="font-mono">R:{fmt(e?.roll, "°", 1)} P:{fmt(e?.pitch, "°", 1)} Y:{fmt(e?.yaw, "°", 1)}</span>
}

function normaliseHeading(deg?: number) {
  if (deg == null || Number.isNaN(deg)) return undefined
  const normalised = ((deg % 360) + 360) % 360
  return normalised
}

function headingToCardinal(deg: number) {
  const labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
  const normalised = ((deg % 360) + 360) % 360
  const idx = Math.round(normalised / 45) % labels.length
  return labels[idx]
}

export default function NavigationPanel() {
  const { data, error } = useSensors(1000)
  const ts = data?.ts ? new Date(data.ts).toLocaleTimeString() : "—"
  const speedLeft = data?.encoders?.left?.speed_mm_s
  const speedRight = data?.encoders?.right?.speed_mm_s
  const speeds = [speedLeft, speedRight].filter((v): v is number => typeof v === "number")
  const speedAvg = speeds.length ? speeds.reduce((a, b) => a + b, 0) / speeds.length : undefined
  const maxSpeed = 500 // mm/s, adjust if you know your top speed
  const obstacleDistanceMm = data?.tof?.distance_mm

  // Average distance travelled (mm) from both encoders (signed)
  const distLeftMm = (() => {
    const mm = data?.encoders?.left?.distance_mm
    if (typeof mm === "number") return mm
    const m = data?.encoders?.left?.distance_m
    return typeof m === "number" ? m * 1000.0 : undefined
  })()
  const distRightMm = (() => {
    const mm = data?.encoders?.right?.distance_mm
    if (typeof mm === "number") return mm
    const m = data?.encoders?.right?.distance_m
    return typeof m === "number" ? m * 1000.0 : undefined
  })()
  const dists = [distLeftMm, distRightMm].filter((v): v is number => typeof v === "number")
  const distAvgMm = dists.length ? dists.reduce((a, b) => a + b, 0) / dists.length : undefined
  const headingRawDeg = data?.bno055?.euler?.yaw
  const headingDeg = normaliseHeading(headingRawDeg)
  const headingCardinal = headingDeg != null ? headingToCardinal(headingDeg) : undefined

  return (
    <div className="space-y-4">
      <Card>
        <CardHeader className="flex flex-row items-center justify-between space-y-0">
          <CardTitle className="flex items-center gap-2 text-base">
            <Activity className="h-4 w-4" /> Navigation
          </CardTitle>
          {error ? <Badge variant="destructive">Error</Badge> : <Badge variant="secondary">{ts}</Badge>}
        </CardHeader>
        <CardContent className="space-y-3 text-sm">
          <div className="grid gap-3 md:grid-cols-3">
            <div className="flex flex-col gap-3 rounded-md border p-3">
              <div className="flex items-center gap-2 text-muted-foreground">
                <Gauge className="h-4 w-4" /> Speed
              </div>
              <SpeedGauge value={speedAvg} max={maxSpeed} />
              <div className="text-xs text-muted-foreground">
                {speeds.length ? `avg from ${speeds.length} encoder${speeds.length > 1 ? "s" : ""}` : "no encoder data"}
              </div>
            </div>
            <div className="flex flex-col gap-3 rounded-md border p-3">
              <div className="flex items-center gap-2 text-muted-foreground">
                <Radar className="h-4 w-4" /> Distance
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">ToF distance</span>
                <span className="font-mono">{obstacleDistanceMm != null ? `${obstacleDistanceMm.toFixed(0)} mm` : "—"}</span>
              </div>
              <div className="flex items-center justify-between text-sm">
                <span className="text-muted-foreground">Avg travelled</span>
                <span className="font-mono">{distAvgMm != null ? `${distAvgMm.toFixed(0)} mm` : "—"}</span>
              </div>
            </div>
            <div className="flex flex-col gap-3 rounded-md border p-3">
              <div className="flex items-center gap-2 text-muted-foreground">
                <CompassIcon className="h-4 w-4" /> Heading
              </div>
              <HeadingCompass heading={headingDeg} />
              <div className="flex flex-col items-center text-xs text-muted-foreground">
                <span className="font-mono text-sm">{headingDeg != null ? `${headingDeg.toFixed(0)}°` : "—"}</span>
                <span className="uppercase tracking-wide">{headingCardinal ?? "no data"}</span>
              </div>
            </div>
          </div>
        </CardContent>
      </Card>

      <SensorsTelemetryCard data={data} error={error} timestamp={ts} />
    </div>
  )
}

function SpeedGauge({ value, max }: { value?: number; max: number }) {
  const clamped = Math.max(-max, Math.min(max, value ?? 0))
  const ratio = (clamped + max) / (2 * max)
  return (
    <div className="flex flex-col items-center gap-3">
      <div className="relative h-3 w-full max-w-[220px] rounded-full bg-muted">
        <div className="absolute inset-y-0 left-1/2 w-px -translate-x-1/2 bg-border" />
        <div
          className="absolute top-1/2 h-2 w-2 -translate-x-1/2 -translate-y-1/2 rounded-full border border-primary bg-background shadow"
          style={{ left: `${ratio * 100}%` }}
        />
        <div className="absolute -bottom-4 left-0 right-0 flex justify-between text-[10px] uppercase tracking-wide text-muted-foreground">
          <span>-{max}</span>
          <span>0</span>
          <span>{max}</span>
        </div>
      </div>
      <div className="font-mono text-xl">{fmt(value, "mm/s", 0)}</div>
    </div>
  )
}

function HeadingCompass({ heading }: { heading?: number }) {
  if (heading == null || Number.isNaN(heading)) {
    return (
      <div className="flex h-24 w-full items-center justify-center rounded-md border bg-muted text-xs text-muted-foreground">
        No heading data
      </div>
    )
  }

  return (
    <div className="relative mx-auto h-24 w-24">
      <div className="absolute inset-0 rounded-full border border-border bg-background" />
      <div className="absolute inset-3 rounded-full border border-dashed border-muted-foreground/60" />
      <span className="absolute left-1/2 top-1 -translate-x-1/2 text-[10px] font-semibold text-muted-foreground">N</span>
      <span className="absolute right-1 top-1/2 -translate-y-1/2 text-[10px] font-semibold text-muted-foreground">E</span>
      <span className="absolute left-1/2 bottom-1 -translate-x-1/2 text-[10px] font-semibold text-muted-foreground">S</span>
      <span className="absolute left-1 top-1/2 -translate-y-1/2 text-[10px] font-semibold text-muted-foreground">W</span>
      <div
        className="absolute left-1/2 top-1/2 h-10 w-1 -translate-x-1/2 -translate-y-full origin-bottom rounded-full bg-primary"
        style={{ transform: `rotate(${heading}deg)` }}
      />
      <div className="absolute left-1/2 top-1/2 h-2 w-2 -translate-x-1/2 -translate-y-1/2 rounded-full bg-primary" />
    </div>
  )
}

function SensorsTelemetryCard({
  data,
  error,
  timestamp,
}: {
  data: SensorsStatus | null | undefined
  error: string | null
  timestamp: string
}) {
  const encConnected = Boolean(data?.encoders?.left?.connected || data?.encoders?.right?.connected)

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <Activity className="h-4 w-4" /> Sensors
        </CardTitle>
        {error ? <Badge variant="destructive">Error</Badge> : <Badge variant="secondary">{timestamp}</Badge>}
      </CardHeader>
      <CardContent className="space-y-4 text-sm">
        <Collapsible defaultOpen>
          <CollapsibleTrigger asChild>
            <Button variant="ghost" size="sm" className="w-full justify-between">
              <span className="flex items-center gap-2">
                <Gauge className="h-4 w-4" /> Encoders
              </span>
              <ChevronDown className="h-4 w-4" />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent className="space-y-3 pt-3">
            <div className="flex items-center justify-between">
              <span className="text-muted-foreground">Status</span>
              <Button
                size="sm"
                variant={encConnected ? "secondary" : "outline"}
                className={!encConnected ? "opacity-75" : ""}
                disabled
              >
                <Plug className="mr-1 h-3.5 w-3.5" /> {encConnected ? "Connected" : "No hardware"}
              </Button>
            </div>
            <div className="grid grid-cols-1 gap-3 md:grid-cols-2">
              <EncoderCard label="Left Encoder" data={data?.encoders?.left} />
              <EncoderCard label="Right Encoder" data={data?.encoders?.right} />
            </div>
          </CollapsibleContent>
        </Collapsible>

        <Collapsible>
          <CollapsibleTrigger asChild>
            <Button variant="ghost" size="sm" className="w-full justify-between">
              <span className="flex items-center gap-2">
                <Thermometer className="h-4 w-4" /> BNO055
              </span>
              <ChevronDown className="h-4 w-4" />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent className="space-y-3 pt-3">
            <div className="grid grid-cols-1 gap-2 text-xs md:grid-cols-2">
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Euler</span>
                <Euler e={data?.bno055?.euler} />
              </div>
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Ang vel</span>
                <V3 v={data?.bno055?.ang_vel_rad_s} unit="rad/s" />
              </div>
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Accel</span>
                <V3 v={data?.bno055?.accel_m_s2} unit="m/s²" />
              </div>
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Lin accel</span>
                <V3 v={data?.bno055?.lin_accel_m_s2} unit="m/s²" />
              </div>
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Gravity</span>
                <V3 v={data?.bno055?.gravity_m_s2} unit="m/s²" />
              </div>
              <div className="flex items-center justify-between rounded-md border p-2">
                <span>Mag</span>
                <V3 v={data?.bno055?.mag_uT} unit="µT" />
              </div>
            </div>
            <div className="grid gap-2 text-xs">
              <div className="flex items-center justify-between text-muted-foreground">
                <span>Quat</span>
                <span className="font-mono">
                  [{fmt(data?.bno055?.quat?.w)}, {fmt(data?.bno055?.quat?.x)}, {fmt(data?.bno055?.quat?.y)}, {fmt(data?.bno055?.quat?.z)}]
                </span>
              </div>
              <div className="flex items-center justify-between text-muted-foreground">
                <span>Temp</span>
                <span className="font-mono">{fmt(data?.bno055?.temp_c, "°C", 1)}</span>
              </div>
            </div>
          </CollapsibleContent>
        </Collapsible>
      </CardContent>
    </Card>
  )
}

function EncoderCard({ label, data }: { label: string; data?: EncoderData }) {
  return (
    <div className="rounded-md border p-2">
      <div className="mb-1 flex items-center gap-2 text-muted-foreground">
        <Gauge className="h-3.5 w-3.5" /> {label}
      </div>
      <div className="grid grid-cols-3 gap-2 font-mono text-xs">
        <div>ticks</div>
        <div className="col-span-2">{data?.ticks ?? "—"}</div>
        <div>dist</div>
        <div className="col-span-2">
          {data?.distance_mm != null ? `${data.distance_mm.toFixed(0)} mm` : fmt(data?.distance_m, "m", 3)}
        </div>
        <div>rpm</div>
        <div className="col-span-2">{fmt(data?.rpm)}</div>
        <div>speed</div>
        <div className="col-span-2">
          {data?.speed_mm_s != null ? `${data.speed_mm_s.toFixed(0)} mm/s` : "—"}
        </div>
      </div>
    </div>
  )
}
