import React from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Gauge, Cpu, Activity, MonitorCog, Wifi, WifiOff } from "lucide-react"
import { useFps, usePing } from "@/components/hooks/useDiagnostics"

/**
 * Diagnostics panel component.
 *
 * Displays live diagnostic metrics including UI FPS, CPU temp (placeholder),
 * network ping, and transport connection status.
 *
 * @param transport - Object containing current transport status string.
 * @returns JSX card listing diagnostic items.
 *
 * @remarks
 * - Uses {@link useFps} hook to calculate UI frames per second.
 * - Uses {@link usePing} to measure round-trip latency and connection health.
 * - CPU Temp is currently a placeholder until wired to `/api/diag`.
 * - Fully responsive grid layout.
 *
 * @example
 * ```tsx
 * <DiagnosticsPanel transport={transport} />
 * ```
 */
export default function DiagnosticsPanel({ transport }: { transport: { status: string } }) {
  const fps = useFps()
  const { latency, ok } = usePing(10000)

  return (
    <Card>
      <CardHeader className="space-y-1">
        <CardTitle className="flex items-center gap-2 text-base">
          <MonitorCog className="h-4 w-4" /> Diagnostics
        </CardTitle>
      </CardHeader>
      <CardContent className="grid grid-cols-2 gap-3 text-sm">
        <DiagItem icon={<Gauge className="h-4 w-4" />} label="UI FPS" value={`${fps} fps`} />
        <DiagItem icon={<Cpu className="h-4 w-4" />} label="CPU Temp" value={"—"} hint="Wire to /api/diag later" />
        <DiagItem
          icon={ok ? <Wifi className="h-4 w-4" /> : <WifiOff className="h-4 w-4" />}
          label="Ping"
          value={latency != null ? `${latency.toFixed(0)} ms` : "N/A"}
        />
        <DiagItem icon={<Activity className="h-4 w-4" />} label="Transport" value={transport.status} />
      </CardContent>
    </Card>
  )
}

/**
 * Simple key-value row for displaying a diagnostic metric.
 * @param icon - Icon element representing the metric.
 * @param label - Human-readable metric label.
 * @param value - Current value to display.
 * @param hint - Optional tooltip shown on hover.
 */
function DiagItem({
  icon,
  label,
  value,
  hint,
}: {
  icon: React.ReactNode
  label: string
  value: string
  hint?: string
}) {
  return (
    <div className="flex items-center justify-between rounded-md border p-2">
      <div className="flex items-center gap-2 text-muted-foreground">
        {icon}
        <span>{label}</span>
      </div>
      <div className="font-mono" title={hint}>
        {value}
      </div>
    </div>
  )
}