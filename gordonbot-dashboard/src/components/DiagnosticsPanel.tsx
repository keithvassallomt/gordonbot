import React from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Gauge, Cpu, Activity, MonitorCog, Wifi, WifiOff, ChevronDown } from "lucide-react"
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible"
import { Button } from "@/components/ui/button"
import KV from "@/components/KV"
import { useFps, usePing, useSystemDiagnostics } from "@/components/hooks/useDiagnostics"

function fmt(n?: number | null, unit?: string, digits = 0) {
  if (n == null || Number.isNaN(n)) return "—"
  return unit ? `${n.toFixed(digits)} ${unit}` : n.toFixed(digits)
}

function fmtBytes(n?: number | null) {
  if (n == null || !Number.isFinite(n)) return "—"
  const units = ["B", "KB", "MB", "GB", "TB"]
  let i = 0
  let v = n
  while (v >= 1024 && i < units.length - 1) { v /= 1024; i++ }
  return `${v.toFixed(1)} ${units[i]}`
}

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
  const diag = useSystemDiagnostics(5000)

  return (
    <Card>
      <CardHeader className="space-y-1">
        <CardTitle className="flex items-center gap-2 text-base">
          <MonitorCog className="h-4 w-4" /> Diagnostics
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-2 gap-3 text-sm">
          <DiagItem icon={<Gauge className="h-4 w-4" />} label="UI FPS" value={`${fps} fps`} />
          <DiagItem icon={<Cpu className="h-4 w-4" />} label="CPU Temp" value={diag.cpuTempC != null ? `${diag.cpuTempC.toFixed(1)} °C` : "—"} />
          <DiagItem icon={<Activity className="h-4 w-4" />} label="CPU Util" value={diag.cpuUtilPercent != null ? `${diag.cpuUtilPercent.toFixed(0)} %` : "—"} />
          <DiagItem
            icon={ok ? <Wifi className="h-4 w-4" /> : <WifiOff className="h-4 w-4" />}
            label="Ping"
            value={latency != null ? `${latency.toFixed(0)} ms` : "N/A"}
          />
          <DiagItem icon={<MonitorCog className="h-4 w-4" />} label="Transport" value={transport.status} />
        </div>
        <Collapsible className="col-span-2">
          <CollapsibleTrigger asChild>
            <Button variant="ghost" size="sm" className="mt-1 w-full justify-between">
              More details <ChevronDown className="h-4 w-4" />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent>
            {diag.error && (
              <p className="mt-2 rounded-md border bg-destructive/10 p-2 text-sm text-destructive">
                Failed to load system diagnostics: {diag.error}
              </p>
            )}
            <div className="mt-2 grid grid-cols-2 gap-3 text-sm">
              <KV label="RP1 Temp" value={fmt(diag.rp1TempC, "°C", 1)} />
              <KV label="Uptime" value={diag.uptimeSeconds != null ? `${Math.floor(diag.uptimeSeconds / 3600)}h ${(Math.floor(diag.uptimeSeconds / 60) % 60)}m` : "—"} />
              <KV label="Boot" value={diag.bootTime != null ? new Date(diag.bootTime * 1000).toLocaleString() : "—"} className="col-span-2" />

              <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">CPU Load (Raw)</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Load 1m" value={fmt(diag.load1, "", 2)} />
                    <KV label="Load 5m" value={fmt(diag.load5, "", 2)} />
                    <KV label="Load 15m" value={fmt(diag.load15, "", 2)} />
                  </div>
              </div>

              <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">CPU Load (Scaled)</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Load/CPU 1m" value={fmt(diag.load1PerCpu, "", 2)} />
                    <KV label="Load/CPU 5m" value={fmt(diag.load5PerCpu, "", 2)} />
                    <KV label="Load/CPU 15m" value={fmt(diag.load15PerCpu, "", 2)} />
                  </div>
              </div>
              
              <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">Memory</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Mem Total" value={fmtBytes(diag.memory?.total)} />
                    <KV label="Mem Used" value={fmtBytes(diag.memory?.used)} />
                    <KV label="Mem Avail" value={fmtBytes(diag.memory?.available)} />
                    <KV label="Mem %" value={diag.memory?.percent != null ? `${diag.memory.percent.toFixed(0)} %` : "—"} />
                  </div>
              </div>

              <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">Swap</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Swap Total" value={fmtBytes(diag.swap?.total)} />
                    <KV label="Swap Used" value={fmtBytes(diag.swap?.used)} />
                    <KV label="Swap %" value={diag.swap?.percent != null ? `${diag.swap.percent.toFixed(0)} %` : "—"} />
                  </div>
              </div>

              <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">Disk</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Disk / Total" value={fmtBytes(diag.diskRoot?.total)} />
                    <KV label="Disk / Used" value={fmtBytes(diag.diskRoot?.used)} />
                    <KV label="Disk / Free" value={fmtBytes(diag.diskRoot?.free)} />
                    <KV label="Disk / %" value={diag.diskRoot?.percent != null ? `${diag.diskRoot.percent.toFixed(0)} %` : "—"} />
                  </div>
              </div>

              {diag.throttling && (
                <div className="col-span-2 rounded-md border p-2">
                  <div className="text-xs font-semibold uppercase tracking-wide text-muted-foreground mb-1">Pi Throttling Flags</div>
                  <div className="grid grid-cols-2 gap-2 text-sm">
                    <KV label="Under-volt (now)" value={String(diag.throttling.under_voltage_now ?? false)} />
                    <KV label="Freq capped (now)" value={String(diag.throttling.freq_capped_now ?? false)} />
                    <KV label="Throttled (now)" value={String(diag.throttling.throttled_now ?? false)} />
                    <KV label="Soft temp limit (now)" value={String(diag.throttling.soft_temp_limit_now ?? false)} />
                    <KV label="Under-volt (ever)" value={String(diag.throttling.under_voltage_occured ?? false)} />
                    <KV label="Freq capped (ever)" value={String(diag.throttling.freq_capped_occured ?? false)} />
                    <KV label="Throttled (ever)" value={String(diag.throttling.throttled_occured ?? false)} />
                    <KV label="Soft temp limit (ever)" value={String(diag.throttling.soft_temp_limit_occured ?? false)} />
                  </div>
                </div>
              )}
            </div>
          </CollapsibleContent>
        </Collapsible>
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