import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible"
import { Progress } from "@/components/ui/progress"
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "@/components/ui/tooltip"
import { Battery, PlugZap, Power, ChevronDown } from "lucide-react"
import { useBattery } from "@/components/hooks/useBattery"

/**
 * Format a numeric value with an optional unit.
 * @param n - The number to format.
 * @param unit - Optional unit string (e.g. "V").
 * @returns Formatted string or em-dash if invalid.
 */
function fmt(n?: number, unit?: string, precision: number = 2) {
  if (n == null || isNaN(n)) return "—"
  return unit ? `${n.toFixed(precision)} ${unit}` : n.toFixed(precision)
}

/**
 * Key-value display row used for advanced battery details.
 * @param label - Human-readable label.
 * @param value - Formatted value string.
 */
function KV({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between rounded-md border p-2">
      <div className="text-muted-foreground">{label}</div>
      <div className="font-mono">{value}</div>
    </div>
  )
}

/**
 * Battery panel component.
 *
 * Displays the robot battery status including percentage, charging indicator,
 * progress bar, and advanced details (voltage, current, capacity, per-cell data).
 *
 * @remarks
 * - Uses {@link useBattery} hook for live battery data.
 * - Polls every 10 seconds and allows manual refresh.
 * - Collapsible section reveals advanced details.
 * - Fully responsive and styled with shadcn/ui components.
 *
 * @example
 * ```tsx
 * <BatteryPanel />
 * ```
 */
export default function BatteryPanel() {
  const { data, error, refresh } = useBattery(10000)
  const percent = Math.max(0, Math.min(100, data?.percent ?? 0))
  const charging = data?.charging ?? false

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <Battery className="h-4 w-4" /> Battery
        </CardTitle>
        <div className="flex items-center gap-2">
          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Badge
                  variant={
                    charging === "charging" || charging === "fast_charging"
                      ? "default"
                      : charging === "discharging"
                      ? "destructive"
                      : "secondary"
                  }
                  className="flex items-center gap-1"
                >
                  {charging === "charging" && <PlugZap className="h-3 w-3" />}
                  {charging === "fast_charging" && <PlugZap className="h-3 w-3 animate-pulse" />}
                  {charging === "discharging" && <Power className="h-3 w-3" />}
                  {charging === "idle" && <Battery className="h-3 w-3" />}
                  {charging === "charging" && "Charging"}
                  {charging === "fast_charging" && "Fast Charging"}
                  {charging === "discharging" && "Discharging"}
                  {charging === "idle" && "Idle"}
                </Badge>
              </TooltipTrigger>
              <TooltipContent>Charging state</TooltipContent>
            </Tooltip>
          </TooltipProvider>
          <Button size="sm" variant="outline" onClick={refresh}>
            Refresh
          </Button>
        </div>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center justify-between">
          <div className="text-sm font-medium">Charge</div>
          <div className="text-sm tabular-nums font-semibold">{percent.toFixed(0)}%</div>
        </div>
        <Progress value={percent} />

        <Collapsible>
          <CollapsibleTrigger asChild>
            <Button variant="ghost" size="sm" className="mt-1 w-full justify-between">
              Advanced details <ChevronDown className="h-4 w-4" />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent>
            {error && (
              <p className="rounded-md border bg-destructive/10 p-2 text-sm text-destructive">
                Failed to load battery info: {error}
              </p>
            )}
            <div className="grid grid-cols-2 gap-3 pt-2 text-sm">
              <KV label="VBUS Voltage" value={fmt(data?.vbusVoltage, "V")} />
              <KV label="VBUS Current" value={fmt(data?.vbusCurrent, "A")} />
              <KV label="VBUS Power" value={fmt(data?.vbusPower, "W")} />
              <KV label="Battery Voltage" value={fmt(data?.batteryVoltage, "V")} />
              <KV label="Battery Current" value={fmt(data?.batteryCurrent, "A")} />
              <KV label="Remaining Capacity" value={fmt(data?.remainingCapacity, "mAh", 0)} />
              <KV
                label="Avg Time to Full"
                value={data?.avgTimeToFullMin != null ? `${data.avgTimeToFullMin} min` : "—"}
              />
            </div>
            {Array.isArray(data?.cells) && (
              <div className="pt-3">
                <div className="mb-1 text-xs font-semibold uppercase tracking-wide text-muted-foreground">
                  Cell Voltages
                </div>
                <div className="grid grid-cols-4 gap-2 text-sm">
                  {data!.cells!.map((v, i) => (
                    <div key={i} className="rounded-md border p-2 text-center">
                      <div className="text-xs text-muted-foreground">Cell {i + 1}</div>
                      <div className="font-mono">{fmt(v, "V")}</div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </CollapsibleContent>
        </Collapsible>
      </CardContent>
    </Card>
  )
}