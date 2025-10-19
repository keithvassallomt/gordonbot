import React from "react"
import { AlertCircle, CheckCircle, AlertTriangle, XCircle, Activity, Play, Pause } from "lucide-react"
import { useScanQuality, type QualityLevel } from "@/components/hooks/useScanQuality"
import { Button } from "@/components/ui/button"

const QUALITY_COLORS: Record<QualityLevel, { bg: string; text: string; icon: React.ReactNode }> = {
  excellent: {
    bg: "bg-green-500/10 border-green-500/30",
    text: "text-green-600 dark:text-green-400",
    icon: <CheckCircle className="h-4 w-4" />,
  },
  good: {
    bg: "bg-blue-500/10 border-blue-500/30",
    text: "text-blue-600 dark:text-blue-400",
    icon: <CheckCircle className="h-4 w-4" />,
  },
  fair: {
    bg: "bg-yellow-500/10 border-yellow-500/30",
    text: "text-yellow-600 dark:text-yellow-400",
    icon: <AlertTriangle className="h-4 w-4" />,
  },
  poor: {
    bg: "bg-orange-500/10 border-orange-500/30",
    text: "text-orange-600 dark:text-orange-400",
    icon: <AlertCircle className="h-4 w-4" />,
  },
  critical: {
    bg: "bg-red-500/10 border-red-500/30",
    text: "text-red-600 dark:text-red-400",
    icon: <XCircle className="h-4 w-4" />,
  },
}

export default function ScanQualityPanel() {
  const { quality, status, monitoringEnabled, toggleMonitoring } = useScanQuality()

  // Show toggle button and paused message when monitoring is disabled
  if (!monitoringEnabled || status !== "connected" || !quality || quality.status !== "ok" || !quality.latest) {
    return (
      <div className="space-y-3">
        <div className="flex items-center justify-between">
          <div className="text-sm font-medium text-muted-foreground">Scan Quality</div>
          <Button
            variant="outline"
            size="sm"
            onClick={toggleMonitoring}
            className="h-7 gap-1.5 px-2"
          >
            {monitoringEnabled ? (
              <>
                <Pause className="h-3.5 w-3.5" />
                <span className="text-xs">Pause</span>
              </>
            ) : (
              <>
                <Play className="h-3.5 w-3.5" />
                <span className="text-xs">Resume</span>
              </>
            )}
          </Button>
        </div>
        <div className="rounded-lg border border-dashed p-4 text-center text-sm text-muted-foreground">
          <Activity className="mx-auto mb-2 h-5 w-5 opacity-50" />
          {!monitoringEnabled && "Monitoring paused (power saving)"}
          {monitoringEnabled && status === "connecting" && "Connecting to scan quality monitor..."}
          {monitoringEnabled && status === "disconnected" && "Scan quality monitor disconnected"}
          {monitoringEnabled && status === "connected" && quality?.status === "no_data" && "Waiting for scan data..."}
        </div>
      </div>
    )
  }

  const { latest, averages, config } = quality
  const qualityStyle = QUALITY_COLORS[latest.quality_level]

  return (
    <div className="space-y-3">
      {/* Header with toggle button */}
      <div className="flex items-center justify-between">
        <div className="text-sm font-medium text-muted-foreground">Scan Quality</div>
        <Button
          variant="outline"
          size="sm"
          onClick={toggleMonitoring}
          className="h-7 gap-1.5 px-2"
        >
          <Pause className="h-3.5 w-3.5" />
          <span className="text-xs">Pause</span>
        </Button>
      </div>
      {/* Quality Level Badge */}
      <div className={`flex items-center gap-2 rounded-lg border p-3 ${qualityStyle.bg}`}>
        <div className={qualityStyle.text}>{qualityStyle.icon}</div>
        <div className="flex-1">
          <div className="flex items-center justify-between">
            <span className={`font-semibold capitalize ${qualityStyle.text}`}>{latest.quality_level}</span>
            <span className={`text-sm font-mono ${qualityStyle.text}`}>
              {(latest.quality_score * 100).toFixed(0)}%
            </span>
          </div>
          <div className="mt-1 h-1.5 overflow-hidden rounded-full bg-muted">
            <div
              className={`h-full ${qualityStyle.text.includes("green") ? "bg-green-500" : qualityStyle.text.includes("blue") ? "bg-blue-500" : qualityStyle.text.includes("yellow") ? "bg-yellow-500" : qualityStyle.text.includes("orange") ? "bg-orange-500" : "bg-red-500"}`}
              style={{ width: `${latest.quality_score * 100}%` }}
            />
          </div>
        </div>
      </div>

      {/* Metrics Grid */}
      <div className="grid grid-cols-2 gap-2 text-xs">
        <MetricItem
          label="Points"
          value={latest.point_count}
          unit=""
          threshold={config?.min_point_count}
          isGood={latest.point_count >= (config?.min_point_count ?? 100)}
        />
        <MetricItem
          label="Coverage"
          value={(latest.angular_coverage * 100).toFixed(1)}
          unit="%"
          threshold={config?.min_angular_coverage ? (config.min_angular_coverage * 100).toFixed(0) : undefined}
          isGood={latest.angular_coverage >= (config?.min_angular_coverage ?? 0.7)}
        />
        <MetricItem
          label="Density"
          value={latest.scan_density.toFixed(2)}
          unit="pts/°"
          threshold={config?.min_scan_density?.toFixed(2)}
          isGood={latest.scan_density >= (config?.min_scan_density ?? 0.5)}
        />
        <MetricItem
          label="Max Gap"
          value={latest.max_gap_size.toFixed(1)}
          unit="°"
          threshold={config?.max_allowed_gap?.toFixed(0)}
          isGood={latest.max_gap_size <= (config?.max_allowed_gap ?? 45)}
        />
        <MetricItem
          label="Valid Range"
          value={(latest.valid_range_ratio * 100).toFixed(1)}
          unit="%"
          isGood={latest.valid_range_ratio >= 0.5}
        />
        <MetricItem
          label="Scan Rate"
          value={latest.scan_rate.toFixed(1)}
          unit="Hz"
          threshold={config?.expected_scan_rate?.toFixed(1)}
          isGood={
            Math.abs(latest.scan_rate - (config?.expected_scan_rate ?? 10)) <
            (config?.expected_scan_rate ?? 10) * 0.2
          }
        />
      </div>

      {/* Issues & Warnings */}
      {(latest.issues.length > 0 || latest.warnings.length > 0) && (
        <div className="space-y-1.5">
          {latest.issues.map((issue, idx) => (
            <div key={`issue-${idx}`} className="flex items-start gap-2 rounded-md bg-red-500/10 p-2 text-xs">
              <XCircle className="h-3 w-3 shrink-0 text-red-500" />
              <span className="text-red-700 dark:text-red-400">{issue}</span>
            </div>
          ))}
          {latest.warnings.map((warning, idx) => (
            <div
              key={`warning-${idx}`}
              className="flex items-start gap-2 rounded-md bg-yellow-500/10 p-2 text-xs"
            >
              <AlertTriangle className="h-3 w-3 shrink-0 text-yellow-600" />
              <span className="text-yellow-700 dark:text-yellow-400">{warning}</span>
            </div>
          ))}
        </div>
      )}

      {/* Averages */}
      {averages && (
        <div className="rounded-md bg-muted/50 p-2 text-xs">
          <div className="mb-1 font-medium text-muted-foreground">Recent Average (10 scans)</div>
          <div className="flex justify-between">
            <span>Quality Score:</span>
            <span className="font-mono">{(averages.quality_score * 100).toFixed(0)}%</span>
          </div>
          <div className="flex justify-between">
            <span>Point Count:</span>
            <span className="font-mono">{averages.point_count.toFixed(0)}</span>
          </div>
        </div>
      )}

      {/* Map Quality Section (Phase 2) */}
      {quality.map_quality && !quality.map_quality.is_empty && (
        <div className="space-y-2">
          <div className="text-xs font-medium text-muted-foreground">Map Structure Quality</div>

          <div className="grid grid-cols-2 gap-2 text-xs">
            <MetricItem
              label="Explored"
              value={(quality.map_quality.explored_ratio * 100).toFixed(1)}
              unit="%"
              isGood={quality.map_quality.explored_ratio >= 0.1}
            />
            <MetricItem
              label="Entropy"
              value={quality.map_quality.entropy.toFixed(2)}
              unit=""
              isGood={quality.map_quality.entropy >= 0.5}
            />
            <MetricItem
              label="Noise"
              value={(quality.map_quality.noise_score * 100).toFixed(1)}
              unit="%"
              isGood={quality.map_quality.noise_score < 0.1}
            />
            <MetricItem
              label="Sharpness"
              value={quality.map_quality.wall_sharpness.toFixed(1)}
              unit=""
              isGood={quality.map_quality.wall_sharpness >= 30}
            />
          </div>

          {(quality.map_quality.warnings.length > 0 || quality.map_quality.issues.length > 0) && (
            <div className="space-y-1">
              {quality.map_quality.issues.map((issue, idx) => (
                <div key={`map-issue-${idx}`} className="flex items-start gap-2 rounded-md bg-red-500/10 p-2 text-xs">
                  <XCircle className="h-3 w-3 shrink-0 text-red-500" />
                  <span className="text-red-700 dark:text-red-400">{issue}</span>
                </div>
              ))}
              {quality.map_quality.warnings.map((warning, idx) => (
                <div
                  key={`map-warning-${idx}`}
                  className="flex items-start gap-2 rounded-md bg-yellow-500/10 p-2 text-xs"
                >
                  <AlertTriangle className="h-3 w-3 shrink-0 text-yellow-600" />
                  <span className="text-yellow-700 dark:text-yellow-400">{warning}</span>
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  )
}

interface MetricItemProps {
  label: string
  value: number | string
  unit?: string
  threshold?: number | string
  isGood?: boolean
}

function MetricItem({ label, value, unit = "", threshold, isGood }: MetricItemProps) {
  return (
    <div className="rounded-md border bg-card p-2">
      <div className="mb-1 text-xs text-muted-foreground">{label}</div>
      <div className="flex items-baseline justify-between">
        <span className={`font-mono text-sm font-semibold ${isGood === false ? "text-red-500" : ""}`}>
          {value}
          {unit}
        </span>
        {threshold !== undefined && (
          <span className="text-xs text-muted-foreground">
            ≥{threshold}
            {unit}
          </span>
        )}
      </div>
    </div>
  )
}
