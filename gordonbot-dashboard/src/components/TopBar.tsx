import { Battery, MonitorCog, Moon, Sun, Wifi, WifiOff, Radar } from "lucide-react"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"

type ThemeMode = "system" | "light" | "dark"

/**
 * Props for the {@link TopBar} component.
 * @property mode - Current theme mode (system, light, or dark).
 * @property onModeChange - Callback fired when user selects a new mode.
 * @property transportStatus - Current transport connection status.
 * @property batteryPercent - Optional battery percentage to display.
 */
interface TopBarProps {
  mode: ThemeMode
  onModeChange: (mode: ThemeMode) => void
  transportStatus: "disconnected" | "connecting" | "connected"
  batteryPercent?: number
}

/**
 * Top navigation bar component.
 *
 * Displays robot identity, transport connection status, battery percent,
 * and theme switcher dropdown.
 *
 * @param props - {@link TopBarProps}.
 * @returns JSX top bar.
 *
 * @remarks
 * - Sticky at the top with backdrop blur.
 * - Transport status shown with icon and badge.
 * - Battery percent shown compactly with icon.
 * - Theme dropdown allows switching between system, light, and dark.
 *
 * @example
 * ```tsx
 * <TopBar
 *   mode={mode}
 *   onModeChange={setMode}
 *   transportStatus="connected"
 *   batteryPercent={80}
 * />
 * ```
 */
export default function TopBar({
  mode,
  onModeChange,
  transportStatus,
  batteryPercent,
}: TopBarProps) {
  return (
    <div className="sticky top-0 z-10 w-full border-b bg-background/80 backdrop-blur supports-[backdrop-filter]:bg-background/60">
      <div className="mx-auto flex max-w-7xl items-center justify-between px-4 py-3">
        <div className="flex items-center gap-2">
          <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-primary text-primary-foreground">
            <Radar className="h-4 w-4" />
          </div>
          <div>
            <div className="text-sm font-semibold leading-4">GordonBot</div>
            <div className="text-[11px] text-muted-foreground">Robot Dashboard</div>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <Badge
            variant={transportStatus === "connected" ? "default" : "secondary"}
            className="flex items-center gap-1"
          >
            {transportStatus === "connected" ? (
              <Wifi className="h-3 w-3" />
            ) : (
              <WifiOff className="h-3 w-3" />
            )}
            {transportStatus}
          </Badge>

          <div className="flex items-center gap-1 rounded-md border px-2 py-1 text-xs">
            <Battery className="mr-1 h-3 w-3" />
            {batteryPercent != null ? `${batteryPercent.toFixed(0)}%` : "—"}
          </div>

          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="outline" size="icon" className="h-8 w-8">
                {mode === "dark" ? (
                  <Moon className="h-4 w-4" />
                ) : mode === "light" ? (
                  <Sun className="h-4 w-4" />
                ) : (
                  <MonitorCog className="h-4 w-4" />
                )}
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end" className="w-44">
              <DropdownMenuLabel>Theme</DropdownMenuLabel>
              <DropdownMenuSeparator />
              <DropdownMenuItem onClick={() => onModeChange("system")}>System</DropdownMenuItem>
              <DropdownMenuItem onClick={() => onModeChange("light")}>Light</DropdownMenuItem>
              <DropdownMenuItem onClick={() => onModeChange("dark")}>Dark</DropdownMenuItem>
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </div>
    </div>
  )
}