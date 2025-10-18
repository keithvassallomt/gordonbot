import { useEffect, useState } from "react";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { TooltipProvider } from "@/components/ui/tooltip";
import { Camera, Map as MapIcon, Radar, Github } from "lucide-react"

import TopBar from "./TopBar"
import CameraPanel from "./CameraPanel"
import SlamMapPanel from "./SlamMapPanel"
import BatteryPanel from "./BatteryPanel";
import DiagnosticsPanel from "./DiagnosticsPanel"
import OrientationPanel from "./OrientationPanel"
import ControlPanel from "./ControlPanel";
import LidarPanel from "./LidarPanel";
import NavigationPanel from "./NavigationPanel"
import { CameraOverlay } from "./CameraOverlay"

import { useBattery } from "./hooks/useBattery";
import { useControlTransport } from "./hooks/useControlTransport"
import { useThemeMode } from "./hooks/useThemeMode"
import { SlamModeProvider } from "./contexts/SlamModeContext"
import { CameraStreamProvider } from "./contexts/CameraStreamContext"

import { CONTROL_WS_PATH } from "./config"

// ========================
// Main Layout
// ========================

/**
 * GordonBot main dashboard layout.
 *
 * Composes all panels (TopBar, Camera, Map, Control, Battery, Diagnostics)
 * into a responsive grid layout.
 *
 * @returns JSX root of the dashboard.
 *
 * @remarks
 * - Left section: Camera and Map tabs (span 3 cols on desktop).
 * - Right section: Control panel, Battery panel, Diagnostics panel (span 2 cols on desktop).
 * - Theme is managed via {@link useThemeMode}.
 * - Transport is provided by {@link useControlTransport}.
 * - Battery state is fetched via {@link useBattery}.
 * - Fully responsive using Tailwind grid + shadcn/ui.
 *
 * @example
 * ```tsx
 * <GordonBotDashboard />
 * ```
 */
export default function GordonBotDashboard() {
  const theme = useThemeMode();
  const transport = useControlTransport(CONTROL_WS_PATH)
  const { data: battery } = useBattery(10000);
  const [activeTab, setActiveTab] = useState<"camera" | "lidar" | "map">("camera")
  const showCameraOverlay = activeTab === "map"

  // Auto-connect drive controls on load; clean up on unmount
  useEffect(() => {
    transport.connect();
    return () => { transport.disconnect(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  return (
    <CameraStreamProvider autoStart={true}>
      <SlamModeProvider>
        <TooltipProvider>
          <div className="min-h-dvh w-full bg-background text-foreground">
          <TopBar
            mode={theme.mode}
            onModeChange={theme.setMode}
            transportStatus={transport.status}
            batteryPercent={battery?.percent}
          />

        <main className="mx-auto max-w-7xl gap-4 px-4 py-4 grid grid-cols-1 lg:grid-cols-5">
          {/* Left: Camera/Map/LIDAR tabs (span 3 cols on desktop) */}
          <section className="lg:col-span-3 space-y-4">
            <div>
              <Tabs value={activeTab} onValueChange={(value) => setActiveTab(value as typeof activeTab)} className="w-full">
                <TabsList className="grid w-full grid-cols-3">
                  <TabsTrigger value="camera" className="flex items-center gap-2"><Camera className="h-4 w-4"/> Camera</TabsTrigger>
                  <TabsTrigger value="lidar" className="flex items-center gap-2"><Radar className="h-4 w-4"/> LiDAR</TabsTrigger>
                  <TabsTrigger value="map" className="flex items-center gap-2"><MapIcon className="h-4 w-4"/> Map</TabsTrigger>
                </TabsList>
                <TabsContent value="camera" className="mt-3">
                  <CameraPanel />
                </TabsContent>
                <TabsContent value="lidar" className="mt-3">
                  <LidarPanel />
                </TabsContent>
                <TabsContent value="map" className="mt-3">
                  <SlamMapPanel />
                </TabsContent>
              </Tabs>
            </div>
            <NavigationPanel />
          </section>

          {/* Right: Controls + Battery + Diagnostics (span 2 cols on desktop) */}
          <aside className="lg:col-span-2 space-y-4">
            <ControlPanel transport={transport} />
            <BatteryPanel />
            <OrientationPanel />
            <DiagnosticsPanel transport={transport} />
          </aside>
        </main>

        <footer className="mx-auto max-w-7xl px-4 pb-6 pt-2 text-center text-xs text-muted-foreground">
          <div className="flex items-center justify-center gap-2">
            <span>GordonBot Dashboard</span>
            <a
              href="https://github.com/keithvassallomt/gordonbot"
              target="_blank"
              rel="noopener noreferrer"
              className="inline-flex items-center gap-1 hover:text-foreground transition-colors"
              aria-label="View on GitHub"
            >
              <Github className="h-3.5 w-3.5" />
            </a>
          </div>
        </footer>
        </div>
        {showCameraOverlay && <CameraOverlay />}
      </TooltipProvider>
    </SlamModeProvider>
    </CameraStreamProvider>
  );
}
