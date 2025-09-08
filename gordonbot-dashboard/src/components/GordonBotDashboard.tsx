import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { useEffect } from "react";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { TooltipProvider } from "@/components/ui/tooltip";
import { Camera, Map as MapIcon } from "lucide-react"

import TopBar from "./TopBar"
import CameraPanel from "./CameraPanel"
import SensorsPanel from "./SensorsPanel"
import MapCanvas from "./MapCanvas";
import BatteryPanel from "./BatteryPanel";
import DiagnosticsPanel from "./DiagnosticsPanel"
import ControlPanel from "./ControlPanel";

import { useBattery } from "./hooks/useBattery";
import { useControlTransport } from "./hooks/useControlTransport"
import { useThemeMode } from "./hooks/useThemeMode"

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

  // Auto-connect drive controls on load; clean up on unmount
  useEffect(() => {
    transport.connect();
    return () => { transport.disconnect(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  return (
    <TooltipProvider>
      <div className="min-h-dvh w-full bg-background text-foreground">
        <TopBar
          mode={theme.mode}
          onModeChange={theme.setMode}
          transportStatus={transport.status}
          batteryPercent={battery?.percent}
        />

        <main className="mx-auto max-w-7xl gap-4 px-4 py-4 grid grid-cols-1 lg:grid-cols-5">
          {/* Left: Camera/Map tabs (span 3 cols on desktop) */}
          <section className="lg:col-span-3 space-y-4">
            <Tabs defaultValue="camera" className="w-full">
              <TabsList className="grid w-full grid-cols-2">
                <TabsTrigger value="camera" className="flex items-center gap-2"><Camera className="h-4 w-4"/> Camera</TabsTrigger>
                <TabsTrigger value="map" className="flex items-center gap-2"><MapIcon className="h-4 w-4"/> Map</TabsTrigger>
              </TabsList>
              <TabsContent value="camera" className="mt-3">
                <CameraPanel />
                <div className="mt-4">
                  <SensorsPanel />
                </div>
              </TabsContent>
              <TabsContent value="map" className="mt-3">
                <Card className="h-[400px] sm:h-[480px] lg:h-[560px]">
                  <CardHeader>
                    <CardTitle className="flex items-center gap-2 text-base"><MapIcon className="h-4 w-4"/> SLAM Map (placeholder)</CardTitle>
                  </CardHeader>
                  <CardContent className="h-full">
                    <MapCanvas />
                  </CardContent>
                </Card>
              </TabsContent>
            </Tabs>
          </section>

          {/* Right: Controls + Battery + Diagnostics (span 2 cols on desktop) */}
          <aside className="lg:col-span-2 space-y-4">
            <ControlPanel transport={transport} />
            <BatteryPanel />
            <DiagnosticsPanel transport={transport} />
          </aside>
        </main>

        <footer className="mx-auto max-w-7xl px-4 pb-6 pt-2 text-center text-xs text-muted-foreground">
          Built with React, Tailwind, and shadcn/ui • Keyboard + On-screen controls • Theme persists in localStorage
        </footer>
      </div>
    </TooltipProvider>
  );
}
