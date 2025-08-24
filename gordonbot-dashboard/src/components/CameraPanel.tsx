import { useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Activity, Camera as CameraIcon } from "lucide-react"


/**
 * Camera panel component.
 *
 * Displays the robot camera feed (WebRTC or placeholder).
 * Allows starting a local webcam demo via getUserMedia for testing,
 * and stopping it to release tracks.
 *
 * @remarks
 * - Placeholder UI shown when inactive.
 * - Responsive with aspect ratio preserved.
 * - Styled with shadcn/ui components.
 *
 * @example
 * ```tsx
 * <CameraPanel />
 * ```
 */
export default function CameraPanel() {
  const videoRef = useRef<HTMLVideoElement | null>(null)
  const [active, setActive] = useState(false)

  /**
   * Start local demo mode using the device camera.
   * Assigns MediaStream to video element and plays it.
   * Sets `active` to true on success.
   */
  const startLocalDemo = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false })
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        await videoRef.current.play()
        setActive(true)
      }
    } catch {
      setActive(false)
    }
  }

  /**
   * Stop the current video stream and release camera resources.
   * Clears the video element srcObject and sets `active` to false.
   */
  const stop = () => {
    const v = videoRef.current
    const s = (v?.srcObject as MediaStream | null)
    s?.getTracks().forEach((t) => t.stop())
    if (v) v.srcObject = null
    setActive(false)
  }

  return (
    <Card className="h-full">
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <CameraIcon className="h-4 w-4" /> Camera
        </CardTitle>
        <div className="flex items-center gap-2">
          <Badge variant={active ? "default" : "secondary"} className="flex items-center gap-1">
            {active ? <Activity className="h-3 w-3" /> : <CameraIcon className="h-3 w-3" />}
            {active ? "Live" : "Idle"}
          </Badge>
          <div className="flex gap-2">
            {!active ? (
              <Button size="sm" onClick={startLocalDemo}>
                Demo Connect
              </Button>
            ) : (
              <Button size="sm" variant="secondary" onClick={stop}>
                Stop
              </Button>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="aspect-video w-full overflow-hidden rounded-lg border bg-muted">
          <video ref={videoRef} className="h-full w-full object-cover" playsInline muted />
          {!active && (
            <div className="flex h-full w-full items-center justify-center bg-gradient-to-br from-background to-muted">
              <div className="text-center text-sm opacity-70">
                <p className="font-medium">WebRTC feed placeholder</p>
                <p>Use the button above to start a local camera demo, or wire this panel to your robot stream.</p>
              </div>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}