import { useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Activity, Camera as CameraIcon } from "lucide-react"
import { API_BASE, VIDEO_MJPEG_ENDPOINT } from "./config"


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

  const mjpegUrl = useMemo(() => API_BASE + VIDEO_MJPEG_ENDPOINT + `?t=${Date.now()}` , [])

  return (
    <Card className="h-full">
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <CameraIcon className="h-4 w-4" /> Camera
        </CardTitle>
        <div className="flex items-center gap-2">
          <Badge variant="default" className="flex items-center gap-1">
            <Activity className="h-3 w-3" /> Live
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
        <div className="aspect-video w-full overflow-hidden rounded-lg border bg-muted relative">
          {/* Robot MJPEG feed */}
          <img src={mjpegUrl} alt="Robot camera" className="absolute inset-0 h-full w-full object-cover" />
          {/* Optional local demo overlay */}
          <video ref={videoRef} className="absolute inset-0 h-full w-full object-cover" playsInline muted style={{ display: active ? 'block' : 'none' }} />
        </div>
      </CardContent>
    </Card>
  )
}
