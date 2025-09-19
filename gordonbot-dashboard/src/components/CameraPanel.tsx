import { useEffect, useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Activity, Camera as CameraIcon } from "lucide-react"
import { API_BASE, VIDEO_MJPEG_ENDPOINT, VIDEO_STATUS_ENDPOINT, VIDEO_WHEP_BASE, VIDEO_WHEP_STREAM_RAW, VIDEO_WHEP_STREAM_ANNOT } from "./config"


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
  const pcRef = useRef<RTCPeerConnection | null>(null)
  const [active, setActive] = useState(false)
  const [connecting, setConnecting] = useState(false)
  const [streamTech, setStreamTech] = useState<"WebRTC" | "MJPEG" | "None">("None")
  const [streamKind, setStreamKind] = useState<"raw" | "annotated">("raw")
  const [decoderMode, setDecoderMode] = useState<"cpu" | "hailo" | null>(null)

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
    try {
      pcRef.current?.close()
    } catch {}
    pcRef.current = null
    setStreamTech("None")
  }

  const mjpegUrl = useMemo(() => API_BASE + VIDEO_MJPEG_ENDPOINT + `?t=${Date.now()}` , [])
  const whepUrl = useMemo(() => {
    const stream = streamKind === "raw" ? VIDEO_WHEP_STREAM_RAW : VIDEO_WHEP_STREAM_ANNOT
    return API_BASE + VIDEO_WHEP_BASE + stream
  }, [streamKind])

  const containerClass = "aspect-video"
  const videoObjectClass = "object-cover"

  const startWebRTC = async () => {
    if (!whepUrl) return
    if (pcRef.current) return
    setConnecting(true)
    try {
      const pc = new RTCPeerConnection()
      pcRef.current = pc

      // Prepare a stream for the video element
      const stream = new MediaStream()
      if (videoRef.current) {
        videoRef.current.srcObject = stream
      }
      pc.addTransceiver("video", { direction: "recvonly" })
      pc.ontrack = (ev) => {
        stream.addTrack(ev.track)
      }

      // Create offer
      const offer = await pc.createOffer()
      await pc.setLocalDescription(offer)

      // Wait for ICE gathering to complete (no trickle for simplicity)
      await new Promise<void>((resolve) => {
        if (pc.iceGatheringState === "complete") {
          resolve()
        } else {
          const check = () => {
            if (pc.iceGatheringState === "complete") {
              pc.removeEventListener("icegatheringstatechange", check)
              resolve()
            }
          }
          pc.addEventListener("icegatheringstatechange", check)
          // Fallback timeout
          setTimeout(() => {
            pc.removeEventListener("icegatheringstatechange", check)
            resolve()
          }, 1500)
        }
      })

      const local = pc.localDescription
      if (!local) throw new Error("No localDescription after createOffer")

      // Send offer to WHEP endpoint
      const resp = await fetch(whepUrl, {
        method: "POST",
        headers: { "Content-Type": "application/sdp" },
        body: local.sdp || "",
      })
      if (!resp.ok) throw new Error(`WHEP POST failed: ${resp.status}`)
      const answerSdp = await resp.text()
      await pc.setRemoteDescription({ type: "answer", sdp: answerSdp })

      // Start playing
      if (videoRef.current) {
        await videoRef.current.play()
      }
      setActive(true)
      setStreamTech("WebRTC")
    } catch {
      // Reset state on error
      try { pcRef.current?.close() } catch {}
      pcRef.current = null
      setActive(false)
      setStreamTech("None")
    } finally {
      setConnecting(false)
    }
  }

  useEffect(() => {
    let cancelled = false
    const fetchStatus = async () => {
      try {
        const res = await fetch(API_BASE + VIDEO_STATUS_ENDPOINT, { cache: "no-store" })
        if (!res.ok) return
        const data: { decoder?: string | null; detect_enabled?: boolean } = await res.json()
        if (cancelled) return
        if (data.detect_enabled && typeof data.decoder === "string") {
          const normalized = data.decoder.toLowerCase()
          setDecoderMode(normalized === "hailo" ? "hailo" : "cpu")
        } else {
          setDecoderMode(null)
        }
      } catch {
        if (!cancelled) setDecoderMode(null)
      }
    }
    void fetchStatus()
    return () => {
      cancelled = true
    }
  }, [])

  // Auto-connect to WebRTC on mount if available
  useEffect(() => {
    if (whepUrl && !active && !connecting && !pcRef.current) {
      // Fire and forget; internal guards prevent double-start
      void startWebRTC()
    }
    // On unmount, ensure we stop any active stream
    return () => {
      stop()
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // If stream kind changes while connected, restart the WebRTC session
  useEffect(() => {
    if (pcRef.current) {
      stop()
      void startWebRTC()
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [streamKind])

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
          <Badge variant="outline">{streamTech}</Badge>
          {streamKind === "annotated" && decoderMode && (
            <Badge
              className={
                decoderMode === "hailo"
                  ? "border-transparent bg-emerald-500 text-white"
                  : "border-transparent bg-destructive text-white"
              }
            >
              {decoderMode === "hailo" ? "Hailo decode" : "CPU decode"}
            </Badge>
          )}
          <div className="flex items-center gap-1 rounded-md border px-2 py-1 text-xs">
            <label className="mr-1">Stream</label>
            <select
              className="bg-transparent outline-none"
              value={streamKind}
              onChange={(e) => setStreamKind(e.target.value as "raw" | "annotated")}
            >
              <option value="raw">Raw</option>
              <option value="annotated">Annotated</option>
            </select>
          </div>
          <div className="flex gap-2">
            {VIDEO_WHEP_BASE ? (
              !active ? (
                <Button size="sm" onClick={startWebRTC} disabled={connecting}>
                  {connecting ? "Connecting..." : "Connect"}
                </Button>
              ) : (
                <Button size="sm" variant="secondary" onClick={stop}>
                  Stop
                </Button>
              )
            ) : (
              !active ? (
                <Button size="sm" onClick={startLocalDemo}>
                  Demo Connect
                </Button>
              ) : (
                <Button size="sm" variant="secondary" onClick={stop}>
                  Stop
                </Button>
              )
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className={`${containerClass} w-full overflow-hidden rounded-lg border bg-black relative`}
        >
          {whepUrl ? (
            <video ref={videoRef} className={`absolute inset-0 h-full w-full ${videoObjectClass}`} playsInline muted />
          ) : (
            <>
              {/* Robot MJPEG feed (fallback) */}
              <img src={mjpegUrl} alt="Robot camera" className={`absolute inset-0 h-full w-full ${videoObjectClass}`} />
              {/* Optional local demo overlay */}
              <video ref={videoRef} className={`absolute inset-0 h-full w-full ${videoObjectClass}`} playsInline muted style={{ display: active ? 'block' : 'none' }} />
            </>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
