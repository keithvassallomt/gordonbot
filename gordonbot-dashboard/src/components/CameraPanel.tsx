import { useEffect, useMemo, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Activity, Camera as CameraIcon } from "lucide-react"
import { API_BASE, VIDEO_MJPEG_ENDPOINT, VIDEO_STATUS_ENDPOINT, VIDEO_RAW_START_ENDPOINT, VIDEO_RAW_STOP_ENDPOINT, VIDEO_WHEP_BASE, VIDEO_WHEP_STREAM_RAW, VIDEO_WHEP_STREAM_ANNOT } from "./config"


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
  const [rawFeedActive, setRawFeedActive] = useState(false)
  const [rawFeedStartSupported, setRawFeedStartSupported] = useState(true)
  const [initialising, setInitialising] = useState(true)
  const autoStartTimeoutRef = useRef<number | null>(null)

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
  const stopRawFeed = async () => {
    if (!rawFeedStartSupported) return
    try {
      const res = await fetch(API_BASE + VIDEO_RAW_STOP_ENDPOINT, { method: "POST" })
      if (res.ok) {
        setRawFeedActive(false)
      } else if (res.status === 404) {
        setRawFeedStartSupported(false)
      }
    } catch {
      setRawFeedActive(false)
    }
  }

  const stop = async ({ keepRawPublisher = false }: { keepRawPublisher?: boolean } = {}) => {
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
    setInitialising(false)
    if (!keepRawPublisher && rawFeedActive) {
      await stopRawFeed()
    }
  }

  const mjpegUrl = useMemo(() => API_BASE + VIDEO_MJPEG_ENDPOINT + `?t=${Date.now()}` , [])
  const whepUrl = useMemo(() => {
    const stream = streamKind === "raw" ? VIDEO_WHEP_STREAM_RAW : VIDEO_WHEP_STREAM_ANNOT
    return API_BASE + VIDEO_WHEP_BASE + stream
  }, [streamKind])

  const containerClass = "aspect-video"
  const videoObjectClass = "object-cover"

  const ensureRawFeed = async () => {
    if (!rawFeedStartSupported || rawFeedActive) return
    try {
      const res = await fetch(API_BASE + VIDEO_RAW_START_ENDPOINT, { method: "POST" })
      if (res.ok) {
        setRawFeedActive(true)
      } else if (res.status === 404) {
        setRawFeedStartSupported(false)
      }
    } catch {
      setRawFeedActive(false)
    }
  }

  const startWebRTC = async ({ auto = false }: { auto?: boolean } = {}) => {
    if (!whepUrl) return false
    if (pcRef.current) return true
    if (autoStartTimeoutRef.current !== null) {
      window.clearTimeout(autoStartTimeoutRef.current)
      autoStartTimeoutRef.current = null
    }
    if (!auto) {
      setInitialising(false)
    }
    setConnecting(true)
    try {
      await ensureRawFeed()
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
      if (streamKind === "raw") setRawFeedActive(true)
      setInitialising(false)
      return true
    } catch {
      // Reset state on error
      try { pcRef.current?.close() } catch {}
      pcRef.current = null
      setActive(false)
      setStreamTech("None")
      if (auto) {
        setInitialising(true)
      }
      return false
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
        const data: { decoder?: string | null; detect_enabled?: boolean; raw_active?: boolean } = await res.json()
        if (cancelled) return
        if (data.detect_enabled && typeof data.decoder === "string") {
          const normalized = data.decoder.toLowerCase()
          setDecoderMode(normalized === "hailo" ? "hailo" : "cpu")
        } else {
          setDecoderMode(null)
        }
        if (typeof data.raw_active === "boolean") {
          setRawFeedActive(data.raw_active)
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

  // Auto-connect to WebRTC after a short delay so the dashboard can stabilise.
  useEffect(() => {
    if (!whepUrl) {
      setInitialising(false)
      return () => { void stop() }
    }
    let cancelled = false
    const initialDelayMs = 2500
    const retryDelayMs = 3000

    const scheduleAttempt = (delay: number) => {
      if (cancelled) return
      autoStartTimeoutRef.current = window.setTimeout(async () => {
        autoStartTimeoutRef.current = null
        if (cancelled || pcRef.current) {
          return
        }
        const success = await startWebRTC({ auto: true })
        if (!success && !cancelled) {
          scheduleAttempt(retryDelayMs)
        }
      }, delay)
    }

    scheduleAttempt(initialDelayMs)

    return () => {
      cancelled = true
      if (autoStartTimeoutRef.current !== null) {
        window.clearTimeout(autoStartTimeoutRef.current)
        autoStartTimeoutRef.current = null
      }
      void stop()
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // If stream kind changes while connected, restart the WebRTC session
  useEffect(() => {
    if (pcRef.current) {
      const preserveRaw = streamKind === "annotated"
      const restart = async () => {
        await stop({ keepRawPublisher: preserveRaw })
        await startWebRTC()
      }
      void restart()
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
              onChange={(e) => {
                const next = e.target.value as "raw" | "annotated"
                setStreamKind(next)
              }}
            >
              <option value="raw">Raw</option>
              <option value="annotated">Annotated</option>
            </select>
          </div>
          <div className="flex gap-2">
            {VIDEO_WHEP_BASE ? (
              !active ? (
                <Button size="sm" onClick={() => { void startWebRTC() }} disabled={connecting}>
                  {connecting ? "Connecting..." : "Connect"}
                </Button>
              ) : (
                <Button size="sm" variant="secondary" onClick={() => { void stop() }}>
                  Disconnect
                </Button>
              )
            ) : (
              !active ? (
                <Button size="sm" onClick={startLocalDemo}>
                  Demo Connect
                </Button>
              ) : (
                <Button size="sm" variant="secondary" onClick={() => { void stop() }}>
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
          {(whepUrl && (initialising || connecting)) && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-3 bg-black/60 text-white">
              <div className="h-9 w-9 animate-spin rounded-full border-2 border-white/40 border-t-transparent" />
              <p className="text-sm font-medium">
                {initialising ? "Initialising camera feed" : "Connecting to camera"}
              </p>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
