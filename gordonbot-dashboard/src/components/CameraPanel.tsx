import { useEffect, useRef, useState } from "react"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Activity, Camera as CameraIcon, Volume2, VolumeX } from "lucide-react"
import { useSharedCameraStream } from "./contexts/CameraStreamContext"

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
  const {
    registerVideoElement,
    startLocalDemo,
    startWebRTC,
    stop,
    setStreamKind,
    active,
    connecting,
    streamTech,
    streamKind,
    decoderMode,
    initialising,
    whepUrl,
    mjpegUrl,
    getCurrentStream,
    hasAudioTrack,
  } = useSharedCameraStream()

  const videoRef = useRef<HTMLVideoElement | null>(null)
  const audioRef = useRef<HTMLAudioElement | null>(null)
  const [audioEnabled, setAudioEnabled] = useState(false)
  const [audioLoading, setAudioLoading] = useState(false)
  const [audioError, setAudioError] = useState<string | null>(null)

  const audioButtonDisabled = audioLoading || (!hasAudioTrack && !audioEnabled)

  // Register video element with shared stream
  useEffect(() => {
    if (!videoRef.current) return
    return registerVideoElement(videoRef.current)
  }, [registerVideoElement])

  useEffect(() => {
    const audioEl = audioRef.current
    if (!audioEnabled) {
      if (audioEl) {
        audioEl.pause()
        audioEl.srcObject = null
      }
      return
    }

    const stream = getCurrentStream()

    if (!stream || stream.getAudioTracks().length === 0 || !hasAudioTrack) {
      if (audioEl) {
        audioEl.pause()
        audioEl.srcObject = null
      }
      setAudioEnabled(false)
      setAudioError("Camera audio track unavailable")
      return
    }

    if (audioEl) {
      audioEl.srcObject = stream
      audioEl.muted = false
      const playResult = audioEl.play()
      if (playResult && typeof playResult.catch === "function") {
        playResult.catch((err: unknown) => {
          console.debug("audio play failed", err)
          setAudioError("Unable to start audio playback")
          setAudioEnabled(false)
        })
      }
    }
  }, [audioEnabled, getCurrentStream, hasAudioTrack])

  useEffect(() => {
    return () => {
      const audioEl = audioRef.current
      if (audioEl) {
        audioEl.pause()
        audioEl.srcObject = null
      }
    }
  }, [])

  const handleToggleAudio = async () => {
    if (audioEnabled) {
      const audioEl = audioRef.current
      if (audioEl) {
        audioEl.pause()
        audioEl.srcObject = null
      }
      setAudioEnabled(false)
      setAudioError(null)
      return
    }

    setAudioLoading(true)
    try {
      setAudioError(null)
      if (!active) {
        const success = await startWebRTC()
        if (!success) {
          throw new Error("Unable to connect camera stream")
        }
      }
      const stream = getCurrentStream() ?? (videoRef.current?.srcObject as MediaStream | null)
      if (!stream) {
        throw new Error("Camera stream unavailable")
      }
      if (stream.getAudioTracks().length === 0 || !hasAudioTrack) {
        throw new Error("Camera audio track unavailable")
      }
      setAudioError(null)
      setAudioEnabled(true)
    } catch (error) {
      const message = error instanceof Error ? error.message : "Failed to start audio"
      setAudioError(message)
      setAudioEnabled(false)
    } finally {
      setAudioLoading(false)
    }
  }

  const containerClass = "aspect-video"
  const videoObjectClass = "object-cover"

  return (
    <Card className="h-full">
      <CardHeader className="flex flex-row items-center justify-between space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <CameraIcon className="h-4 w-4" /> Camera
        </CardTitle>
        <div className="flex flex-col gap-2">
          <div className="flex flex-wrap items-center justify-end gap-2">
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
                onChange={(event) => {
                  const next = event.target.value as typeof streamKind
                  setStreamKind(next)
                }}
              >
                <option value="raw">Raw</option>
                <option value="annotated">Annotated</option>
              </select>
            </div>
            <div className="flex gap-2">
              {whepUrl ? (
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
                  <Button size="sm" onClick={() => { void startLocalDemo() }}>
                    Demo Connect
                  </Button>
                ) : (
                  <Button size="sm" variant="secondary" onClick={() => { void stop() }}>
                    Stop
                  </Button>
                )
              )}
            </div>
            {whepUrl && (
              <Button
                size="sm"
                variant={audioEnabled ? "default" : "outline"}
                onClick={() => { void handleToggleAudio() }}
                disabled={audioButtonDisabled}
              >
                {audioLoading ? (
                  "Audio…"
                ) : (
                  <>
                    {audioEnabled ? <VolumeX className="mr-1 h-4 w-4" /> : <Volume2 className="mr-1 h-4 w-4" />}
                    {audioEnabled ? "Mute audio" : "Hear audio"}
                  </>
                )}
              </Button>
            )}
            {!hasAudioTrack && whepUrl && !audioEnabled && !audioError && (
              <div className="flex items-center text-muted-foreground">
                <VolumeX className="h-3.5 w-3.5 opacity-60" aria-hidden="true" />
              </div>
            )}
          </div>
          {audioError && (
            <div className="w-full text-right text-xs text-destructive">
              {audioError}
            </div>
          )}
        </div>
      </CardHeader>
      <CardContent>
        <div className={`${containerClass} w-full overflow-hidden rounded-lg border bg-black relative`}>
          {whepUrl ? (
            <video ref={videoRef} className={`absolute inset-0 h-full w-full ${videoObjectClass}`} playsInline muted />
          ) : (
            <>
              <img src={mjpegUrl} alt="Robot camera" className={`absolute inset-0 h-full w-full ${videoObjectClass}`} />
              <video
                ref={videoRef}
                className={`absolute inset-0 h-full w-full ${videoObjectClass}`}
                playsInline
                muted
                style={{ display: active ? "block" : "none" }}
              />
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
        <audio ref={audioRef} className="hidden" />
      </CardContent>
    </Card>
  )
}
