import { useCallback, useEffect, useMemo, useRef, useState } from "react"

import {
  API_BASE,
  VIDEO_MJPEG_ENDPOINT,
  VIDEO_RAW_START_ENDPOINT,
  VIDEO_RAW_STOP_ENDPOINT,
  VIDEO_STATUS_ENDPOINT,
  VIDEO_WHEP_BASE,
  VIDEO_WHEP_STREAM_ANNOT,
  VIDEO_WHEP_STREAM_RAW,
} from "@/components/config"

export type StreamTech = "WebRTC" | "MJPEG" | "None"
export type StreamKind = "raw" | "annotated"
export type DecoderMode = "cpu" | "hailo" | null

export interface UseCameraStreamOptions {
  /**
   * Automatically attempt to establish the WHEP connection on mount.
   * Defaults to true to match the legacy CameraPanel behaviour.
   */
  autoStart?: boolean
  /**
   * Delay (ms) before the first auto-start attempt.
   */
  autoStartDelayMs?: number
  /**
   * Delay (ms) before retrying an auto-start after a failure.
   */
  autoRetryDelayMs?: number
}

export interface CameraStreamControls {
  videoRef: React.RefObject<HTMLVideoElement | null>
  startLocalDemo: () => Promise<void>
  startWebRTC: (opts?: { auto?: boolean }) => Promise<boolean>
  stop: (opts?: { keepRawPublisher?: boolean }) => Promise<void>
  setStreamKind: React.Dispatch<React.SetStateAction<StreamKind>>
}

export interface CameraStreamState extends CameraStreamControls {
  active: boolean
  connecting: boolean
  streamTech: StreamTech
  streamKind: StreamKind
  decoderMode: DecoderMode
  rawFeedActive: boolean
  rawFeedStartSupported: boolean
  initialising: boolean
  whepUrl: string | null
  mjpegUrl: string
  mediaStream: MediaStream | null
  getCurrentStream: () => MediaStream | null
  hasAudioTrack: boolean
  connectionAttempt: number
  waitingForStream: boolean
}

export function useCameraStream(options: UseCameraStreamOptions = {}): CameraStreamState {
  const {
    autoStart = true,
    autoStartDelayMs = 2500,
    autoRetryDelayMs = 3000,
  } = options

  const videoRef = useRef<HTMLVideoElement | null>(null)
  const pcRef = useRef<RTCPeerConnection | null>(null)
  const autoStartTimeoutRef = useRef<number | null>(null)

  const [active, setActive] = useState(false)
  const [connecting, setConnecting] = useState(false)
  const [streamTech, setStreamTech] = useState<StreamTech>("None")
  const [streamKind, setStreamKind] = useState<StreamKind>("raw")
  const [decoderMode, setDecoderMode] = useState<DecoderMode>(null)
  const [rawFeedActive, setRawFeedActive] = useState(false)
  const [rawFeedStartSupported, setRawFeedStartSupported] = useState(true)
  const [initialising, setInitialising] = useState(true)
  const [mediaStream, setMediaStream] = useState<MediaStream | null>(null)
  const [hasAudioTrack, setHasAudioTrack] = useState(false)
  const [connectionAttempt, setConnectionAttempt] = useState(0)
  const [waitingForStream, setWaitingForStream] = useState(false)
  const streamRef = useRef<MediaStream | null>(null)

  const updateMediaStream = useCallback((stream: MediaStream | null) => {
    streamRef.current = stream
    setMediaStream(stream)
    if (!stream) {
      setHasAudioTrack(false)
      return
    }
    const audioTracks = stream.getAudioTracks()
    const hasLiveAudio = audioTracks.some((track) => track.readyState === "live" && !track.muted)
    setHasAudioTrack(hasLiveAudio)
  }, [])

  const getCurrentStream = useCallback(() => streamRef.current, [])

  const whepUrl = useMemo(() => {
    if (!VIDEO_WHEP_BASE) return null
    const stream = streamKind === "annotated" ? VIDEO_WHEP_STREAM_ANNOT : VIDEO_WHEP_STREAM_RAW
    return API_BASE + VIDEO_WHEP_BASE + stream
  }, [streamKind])

  const mjpegUrl = useMemo(() => `${API_BASE + VIDEO_MJPEG_ENDPOINT}?t=${Date.now()}`, [])

  const stopRawFeed = useCallback(async () => {
    if (!rawFeedStartSupported || !rawFeedActive) return
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
  }, [rawFeedActive, rawFeedStartSupported])

  const ensureRawFeed = useCallback(async () => {
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
  }, [rawFeedActive, rawFeedStartSupported])

  const startLocalDemo = useCallback(async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: true, audio: false })
      if (videoRef.current) {
        videoRef.current.srcObject = stream
        await videoRef.current.play()
        setActive(true)
        setStreamTech("MJPEG")
        updateMediaStream(stream)
      }
    } catch {
      setActive(false)
    }
  }, [updateMediaStream])

  const stop = useCallback(
    async ({ keepRawPublisher = false }: { keepRawPublisher?: boolean } = {}) => {
      const existing = videoRef.current?.srcObject as MediaStream | null
      existing?.getTracks().forEach((track) => track.stop())
      if (videoRef.current) {
        videoRef.current.srcObject = null
      }
      setActive(false)
      setStreamTech("None")
      try {
        pcRef.current?.close()
      } catch {
        // ignore close errors
      }
      pcRef.current = null
      setInitialising(false)
      const currentStream = streamRef.current
      currentStream?.getTracks().forEach((track) => {
        try {
          track.stop()
        } catch {
          // ignore stop errors
        }
      })
      updateMediaStream(null)
      if (!keepRawPublisher) {
        await stopRawFeed()
      }
    },
    [stopRawFeed, updateMediaStream],
  )

  const checkStreamReady = useCallback(async (streamName: string): Promise<boolean> => {
    try {
      const response = await fetch(`${API_BASE}/api/video/whep/${streamName}/ready`, {
        cache: "no-store"
      })
      if (!response.ok) return false
      const data: { ready?: boolean } = await response.json()
      return data.ready === true
    } catch {
      return false
    }
  }, [])

  const startWebRTC = useCallback(
    async ({ auto = false }: { auto?: boolean } = {}) => {
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

      // Check if stream is ready before attempting connection
      const streamName = streamKind === "annotated" ? VIDEO_WHEP_STREAM_ANNOT : VIDEO_WHEP_STREAM_RAW
      const ready = await checkStreamReady(streamName)
      if (!ready) {
        setWaitingForStream(true)
        setConnecting(false)
        if (auto) {
          setInitialising(true)
        }
        return false
      }

      try {
        await ensureRawFeed()
        const pc = new RTCPeerConnection()
        pcRef.current = pc

        const stream = new MediaStream()
        if (videoRef.current) {
          videoRef.current.srcObject = stream
          // @ts-expect-error latencyHint is still experimental
          videoRef.current.latencyHint = "interactive"
        }

        pc.addTransceiver("video", { direction: "recvonly" })
        pc.addTransceiver("audio", { direction: "recvonly" })
        pc.ontrack = (event) => {
          stream.addTrack(event.track)
          updateMediaStream(stream)
          event.track.addEventListener("ended", () => {
            updateMediaStream(stream)
          })
          event.track.addEventListener("mute", () => {
            updateMediaStream(stream)
          })
          event.track.addEventListener("unmute", () => {
            updateMediaStream(stream)
          })
        }
        updateMediaStream(stream)

        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        await new Promise<void>((resolve) => {
          if (pc.iceGatheringState === "complete") {
            resolve()
            return
          }
          const handleState = () => {
            if (pc.iceGatheringState === "complete") {
              pc.removeEventListener("icegatheringstatechange", handleState)
              resolve()
            }
          }
          pc.addEventListener("icegatheringstatechange", handleState)
          setTimeout(() => {
            pc.removeEventListener("icegatheringstatechange", handleState)
            resolve()
          }, 500)
        })

        const local = pc.localDescription
        if (!local) throw new Error("No local description after createOffer")

        const response = await fetch(whepUrl, {
          method: "POST",
          headers: { "Content-Type": "application/sdp" },
          body: local.sdp || "",
        })

        if (!response.ok) {
          throw new Error(`WHEP POST failed: ${response.status}`)
        }

        const answerSdp = await response.text()
        await pc.setRemoteDescription({ type: "answer", sdp: answerSdp })

        if (videoRef.current) {
          await videoRef.current.play()
        }
        setActive(true)
        setStreamTech("WebRTC")
        if (streamKind === "raw") {
          setRawFeedActive(true)
        }
        setInitialising(false)
        setWaitingForStream(false)
        setConnectionAttempt(0)
        return true
      } catch (error) {
        try {
          pcRef.current?.close()
        } catch {
          // ignore
        }
        pcRef.current = null
        setActive(false)
        setStreamTech("None")
        if (auto) {
          setInitialising(true)
        }
        updateMediaStream(null)
        // Don't log 404 errors - they're expected when stream is starting up
        if (error instanceof Error && !error.message.includes('404')) {
          console.debug('WebRTC connection failed:', error.message)
        }
        return false
      } finally {
        setConnecting(false)
      }
    },
    [ensureRawFeed, streamKind, updateMediaStream, whepUrl, checkStreamReady],
  )

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

  useEffect(() => {
    if (!autoStart || !whepUrl) {
      setInitialising(false)
      return () => {
        void stop()
      }
    }

    let cancelled = false

    const scheduleAttempt = (delay: number) => {
      if (cancelled) return
      autoStartTimeoutRef.current = window.setTimeout(async () => {
        autoStartTimeoutRef.current = null
        if (cancelled || pcRef.current) {
          return
        }
        setConnectionAttempt((prev) => prev + 1)
        const success = await startWebRTC({ auto: true })
        if (!success && !cancelled) {
          scheduleAttempt(autoRetryDelayMs)
        }
      }, delay)
    }

    scheduleAttempt(autoStartDelayMs)

    return () => {
      cancelled = true
      if (autoStartTimeoutRef.current !== null) {
        window.clearTimeout(autoStartTimeoutRef.current)
        autoStartTimeoutRef.current = null
      }
      void stop()
    }
  }, [autoRetryDelayMs, autoStart, autoStartDelayMs, startWebRTC, stop, whepUrl])

  useEffect(() => {
    if (!pcRef.current) return

    const preserveRaw = streamKind === "annotated"
    const restart = async () => {
      await stop({ keepRawPublisher: preserveRaw })
      await startWebRTC()
    }
    void restart()
  }, [startWebRTC, stop, streamKind])

  return {
    videoRef,
    startLocalDemo,
    startWebRTC,
    stop,
    setStreamKind,
    active,
    connecting,
    streamTech,
    streamKind,
    decoderMode,
    rawFeedActive,
    rawFeedStartSupported,
    initialising,
    whepUrl,
    mjpegUrl,
    mediaStream,
    getCurrentStream,
    hasAudioTrack,
    connectionAttempt,
    waitingForStream,
  }
}
