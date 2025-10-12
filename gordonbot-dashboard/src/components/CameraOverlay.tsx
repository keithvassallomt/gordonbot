import { useEffect, useMemo, useRef, useState } from "react"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Camera, ChevronDown, ChevronUp, GripVertical } from "lucide-react"

import { useCameraStream } from "./hooks/useCameraStream"

const MIN_WIDTH = 240
const MIN_HEIGHT = 135
const VIEWPORT_PADDING = 16
const COLLAPSED_HEIGHT = 56

const clampWithin = (value: number, min: number, max: number) => {
  if (!Number.isFinite(max) || max < min) {
    return min
  }
  return Math.min(Math.max(value, min), max)
}

export function CameraOverlay(): JSX.Element {
  const {
    videoRef,
    setStreamKind,
    active,
    connecting,
    streamTech,
    streamKind,
    decoderMode,
    initialising,
    whepUrl,
    mjpegUrl,
  } = useCameraStream()

  const [collapsed, setCollapsed] = useState(false)
  const [position, setPosition] = useState({ x: VIEWPORT_PADDING, y: 120 })
  const [size, setSize] = useState({ width: 320, height: 180 })
  const [isDragging, setIsDragging] = useState(false)
  const [isResizing, setIsResizing] = useState(false)

  const positionRef = useRef(position)
  const sizeRef = useRef(size)
  const collapsedRef = useRef(collapsed)

  useEffect(() => {
    positionRef.current = position
  }, [position])

  useEffect(() => {
    sizeRef.current = size
  }, [size])

  useEffect(() => {
    collapsedRef.current = collapsed
  }, [collapsed])

  const effectiveSize = useMemo(() => {
    return collapsed
      ? { width: Math.max(MIN_WIDTH, size.width), height: COLLAPSED_HEIGHT }
      : size
  }, [collapsed, size.height, size.width])

  useEffect(() => {
    const maxX = window.innerWidth - effectiveSize.width - VIEWPORT_PADDING
    const maxY = window.innerHeight - effectiveSize.height - VIEWPORT_PADDING
    setPosition((prev) => ({
      x: clampWithin(prev.x, VIEWPORT_PADDING, maxX),
      y: clampWithin(prev.y, VIEWPORT_PADDING, maxY),
    }))
  }, [effectiveSize.height, effectiveSize.width])

  useEffect(() => {
    const handleResize = () => {
      const currentSize = sizeRef.current
      const currentPosition = positionRef.current
      const isCollapsed = collapsedRef.current

      const targetWidth = isCollapsed ? Math.max(MIN_WIDTH, currentSize.width) : currentSize.width
      const targetHeight = isCollapsed ? COLLAPSED_HEIGHT : currentSize.height

      const maxX = window.innerWidth - targetWidth - VIEWPORT_PADDING
      const maxY = window.innerHeight - targetHeight - VIEWPORT_PADDING

      setPosition({
        x: clampWithin(currentPosition.x, VIEWPORT_PADDING, maxX),
        y: clampWithin(currentPosition.y, VIEWPORT_PADDING, maxY),
      })

      if (!isCollapsed) {
        const maxWidth = Math.max(MIN_WIDTH, window.innerWidth - positionRef.current.x - VIEWPORT_PADDING)
        const maxHeight = Math.max(MIN_HEIGHT, window.innerHeight - positionRef.current.y - VIEWPORT_PADDING)
        setSize((prev) => ({
          width: Math.min(prev.width, maxWidth),
          height: Math.min(prev.height, maxHeight),
        }))
      }
    }

    window.addEventListener("resize", handleResize)
    return () => {
      window.removeEventListener("resize", handleResize)
    }
  }, [])

  const handleDragPointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    event.preventDefault()
    const startX = event.clientX
    const startY = event.clientY
    const offsetX = startX - position.x
    const offsetY = startY - position.y
    setIsDragging(true)

    const handlePointerMove = (moveEvent: PointerEvent) => {
      const proposedX = moveEvent.clientX - offsetX
      const proposedY = moveEvent.clientY - offsetY
      const maxX = window.innerWidth - effectiveSize.width - VIEWPORT_PADDING
      const maxY = window.innerHeight - effectiveSize.height - VIEWPORT_PADDING
      setPosition({
        x: clampWithin(proposedX, VIEWPORT_PADDING, maxX),
        y: clampWithin(proposedY, VIEWPORT_PADDING, maxY),
      })
    }

    const handlePointerUp = () => {
      setIsDragging(false)
      window.removeEventListener("pointermove", handlePointerMove)
      window.removeEventListener("pointerup", handlePointerUp)
    }

    window.addEventListener("pointermove", handlePointerMove)
    window.addEventListener("pointerup", handlePointerUp)
  }

  const handleResizePointerDown = (event: React.PointerEvent<HTMLDivElement>) => {
    event.preventDefault()
    event.stopPropagation()
    const startX = event.clientX
    const startY = event.clientY
    const startWidth = size.width
    const startHeight = size.height
    const startPosition = position
    setIsResizing(true)

    const handlePointerMove = (moveEvent: PointerEvent) => {
      const deltaX = moveEvent.clientX - startX
      const deltaY = moveEvent.clientY - startY

      const maxWidth = Math.max(MIN_WIDTH, window.innerWidth - startPosition.x - VIEWPORT_PADDING)
      const maxHeight = Math.max(MIN_HEIGHT, window.innerHeight - startPosition.y - VIEWPORT_PADDING)

      const nextWidth = clampWithin(startWidth + deltaX, MIN_WIDTH, maxWidth)
      const nextHeight = clampWithin(startHeight + deltaY, MIN_HEIGHT, maxHeight)

      setSize({ width: nextWidth, height: nextHeight })
    }

    const handlePointerUp = () => {
      setIsResizing(false)
      window.removeEventListener("pointermove", handlePointerMove)
      window.removeEventListener("pointerup", handlePointerUp)
    }

    window.addEventListener("pointermove", handlePointerMove)
    window.addEventListener("pointerup", handlePointerUp)
  }

  const toggleCollapsed = () => {
    setCollapsed((prev) => !prev)
  }

  const liveBadgeVariant = streamTech === "WebRTC" ? "default" : "secondary"

  const overlayStyle: React.CSSProperties = {
    top: position.y,
    left: position.x,
    width: effectiveSize.width,
    ...(collapsed ? {} : { height: effectiveSize.height }),
  }

  return (
    <div
      className={`fixed z-40 flex flex-col overflow-hidden rounded-lg border border-border bg-background/95 shadow-lg backdrop-blur ${
        isDragging ? "cursor-grabbing" : ""
      }`}
      style={overlayStyle}
    >
      <div
        className={`flex items-center justify-between gap-2 border-b px-3 py-2 ${
          isDragging ? "cursor-grabbing" : "cursor-grab"
        }`}
        onPointerDown={handleDragPointerDown}
      >
        <div className="flex items-center gap-2 select-none">
          <GripVertical className="h-4 w-4 text-muted-foreground" />
          <Camera className="h-4 w-4 text-muted-foreground" />
          <span className="text-sm font-medium">Camera view</span>
        </div>
        <div className="flex items-center gap-2">
          <Badge variant={liveBadgeVariant} className="text-[10px] uppercase">
            {streamTech}
          </Badge>
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7"
            onClick={toggleCollapsed}
            aria-label={collapsed ? "Expand camera overlay" : "Collapse camera overlay"}
          >
            {collapsed ? <ChevronDown className="h-4 w-4" /> : <ChevronUp className="h-4 w-4" />}
          </Button>
        </div>
      </div>

      {!collapsed && (
        <div className="relative flex-1 bg-black">
          {whepUrl ? (
            <video ref={videoRef} className="absolute inset-0 h-full w-full object-cover" playsInline muted />
          ) : (
            <>
              <img src={mjpegUrl} alt="Robot camera" className="absolute inset-0 h-full w-full object-cover" />
              <video
                ref={videoRef}
                className="absolute inset-0 h-full w-full object-cover"
                playsInline
                muted
                style={{ display: active ? "block" : "none" }}
              />
            </>
          )}

          {(whepUrl && (initialising || connecting)) && (
            <div className="absolute inset-0 flex flex-col items-center justify-center gap-2 bg-black/60 text-white">
              <div className="h-8 w-8 animate-spin rounded-full border-2 border-white/40 border-t-transparent" />
              <span className="text-xs font-medium">
                {initialising ? "Initialising feed…" : "Connecting…"}
              </span>
            </div>
          )}

          <div className="pointer-events-auto absolute left-2 top-2 flex items-center gap-2">
            <Badge variant="secondary" className="text-[10px] uppercase">
              {streamKind}
            </Badge>
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
          </div>

          <div className="pointer-events-auto absolute right-2 top-2">
            <select
              className="rounded-md border border-border bg-background/90 px-2 py-1 text-xs text-foreground shadow-sm outline-none"
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

          <div className="pointer-events-none absolute inset-x-2 bottom-2 flex justify-end gap-2 text-[10px] text-muted-foreground">
            <span className="pointer-events-auto flex items-center gap-1 rounded-md bg-background/80 px-2 py-1 shadow">
              {streamTech}
            </span>
          </div>

          <div
            className={`pointer-events-auto absolute bottom-1 right-1 h-4 w-4 rounded-sm ${
              isResizing ? "cursor-grabbing bg-primary/60" : "cursor-se-resize bg-muted-foreground/60"
            }`}
            onPointerDown={handleResizePointerDown}
            aria-hidden="true"
          />
        </div>
      )}
    </div>
  )
}
