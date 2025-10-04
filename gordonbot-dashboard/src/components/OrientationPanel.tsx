import { useCallback, useEffect, useMemo, useRef, useState } from "react"
import { ChevronDown, Compass, RefreshCcw, RotateCcw, Save } from "lucide-react"
import * as THREE from "three"
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from "@/components/ui/collapsible"
import { useOrientation } from "@/components/hooks/useOrientation"
import type { OrientationNotification } from "@/components/hooks/useOrientation"
import { DEBUG_MODE } from "@/components/config"
import { cn } from "@/lib/utils"

const SMOOTHING = 0.15

function normalizeHeading(deg: number) {
  const normalized = ((deg % 360) + 360) % 360
  return normalized
}

function formatAngle(value: number | null) {
  if (value === null || Number.isNaN(value)) return "—"
  return `${value.toFixed(1)}°`
}

function calibrationLevelClass(level: number | undefined) {
  if (level == null) {
    return "border-muted/40 text-muted-foreground"
  }
  if (level >= 3) {
    return "border-emerald-500/40 text-emerald-400"
  }
  if (level >= 2) {
    return "border-amber-500/40 text-amber-400"
  }
  return "border-destructive/40 text-destructive"
}

export default function OrientationPanel() {
  const {
    frame,
    status,
    startCalibration,
    abortCalibration,
    notification,
    acknowledgeNotification,
    reconnecting,
    calibrating,
  } = useOrientation()

  const containerRef = useRef<HTMLDivElement | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const cubeRef = useRef<THREE.Mesh | null>(null)
  const currentQuatRef = useRef(new THREE.Quaternion())
  const targetQuatRef = useRef(new THREE.Quaternion())
  const offsetQuatRef = useRef(new THREE.Quaternion())
  const animationRef = useRef<number | null>(null)
  const hasInitialisedRef = useRef(false)

  // Store alignment rotation so we can account for it in recenter
  const alignmentRotationRef = useRef(new THREE.Quaternion())

  const [angles, setAngles] = useState<{ heading: number | null; pitch: number | null; roll: number | null }>(
    { heading: null, pitch: null, roll: null },
  )
  const [statusNote, setStatusNote] = useState<OrientationNotification | null>(null)
  const [isCalibrating, setIsCalibrating] = useState(false)
  const [calibrationLogs, setCalibrationLogs] = useState<string[]>([])
  const calibrationAbortRef = useRef<AbortController | null>(null)
  const logsEndRef = useRef<HTMLDivElement>(null)
  const frameRef = useRef(frame)
  const [isDebugging, setIsDebugging] = useState(false)
  const [debugLogs, setDebugLogs] = useState<string[]>([])
  const debugLogsEndRef = useRef<HTMLDivElement>(null)

  // Keep frameRef updated
  useEffect(() => {
    frameRef.current = frame
  }, [frame])

  useEffect(() => {
    if (!notification) return
    setStatusNote(notification)
    acknowledgeNotification()
  }, [notification, acknowledgeNotification])

  // Auto-scroll logs
  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" })
  }, [calibrationLogs])

  // Auto-scroll debug logs
  useEffect(() => {
    debugLogsEndRef.current?.scrollIntoView({ behavior: "smooth" })
  }, [debugLogs])

  const addLog = useCallback((message: string) => {
    setCalibrationLogs((prev) => [...prev, `${new Date().toLocaleTimeString()}: ${message}`])
  }, [])

  const addDebugLog = useCallback((message: string) => {
    setDebugLogs((prev) => [...prev, message])
  }, [])

  const handleDebugMotions = useCallback(async () => {
    setIsDebugging(true)
    setDebugLogs([])

    const motions = [
      { name: "tilt left", instruction: "Please TILT LEFT (left side down)" },
      { name: "tilt right", instruction: "Please TILT RIGHT (right side down)" },
      { name: "tilt forward", instruction: "Please TILT FORWARD (nose down)" },
      { name: "tilt backward", instruction: "Please TILT BACKWARD (nose up)" },
      { name: "turn left", instruction: "Please TURN LEFT" },
      { name: "turn right", instruction: "Please TURN RIGHT" },
    ]

    const allData: Record<string, any[]> = {}

    for (const motion of motions) {
      addDebugLog(`\n${motion.instruction}`)

      // Countdown
      for (let i = 3; i > 0; i--) {
        addDebugLog(`Starting in ${i}...`)
        await new Promise((resolve) => setTimeout(resolve, 1000))
      }

      addDebugLog("Recording...")
      const samples: any[] = []
      const startTime = Date.now()
      const duration = 5000 // 5 seconds

      // Collect samples for 5 seconds
      while (Date.now() - startTime < duration) {
        const currentFrame = frameRef.current
        if (currentFrame) {
          samples.push({
            ts: Date.now(),
            qw: currentFrame.qw,
            qx: currentFrame.qx,
            qy: currentFrame.qy,
            qz: currentFrame.qz,
            calib: currentFrame.calib,
          })
        }
        await new Promise((resolve) => setTimeout(resolve, 100)) // Sample at 10Hz
      }

      allData[motion.name] = samples
      addDebugLog(`Recorded ${samples.length} samples for ${motion.name}`)
    }

    // Send data to backend to save
    addDebugLog("\nSaving debug log to file...")
    try {
      const response = await fetch(`${import.meta.env.VITE_API_BASE}/api/orientation/debug-log`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(allData),
      })

      if (response.ok) {
        const result = await response.json()
        addDebugLog(`✓ Debug log saved to: ${result.path}`)
      } else {
        const errorText = await response.text()
        addDebugLog(`✗ Failed to save debug log: ${errorText}`)
      }
    } catch (error) {
      addDebugLog(`✗ Error: ${error instanceof Error ? error.message : String(error)}`)
    }

    addDebugLog("\nDebug session complete!")
    setIsDebugging(false)
  }, [addDebugLog])

  const handleSimulateSave = useCallback(async () => {
    addLog("Testing calibration save with dummy data...")
    try {
      const response = await fetch(`${import.meta.env.VITE_API_BASE}/api/orientation/calibration/save`, {
        method: "POST",
      })

      if (response.ok) {
        addLog("✓ Save successful! Calibration system is working.")
      } else {
        const errorText = await response.text()
        addLog(`✗ Save failed: ${errorText}`)
      }
    } catch (error) {
      addLog(`✗ Network error: ${error instanceof Error ? error.message : String(error)}`)
    }
  }, [addLog])

  const handleCalibrate = useCallback(async () => {
    if (isCalibrating) {
      // Abort
      calibrationAbortRef.current?.abort()
      calibrationAbortRef.current = null
      setIsCalibrating(false)
      addLog("Calibration aborted")
      return
    }

    // Start calibration
    setIsCalibrating(true)
    setCalibrationLogs([])
    const abortController = new AbortController()
    calibrationAbortRef.current = abortController
    addLog("Starting calibration process...")
    addLog("Move the sensor in various orientations...")

    try {
      const checkInterval = 500
      const timeout = 120000 // 2 minutes max
      const startTime = Date.now()

      const calibrated = { sys: false, gyro: false, accel: false, mag: false }

      while (!abortController.signal.aborted && Date.now() - startTime < timeout) {
        await new Promise((resolve) => setTimeout(resolve, checkInterval))

        const currentFrame = frameRef.current
        if (!currentFrame?.calib) continue

        const { sys, gyro, accel, mag } = currentFrame.calib

        // Check each sensor
        if (!calibrated.sys && sys === 3) {
          calibrated.sys = true
          addLog("SYS successfully calibrated")
        }
        if (!calibrated.gyro && gyro === 3) {
          calibrated.gyro = true
          addLog("GYR successfully calibrated")
        }
        if (!calibrated.accel && accel === 3) {
          calibrated.accel = true
          addLog("ACC successfully calibrated")
        }
        if (!calibrated.mag && mag === 3) {
          calibrated.mag = true
          addLog("MAG successfully calibrated")
        }

        // All sensors calibrated?
        if (calibrated.sys && calibrated.gyro && calibrated.accel && calibrated.mag) {
          addLog("All sensors calibrated! Saving offsets...")

          // Call backend to save offsets
          const response = await fetch(`${import.meta.env.VITE_API_BASE}/api/orientation/calibration/save`, {
            method: "POST",
          })

          if (response.ok) {
            addLog("Calibration complete and saved!")
            setIsCalibrating(false)
            calibrationAbortRef.current = null
            return
          } else {
            const errorText = await response.text()
            addLog(`Failed to save calibration: ${errorText}`)
            setIsCalibrating(false)
            calibrationAbortRef.current = null
            return
          }
        }
      }

      if (abortController.signal.aborted) {
        // Already logged "aborted"
        return
      }

      addLog("Calibration timed out after 2 minutes")
      setIsCalibrating(false)
      calibrationAbortRef.current = null
    } catch (error) {
      addLog(`Calibration error: ${error instanceof Error ? error.message : String(error)}`)
      setIsCalibrating(false)
      calibrationAbortRef.current = null
    }
  }, [isCalibrating, addLog])

  // Initial Three.js scene setup
  useEffect(() => {
    const container = containerRef.current
    if (!container) return

    const scene = new THREE.Scene()
    scene.background = null

    const camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 100)
    camera.position.set(3, 3, 4)
    camera.lookAt(0, 0, 0)

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(container.clientWidth, container.clientHeight)
    container.appendChild(renderer.domElement)

    rendererRef.current = renderer

    // Ground grid
    const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222)
    gridHelper.position.y = -1.15 // Position at bottom of robot
    scene.add(gridHelper)

    // Robot dimensions: 10cm wide (X), 23cm tall (Y), 12cm long (Z)
    const geometry = new THREE.BoxGeometry(1.0, 2.3, 1.2)
    const material = new THREE.MeshStandardMaterial({
      color: 0x2563eb,
      metalness: 0.3,
      roughness: 0.7,
    })
    const cube = new THREE.Mesh(geometry, material)
    cubeRef.current = cube
    scene.add(cube)

    const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x1f2937 })
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), edgeMaterial)
    cube.add(edges)

    // Add face labels using sprites
    const createLabel = (text: string, position: THREE.Vector3) => {
      // Create a new canvas for each label to avoid texture sharing
      const canvas = document.createElement("canvas")
      const context = canvas.getContext("2d")!
      canvas.width = 256
      canvas.height = 128

      context.clearRect(0, 0, canvas.width, canvas.height)
      context.fillStyle = "rgba(255, 255, 255, 0.9)"
      context.font = "bold 48px sans-serif"
      context.textAlign = "center"
      context.textBaseline = "middle"
      context.fillText(text, canvas.width / 2, canvas.height / 2)

      const texture = new THREE.CanvasTexture(canvas)
      const spriteMaterial = new THREE.SpriteMaterial({ map: texture })
      const sprite = new THREE.Sprite(spriteMaterial)
      sprite.scale.set(0.5, 0.25, 1)
      sprite.position.copy(position)
      return sprite
    }

    // Add labels to each face
    // Box geometry: width (X) = 1.0, height (Y) = 2.3, depth (Z) = 1.2
    cube.add(createLabel("TOP", new THREE.Vector3(0, 1.2, 0))) // +Y top
    cube.add(createLabel("BOTTOM", new THREE.Vector3(0, -1.2, 0))) // -Y bottom
    cube.add(createLabel("FRONT", new THREE.Vector3(0, 0, 0.65))) // +Z front
    cube.add(createLabel("BACK", new THREE.Vector3(0, 0, -0.65))) // -Z back
    cube.add(createLabel("LEFT", new THREE.Vector3(-0.55, 0, 0))) // -X left
    cube.add(createLabel("RIGHT", new THREE.Vector3(0.55, 0, 0))) // +X right

    // Apply mesh rotation to stand upright
    cube.rotation.x = Math.PI / 2

    // Add XYZ axes helper in corner
    const axesHelper = new THREE.AxesHelper(1.5)
    axesHelper.position.set(-3.5, -0.5, -3.5)
    scene.add(axesHelper)

    const directional = new THREE.DirectionalLight(0xffffff, 0.85)
    directional.position.set(3, 5, 4)
    scene.add(directional)
    scene.add(new THREE.AmbientLight(0xffffff, 0.35))

    // Add orbit controls for mouse interaction
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.05
    controls.screenSpacePanning = false
    controls.minDistance = 2
    controls.maxDistance = 10
    controls.maxPolarAngle = Math.PI

    const animate = () => {
      animationRef.current = requestAnimationFrame(animate)
      const current = currentQuatRef.current
      const target = targetQuatRef.current
      if (!current.equals(target)) {
        current.slerp(target, SMOOTHING)
      }
      cube.quaternion.copy(current)
      controls.update() // Update controls
      renderer.render(scene, camera)
    }
    animate()

    const handleResize = () => {
      if (!containerRef.current || !rendererRef.current) return
      const width = containerRef.current.clientWidth
      const height = containerRef.current.clientHeight
      rendererRef.current.setSize(width, height)
      camera.aspect = width / height
      camera.updateProjectionMatrix()
    }

    window.addEventListener("resize", handleResize)

    return () => {
      window.removeEventListener("resize", handleResize)
      if (animationRef.current !== null) {
        cancelAnimationFrame(animationRef.current)
        animationRef.current = null
      }
      controls.dispose()
      edges.geometry.dispose()
      edgeMaterial.dispose()
      geometry.dispose()
      material.dispose()
      renderer.dispose()
      renderer.forceContextLoss?.()
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement)
      }
    }
  }, [])

  // Initialize alignment rotation once
  useEffect(() => {
    // Sensor mounting: upside-down (180° around X) + rotated 90° to the right (90° around Z)
    // When robot back faces you: sensor USB-C edge points right, component side faces down
    const flipUpsideDown = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI) // 180° flip
    const rotate90Right = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2) // -90° around Z
    alignmentRotationRef.current = flipUpsideDown.multiply(rotate90Right)
  }, [])

  // Update target quaternion and derived angles when new data arrives
  useEffect(() => {
    if (!frame) return
    const { qx = 0, qy = 0, qz = 0, qw = 1 } = frame

    // Remap quaternion components based on sensor mounting (from debug data analysis)
    // Sensor qz → dominant for ALL motions (vertical axis, upside-down)
    // Sensor qx → changes most for tilt left/right (roll)
    // Sensor qy → changes for tilt forward/back (pitch)
    // Mapping: sensor(qx,qy,qz,qw) → robot(qy, qz, qx, qw)
    // Negate X and Z to correct tilt direction (heading stays correct)
    const incoming = new THREE.Quaternion(-qy, qz, -qx, qw).normalize()

    const adjusted = incoming.clone().premultiply(offsetQuatRef.current).normalize()

    targetQuatRef.current.copy(adjusted)
    if (!hasInitialisedRef.current) {
      currentQuatRef.current.copy(adjusted)
      hasInitialisedRef.current = true
    }

    const euler = new THREE.Euler().setFromQuaternion(adjusted, "YZX")
    const headingDeg = normalizeHeading(THREE.MathUtils.radToDeg(euler.y))
    const pitchDeg = THREE.MathUtils.radToDeg(euler.x)  // X-axis = pitch (forward/back tilt)
    const rollDeg = THREE.MathUtils.radToDeg(euler.z)   // Z-axis = roll (left/right tilt)

    setAngles({ heading: headingDeg, pitch: pitchDeg, roll: rollDeg })
  }, [frame])

  const handleRecenter = useCallback(() => {
    if (!frame) return
    const { qx = 0, qy = 0, qz = 0, qw = 1 } = frame

    // Apply same remapping as in the main effect
    const incoming = new THREE.Quaternion(-qy, qz, -qx, qw).normalize()

    // Create offset that cancels out the current rotation
    const offset = incoming.clone().invert()
    offsetQuatRef.current.copy(offset)

    // Apply offset to get identity (zero rotation)
    const adjusted = incoming.clone().premultiply(offset).normalize()
    targetQuatRef.current.copy(adjusted)
    currentQuatRef.current.copy(adjusted)

    // All angles should now be zero
    setAngles({
      heading: 0,
      pitch: 0,
      roll: 0,
    })
  }, [frame])

  const statusMeta = useMemo(() => {
    if (calibrating) {
      return {
        badgeLabel: "Calibrating",
        badgeVariant: "secondary" as const,
        message: "Running calibration routine",
      }
    }
    if (statusNote) {
      const badgeVariant =
        statusNote.kind === "success"
          ? ("secondary" as const)
          : statusNote.kind === "error"
            ? ("destructive" as const)
            : ("outline" as const)
      const badgeLabel =
        statusNote.kind === "success"
          ? "Success"
          : statusNote.kind === "error"
            ? "Error"
            : "Info"
      return { badgeLabel, badgeVariant, message: statusNote.message ?? "Status updated" }
    }
    if (status === "connected" && frame) {
      return {
        badgeLabel: "Live",
        badgeVariant: "secondary" as const,
        message: "Orientation stream live",
      }
    }
    if (status === "connecting" || reconnecting) {
      return {
        badgeLabel: reconnecting ? "Reconnecting" : "Connecting",
        badgeVariant: "outline" as const,
        message: "Connecting to orientation stream…",
      }
    }
    return {
      badgeLabel: "Offline",
      badgeVariant: "destructive" as const,
      message: "Orientation stream offline",
    }
  }, [calibrating, frame, reconnecting, status, statusNote])

  const calib = frame?.calib ?? {}
  const calibrationItems = [
    { label: "SYS", value: calib.sys },
    { label: "GYR", value: calib.gyro },
    { label: "ACC", value: calib.accel },
    { label: "MAG", value: calib.mag },
  ]

  return (
    <Card className="relative">
      <CardHeader className="space-y-0">
        <CardTitle className="flex items-center gap-2 text-base">
          <Compass className="h-4 w-4" /> Orientation
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-4 text-sm">
        <div className="flex flex-col gap-2 rounded-md border p-3 sm:flex-row sm:items-center sm:justify-between">
          <div>
            <div className="text-xs uppercase tracking-wide text-muted-foreground">Status</div>
            <div className="text-sm">{statusMeta.message}</div>
          </div>
          <Badge variant={statusMeta.badgeVariant}>{statusMeta.badgeLabel}</Badge>
        </div>
        <div ref={containerRef} className="h-48 w-full overflow-hidden rounded-md border bg-muted/40" />

        <div className="grid grid-cols-3 gap-3 text-center">
          <div className="rounded-md border p-2">
            <div className="text-xs uppercase tracking-wide text-muted-foreground">Heading</div>
            <div className="font-mono text-lg">{formatAngle(angles.heading)}</div>
          </div>
          <div className="rounded-md border p-2">
            <div className="text-xs uppercase tracking-wide text-muted-foreground">Pitch</div>
            <div className="font-mono text-lg">{formatAngle(angles.pitch)}</div>
          </div>
          <div className="rounded-md border p-2">
            <div className="text-xs uppercase tracking-wide text-muted-foreground">Roll</div>
            <div className="font-mono text-lg">{formatAngle(angles.roll)}</div>
          </div>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          {calibrationItems.map((item) => (
            <Badge
              key={item.label}
              variant="outline"
              className={cn("px-2 py-1 font-mono text-xs", calibrationLevelClass(item.value))}
            >
              {item.label}: {item.value ?? "—"}
            </Badge>
          ))}
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <Button variant="secondary" size="sm" onClick={handleRecenter} disabled={!frame || calibrating}>
            <RotateCcw className="mr-2 h-4 w-4" /> Recenter
          </Button>
          {DEBUG_MODE && (
            <>
              <Button
                variant={calibrating ? "destructive" : "outline"}
                size="sm"
                onClick={calibrating ? abortCalibration : startCalibration}
                disabled={!calibrating && status !== "connected"}
              >
                <RefreshCcw className="mr-2 h-4 w-4" /> {calibrating ? "Abort" : "Figure 8 Test"}
              </Button>
              <Button
                variant="outline"
                size="sm"
                onClick={handleDebugMotions}
                disabled={isDebugging || !frame}
              >
                {isDebugging ? "Debugging..." : "Debug Axes"}
              </Button>
            </>
          )}
        </div>

        {debugLogs.length > 0 && (
          <div className="max-h-48 overflow-y-auto rounded-md border bg-muted/40 p-3 font-mono text-xs">
            {debugLogs.map((log, idx) => (
              <div key={idx} className="text-muted-foreground whitespace-pre-wrap">
                {log}
              </div>
            ))}
            <div ref={debugLogsEndRef} />
          </div>
        )}

        <Collapsible>
          <CollapsibleTrigger asChild>
            <Button variant="ghost" size="sm" className="w-full justify-between">
              <span className="text-xs font-semibold uppercase tracking-wide">Calibration</span>
              <ChevronDown className="h-4 w-4" />
            </Button>
          </CollapsibleTrigger>
          <CollapsibleContent className="space-y-3 pt-3">
            <div className="flex gap-2">
              <Button
                variant={isCalibrating ? "destructive" : "outline"}
                size="sm"
                onClick={handleCalibrate}
                disabled={!frame && !isCalibrating}
                className="flex-1"
              >
                <Save className="mr-2 h-4 w-4" />
                {isCalibrating ? "ABORT" : "Calibrate..."}
              </Button>
              {DEBUG_MODE && (
                <Button
                  variant="ghost"
                  size="sm"
                  onClick={handleSimulateSave}
                  disabled={isCalibrating}
                  className="flex-1"
                >
                  Simulate Save
                </Button>
              )}
            </div>

            {calibrationLogs.length > 0 && (
              <div className="max-h-32 overflow-y-auto rounded-md border bg-muted/40 p-2 font-mono text-xs">
                {calibrationLogs.map((log, idx) => (
                  <div key={idx} className="text-muted-foreground">
                    {log}
                  </div>
                ))}
                <div ref={logsEndRef} />
              </div>
            )}
          </CollapsibleContent>
        </Collapsible>
      </CardContent>

    </Card>
  )
}
