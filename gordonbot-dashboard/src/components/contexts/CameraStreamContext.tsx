import React, { createContext, useContext, ReactNode, useCallback, useEffect, useRef } from "react"
import { useCameraStream, type CameraStreamState } from "../hooks/useCameraStream"

/**
 * Camera stream context providing shared video stream across components.
 *
 * This context ensures that only ONE WebRTC connection is established
 * for the camera feed, shared between CameraPanel and CameraOverlay.
 *
 * Each component gets its own videoRef to avoid ref collision, but they
 * share the same MediaStream.
 */

type SharedCameraStreamState = Omit<CameraStreamState, 'videoRef'> & {
  /**
   * Register a video element to receive the camera stream.
   * Returns a cleanup function.
   */
  registerVideoElement: (videoElement: HTMLVideoElement) => () => void
}

const CameraStreamContext = createContext<SharedCameraStreamState | null>(null)

interface CameraStreamProviderProps {
  children: ReactNode
  /**
   * Set to true if the camera should automatically start on mount.
   * Default: true (matches legacy behavior)
   */
  autoStart?: boolean
}

export function CameraStreamProvider({ children, autoStart = true }: CameraStreamProviderProps) {
  const cameraStream = useCameraStream({ autoStart })
  const videoElementsRef = useRef<Set<HTMLVideoElement>>(new Set())

  // When the stream changes, update all registered video elements
  useEffect(() => {
    const stream = cameraStream.mediaStream
    videoElementsRef.current.forEach((videoElement) => {
      if (videoElement.srcObject !== stream) {
        videoElement.srcObject = stream
        if (stream) {
          videoElement.play().catch(() => {
            // Ignore autoplay errors
          })
        }
      }
    })
  }, [cameraStream.mediaStream])

  const registerVideoElement = useCallback((videoElement: HTMLVideoElement) => {
    // Add to registered elements
    videoElementsRef.current.add(videoElement)

    // Set the current stream immediately
    if (cameraStream.mediaStream) {
      videoElement.srcObject = cameraStream.mediaStream
      videoElement.play().catch(() => {
        // Ignore autoplay errors
      })
    }

    // Return cleanup function
    return () => {
      videoElementsRef.current.delete(videoElement)
      videoElement.srcObject = null
    }
  }, [cameraStream.mediaStream])

  const sharedState: SharedCameraStreamState = {
    ...cameraStream,
    registerVideoElement,
  }

  // Remove videoRef from the shared state since each component manages its own
  // @ts-expect-error We're intentionally removing videoRef
  delete sharedState.videoRef

  return <CameraStreamContext.Provider value={sharedState}>{children}</CameraStreamContext.Provider>
}

/**
 * Hook to access the shared camera stream.
 *
 * @throws Error if used outside CameraStreamProvider
 */
export function useSharedCameraStream(): SharedCameraStreamState {
  const context = useContext(CameraStreamContext)
  if (!context) {
    throw new Error("useSharedCameraStream must be used within CameraStreamProvider")
  }
  return context
}
