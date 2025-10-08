import { createContext, useContext, useState, useEffect, type ReactNode } from "react"
import { API_BASE } from "../config"

export type SlamMode = "create" | "localisation"
export type SpeedMode = "normal" | "creep"

interface SlamModeContextType {
  slamMode: SlamMode
  setSlamMode: (mode: SlamMode) => void
  speedMode: SpeedMode
  setSpeedMode: (mode: SpeedMode) => void
}

const SlamModeContext = createContext<SlamModeContextType | undefined>(undefined)

export function SlamModeProvider({ children }: { children: ReactNode }) {
  const [slamMode, setSlamMode] = useState<SlamMode>("create")
  const [speedMode, setSpeedMode] = useState<SpeedMode>("normal")

  // Check for saved map on mount and set default mode
  useEffect(() => {
    const checkSavedMap = async () => {
      try {
        const response = await fetch(`${API_BASE}/api/slam/status`)
        if (response.ok) {
          const status = await response.json()
          if (status.saved_map_exists) {
            setSlamMode("localisation")
          }
        }
      } catch (error) {
        console.error("Error checking saved map status:", error)
      }
    }

    checkSavedMap()
  }, [])

  return (
    <SlamModeContext.Provider value={{ slamMode, setSlamMode, speedMode, setSpeedMode }}>
      {children}
    </SlamModeContext.Provider>
  )
}

export function useSlamMode() {
  const context = useContext(SlamModeContext)
  if (context === undefined) {
    throw new Error("useSlamMode must be used within a SlamModeProvider")
  }
  return context
}
