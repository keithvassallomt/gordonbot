import { createContext, useContext, useState, type ReactNode } from "react"

export type SlamMode = "map"
export type SpeedMode = "normal" | "creep"

interface SlamModeContextType {
  slamMode: SlamMode
  speedMode: SpeedMode
  setSpeedMode: (mode: SpeedMode) => void
}

const SlamModeContext = createContext<SlamModeContextType | undefined>(undefined)

export function SlamModeProvider({ children }: { children: ReactNode }) {
  const [slamMode] = useState<SlamMode>("map")
  const [speedMode, setSpeedMode] = useState<SpeedMode>("normal")

  return (
    <SlamModeContext.Provider value={{ slamMode, speedMode, setSpeedMode }}>
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
