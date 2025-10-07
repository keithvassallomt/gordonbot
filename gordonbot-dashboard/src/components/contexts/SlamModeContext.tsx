import React, { createContext, useContext, useState, ReactNode } from "react"

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
