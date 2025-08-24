import { useEffect, useState } from "react"
import { useMediaQuery } from "./useMediaQuery"

export type ThemeMode = "system" | "light" | "dark"

const THEME_KEY = "gordonbot-theme"

/**
 * React hook to manage theme mode (system, light, dark) with persistence.
 *
 * @returns Object with:
 * - `mode`: Current theme mode ("system" | "light" | "dark").
 * - `setMode`: Setter to update the mode.
 *
 * @remarks
 * - Persists the choice in localStorage under the key `gordonbot-theme`.
 * - If `system` is selected, uses `prefers-color-scheme: dark` media query.
 * - Automatically toggles the `dark` class on `<html>`.
 *
 * @example
 * const { mode, setMode } = useThemeMode();
 * <button onClick={() => setMode("dark")}>Dark Mode</button>
 */
export function useThemeMode() {
  const [mode, setMode] = useState<ThemeMode>(() => (localStorage.getItem(THEME_KEY) as ThemeMode) || "system")
  const systemDark = useMediaQuery("(prefers-color-scheme: dark)")

  useEffect(() => {
    const effective = mode === "system" ? (systemDark ? "dark" : "light") : mode
    const root = document.documentElement
    if (effective === "dark") root.classList.add("dark")
    else root.classList.remove("dark")
    localStorage.setItem(THEME_KEY, mode)
  }, [mode, systemDark])

  return { mode, setMode }
}