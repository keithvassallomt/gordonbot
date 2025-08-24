import { useEffect, useState } from "react"


/**
 * React hook to subscribe to a CSS media query and get its match state.
 *
 * @param query - Media query string (e.g. `"(prefers-color-scheme: dark)"`).
 * @returns Boolean indicating if the query currently matches.
 *
 * @remarks
 * - Updates automatically when the media query result changes.
 * - Cleans up the event listener on unmount.
 *
 * @example
 * const prefersDark = useMediaQuery("(prefers-color-scheme: dark)");
 * console.log(prefersDark ? "Dark mode" : "Light mode");
 */
export function useMediaQuery(query: string) {
  const [matches, setMatches] = useState<boolean>(() => window.matchMedia(query).matches)

  useEffect(() => {
    const mql = window.matchMedia(query)
    const handler = (e: MediaQueryListEvent) => setMatches(e.matches)
    mql.addEventListener("change", handler)
    return () => mql.removeEventListener("change", handler)
  }, [query])

  return matches
}