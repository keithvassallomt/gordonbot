/**
 * Key-value row component.
 *
 * Renders a bordered row with a label on the left and a monospaced value on the right.
 * Used throughout the dashboard for displaying metrics and diagnostic data.
 *
 * @param label - Text label for the value (e.g. "Voltage").
 * @param value - Value string to display (monospaced).
 * @param className - Optional additional CSS classes for styling/spacing.
 *
 * @example
 * ```tsx
 * <KV label="Voltage" value="5.00 V" />
 * ```
 */
export default function KV({
  label,
  value,
  className,
}: { label: string; value: string; className?: string }) {
  return (
    <div className={`flex items-center justify-between rounded-md border p-2 ${className ?? ""}`}>
      <div className="text-muted-foreground">{label}</div>
      <div className="font-mono">{value}</div>
    </div>
  )
}