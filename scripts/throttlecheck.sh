#!/usr/bin/env bash
set -euo pipefail

raw="$(vcgencmd get_throttled 2>/dev/null || true)"
hex="${raw#*=}"
# Convert hex (e.g. 0x50000) to an integer
v=$((16#${hex#0x}))

yn() { (( v & $1 )) && echo YES || echo NO; }

printf "Now: Undervoltage=%s, FreqCap=%s, Throttled=%s, TempLimit=%s | Past: Undervoltage=%s, FreqCap=%s, Throttled=%s, TempLimit=%s\n" \
  "$(yn 0x1)" "$(yn 0x2)" "$(yn 0x4)" "$(yn 0x8)" \
  "$(yn 0x10000)" "$(yn 0x20000)" "$(yn 0x40000)" "$(yn 0x80000)"
