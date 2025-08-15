#!/usr/bin/env bash
set -euo pipefail

raw="$(vcgencmd get_throttled 2>/dev/null || true)"
hex="${raw#*=}"
if [[ -z "${hex:-}" ]]; then
  echo "vcgencmd not found or no output. Are you on Raspberry Pi OS with VideoCore tools installed?"
  exit 1
fi

v=$((16#${hex#0x}))   # convert hex (e.g. 0x50000) to int

# Colours
RED="$(tput setaf 1; tput bold)"
YEL="$(tput setaf 3; tput bold)"
GRN="$(tput setaf 2)"
RST="$(tput sgr0)"

yn_now()  { (( v & $1 )) && echo -e "${RED}YES${RST}" || echo -e "${GRN}NO${RST}"; }
yn_past() { (( v & $1 )) && echo -e "${YEL}YES${RST}" || echo -e "NO"; }

clear
echo "[$(date '+%Y-%m-%d %H:%M:%S')] throttled=${hex} (dec=${v})"
printf "Now : Undervoltage=%s  FreqCap=%s  Throttled=%s  TempLimit=%s\n" \
  "$(yn_now 0x1)" "$(yn_now 0x2)" "$(yn_now 0x4)" "$(yn_now 0x8)"
printf "Past: Undervoltage=%s  FreqCap=%s  Throttled=%s  TempLimit=%s\n" \
  "$(yn_past 0x10000)" "$(yn_past 0x20000)" "$(yn_past 0x40000)" "$(yn_past 0x80000)"
# Optional alert if actively undervolting:
(( v & 0x1 )) && echo -e "${RED}⚠ Undervoltage right now — check PSU/cables!${RST}"
