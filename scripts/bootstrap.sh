#!/usr/bin/env bash
# Bootstrap GordonBot dev environment: Python venv + Node deps
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND_DIR="$ROOT/gordonbot-backend"
FRONTEND_DIR="$ROOT/gordonbot-dashboard"

echo "[bootstrap] Setting up backend venv and dependencies..."
(
  cd "$BACKEND_DIR"
  if [[ ! -d .venv ]]; then
    python3 -m venv .venv
  fi
  # shellcheck disable=SC1091
  source .venv/bin/activate
  python -m pip install --upgrade pip
  pip install -r requirements.txt
)

echo "[bootstrap] Installing frontend dependencies (npm ci)..."
(
  cd "$FRONTEND_DIR"
  if command -v npm >/dev/null 2>&1; then
    npm ci
  else
    echo "npm not found; please install Node.js >= 20 to continue" >&2
    exit 1
  fi
)

echo "[bootstrap] Done! Use scripts/dev.sh to run both services."

