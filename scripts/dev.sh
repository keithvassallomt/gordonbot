#!/usr/bin/env bash
# ==========================
# GordonBot Dev Supervisor
# - Runs backend (FastAPI) and frontend (Vite)
# - Simple, reliable log streaming
# - Interactive controls: (R)eload, (Q)uit
# - Automatic port cleanup
# ==========================

set -euo pipefail

# Configuration
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND_DIR="${BACKEND_DIR:-"$ROOT/gordonbot-backend"}"
FRONTEND_DIR="${FRONTEND_DIR:-"$ROOT/gordonbot-dashboard"}"
BACKEND_PORT="${BACKEND_PORT:-8000}"
FRONTEND_PORT="${FRONTEND_PORT:-5173}"

# Global state
BACKEND_PID=""
FRONTEND_PID=""
RELOADING=false

# Simple logging functions
log() {
    echo "[$(date +'%H:%M:%S')] $*"
}

info() {
    echo "[INFO] $*"
}

warn() {
    echo "[WARN] $*"
}

error() {
    echo "[ERROR] $*" >&2
}

success() {
    echo "[SUCCESS] $*"
}

# Check if process is running
is_alive() {
    [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null
}

# Kill processes on a given port
kill_port() {
    local port="$1"
    local name="$2"
    
    if command -v lsof >/dev/null 2>&1; then
        local pids
        pids=$(lsof -iTCP:"$port" -sTCP:LISTEN -t 2>/dev/null || true)
        if [[ -n "$pids" ]]; then
            warn "Killing existing processes on $name port $port: $pids"
            echo "$pids" | xargs kill 2>/dev/null || true
            sleep 1
            # Force kill if still running
            for pid in $pids; do
                if is_alive "$pid"; then
                    kill -9 "$pid" 2>/dev/null || true
                fi
            done
            success "Port $port cleared"
        fi
    fi
}

# Start backend server
start_backend() {
    info "Starting backend on port $BACKEND_PORT..."
    
    (
        cd "$BACKEND_DIR"
        if [[ -f ".venv/bin/activate" ]]; then
            source .venv/bin/activate
        fi
        exec uvicorn app.main:app --host 0.0.0.0 --port "$BACKEND_PORT" --reload
    ) 2>&1 | sed -u 's/^/[backend] /' &
    
    BACKEND_PID=$!
    sleep 2
    
    if is_alive "$BACKEND_PID"; then
        success "Backend started (PID: $BACKEND_PID)"
    else
        error "Backend failed to start"
        return 1
    fi
}

# Start frontend server
start_frontend() {
    info "Starting frontend on port $FRONTEND_PORT..."
    
    (
        cd "$FRONTEND_DIR"
        exec npm run dev -- --host --port "$FRONTEND_PORT"
    ) 2>&1 | sed -u 's/^/[frontend] /' &
    
    FRONTEND_PID=$!
    sleep 2
    
    if is_alive "$FRONTEND_PID"; then
        success "Frontend started (PID: $FRONTEND_PID)"
    else
        error "Frontend failed to start"
        return 1
    fi
}

# Stop all processes
stop_all() {
    info "Stopping services..."
    [[ -n "$BACKEND_PID" ]] && kill "$BACKEND_PID" 2>/dev/null || true
    [[ -n "$FRONTEND_PID" ]] && kill "$FRONTEND_PID" 2>/dev/null || true
    sleep 1
    
    # Force kill if needed
    [[ -n "$BACKEND_PID" ]] && is_alive "$BACKEND_PID" && kill -9 "$BACKEND_PID" 2>/dev/null || true
    [[ -n "$FRONTEND_PID" ]] && is_alive "$FRONTEND_PID" && kill -9 "$FRONTEND_PID" 2>/dev/null || true
    
    BACKEND_PID=""
    FRONTEND_PID=""
}

# Reload both services
reload_all() {
    if [[ "$RELOADING" == "true" ]]; then
        warn "Reload already in progress..."
        return
    fi
    
    RELOADING=true
    echo
    info "=== RELOADING SERVICES ==="
    
    stop_all
    kill_port "$BACKEND_PORT" "backend"
    kill_port "$FRONTEND_PORT" "frontend"
    
    if start_backend && start_frontend; then
        success "Services reloaded successfully!"
    else
        error "Reload failed!"
    fi
    
    echo "=== RELOAD COMPLETE ==="
    echo
    RELOADING=false
}

# Get current status
get_status() {
    local b_status="stopped"
    local f_status="stopped"
    
    [[ -n "$BACKEND_PID" ]] && is_alive "$BACKEND_PID" && b_status="running ($BACKEND_PID)"
    [[ -n "$FRONTEND_PID" ]] && is_alive "$FRONTEND_PID" && f_status="running ($FRONTEND_PID)"
    
    echo "Backend: $b_status | Frontend: $f_status | [R]eload [Q]uit"
}

# Cleanup on exit
cleanup() {
    echo
    info "Shutting down..."
    stop_all
    echo "Goodbye!"
    exit 0
}

trap cleanup INT TERM EXIT

# Show header
show_header() {
    clear
    echo "==============================================="
    echo "        🤖 GordonBot Dev Server"
    echo "==============================================="
    echo "Backend:  http://localhost:$BACKEND_PORT"
    echo "Frontend: http://localhost:$FRONTEND_PORT"
    echo "==============================================="
    echo
}

# Main function
main() {
    show_header
    
    # Clean ports and start services
    kill_port "$BACKEND_PORT" "backend"
    kill_port "$FRONTEND_PORT" "frontend"
    
    if ! start_backend || ! start_frontend; then
        error "Failed to start services"
        exit 1
    fi
    
    echo
    success "All services started! Use controls below:"
    echo
    
    # Simple status loop
    while true; do
        # Print status on same line
        printf "\r$(get_status)"
        
        # Check if services crashed
        if ([[ -n "$BACKEND_PID" ]] && ! is_alive "$BACKEND_PID") || \
           ([[ -n "$FRONTEND_PID" ]] && ! is_alive "$FRONTEND_PID"); then
            echo
            error "Service crashed! Exiting..."
            exit 1
        fi
        
        # Handle input (non-blocking)
        if read -t 1 -n 1 key 2>/dev/null; then
            case "$key" in
                r|R)
                    reload_all
                    ;;
                q|Q|$'\x03')
                    echo
                    break
                    ;;
            esac
        fi
    done
}

main "$@"
