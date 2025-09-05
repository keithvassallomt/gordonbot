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
# MediaMTX (optional) — started if not already running
MEDIAMTX_DIR="${MEDIAMTX_DIR:-"/home/keith/mediamtx"}"
MEDIAMTX_CMD="${MEDIAMTX_CMD:-"$MEDIAMTX_DIR/mediamtx"}"
MEDIAMTX_HTTP_PORT="${MEDIAMTX_HTTP_PORT:-8889}"
MEDIAMTX_RTSP_PORT="${MEDIAMTX_RTSP_PORT:-8554}"

# Global state
BACKEND_PID=""
FRONTEND_PID=""
MEDIAMTX_PID=""
MEDIAMTX_STARTED_BY_SCRIPT=false
RELOADING=false

# UI / logging state
LOG_FIFO=""
LOG_READER_PID=""
UI_ACTIVE=false
HEADER_LINES=5
FOOTER_LINES=2
TERM_ROWS=0
TERM_COLS=0
LOG_TOP=0
LOG_BOTTOM=0

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

# Detect if MediaMTX is running (either by process name or HTTP port)
mediamtx_running() {
    if command -v pgrep >/dev/null 2>&1; then
        if pgrep -x "mediamtx" >/dev/null 2>&1; then
            return 0
        fi
        # Also check for an exact path invocation
        if [[ -n "$MEDIAMTX_CMD" ]] && pgrep -f "${MEDIAMTX_CMD}" >/dev/null 2>&1; then
            return 0
        fi
    fi
    # Fallback: check HTTP port
    if command -v lsof >/dev/null 2>&1; then
        if lsof -iTCP:"$MEDIAMTX_HTTP_PORT" -sTCP:LISTEN -t >/dev/null 2>&1; then
            return 0
        fi
    fi
    return 1
}

# Start MediaMTX if not already running
start_mediamtx_if_needed() {
    if mediamtx_running; then
        info "MediaMTX already running; will not start a new instance"
        return 0
    fi
    if [[ ! -x "$MEDIAMTX_CMD" ]]; then
        warn "MediaMTX not found or not executable at: $MEDIAMTX_CMD — skipping"
        return 0
    fi
    info "Starting MediaMTX from $MEDIAMTX_CMD..."
    (
        cd "$MEDIAMTX_DIR"
        stdbuf -oL -eL "$MEDIAMTX_CMD" 2>&1 | prefix_logs mediamtx > "$LOG_FIFO"
    ) &
    MEDIAMTX_PID=$!
    MEDIAMTX_STARTED_BY_SCRIPT=true
    sleep 1
    if is_alive "$MEDIAMTX_PID"; then
        success "MediaMTX started (PID: $MEDIAMTX_PID)"
    else
        error "Failed to start MediaMTX"
        MEDIAMTX_STARTED_BY_SCRIPT=false
        MEDIAMTX_PID=""
        return 1
    fi
}

# Start backend server
start_backend() {
    info "Starting backend on port $BACKEND_PORT..."
    
    (
        cd "$BACKEND_DIR"
        if [[ -f ".venv/bin/activate" ]]; then
            # shellcheck disable=SC1091
            source .venv/bin/activate
        fi
        stdbuf -oL -eL uvicorn app.main:app --host 0.0.0.0 --port "$BACKEND_PORT" --reload 2>&1 | prefix_logs backend > "$LOG_FIFO"
    ) &
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
        stdbuf -oL -eL npm run dev -- --host --port "$FRONTEND_PORT" 2>&1 | prefix_logs frontend > "$LOG_FIFO"
    ) &
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
    # Only stop MediaMTX if we started it
    if [[ "$MEDIAMTX_STARTED_BY_SCRIPT" == true && -n "$MEDIAMTX_PID" ]]; then
        kill "$MEDIAMTX_PID" 2>/dev/null || true
    fi
    sleep 1
    
    # Force kill if needed
    [[ -n "$BACKEND_PID" ]] && is_alive "$BACKEND_PID" && kill -9 "$BACKEND_PID" 2>/dev/null || true
    [[ -n "$FRONTEND_PID" ]] && is_alive "$FRONTEND_PID" && kill -9 "$FRONTEND_PID" 2>/dev/null || true
    if [[ "$MEDIAMTX_STARTED_BY_SCRIPT" == true && -n "$MEDIAMTX_PID" ]] && is_alive "$MEDIAMTX_PID"; then
        kill -9 "$MEDIAMTX_PID" 2>/dev/null || true
    fi
    
    BACKEND_PID=""
    FRONTEND_PID=""
    MEDIAMTX_PID=""
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
    local m_status="stopped"
    
    [[ -n "$BACKEND_PID" ]] && is_alive "$BACKEND_PID" && b_status="running ($BACKEND_PID)"
    [[ -n "$FRONTEND_PID" ]] && is_alive "$FRONTEND_PID" && f_status="running ($FRONTEND_PID)"
    if [[ "$MEDIAMTX_STARTED_BY_SCRIPT" == true && -n "$MEDIAMTX_PID" ]] && is_alive "$MEDIAMTX_PID"; then
        m_status="running ($MEDIAMTX_PID)"
    elif mediamtx_running; then
        m_status="running (external)"
    fi
    
    echo "Backend: $b_status | Frontend: $f_status | MediaMTX: $m_status | [R]eload [Q]uit"
}

# Cleanup on exit
cleanup() {
    # Restore UI if active
    if [[ "$UI_ACTIVE" == true ]]; then
        restore_ui
    else
        echo
    fi
    info "Shutting down..."
    stop_all
    # Stop log reader and remove FIFO
    [[ -n "$LOG_READER_PID" ]] && kill "$LOG_READER_PID" 2>/dev/null || true
    [[ -n "$LOG_READER_PID" ]] && kill -9 "$LOG_READER_PID" 2>/dev/null || true
    if [[ -n "$LOG_FIFO" && -p "$LOG_FIFO" ]]; then
        rm -f "$LOG_FIFO" || true
    fi
    echo "Goodbye!"
    exit 0
}

trap cleanup INT TERM EXIT

# Show header
# ----------------------
# UI helpers (fixed header/footer + scrollable logs)
# ----------------------

init_ui() {
    UI_ACTIVE=true
    # Alternate screen and hide cursor
    printf '\e[?1049h\e[H\e[?25l'
    compute_layout
    set_scroll_region
    draw_header
    draw_footer
    # Position cursor at bottom of log area
    printf '\e[%d;1H' "$LOG_BOTTOM"
}

restore_ui() {
    # Reset scroll region and restore terminal
    printf '\e[r\e[?25h\e[?1049l'
    UI_ACTIVE=false
}

compute_layout() {
    TERM_ROWS=$(tput lines 2>/dev/null || echo 24)
    TERM_COLS=$(tput cols 2>/dev/null || echo 80)
    # Ensure minimum sizes
    (( TERM_ROWS < (HEADER_LINES + FOOTER_LINES + 3) )) && HEADER_LINES=3 && FOOTER_LINES=1
    LOG_TOP=$((HEADER_LINES + 1))
    LOG_BOTTOM=$((TERM_ROWS - FOOTER_LINES))
}

set_scroll_region() {
    # Confine scrolling to the log area
    printf '\e[%d;%dr' "$LOG_TOP" "$LOG_BOTTOM"
}

draw_header() {
    local b_status="stopped" f_status="stopped" m_status="stopped"
    [[ -n "$BACKEND_PID" ]] && is_alive "$BACKEND_PID" && b_status="running ($BACKEND_PID)"
    [[ -n "$FRONTEND_PID" ]] && is_alive "$FRONTEND_PID" && f_status="running ($FRONTEND_PID)"
    if [[ "$MEDIAMTX_STARTED_BY_SCRIPT" == true && -n "$MEDIAMTX_PID" ]] && is_alive "$MEDIAMTX_PID"; then
        m_status="running ($MEDIAMTX_PID)"
    elif mediamtx_running; then
        m_status="running (external)"
    fi

    printf '\e7'  # save cursor
    printf '\e[H'
    printf '===============================================\n'
    printf '        🤖 GordonBot Dev Server                \n'
    printf '===============================================\n'
    printf 'Backend:  http://localhost:%s  (%s)\n' "$BACKEND_PORT" "$b_status"
    printf 'Frontend: http://localhost:%s  (%s)\n' "$FRONTEND_PORT" "$f_status"
    printf 'MediaMTX: http://localhost:%s (HTTP) • rtsp://localhost:%s (RTSP)  (%s)\n' "$MEDIAMTX_HTTP_PORT" "$MEDIAMTX_RTSP_PORT" "$m_status"
    printf '\e8'  # restore cursor
}

draw_footer() {
    printf '\e7'
    printf '\e[%d;1H' "$((TERM_ROWS - FOOTER_LINES + 1))"
    # Footer divider and help line
    # Divider
    printf '%-*s\n' "$TERM_COLS" "$(printf '─%.0s' $(seq 1 "$TERM_COLS"))"
    # Help
    printf '%-*s' "$TERM_COLS" "[R]eload  [Q]uit  • Logs scroll in the center; header/footer stay fixed."
    printf '\e8'
}

on_resize() {
    compute_layout
    set_scroll_region
    draw_header
    draw_footer
}

# Centralized log reader using a FIFO
start_log_reader() {
    LOG_FIFO="$(mktemp -u /tmp/gordonbot-logs.XXXXXX)"
    mkfifo "$LOG_FIFO"
    # Keep FIFO open by this shell to avoid writer blocking
    exec {LOGFD}<>"$LOG_FIFO"
    # Background reader: prints each line; scrolling stays within log region
    (
        while IFS= read -r line <&$LOGFD; do
            printf '%s\n' "$line"
        done
    ) &
    LOG_READER_PID=$!
}

# Prefix helper for service logs
prefix_logs() { sed -u "s/^/[$1] /"; }

# Main function
main() {
    init_ui
    start_log_reader
    
    # Clean ports and start services
    kill_port "$BACKEND_PORT" "backend"
    kill_port "$FRONTEND_PORT" "frontend"
    # Do not kill MediaMTX ports; respect existing external instance
    
    # Ensure MediaMTX is running (if available)
    start_mediamtx_if_needed || true
    
    if ! start_backend || ! start_frontend; then
        error "Failed to start services"
        exit 1
    fi
    
    success "All services started!"
    trap on_resize WINCH
    # Input loop; logs handled by background reader
    while true; do
        draw_header
        if read -rsn1 -t 0.5 key < /dev/tty 2>/dev/null; then
            case "$key" in
                r|R) reload_all ;;
                q|Q|$'\x03') break ;;
            esac
        fi
        # Crash detection
        if ([[ -n "$BACKEND_PID" ]] && ! is_alive "$BACKEND_PID") || \
           ([[ -n "$FRONTEND_PID" ]] && ! is_alive "$FRONTEND_PID"); then
            error "Service crashed! Exiting..."
            break
        fi
    done
}

main "$@"
