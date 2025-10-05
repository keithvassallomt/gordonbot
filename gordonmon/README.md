GordonMon — Bubble Tea TUI Supervisor

A terminal UI (Go) that supervises GordonBot services with a sticky header/footer and a mouse-scrollable log.

Features
- Starts/stops/restarts backend (FastAPI) and frontend (Vite) via the Go supervisor
- Optionally starts MediaMTX if not already running
- Sticky header: status, first non-loopback IPv4, derived URLs, spinner while starting
- Middle: scrollable log (mouse wheel, PgUp/PgDn, Home/End)
- Sticky footer: hotkeys (Q quit, R reload, B restart backend, F restart frontend)

Build & Run
```bash
cd gordonmon
go run .           # run in place
# or
go build -o gordonmon
./gordonmon
```

Config (env)
- `BACKEND_DIR` (default: ../gordonbot-backend)
- `FRONTEND_DIR` (default: ../gordonbot-dashboard)
- `MEDIAMTX_DIR` (default: /home/keith/mediamtx)
- `MEDIAMTX_CMD` (default: $MEDIAMTX_DIR/mediamtx)
- `BACKEND_PORT` (default: 8000)
- `FRONTEND_PORT` (default: 5173)
- `MEDIAMTX_HTTP_PORT` (default: 8889)
- `MEDIAMTX_RTSP_PORT` (default: 8554)

Log view mode
- Default: pager-like (does not auto-follow; scroll with mouse or PgUp/PgDn). Toggle at runtime with `P`.
- To start in tail-follow mode instead, set `GORDONMON_PAGER=0` (or `false`).

Notes
- Mouse scrolling in the log pane works in most terminals (iTerm2, Warp, Kitty, etc.).
- On first run Go will download Bubble Tea/Bubbles/Lip Gloss modules.
