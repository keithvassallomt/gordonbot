# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

GordonBot is a Raspberry Pi-based robot control system with a modern web dashboard. The project consists of:

- **Backend** (`gordonbot-backend/`): FastAPI server with REST endpoints and WebSocket control
- **Frontend** (`gordonbot-dashboard/`): React + TypeScript dashboard with Vite, Tailwind, and shadcn/ui
- **Hardware Tests** (`code/`): Python scripts for testing GPIO-based motor control using gpiozero
- **Scripts** (`scripts/`): Development and monitoring utilities

## Development Commands

### Full Stack Development
```bash
# Start both backend and frontend with live reload and combined logs
./scripts/dev.sh

# Development server provides:
# - Backend: FastAPI on :8000 with uvicorn auto-reload
# - Frontend: Vite dev server on :5173 with HMR
# - Interactive controls: (R)eload all, (X)terminate
```

### Backend Only
```bash
cd gordonbot-backend
# Activate virtual environment if it exists
source .venv/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

### Frontend Only
```bash
cd gordonbot-dashboard
npm run dev                    # localhost:5173
npm run dev:host              # bind to all interfaces
npm run build                 # production build
npm run preview               # preview production build
npm run typecheck            # TypeScript checking
npm run lint                 # ESLint
npm run lint:css             # Stylelint for CSS
npm run check                # Run all checks (typecheck + lint + css lint)
npm run format               # Prettier formatting
```

### Hardware Testing (Raspberry Pi)
```bash
# Test motor control (requires GPIO access)
python code/test_motors.py
python code/test_motors_encoders.py

# Monitor power/throttling status
./scripts/power_monitor.sh        # detailed colored output
./scripts/throttlecheck.sh        # compact status line
```

## Architecture

### Backend Structure
- **FastAPI app** (`app/main.py`): Main application with CORS, API mounting, and WebSocket routing
- **REST API** mounted at `/api/` with routers:
  - `/api/ping`: Health check endpoint
  - `/api/battery/status`: Battery telemetry (currently mock data)
- **WebSocket** at `/ws/control`: Real-time drive commands from dashboard
- **Configuration** (`app/core/config.py`): Centralized settings with Pydantic
- **Schemas** (`app/schemas.py`): Pydantic models for API contracts (BatteryData, DriveMessage)

### Frontend Structure
- **Component-based React** with TypeScript and functional components
- **Main Layout** (`GordonBotDashboard.tsx`): Grid-based responsive layout with:
  - Left section: Camera/Map tabs (3 cols on desktop)
  - Right section: Control + Battery + Diagnostics panels (2 cols)
- **Custom Hooks**:
  - `useBattery`: REST API polling for battery data
  - `useControlTransport`: WebSocket management with auto-reconnection
  - `useControls`: Keyboard and touch input handling with dead-man safety
  - `useThemeMode`: Dark/light theme persistence
- **Control System**: Differential drive with configurable dead-man timeout (300ms) and command rate (20Hz)
- **UI Library**: shadcn/ui components with Radix UI primitives and Tailwind CSS

### Communication Flow
1. **REST API**: Frontend polls `/api/battery/status` for telemetry
2. **WebSocket**: Real-time drive commands via `/ws/control` 
3. **Development Proxy**: Vite proxies `/api/*` and `/ws/*` to backend during development
4. **Safety**: Server-side clamping of drive values, client-side dead-man timeout

### Hardware Integration
- **GPIO Control**: Uses `gpiozero` library for motor control via GPIO pins
- **Motor Configuration**: Dual motor setup (Motor A: pins 5,6; Motor B: pins 13,19)  
- **Power Monitoring**: Raspberry Pi throttling/undervoltage detection via `vcgencmd`
- **Deployment Target**: Designed to run on Raspberry Pi with camera and battery monitoring

## Development Notes

- **Port Configuration**: Backend defaults to :8000, frontend to :5173 (configurable via environment variables)
- **Python Dependencies**: FastAPI, uvicorn, pydantic (see `requirements.txt`)
- **Node Requirements**: Node.js >=20.0.0 required for frontend
- **Theme System**: Automatic dark/light mode detection with localStorage persistence
- **Development Workflow**: Use `./scripts/dev.sh` for full-stack development with integrated log streaming
- **Safety Features**: Multiple layers including WebSocket disconnection handling, server-side value clamping, and dead-man timeout
