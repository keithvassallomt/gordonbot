# GordonBot SLAM Overview

## High-Level Flow
- **Sensors**  
  - RPLIDAR C1 publishes scans via the backend `/ws/lidar` WebSocket.  
  - Wheel encoders and the BNO055 IMU are exposed through `/api/sensors`.  
  - The IMU quaternion is remapped in the ROS bridge so `base_link` yaw matches the robot’s true heading.
- **ROS2 Stack (Docker container `gordonbot-ros2-slam`)**  
  - `lidar_bridge`, `odom_bridge`, and `imu_bridge` convert backend telemetry into ROS topics (`/scan`, `/odom`, `/imu`).  
  - `slam_toolbox` consumes `/scan` + `/odom` and produces `/map` plus pose updates via TF (`map → base_link`).  
  - `map_bridge` subscribes to `/map`, extracts the current pose, and serves both over a WebSocket on port `9001`.
- **Backend (FastAPI)**  
  - `app/sockets/slam.py` connects to the ROS bridge WebSocket and rebroadcasts map/pose messages to the dashboard at `/ws/slam`.  
  - `/api/slam/save` saves the current pose graph and occupancy grid, then runs post-processing to clean the map (PGM/YAML pairs written to `ros2_docker/maps`).  
  - `/api/slam/map/processed-grid` reads the processed map from disk, converts it back into a `SlamMapMessage`-compatible payload, and returns it for frontend localisation use.  
  - `/api/slam/clear` deletes saved map artifacts and restarts the ROS container; `/api/slam/status` reports container state and whether a saved map exists.
- **Frontend (React dashboard)**  
  - `useSLAM` subscribes to `/ws/slam` and keeps live map + pose in state.  
  - `MapCanvas` renders both the live grid and the processed grid using the same draw pipeline, so pan/zoom behaves identically in mapping and localisation modes.  
  - `SlamMapPanel` manages two modes: **Create Map** (live mapping) and **Localisation** (post-processed map). It auto-collapses when a saved map is present, exposes raw vs. processed toggles when expanded, and forces mapping mode after a clear.  
  - The drive panel shares a context-based creep/normal speed toggle with the map panel so operator controls stay synchronized.

## Key Features
- Real-time 2D SLAM with `slam_toolbox`, 5 cm resolution.
- Configurable mapping/localisation workflow with clear, save, and post-process actions surfaced in the UI.
- Processed localisation map delivered as structured occupancy data, enabling smooth zoom/pan alongside the live map.
- Frontend/ROS heading alignment fixes ensure the rendered robot pose matches physical motion and prevents map mirroring.

## Operational Notes
- Clearing the map wipes saved assets, restarts the ROS container, expands the panel, and switches to Create Map mode.  
- Saving the map produces:  
  - `saved_map.data` / `saved_map.posegraph` (pose graph)  
  - `saved_map_raw.{pgm,yaml}` (raw grid)  
  - `saved_map_processed.{pgm,yaml}` (cleaned grid for localisation)  
- Localisation mode defaults to the processed map; you can still view the live feed by selecting “Raw”.  
- Drive creep mode affects both keyboard driving and the map panel toggles via shared context.
