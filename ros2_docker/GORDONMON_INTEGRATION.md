# GordonBot SLAM - Gordonmon Integration

## ✅ Integration Complete

The ROS2 SLAM Docker container is now integrated with gordonmon and will automatically start/stop with your GordonBot system.

## How It Works

### Startup (when you run `go run ./gordonmon`)

1. Gordonmon detects the `ros2_docker/` directory
2. Runs `sudo docker compose up -d` in that directory
3. ROS2 SLAM container starts in the background
4. Container automatically:
   - Connects to backend WebSocket `/ws/lidar`
   - Polls `/api/sensors` for encoder and IMU data
   - Starts slam_toolbox
   - Serves map on WebSocket port **9001**

### Shutdown (when you quit gordonmon with 'Q')

1. All processes stop (backend, frontend, mediamtx)
2. Runs `sudo docker compose down` to stop SLAM container
3. Clean shutdown

## Changes Made

### `gordonmon/main.go`

**Added to `startAllCmd()` (line 531-542):**
```go
// ROS2 SLAM Docker
ros2DockerDir := filepath.Join(c.root, "ros2_docker")
if _, err := os.Stat(ros2DockerDir); err == nil {
    program.Send(logMsg{fmt.Sprintf("%s Starting ROS2 SLAM container…", tagStart)})
    slamCmd := "sudo docker compose up -d"
    sp, slamErr := startProcess("ros2_slam", slamCmd, ros2DockerDir)
    if slamErr != nil {
        program.Send(logMsg{fmt.Sprintf("%s ROS2 SLAM failed to start: %v", tagStart, slamErr)})
    } else {
        _ = sp
    }
}
```

**Added to `stopAll()` (line 441-446):**
```go
// Stop ROS2 SLAM Docker container
c := loadCfg()
ros2DockerDir := filepath.Join(c.root, "ros2_docker")
if _, err := os.Stat(ros2DockerDir); err == nil {
    exec.Command("bash", "-c", "cd "+ros2DockerDir+" && sudo docker compose down").Run()
}
```

## Verifying It Works

### Start gordonmon:
```bash
cd /home/keith/gordonbot/gordonmon
go run .
```

You should see in the logs:
```
[start] Starting ROS2 SLAM container…
```

### Check if container is running:
```bash
sudo docker ps | grep gordonbot-ros2-slam
```

You should see:
```
CONTAINER ID   IMAGE                        STATUS         PORTS     NAMES
xxxxxxx        gordonbot-ros2-slam:latest   Up X seconds             gordonbot-ros2-slam
```

### Check map WebSocket is available:
```bash
# This should connect (Ctrl+C to exit)
websocat ws://localhost:9001
```

### Quit gordonmon (press 'Q')

Container should automatically stop. Verify:
```bash
sudo docker ps | grep gordonbot-ros2-slam
# Should return nothing
```

## Manual Control (if needed)

You can still manually control the container:

```bash
cd /home/keith/gordonbot/ros2_docker

# Start manually
sudo docker compose up -d

# Stop manually
sudo docker compose down

# View logs
sudo docker logs -f gordonbot-ros2-slam

# Restart
sudo docker compose restart
```

## Troubleshooting

### Container doesn't start with gordonmon

**Check:**
1. Is `ros2_docker/` directory present?
2. Is Docker installed? (`docker --version`)
3. Check gordonmon logs for errors

**Fix:**
```bash
cd /home/keith/gordonbot/ros2_docker
sudo docker compose build
sudo docker compose up -d
```

### Container starts but no map data

**Check:**
1. Backend is running (`http://localhost:8000/api/ping`)
2. LIDAR is streaming (`ws://localhost:8000/ws/lidar`)
3. Encoders are publishing (`curl http://localhost:8000/api/sensors`)

**Debug:**
```bash
# View container logs
sudo docker logs -f gordonbot-ros2-slam

# Check ROS2 topics (inside container)
sudo docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /map
```

### Container doesn't stop with gordonmon

This is likely a sudo permission issue. You can manually stop:
```bash
cd /home/keith/gordonbot/ros2_docker
sudo docker compose down
```

## Next Steps

1. ✅ **Gordonmon integration** - COMPLETE
2. **FastAPI bridge** - Create `/ws/slam` endpoint to proxy map from port 9001
3. **Frontend** - Update MapCanvas.tsx to consume `/ws/slam`
4. **Testing** - Drive robot and verify map builds

## Notes

- Container runs in **detached mode** (`-d` flag), so it won't block gordonmon
- Container uses **host networking** for low-latency ROS2 DDS communication
- Logs are accessible via `sudo docker logs gordonbot-ros2-slam`
- Maps are persisted in `ros2_docker/maps/` directory
