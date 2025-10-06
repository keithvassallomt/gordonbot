# GordonBot SLAM Quick Start

## Prerequisites

- GordonBot backend running on `http://localhost:8000`
- LIDAR WebSocket streaming on `/ws/lidar`
- Encoder data available on `/api/sensors`
- Docker and Docker Compose installed

## Starting SLAM

```bash
cd /home/keith/gordonbot/ros2_docker

# Start the ROS2 SLAM container
sudo docker compose up -d

# View logs
sudo docker logs -f gordonbot-ros2-slam

# Stop
sudo docker compose down
```

## What's Running

When you start the container, these nodes launch automatically:

1. **lidar_bridge** - Connects to `ws://localhost:8000/ws/lidar` and publishes `/scan` topic
2. **odom_bridge** - Polls `/api/sensors` for encoder data and publishes `/odom` topic
3. **imu_bridge** - Polls `/api/sensors` for BNO055 data and publishes `/imu` topic
4. **slam_toolbox** - Consumes `/scan` + `/odom` and produces `/map` topic
5. **map_bridge** - Serves `/map` via WebSocket on port **9001**

## Checking Status

```bash
# Check if container is running
sudo docker ps | grep gordonbot-ros2-slam

# View all logs
sudo docker logs gordonbot-ros2-slam

# Follow logs in real-time
sudo docker logs -f gordonbot-ros2-slam

# Enter container for debugging
sudo docker exec -it gordonbot-ros2-slam bash
```

## Inside the Container (Debugging)

```bash
# Enter container
sudo docker exec -it gordonbot-ros2-slam bash

# Source ROS2
source /ros2_ws/install/setup.bash

# List active topics
ros2 topic list

# Check scan data
ros2 topic hz /scan
ros2 topic echo /scan --once

# Check odometry
ros2 topic hz /odom
ros2 topic echo /odom --once

# Check map
ros2 topic echo /map --once

# View TF tree
ros2 run tf2_tools view_frames
```

## Expected Topics

When everything is working, you should see:

- `/scan` - LaserScan data from LIDAR (~10 Hz)
- `/odom` - Wheel encoder odometry (~20 Hz)
- `/imu` - BNO055 IMU data (~50 Hz)
- `/map` - SLAM occupancy grid (updates every 2 seconds)
- `/tf` - Transform tree (odom → base_link → laser)

## Map WebSocket

The map bridge serves map data on **`ws://localhost:9001`**

Message format:
```json
{
  "type": "map",
  "ts": 1759737513212,
  "width": 384,
  "height": 384,
  "resolution": 0.05,
  "origin": {
    "x": -9.6,
    "y": -9.6,
    "theta": 0.0
  },
  "data": [/* occupancy grid array */]
}
```

## Troubleshooting

### Container won't start
```bash
# Check logs for errors
sudo docker logs gordonbot-ros2-slam

# Rebuild image
sudo docker compose build --no-cache
sudo docker compose up -d
```

### No scan data on /scan topic
- Check LIDAR is streaming: `curl http://localhost:8000/ws/lidar` should connect
- Check backend LIDAR service is running
- Inside container: `ros2 topic echo /scan --once` should show data

### No odometry on /odom topic
- Check encoders endpoint: `curl http://localhost:8000/api/sensors` should return encoder data
- Verify `distance_m` field is present in encoder data

### SLAM not building map
- Verify both `/scan` and `/odom` are publishing: `ros2 topic list`
- Check slam_toolbox logs: `sudo docker logs gordonbot-ros2-slam | grep slam`
- Ensure robot is moving (SLAM needs motion to build map)

### Map not streaming to WebSocket
- Check map_bridge is running: `sudo docker logs gordonbot-ros2-slam | grep map_bridge`
- Test WebSocket: `websocat ws://localhost:9001` (requires websocat tool)
- Verify `/map` topic is publishing: `ros2 topic hz /map`

## Configuration

Edit `config/slam_toolbox_params.yaml` to tune SLAM:

- `resolution: 0.05` - Map resolution (5cm/pixel)
- `max_laser_range: 12.0` - LIDAR max range
- `minimum_travel_distance: 0.05` - Min movement for update
- `do_loop_closing: true` - Enable loop closure

After changing config:
```bash
sudo docker compose down
sudo docker compose up -d
```

## Next Steps

1. **Frontend Integration**: Connect React MapCanvas to `ws://localhost:9001`
2. **Test Mapping**: Drive robot and verify map builds
3. **Save Maps**: Implement map save/load via slam_toolbox service calls
4. **Gordonmon Integration**: Add container to supervisor

## Useful Commands

```bash
# Restart container
sudo docker compose restart

# View resource usage
sudo docker stats gordonbot-ros2-slam

# Remove container and volumes
sudo docker compose down -v

# Rebuild from scratch
sudo docker compose build --no-cache
sudo docker compose up -d
```
