# GordonBot ROS2 SLAM Docker Setup

This directory contains the Docker-based ROS2 SLAM system for GordonBot.

## Architecture

- **ROS2 Humble** running in Docker container
- **Bridge nodes** connect GordonBot FastAPI backend to ROS2 topics
- **slam_toolbox** performs 2D LIDAR SLAM
- **Map bridge** exposes SLAM map via WebSocket for frontend

## Components

### Bridge Nodes (`bridge_nodes/`)

1. **lidar_bridge** - Subscribes to `/ws/lidar` WebSocket, publishes `/scan` topic
2. **odom_bridge** - Polls encoder data from `/api/sensors`, publishes `/odom` topic
3. **imu_bridge** - Polls BNO055 from `/api/sensors`, publishes `/imu` topic
4. **map_bridge** - Subscribes to `/map`, serves via WebSocket on port 9001

### Configuration (`config/`)

- `slam_toolbox_params.yaml` - SLAM algorithm parameters (5cm resolution, async mode)
- `slam_launch.py` - Launch file for all nodes
- `cyclonedds.xml` - DDS middleware configuration

### Maps (`maps/`)

Persistent storage for saved SLAM maps.

## Setup

### 1. Build Docker Image

```bash
cd /home/keith/gordonbot/ros2_docker
docker-compose build
```

### 2. Start SLAM System

```bash
docker-compose up -d
```

### 3. View Logs

```bash
docker-compose logs -f ros2_slam
```

### 4. Stop System

```bash
docker-compose down
```

## Network Architecture

- **host network mode** - Container shares network with host for low-latency DDS
- **Backend connection** - Bridges connect to `http://localhost:8000` (FastAPI)
- **WebSocket server** - Map bridge listens on `ws://localhost:9001`

## Data Flow

```
GordonBot Backend (FastAPI)
    ↓ (WebSocket /ws/lidar)
lidar_bridge → /scan topic
    ↓
slam_toolbox → /map topic
    ↓
map_bridge → WebSocket :9001
    ↓
Frontend (React)
```

## Configuration

### Environment Variables

Set in `docker-compose.yml`:

- `ROS_DOMAIN_ID` - ROS2 domain (default: 0)
- `RMW_IMPLEMENTATION` - DDS implementation (default: cyclonedds)
- `BACKEND_URL` - GordonBot backend URL (default: http://localhost:8000)

### SLAM Parameters

Edit `config/slam_toolbox_params.yaml`:

- `resolution` - Map resolution (default: 0.05 = 5cm/pixel)
- `max_laser_range` - LIDAR max range (default: 12.0m for C1)
- `minimum_travel_distance` - Min movement for update (default: 0.05m)
- `do_loop_closing` - Enable loop closure (default: true)

## Troubleshooting

### Container won't start

Check Docker logs:
```bash
docker-compose logs ros2_slam
```

### No LIDAR data

Verify WebSocket connection:
```bash
# In container
docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
ros2 topic hz /scan
```

### No map updates

Check slam_toolbox status:
```bash
docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
ros2 topic echo /map --once
```

### TF errors

View TF tree:
```bash
docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
```

## Development

### Rebuild after code changes

```bash
docker-compose build --no-cache
docker-compose up -d
```

### Interactive shell

```bash
docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
```

### Manual launch

```bash
docker exec -it gordonbot-ros2-slam bash
source /ros2_ws/install/setup.bash
ros2 launch /config/slam_launch.py
```

## Integration with Gordonmon

Add to `gordonmon/main.go`:

```go
// SLAM container
{
    Name: "ros2_slam",
    Command: "docker-compose",
    Args: []string{"-f", "/home/keith/gordonbot/ros2_docker/docker-compose.yml", "up"},
    Dir: "/home/keith/gordonbot/ros2_docker",
    Enabled: true,
}
```
