from __future__ import annotations

import subprocess
import logging
from pathlib import Path

from fastapi import APIRouter, HTTPException

log = logging.getLogger(__name__)

router = APIRouter()


@router.post("/slam/clear", tags=["slam"])
async def clear_slam_map():
    """
    Clear the SLAM map by restarting the ROS2 container.

    This restarts slam_toolbox with a fresh map.
    """
    try:
        # Path to ros2_docker directory
        ros2_docker_path = Path(__file__).parent.parent.parent.parent / "ros2_docker"

        if not ros2_docker_path.exists():
            raise HTTPException(
                status_code=500,
                detail="ROS2 docker directory not found"
            )

        # Restart the container using docker compose
        result = subprocess.run(
            ["sudo", "docker", "compose", "restart"],
            cwd=str(ros2_docker_path),
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode != 0:
            log.error(f"Failed to restart ROS2 container: {result.stderr}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to restart ROS2 container: {result.stderr}"
            )

        log.info("Successfully restarted ROS2 container to clear SLAM map")
        return {
            "success": True,
            "message": "SLAM map cleared, container restarting"
        }

    except subprocess.TimeoutExpired:
        raise HTTPException(
            status_code=500,
            detail="Container restart timed out"
        )
    except Exception as e:
        log.error(f"Error clearing SLAM map: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error clearing SLAM map: {str(e)}"
        )


@router.get("/slam/status", tags=["slam"])
async def get_slam_status():
    """
    Get SLAM system status.

    Returns information about the ROS2 container and SLAM system.
    """
    try:
        # Check if container is running
        result = subprocess.run(
            ["sudo", "docker", "ps", "--filter", "name=gordonbot-ros2-slam", "--format", "{{.Status}}"],
            capture_output=True,
            text=True,
            timeout=5
        )

        container_running = bool(result.stdout.strip())

        return {
            "container_running": container_running,
            "container_status": result.stdout.strip() if container_running else "Not running"
        }

    except Exception as e:
        log.error(f"Error getting SLAM status: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting SLAM status: {str(e)}"
        )
