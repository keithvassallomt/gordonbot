from __future__ import annotations

import subprocess
import logging
from pathlib import Path
from io import BytesIO

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from PIL import Image

from app.schemas import SlamGoToPointRequest
from app.services.autonomous_drive import (
    GoToInProgressError,
    GoToNotRunningError,
    GoToValidationError,
    cancel_go_to_point,
    get_go_to_status,
    start_go_to_point,
)
from app.services.map_processor import default_processor

log = logging.getLogger(__name__)

router = APIRouter()


@router.post("/slam/clear", tags=["slam"])
async def clear_slam_map():
    """
    Clear the SLAM map by deleting saved map files and restarting the ROS2 container.

    This deletes any saved map and restarts slam_toolbox with a fresh map.
    """
    try:
        # Path to ros2_docker directory
        ros2_docker_path = Path(__file__).parent.parent.parent.parent / "ros2_docker"
        maps_path = ros2_docker_path / "maps"

        if not ros2_docker_path.exists():
            raise HTTPException(
                status_code=500,
                detail="ROS2 docker directory not found"
            )

        # Delete all saved map files if they exist
        saved_map_files = [
            # Pose-graph files (for localization/continued mapping)
            maps_path / "saved_map.data",
            maps_path / "saved_map.posegraph",
            # Raw occupancy grid files
            maps_path / "saved_map_raw.pgm",
            maps_path / "saved_map_raw.yaml",
            # Processed occupancy grid files
            maps_path / "saved_map_processed.pgm",
            maps_path / "saved_map_processed.yaml"
        ]

        deleted_count = 0
        for map_file in saved_map_files:
            if map_file.exists():
                try:
                    map_file.unlink()
                    log.info(f"Deleted saved map file: {map_file.name}")
                    deleted_count += 1
                except Exception as e:
                    log.warning(f"Failed to delete {map_file.name}: {e}")

        log.info(f"Deleted {deleted_count} map files")

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

        log.info("Successfully cleared saved map and restarted ROS2 container")
        return {
            "success": True,
            "message": "SLAM map cleared, saved map deleted, container restarting"
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

        # Check if saved map exists
        maps_path = Path(__file__).parent.parent.parent.parent / "ros2_docker" / "maps"
        saved_map_exists = (maps_path / "saved_map.data").exists() and (maps_path / "saved_map.posegraph").exists()

        return {
            "container_running": container_running,
            "container_status": result.stdout.strip() if container_running else "Not running",
            "saved_map_exists": saved_map_exists
        }

    except Exception as e:
        log.error(f"Error getting SLAM status: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting SLAM status: {str(e)}"
        )


@router.post("/slam/save", tags=["slam"])
async def save_slam_map():
    """
    Save the current SLAM map with post-processing.

    Workflow:
    1. Saves pose-graph (.data + .posegraph) using serialize_map service
    2. Exports occupancy grid (.pgm + .yaml) using save_map service
    3. Applies post-processing to create cleaned version for navigation

    Files created:
    - saved_map.data, saved_map.posegraph (for localization/continued mapping)
    - saved_map_raw.pgm, saved_map_raw.yaml (raw occupancy grid)
    - saved_map_processed.pgm, saved_map_processed.yaml (cleaned for navigation)
    """
    try:
        # Path to ros2_docker directory
        ros2_docker_path = Path(__file__).parent.parent.parent.parent / "ros2_docker"
        maps_path = ros2_docker_path / "maps"

        if not maps_path.exists():
            raise HTTPException(
                status_code=500,
                detail="Maps directory not found"
            )

        # Step 1: Save pose-graph using serialize_map
        log.info("Saving SLAM pose-graph...")
        result = subprocess.run(
            [
                "sudo", "docker", "exec", "gordonbot-ros2-slam",
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && "
                "ros2 service call "
                "/slam_toolbox/serialize_map "
                "slam_toolbox/srv/SerializePoseGraph "
                "\"{filename: '/ros2_ws/maps/saved_map'}\""
            ],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode != 0:
            log.error(f"Failed to serialize SLAM map: {result.stderr}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to serialize SLAM map: {result.stderr}"
            )

        log.info("Successfully saved SLAM pose-graph to /ros2_ws/maps/saved_map")

        # Step 2: Export occupancy grid using save_map
        log.info("Exporting occupancy grid...")
        result = subprocess.run(
            [
                "sudo", "docker", "exec", "gordonbot-ros2-slam",
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && "
                "ros2 service call "
                "/slam_toolbox/save_map "
                "slam_toolbox/srv/SaveMap "
                "\"{name: {data: '/ros2_ws/maps/saved_map_raw'}}\""
            ],
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode != 0:
            log.error(f"Failed to export occupancy grid: {result.stderr}")
            raise HTTPException(
                status_code=500,
                detail=f"Failed to export occupancy grid: {result.stderr}"
            )

        log.info("Successfully exported occupancy grid to /ros2_ws/maps/saved_map_raw")

        # Step 3: Apply post-processing
        log.info("Applying post-processing to occupancy grid...")
        raw_pgm = maps_path / "saved_map_raw.pgm"
        raw_yaml = maps_path / "saved_map_raw.yaml"
        processed_pgm = maps_path / "saved_map_processed.pgm"
        processed_yaml = maps_path / "saved_map_processed.yaml"

        processing_success = default_processor.process_map(
            input_pgm_path=raw_pgm,
            input_yaml_path=raw_yaml,
            output_pgm_path=processed_pgm,
            output_yaml_path=processed_yaml
        )

        if processing_success:
            log.info("Successfully applied post-processing to map")
            message = "SLAM map saved with post-processing (raw + processed versions created)"
        else:
            log.warning("Post-processing failed or disabled, only raw map available")
            message = "SLAM map saved (post-processing failed, only raw version available)"

        return {
            "success": True,
            "message": message,
            "files": {
                "pose_graph": ["saved_map.data", "saved_map.posegraph"],
                "raw_occupancy_grid": ["saved_map_raw.pgm", "saved_map_raw.yaml"],
                "processed_occupancy_grid": ["saved_map_processed.pgm", "saved_map_processed.yaml"] if processing_success else None
            }
        }

    except subprocess.TimeoutExpired:
        raise HTTPException(
            status_code=500,
            detail="Map save operation timed out"
        )
    except Exception as e:
        log.error(f"Error saving SLAM map: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Error saving SLAM map: {str(e)}"
        )


@router.get("/slam/map/processed", tags=["slam"])
async def get_processed_map():
    """
    Get the post-processed map image as PNG.

    Converts the saved_map_processed.pgm file to PNG format for browser compatibility.
    Returns the cleaned map suitable for navigation.
    """
    try:
        maps_path = Path(__file__).parent.parent.parent.parent / "ros2_docker" / "maps"
        processed_pgm = maps_path / "saved_map_processed.pgm"

        if not processed_pgm.exists():
            raise HTTPException(
                status_code=404,
                detail="Processed map not found. Save a map first."
            )

        # Load PGM image and convert to PNG
        img = Image.open(processed_pgm)

        # Convert to PNG in memory
        img_bytes = BytesIO()
        img.save(img_bytes, format="PNG")
        img_bytes.seek(0)

        return StreamingResponse(
            img_bytes,
            media_type="image/png",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )

    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Error retrieving processed map: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving processed map: {str(e)}"
        )


@router.get("/slam/map/processed-grid", tags=["slam"])
async def get_processed_map_grid():
    """
    Get the post-processed map as a SlamMapMessage-compatible payload.

    Returns the processed occupancy grid data so the frontend can render it
    with the same pan/zoom behaviour as the live SLAM map.
    """
    try:
        message = default_processor.load_processed_map_as_message()
        if message is None:
            raise HTTPException(
                status_code=404,
                detail="Processed map not found. Save a map first."
            )
        return message
    except HTTPException:
        raise
    except Exception as e:
        log.error(f"Error retrieving processed map grid: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving processed map grid: {str(e)}"
        )


@router.post("/slam/goto", tags=["slam"])
async def go_to_point(request: SlamGoToPointRequest):
    """Start autonomous navigation toward a target SLAM coordinate."""
    try:
        status = await start_go_to_point(
            request.x,
            request.y,
            tolerance=request.tolerance,
        )
    except GoToValidationError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except GoToInProgressError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:  # pragma: no cover - defensive
        log.exception("Failed to start go-to routine: %s", exc)
        raise HTTPException(status_code=500, detail="Failed to start go-to routine") from exc

    payload = status.to_dict()
    payload["message"] = "Go-to routine started"
    return payload


@router.get("/slam/goto/status", tags=["slam"])
async def go_to_status():
    """Return current go-to status or the result of the last run."""
    status = await get_go_to_status()
    return status.to_dict()


@router.post("/slam/goto/cancel", tags=["slam"])
async def cancel_go_to():
    """Request cancellation of the active go-to routine."""
    try:
        await cancel_go_to_point()
    except GoToNotRunningError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:  # pragma: no cover - defensive
        log.exception("Failed to cancel go-to routine: %s", exc)
        raise HTTPException(status_code=500, detail="Failed to cancel go-to routine") from exc

    status = await get_go_to_status()
    payload = status.to_dict()
    payload["message"] = "Go-to routine cancelled"
    return payload
