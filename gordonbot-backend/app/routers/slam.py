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
    """Clear saved SLAM map artifacts and reset slam_toolbox in-place."""
    try:
        ros2_docker_path = Path(__file__).parent.parent.parent.parent / "ros2_docker"
        maps_path = ros2_docker_path / "maps"

        if not ros2_docker_path.exists():
            raise HTTPException(status_code=500, detail="ROS2 docker directory not found")

        saved_map_files = [
            maps_path / "saved_map.data",
            maps_path / "saved_map.posegraph",
            maps_path / "saved_map_raw.pgm",
            maps_path / "saved_map_raw.yaml",
            maps_path / "saved_map_processed.pgm",
            maps_path / "saved_map_processed.yaml",
        ]

        deleted_count = 0
        for map_file in saved_map_files:
            if map_file.exists():
                try:
                    map_file.unlink()
                    log.info("Deleted saved map file: %s", map_file.name)
                    deleted_count += 1
                except Exception as exc:
                    log.warning("Failed to delete %s: %s", map_file.name, exc)

        log.info("Deleted %d map files", deleted_count)

        result = subprocess.run(
            ["docker", "compose", "restart"],
            cwd=str(ros2_docker_path),
            capture_output=True,
            text=True,
            timeout=60,
        )

        if result.returncode != 0:
            log.error("Failed to restart ROS2 container: %s", result.stderr)
            raise HTTPException(status_code=500, detail="Failed to restart ROS2 container")

        log.info("Successfully cleared saved map and restarted ROS2 container")
        return {"success": True, "message": "SLAM map cleared and container restarting"}

    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Container restart timed out")
    except Exception as exc:
        log.error("Error clearing SLAM map: %s", exc)
        raise HTTPException(status_code=500, detail=f"Error clearing SLAM map: {exc}")


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
    raise HTTPException(status_code=501, detail="SLAM map saving is disabled")


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
