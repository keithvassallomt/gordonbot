from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter

from app.sockets import scan_quality as scan_quality_socket

log = logging.getLogger(__name__)

router = APIRouter(tags=["scan_quality"])


@router.get("/scan-quality")
async def get_scan_quality() -> dict:
    """
    Get the latest scan quality metrics.

    Returns quality data from the ROS2 scan quality monitor including:
    - Quality level and score
    - Point count, density, and coverage metrics
    - Issues and warnings
    - Configuration thresholds
    """
    quality = scan_quality_socket.get_latest_quality()

    if quality is None:
        return {
            "status": "no_data",
            "message": "No scan quality data available yet"
        }

    return quality


@router.post("/scan-quality/enable")
async def enable_scan_quality() -> dict:
    """
    Enable scan quality monitoring (manual override).

    Starts the ROS2 scan quality monitor node and begins analyzing LIDAR scans.
    This is a power-intensive operation and should only be enabled when needed.

    Sets manual override flag, keeping monitoring enabled even when no clients are connected.
    """
    success = await scan_quality_socket.enable_monitoring(manual=True)

    if success:
        return {
            "status": "enabled",
            "message": "Scan quality monitoring enabled (manual override)"
        }
    else:
        return {
            "status": "error",
            "message": "Failed to enable scan quality monitoring"
        }


@router.post("/scan-quality/disable")
async def disable_scan_quality() -> dict:
    """
    Disable scan quality monitoring (clears manual override).

    Stops the ROS2 scan quality monitor node to save power.
    Clears the manual override flag and disables monitoring immediately.
    """
    success = await scan_quality_socket.disable_monitoring(manual=True)

    if success:
        return {
            "status": "disabled",
            "message": "Scan quality monitoring disabled"
        }
    else:
        return {
            "status": "error",
            "message": "Failed to disable scan quality monitoring"
        }


@router.get("/scan-quality/status")
async def get_scan_quality_status() -> dict:
    """
    Get the current status of scan quality monitoring.

    Returns:
        - enabled: Whether monitoring is currently active
        - client_count: Number of connected WebSocket clients
        - manual_override: Whether manual override is enabled
    """
    status = scan_quality_socket.get_monitoring_status()
    return status
