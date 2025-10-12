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
