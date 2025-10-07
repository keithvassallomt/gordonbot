from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter, HTTPException

from app.schemas import LidarScanData, LidarStatus, LidarPointData
from app.services.lidar import get_lidar_service

log = logging.getLogger(__name__)

router = APIRouter(tags=["lidar"])


@router.get("/lidar/status", response_model=LidarStatus)
async def get_lidar_status() -> LidarStatus:
    """
    Get LIDAR device status.

    Returns:
        LidarStatus: Current LIDAR connection and operation status
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    status_dict = await service.get_status()

    return LidarStatus(**status_dict)


@router.get("/lidar/scan", response_model=Optional[LidarScanData])
async def get_latest_scan() -> Optional[LidarScanData]:
    """
    Get the most recent complete LIDAR scan.

    Returns:
        LidarScanData: Latest point cloud data, or None if no scan available
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    scan = await service.get_latest_scan()

    if scan is None:
        return None

    # Convert service dataclass to Pydantic schema
    return LidarScanData(
        ts=scan.ts,
        points=[
            LidarPointData(
                angle=point.angle,
                distance_mm=point.distance_mm,
                quality=point.quality,
            )
            for point in scan.points
        ],
        scan_rate_hz=scan.scan_rate_hz,
    )


@router.post("/lidar/start")
async def start_lidar_scan() -> dict:
    """
    Start LIDAR scanning.

    Returns:
        dict: Success status and message
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    # If already running, just return success
    if service.running:
        return {"ok": True, "message": "LIDAR scan already running"}

    # Try to start scan
    success = await service.start_scan()

    if not success:
        # If first attempt fails, try reconnecting and starting again
        log.info("First start attempt failed, trying to reconnect...")
        await service.disconnect()
        connected = await service.connect()
        if connected:
            success = await service.start_scan()

    if not success:
        raise HTTPException(status_code=500, detail="Failed to start LIDAR scan")

    return {"ok": True, "message": "LIDAR scan started"}


@router.post("/lidar/stop")
async def stop_lidar_scan() -> dict:
    """
    Stop LIDAR scanning.

    Returns:
        dict: Success status and message
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    await service.stop_scan()

    return {"ok": True, "message": "LIDAR scan stopped"}


@router.post("/lidar/restart")
async def restart_lidar_scan() -> dict:
    """
    Restart LIDAR scanning (stop then start).

    Returns:
        dict: Success status and message
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    # Stop if running
    if service.running:
        await service.stop_scan()

    # Start scan
    success = await service.start_scan()

    if not success:
        raise HTTPException(status_code=500, detail="Failed to restart LIDAR scan")

    return {"ok": True, "message": "LIDAR scan restarted"}


@router.get("/lidar/diagnostics")
async def get_lidar_diagnostics() -> dict:
    """
    Get detailed LIDAR diagnostic information.

    Returns:
        dict: Diagnostic stats including filter rates, errors, task status
    """
    service = get_lidar_service()

    if service is None:
        raise HTTPException(status_code=503, detail="LIDAR service not initialized")

    return await service.get_diagnostics()
