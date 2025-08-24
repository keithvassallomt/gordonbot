from __future__ import annotations
from fastapi import APIRouter
from typing import Any, Dict
from app.services.diagnostics import get_system_diagnostics

router = APIRouter()

@router.get("/diag/system", tags=["system"])
async def system_diagnostics() -> Dict[str, Any]:
    return get_system_diagnostics()