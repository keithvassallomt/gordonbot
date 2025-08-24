from __future__ import annotations
from fastapi import APIRouter
from fastapi.responses import PlainTextResponse

router = APIRouter()

@router.get("/ping", response_class=PlainTextResponse, tags=["diagnostics"])
async def ping() -> str:
    return "pong"