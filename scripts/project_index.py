#!/usr/bin/env python3
"""
Generate a machine-readable project index for GordonBot.
Outputs JSON to stdout; intended to be redirected to .codex/scan.json
"""
from __future__ import annotations

import json
import os
import re
from pathlib import Path
import ast

ROOT = Path(__file__).resolve().parents[1]

def read_text(p: Path) -> str:
    try:
        return p.read_text(encoding="utf-8")
    except Exception:
        return ""

HTTP_DECOS = {"get", "post", "put", "patch", "delete", "options", "head"}

def parse_routes(routers_dir: Path, api_prefix: str = "/api"):
    routes = []
    for py in sorted(routers_dir.glob("*.py")):
        text = read_text(py)
        try:
            tree = ast.parse(text)
        except SyntaxError:
            tree = None

        file_rel = str(py.relative_to(ROOT))

        if tree is not None:
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    func_name = node.name
                    for deco in node.decorator_list:
                        # Expect: @router.<method>("/path", ...)
                        if isinstance(deco, ast.Call) and isinstance(deco.func, ast.Attribute):
                            method = deco.func.attr
                            value = deco.func.value
                            if method in HTTP_DECOS and isinstance(value, ast.Name) and value.id == "router":
                                # Path is first arg as a string literal
                                if deco.args and isinstance(deco.args[0], (ast.Str, ast.Constant)) and isinstance(getattr(deco.args[0], 's', None) or getattr(deco.args[0], 'value', None), str):
                                    path = (getattr(deco.args[0], 's', None) or getattr(deco.args[0], 'value', None))
                                    full = api_prefix.rstrip("/") + path
                                    routes.append({
                                        "method": method.upper(),
                                        "path": full,
                                        "file": file_rel,
                                        "function": func_name,
                                    })
        # Fallback regex for any missed entries
        deco_re = re.compile(r"@router\.(get|post|put|patch|delete|options|head)\(\s*([\'\"])(.+?)\2")
        for m in deco_re.finditer(text):
            method = m.group(1).upper()
            path = m.group(3)
            full = api_prefix.rstrip("/") + path
            # Avoid duplicates (match by method+path)
            if not any(r["method"] == method and r["path"] == full for r in routes):
                routes.append({"method": method, "path": full, "file": file_rel})
    return sorted(routes, key=lambda r: (r["path"], r.get("function", "~")))

def parse_websockets(sockets_dir: Path, default_ws_path: str | None = None):
    wss = []
    deco_literal = re.compile(r"@router\.websocket\(\s*([\'\"])(.+?)\1\s*\)")
    deco_expr = re.compile(r"@router\.websocket\(\s*([^)]+)\)")
    for py in sorted(sockets_dir.glob("*.py")):
        text = read_text(py)
        # Literal decorator
        for m in deco_literal.finditer(text):
            wss.append({"path": m.group(2), "file": str(py.relative_to(ROOT))})
        # Non-literal decorator (e.g., @router.websocket(settings.control_ws_path))
        if default_ws_path and not any("file" in d and d["file"] == str(py.relative_to(ROOT)) for d in wss):
            if deco_expr.search(text):
                wss.append({"path": default_ws_path, "file": str(py.relative_to(ROOT)), "inferred": True})
    return wss

def parse_default_ws_path(config_path: Path) -> str | None:
    text = read_text(config_path)
    m = re.search(r"control_ws_path\s*:\s*str\s*=\s*\"([^\"]+)\"", text)
    return m.group(1) if m else None

def read_backend_env(env_path: Path):
    env = {}
    for line in read_text(env_path).splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" in line:
            k, v = line.split("=", 1)
            env[k.strip()] = v.strip()
    return env

def read_frontend_package(pkg_path: Path):
    try:
        return json.loads(read_text(pkg_path))
    except Exception:
        return {}

def main():
    backend = ROOT / "gordonbot-backend"
    frontend = ROOT / "gordonbot-dashboard"

    api_prefix = "/api"  # from app/main.py and config

    routes = parse_routes(backend / "app" / "routers", api_prefix=api_prefix)
    default_ws_path = parse_default_ws_path(backend / "app" / "core" / "config.py")
    wss = parse_websockets(backend / "app" / "sockets", default_ws_path)
    backend_env = read_backend_env(backend / ".env") if (backend / ".env").exists() else {}
    frontend_pkg = read_frontend_package(frontend / "package.json")

    data = {
        "project": {
            "name": "GordonBot",
            "root": str(ROOT),
        },
        "runtime": {
            "ports": {"backend": 8000, "frontend": 5173, "mediamtx_http": 8889, "mediamtx_rtsp": 8554},
            "entrypoints": {
                "backend": {
                    "module": "app.main:app",
                    "cwd": "gordonbot-backend",
                    "command": "uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload"
                },
                "frontend": {
                    "cwd": "gordonbot-dashboard",
                    "command": "npm run dev:host"
                }
            }
        },
        "backend": {
            "api_prefix": api_prefix,
            "routes": routes,
            "websockets": wss,
            "env": backend_env,
            "requirements_file": "gordonbot-backend/requirements.txt",
        },
        "frontend": {
            "env_example": {"VITE_API_BASE": "http://127.0.0.1:8000"},
            "scripts": frontend_pkg.get("scripts", {}),
            "dependencies": frontend_pkg.get("dependencies", {}),
            "devDependencies": frontend_pkg.get("devDependencies", {}),
        },
        "media": {
            "whep": {
                "proxy_base_path": "/api/video/whep/",
                "streams": ["gordon", "gordon-annot"],
                "mediamtx_env": ["MEDIAMTX_WHEP_BASE", "MEDIAMTX_WHEP_STYLE"],
            },
            "rtsp": {
                "env": ["CAMERA_RTSP_URL", "CAMERA_RTSP_ANNOT_URL", "CAMERA_BITRATE"],
            }
        },
        "scripts": {
            "run": "go run ./gordonmon"
        }
    }

    print(json.dumps(data, indent=2, sort_keys=True))

if __name__ == "__main__":
    main()
