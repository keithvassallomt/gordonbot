from __future__ import annotations
from pydantic import BaseModel
import os
try:
    from dotenv import load_dotenv  # type: ignore
    load_dotenv()
except Exception:
    # dotenv is optional; ignore if not installed
    pass

def _getenv_bool(name: str, default: bool = False) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val.strip().lower() in {"1", "true", "yes", "on"}

def _getenv_list(name: str, default: list[str] | None = None) -> list[str]:
    raw = os.getenv(name)
    if raw is None:
        return list(default or [])
    # Split on commas and whitespace, keep non-empty
    parts = [p.strip() for p in raw.replace("\n", ",").split(",")]
    return [p for p in parts if p]

class Settings(BaseModel):
    api_prefix: str = "/api"
    control_ws_path: str = "/ws/control"
    verbose: bool = _getenv_bool("VERBOSE", False)
    camera_rtsp_url: str | None = os.getenv("CAMERA_RTSP_URL")
    camera_bitrate: int = int(os.getenv("CAMERA_BITRATE", "2000000"))
    # Optional annotated stream publisher
    camera_rtsp_annot_url: str | None = os.getenv("CAMERA_RTSP_ANNOT_URL")
    annot_fps: int = int(os.getenv("ANNOT_FPS", "10"))
    annot_min_area: int = int(os.getenv("ANNOT_MIN_AREA", "600"))  # min bbox area in downscaled space
    # Base URL for MediaMTX HTTP signaling. Can include or omit '/whep'.
    # Combined with MEDIAMTX_WHEP_STYLE to form the final target.
    mediamtx_whep_base: str = os.getenv("MEDIAMTX_WHEP_BASE", "http://127.0.0.1:8889/whep")
    # WHEP URL style: 'prefix' -> '/whep/{stream}', 'suffix' -> '/{stream}/whep'
    # Defaults to 'prefix' to preserve existing behavior.
    mediamtx_whep_style: str = os.getenv("MEDIAMTX_WHEP_STYLE", "suffix")
    # Comma-separated list of allowed CORS origins, e.g. "http://localhost:5173,http://192.168.0.5:5173"
    cors_origins: list[str] = _getenv_list(
        "CORS_ORIGINS",
        [
            "http://127.0.0.1:5173",
            "http://localhost:5173",
        ],
    )
    # Optional regex for allowed CORS origins; if set, it is used instead of the list
    cors_allow_origin_regex: str | None = os.getenv(
        "CORS_ALLOW_ORIGIN_REGEX",
        # Default: allow common private/LAN dev hosts
        r"^https?://(localhost|127\.0\.0\.1|10\.\d+\.\d+\.\d+|192\.168\.\d+\.\d+|172\.(1[6-9]|2[0-9]|3[0-1])\.\d+\.\d+)(:\d+)?$",
    )
    # Object detection (OpenCV DNN) for annotated stream
    detect_enabled: bool = _getenv_bool("DETECT_ENABLED", False)
    # Path to an ONNX object detection model (e.g. YOLOv5/8). If unset, motion detection is used.
    detect_onnx_path: str | None = os.getenv("DETECT_ONNX_PATH")
    # Class labels: 'coco' for built-in COCO80, or path to a labels.txt (one label per line)
    detect_labels: str = os.getenv("DETECT_LABELS", "coco")
    # Confidence and NMS thresholds
    detect_conf_threshold: float = float(os.getenv("DETECT_CONF_THRESHOLD", "0.40"))
    detect_nms_threshold: float = float(os.getenv("DETECT_NMS_THRESHOLD", "0.45"))
    # Model input square size (e.g., 640 for YOLOv5/8). Adjust to match your model.
    detect_input_size: int = int(os.getenv("DETECT_INPUT_SIZE", "640"))
    # Run detection every N frames to reduce CPU (>=1; 1 = every frame)
    detect_interval: int = max(1, int(os.getenv("DETECT_INTERVAL", "2")))

    # Encoders / odometry (defaults based on Pololu 100:1 + 12 CPR motor encoder)
    # The Pololu example suggests ~1204 counts per gearbox output revolution for 100:1.
    encoder_counts_per_rev_output: int = int(os.getenv("ENCODER_COUNTS_PER_REV_OUTPUT", "1204"))
    # Wheel diameter in meters (front drive wheel, 3 cm)
    wheel_diameter_m: float = float(os.getenv("WHEEL_DIAMETER_M", "0.03"))
    # Encoder GPIO pins (BCM) for left/right motors (Channel A/B)
    encoder_left_a: int = int(os.getenv("ENCODER_LEFT_A", "12"))
    encoder_left_b: int = int(os.getenv("ENCODER_LEFT_B", "27"))
    encoder_right_a: int = int(os.getenv("ENCODER_RIGHT_A", "22"))
    encoder_right_b: int = int(os.getenv("ENCODER_RIGHT_B", "26"))
    # Optional empirical scales to correct for slip / effective pitch diameter
    encoder_scale_fwd: float = float(os.getenv("ENCODER_SCALE_FWD", "1.024"))
    encoder_scale_rev: float = float(os.getenv("ENCODER_SCALE_REV", "1.068"))
    # Optional per-motor overrides; if unset, left scales apply to right as well
    encoder_scale_fwd_right: float = float(os.getenv("ENCODER_SCALE_FWD_RIGHT", os.getenv("ENCODER_SCALE_FWD", "1.024")))
    encoder_scale_rev_right: float = float(os.getenv("ENCODER_SCALE_REV_RIGHT", os.getenv("ENCODER_SCALE_REV", "1.068")))

# Simple settings instance (expand later for env vars)
settings = Settings()
