from __future__ import annotations
from pydantic import BaseModel
from pathlib import Path
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

BACKEND_ROOT = Path(__file__).resolve().parents[2]


class Settings(BaseModel):
    api_prefix: str = "/api"
    control_ws_path: str = "/ws/control"
    orientation_ws_path: str = "/ws/orientation"
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
    detect_backend: str = os.getenv("DETECT_BACKEND", "cpu")
    # Path to an ONNX object detection model (e.g. YOLOv5/8). If unset, motion detection is used.
    detect_onnx_path: str | None = os.getenv("DETECT_ONNX_PATH")
    # Path to a compiled Hailo HEF when using detect_backend="hailo"
    detect_hailo_hef_path: str | None = os.getenv("DETECT_HAILO_HEF_PATH")
    # Optional python function for custom Hailo post-processing ("module:function")
    detect_hailo_postprocess: str | None = os.getenv("DETECT_HAILO_POSTPROCESS")
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
    track_thickness_m: float = float(os.getenv("TRACK_THICKNESS", "0.002"))
    # Encoder GPIO pins (BCM) for right/left motors (Phase A/B)
    encoder_right_pa: int = int(os.getenv("ENCODER_RIGHT_PA", "12"))
    encoder_right_pb: int = int(os.getenv("ENCODER_RIGHT_PB", "26"))
    encoder_left_pa: int = int(os.getenv("ENCODER_LEFT_PA", "22"))
    encoder_left_pb: int = int(os.getenv("ENCODER_LEFT_PB", "27"))
    # Optional empirical scales to correct for slip / effective pitch diameter
    encoder_scale_fwd: float = float(os.getenv("ENCODER_SCALE_FWD", "1.024"))
    encoder_scale_rev: float = float(os.getenv("ENCODER_SCALE_REV", "1.068"))
    # Optional per-motor overrides; if unset, left scales apply to right as well
    encoder_scale_fwd_right: float = float(os.getenv("ENCODER_SCALE_FWD_RIGHT", os.getenv("ENCODER_SCALE_FWD", "1.024")))
    encoder_scale_rev_right: float = float(os.getenv("ENCODER_SCALE_REV_RIGHT", os.getenv("ENCODER_SCALE_REV", "1.068")))

    # Wake word detection (Porcupine)
    wakeword_enabled: bool = _getenv_bool("WAKEWORD_ENABLED", False)
    wakeword_access_key: str | None = os.getenv("PICOVOICE_ACCESS_KEY") or os.getenv("PICOVOICE_KEY")
    wakeword_keyword_path: str = os.getenv(
        "WAKEWORD_KEYWORD_PATH",
        str(BACKEND_ROOT / "models" / "porcupine" / "gordon.ppn"),
    )
    wakeword_sensitivity: float = float(os.getenv("WAKEWORD_SENSITIVITY", "0.6"))
    wakeword_audio_device_index: int | None = (
        int(os.environ["WAKEWORD_AUDIO_DEVICE_INDEX"])
        if os.getenv("WAKEWORD_AUDIO_DEVICE_INDEX") not in (None, "")
        else None
    )

    # Optional: skip Porcupine init when dependencies unavailable (dev laptops)
    wakeword_allow_missing_deps: bool = _getenv_bool("WAKEWORD_ALLOW_MISSING_DEPS", True)

    # Wake word audio feedback
    wakeword_audio_enabled: bool = _getenv_bool("WAKEWORD_AUDIO_ENABLED", True)
    wakeword_audio_path: str = os.getenv(
        "WAKEWORD_AUDIO_PATH",
        str(BACKEND_ROOT / "assets" / "audio" / "wake.wav"),
    )
    wakeword_audio_command: str = os.getenv(
        "WAKEWORD_AUDIO_CMD",
        "ffplay -nodisp -autoexit {path}",
    )

    ack_audio_enabled: bool = _getenv_bool("ACK_AUDIO_ENABLED", True)
    ack_audio_path: str = os.getenv(
        "ACK_AUDIO_PATH",
        str(BACKEND_ROOT / "assets" / "audio" / "ack.wav"),
    )
    ack_audio_command: str = os.getenv(
        "ACK_AUDIO_CMD",
        wakeword_audio_command,
    )

    intercom_volume_gain: float = float(os.getenv("INTERCOM_VOLUME_GAIN", "1.8"))

    # Speech capture / ASR
    speech_vad_aggressiveness: int = max(0, min(3, int(os.getenv("SPEECH_VAD_AGGRESSIVENESS", "2"))))
    speech_vad_silence_ms: int = max(100, int(os.getenv("SPEECH_VAD_SILENCE_MS", "700")))
    speech_vad_max_ms: int = max(500, int(os.getenv("SPEECH_VAD_MAX_MS", "10000")))
    speech_pre_roll_ms: int = max(0, int(os.getenv("SPEECH_PRE_ROLL_MS", "200")))
    # Camera resolutions (main = raw stream; detect = annotated pipeline)
    camera_main_width: int = int(os.getenv("CAMERA_MAIN_WIDTH", os.getenv("CAMERA_WIDTH", "1920")))
    camera_main_height: int = int(os.getenv("CAMERA_MAIN_HEIGHT", os.getenv("CAMERA_HEIGHT", "1080")))
    camera_detect_width: int = int(os.getenv("CAMERA_DETECT_WIDTH", "640"))
    camera_detect_height: int = int(os.getenv("CAMERA_DETECT_HEIGHT", "480"))

    speech_backend: str = os.getenv("SPEECH_BACKEND", "auto")
    speech_model: str = os.getenv("SPEECH_MODEL", "tiny.en")
    speech_device: str = os.getenv("SPEECH_DEVICE", "auto")
    speech_compute_type: str = os.getenv("SPEECH_COMPUTE_TYPE", "auto")
    speech_save_recordings: bool = _getenv_bool("SPEECH_SAVE_RECORDINGS", False)
    speech_recording_dir: str = os.getenv(
        "SPEECH_RECORDING_DIR",
        str(BACKEND_ROOT / "assets" / "recording"),
    )
    speech_api_base: str = os.getenv("SPEECH_API_BASE", "https://api.openai.com/v1")
    speech_api_model: str = os.getenv("SPEECH_API_MODEL", "whisper-1")
    speech_api_key: str | None = os.getenv("SPEECH_API_KEY")
    speech_api_org: str | None = os.getenv("SPEECH_API_ORG")
    speech_api_timeout: float = float(os.getenv("SPEECH_API_TIMEOUT", "30"))
    whispercpp_bin: str = os.getenv(
        "WHISPER_CPP_BIN",
        str(BACKEND_ROOT / "third_party" / "whisper_cpp" / "build" / "bin" / "whisper-cli"),
    )
    whispercpp_model: str = os.getenv(
        "WHISPER_CPP_MODEL",
        str(BACKEND_ROOT / "models" / "whisper" / "ggml-tiny.en-q5_1.bin"),
    )
    whispercpp_threads: int = int(
        os.getenv(
            "WHISPER_CPP_THREADS",
            str(max(1, (os.cpu_count() or 1))),
        )
    )

    # Speech synthesis
    tts_backend: str = os.getenv("TTS_BACKEND", "auto")
    espeak_ng_bin: str = os.getenv("ESPEAK_NG_BIN", "espeak-ng")
    espeak_voice: str = os.getenv("ESPEAK_VOICE", "en-US")
    espeak_rate: int = int(os.getenv("ESPEAK_RATE", "170"))
    espeak_pitch: int = int(os.getenv("ESPEAK_PITCH", "50"))
    espeak_amplitude: int = int(os.getenv("ESPEAK_AMPLITUDE", "200"))
    tts_use_filter: bool = _getenv_bool("TTS_USE_FILTER", True)
    sox_bin: str = os.getenv("SOX_BIN", "sox")
    tts_sox_effects: str = os.getenv(
        "TTS_SOX_EFFECTS",
        "pitch +150 tremolo 12 40 flanger echo 0.4 0.5 25 0.3 echo 0.3 0.4 50 0.2 gain -1",
    )
    tts_sox_destination: str = os.getenv("TTS_SOX_DESTINATION", "-t alsa default")
    tts_api_base: str = os.getenv("TTS_API_BASE", "https://api.openai.com/v1")
    tts_api_timeout: float = float(os.getenv("TTS_API_TIMEOUT", "30"))
    tts_openai_model: str = os.getenv("TTS_OPENAI_MODEL", "gpt-4o-mini-tts")
    tts_openai_voice: str = os.getenv("TTS_OPENAI_VOICE", "onyx")
    voice_reply_model: str = os.getenv("VOICE_REPLY_MODEL", "gpt-4o-mini")
    voice_reply_system_prompt: str = os.getenv(
        "VOICE_REPLY_SYSTEM_PROMPT",
        "You are the voice of GordonBot. Provide concise, helpful spoken answers.",
    )

    bno055_calibration_path: str = os.getenv(
        "BNO055_CALIBRATION_PATH",
        str(BACKEND_ROOT / "assets" / "calibration" / "bno055_offsets.json"),
    )

# Simple settings instance (expand later for env vars)
settings = Settings()
