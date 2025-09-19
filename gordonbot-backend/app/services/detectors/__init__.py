from __future__ import annotations

"""Detector factory helpers."""

from typing import Sequence
import logging

from .base import Detection, Detector
from .cpu import CpuDetector

log = logging.getLogger(__name__)


def _load_labels(label_source: str | None) -> list[str]:
    if not label_source:
        return []
    if label_source.lower() == "coco":
        return [
            "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
            "fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
            "elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
            "skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
            "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
            "pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
            "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
            "hair drier","toothbrush"
        ]
    try:
        with open(label_source, "r", encoding="utf-8") as fh:
            return [ln.strip() for ln in fh if ln.strip()]
    except Exception as exc:  # pragma: no cover - IO failure fallback
        log.warning("detector: failed to read labels from %s: %s", label_source, exc)
        return []


def create_detector(settings) -> Detector | None:
    """Create the configured detector from settings.

    Currently supports the existing OpenCV ONNX workflow. The accelerator
    implementation will plug into this factory in a later step.
    """
    if not getattr(settings, "detect_enabled", False):
        return None
    onnx_path = getattr(settings, "detect_onnx_path", None)
    if not onnx_path:
        log.warning("detector: detect_enabled but no DETECT_ONNX_PATH provided")
        return None
    labels = _load_labels(getattr(settings, "detect_labels", None))
    conf = float(getattr(settings, "detect_conf_threshold", 0.40))
    nms = float(getattr(settings, "detect_nms_threshold", 0.45))
    input_size = int(getattr(settings, "detect_input_size", 640))
    interval = max(1, int(getattr(settings, "detect_interval", 2)))
    try:
        return CpuDetector(
            onnx_path=onnx_path,
            labels=labels,
            conf_threshold=conf,
            nms_threshold=nms,
            input_size=input_size,
            interval=interval,
        )
    except Exception as exc:
        log.warning("detector: failed to create CPU detector: %s", exc)
        return None


__all__ = ["Detection", "Detector", "create_detector"]
