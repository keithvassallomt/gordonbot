from __future__ import annotations

"""CPU-based object detector using OpenCV DNN."""

from pathlib import Path
from typing import Sequence, List
import logging

import numpy as np  # type: ignore

from .base import Detection, Detector

log = logging.getLogger(__name__)

try:
    import cv2  # type: ignore
    _CV_AVAILABLE = True
except Exception as exc:  # pragma: no cover - dev machines without OpenCV
    cv2 = None  # type: ignore
    _CV_AVAILABLE = False
    log.debug("OpenCV import failed for CPU detector: %s", exc)


class CpuDetector(Detector):
    """YOLO-style detector backed by OpenCV DNN."""

    def __init__(
        self,
        onnx_path: str,
        labels: Sequence[str] | None,
        conf_threshold: float,
        nms_threshold: float,
        input_size: int,
        interval: int = 1,
    ) -> None:
        if not _CV_AVAILABLE:
            raise RuntimeError("OpenCV (cv2) is not available for CpuDetector")
        super().__init__(interval=interval)
        model_path = Path(onnx_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        self._net = cv2.dnn.readNetFromONNX(str(model_path))  # type: ignore[attr-defined]
        try:
            self._net.setPreferableBackend(getattr(cv2.dnn, "DNN_BACKEND_OPENCV", 3))
            self._net.setPreferableTarget(getattr(cv2.dnn, "DNN_TARGET_CPU", 0))
        except Exception:
            pass
        self._labels = list(labels or [])
        self._conf = float(conf_threshold)
        self._nms = float(nms_threshold)
        self._input_size = int(input_size)

    def detect(self, frame_bgr: "np.ndarray") -> Sequence[Detection]:  # type: ignore[name-defined]
        blob = cv2.dnn.blobFromImage(
            frame_bgr,
            scalefactor=1 / 255.0,
            size=(self._input_size, self._input_size),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        out = self._net.forward()
        detections: List[Detection] = []
        if out is None:
            return detections
        out = np.squeeze(out)
        if out.ndim == 1:
            out = np.expand_dims(out, axis=0)
        ih, iw = frame_bgr.shape[:2]
        boxes: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        for row in out:
            if row.shape[0] < 85:
                continue
            obj_conf = float(row[4])
            class_scores = row[5:]
            class_id = int(np.argmax(class_scores))
            class_conf = float(class_scores[class_id])
            conf = obj_conf * class_conf
            if conf < self._conf:
                continue
            cx, cy, w0, h0 = row[0], row[1], row[2], row[3]
            x = int((cx - w0 / 2) * iw / self._input_size)
            y = int((cy - h0 / 2) * ih / self._input_size)
            w_box = int(w0 * iw / self._input_size)
            h_box = int(h0 * ih / self._input_size)
            boxes.append([max(0, x), max(0, y), max(1, w_box), max(1, h_box)])
            scores.append(conf)
            class_ids.append(class_id)
        if not boxes:
            return detections
        idxs = cv2.dnn.NMSBoxes(boxes, scores, self._conf, self._nms)  # type: ignore[attr-defined]
        if isinstance(idxs, tuple) or isinstance(idxs, np.ndarray):
            idxs = list(np.array(idxs).reshape(-1))
        elif isinstance(idxs, list):
            idxs = [int(i) for i in idxs]
        for i in idxs or []:
            x, y, w_box, h_box = boxes[i]
            cls = class_ids[i]
            label = self._labels[cls] if 0 <= cls < len(self._labels) else str(cls)
            detections.append(
                Detection(
                    x=x,
                    y=y,
                    width=w_box,
                    height=h_box,
                    label=label,
                    confidence=float(scores[i]),
                )
            )
        return detections


__all__ = ["CpuDetector"]
