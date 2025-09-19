from __future__ import annotations

"""Custom post-processing helpers for Hailo detectors."""

from typing import Dict, Iterable, Sequence

import numpy as np  # type: ignore

from .base import Detection


def hailo_yolov8_nms_postprocess(
    outputs: Dict[str, Iterable[object]],
    meta: dict[str, object],
    labels: Sequence[str],
    conf_threshold: float,
    _nms_threshold: float,
) -> tuple[Sequence[Detection], list[tuple[int, float, float, float, float, float]]]:
    """Decode Hailo's YOLOv8 NMS output into Detection instances.

    The HEF exposes ``yolov8s/yolov8_nms_postprocess`` as a list with one
    entry per class. Each entry is either an empty list or a list of numpy
    arrays, where each array has shape ``(N, 5)`` containing
    ``[x1, y1, x2, y2, score]`` in relative coordinates.
    """
    key = None
    for candidate in outputs.keys():
        # Prefer exact match but fall back to substring if needed
        if "nms" in candidate:
            key = candidate
            if candidate.endswith("nms_postprocess"):
                break
    if key is None:
        return []

    det_list: list[Detection] = []
    raw_info: list[tuple[int, float, float, float, float, float]] = []
    input_h, input_w = meta.get("input_hw", (640, 640))  # type: ignore[assignment]
    orig_h, orig_w = meta.get("orig_hw", (input_h, input_w))  # type: ignore[assignment]
    scale = float(meta.get("scale", 1.0))
    pad_top, pad_left = meta.get("pad", (0, 0))  # type: ignore[assignment]
    data = outputs.get(key)
    if not isinstance(data, Iterable):
        return []
    # Some HEFs wrap the per-class list inside an extra singleton list
    if isinstance(data, list) and len(data) == 1 and isinstance(data[0], Iterable):
        classes_iter = list(data[0])
    else:
        classes_iter = list(data)

    for class_idx, per_class in enumerate(classes_iter):
        if per_class is None:
            continue
        # Hailo returns a list of arrays per class
        arrays: list[np.ndarray]
        if isinstance(per_class, np.ndarray):
            arrays = [np.asarray(per_class)]
        elif isinstance(per_class, list):
            arrays = [
                np.asarray(arr)
                for arr in per_class
                if isinstance(arr, np.ndarray) and arr.size > 0
            ]
        else:
            continue
        for arr in arrays:
            if arr.size == 0:
                continue
            arr = arr.reshape(-1, arr.shape[-1])
            for coord0, coord1, coord2, coord3, score, *_ in arr:
                conf = float(score)
                if conf < conf_threshold:
                    continue
                # Hailo's YOLOv8 NMS output orders coordinates as (y1, x1, y2, x2)
                y1f = float(coord0)
                x1f = float(coord1)
                y2f = float(coord2)
                x2f = float(coord3)

                # YOLO outputs are normalized to padded input
                x1_net = x1f * input_w
                y1_net = y1f * input_h
                x2_net = x2f * input_w
                y2_net = y2f * input_h

                x1_orig = (x1_net - pad_left) / max(scale, 1e-6)
                y1_orig = (y1_net - pad_top) / max(scale, 1e-6)
                x2_orig = (x2_net - pad_left) / max(scale, 1e-6)
                y2_orig = (y2_net - pad_top) / max(scale, 1e-6)

                x1_orig = max(0.0, min(float(orig_w - 1), x1_orig))
                y1_orig = max(0.0, min(float(orig_h - 1), y1_orig))
                x2_orig = max(0.0, min(float(orig_w - 1), x2_orig))
                y2_orig = max(0.0, min(float(orig_h - 1), y2_orig))

                w_px = max(1, int(round(x2_orig - x1_orig)))
                h_px = max(1, int(round(y2_orig - y1_orig)))
                x_px = int(round(x1_orig))
                y_px = int(round(y1_orig))
                label = labels[int(class_idx)] if 0 <= class_idx < len(labels) else str(class_idx)
                det_list.append(
                    Detection(
                        x=x_px,
                        y=y_px,
                        width=w_px,
                        height=h_px,
                        label=label,
                        confidence=conf,
                    )
                )
                raw_info.append((class_idx, x1f, y1f, x2f, y2f, conf))
    return det_list, raw_info


__all__ = ["hailo_yolov8_nms_postprocess"]
