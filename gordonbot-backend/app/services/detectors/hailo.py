from __future__ import annotations

"""Detector implementation accelerated by the Hailo AI Hat+ (Hailo-8L)."""

from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Sequence
import logging
import threading

import numpy as np  # type: ignore

from .base import Detection, Detector

log = logging.getLogger(__name__)

try:  # pragma: no cover - optional hardware dependency
    import cv2  # type: ignore
    _CV_AVAILABLE = True
except Exception as exc:  # pragma: no cover - optional hardware dependency
    cv2 = None  # type: ignore
    _CV_AVAILABLE = False
    log.debug("OpenCV unavailable for Hailo detector: %s", exc)

def _import_hailo_runtime():
    candidates = [
        "hailort",
        "hailo",
        "hailo_platform",
    ]
    suffixes = ("pyhailort.pyhailort", "pyhailort", "runtime", "hailort")

    def normalise(module):
        if module is None:
            return None
        hef = getattr(module, "Hef", None)
        if hef is None and hasattr(module, "HEF"):
            setattr(module, "Hef", getattr(module, "HEF"))
            hef = getattr(module, "Hef")
        infer = getattr(module, "InferVStreams", None)
        if infer is None and hasattr(module, "InferVstreams"):
            setattr(module, "InferVStreams", getattr(module, "InferVstreams"))
            infer = getattr(module, "InferVStreams")
        if hef is not None and infer is not None and hasattr(module, "VDevice"):
            return module
        return None

    for name in candidates:
        module = None
        try:  # pragma: no cover - optional hardware dependency
            module = import_module(name)
        except Exception as exc:
            log.debug("Hailo runtime import failed (%s): %s", name, exc)
        runtime = normalise(module)
        if runtime is not None:
            return runtime
        for suffix in suffixes:
            try:
                sub = import_module(f"{name}.{suffix}")
            except Exception:
                continue
            runtime = normalise(sub)
            if runtime is not None:
                return runtime
    return None


hailo = _import_hailo_runtime()
_HAILO_AVAILABLE = hailo is not None
if not _HAILO_AVAILABLE:
    hailo = None  # type: ignore
    log.debug("Hailo runtime unavailable; install hailort on the target device")


_PostprocessFn = Callable[[Dict[str, np.ndarray], tuple[int, int], Sequence[str], float, float], Sequence[Detection]]


def _resolve_postprocess(spec: str | None) -> _PostprocessFn | None:
    """Load a custom post-process function from "module:function"."""
    if not spec:
        return None
    mod_name, _, func_name = spec.partition(":")
    if not mod_name or not func_name:
        raise ValueError("detect_hailo_postprocess must be in 'module:function' format")
    module = import_module(mod_name)
    fn = getattr(module, func_name, None)
    if fn is None or not callable(fn):
        raise AttributeError(f"Function {func_name!r} not found in module {mod_name!r}")
    return fn  # type: ignore[return-value]


def _default_postprocess(
    outputs: Dict[str, np.ndarray],
    input_hw: tuple[int, int],
    labels: Sequence[str],
    conf_threshold: float,
    nms_threshold: float,
) -> Sequence[Detection]:
    """Best-effort YOLOv5-style post-process for HEFs exporting a single tensor.

    Assumes the first output tensor contains detections in the format
    (cx, cy, w, h, obj_conf, class_scores...)
    """
    if not outputs:
        return []
    # Use the first tensor by default
    value = next(iter(outputs.values()))
    if value is None:
        return []
    if isinstance(value, list):
        tensors: list[np.ndarray] = []
        for item in value:
            if isinstance(item, np.ndarray):
                tensors.append(np.asarray(item))
            elif isinstance(item, list):
                tensors.extend(
                    np.asarray(arr)
                    for arr in item
                    if isinstance(arr, np.ndarray) and arr.size > 0
                )
        if not tensors:
            return []
        tensor = np.concatenate([t.reshape(-1, t.shape[-1]) for t in tensors], axis=0)
    else:
        tensor = np.asarray(value)
    if tensor.ndim >= 3:
        tensor = tensor.reshape(-1, tensor.shape[-1])
    elif tensor.ndim == 1:
        tensor = tensor.reshape(-1, 1)
    arr = tensor.astype(np.float32, copy=False)
    if arr.size == 0:
        return []
    ih, iw = input_hw
    size = float(max(ih, iw)) if max(ih, iw) else 1.0
    boxes: list[List[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []
    for row in arr:
        if row.shape[0] < 6:
            continue
        obj_conf = float(row[4])
        class_scores = row[5:]
        if class_scores.size == 0:
            continue
        class_id = int(np.argmax(class_scores))
        class_conf = float(class_scores[class_id])
        conf = obj_conf * class_conf
        if conf < conf_threshold:
            continue
        cx, cy, w0, h0 = map(float, row[:4])
        x = int((cx - w0 / 2) * iw / size)
        y = int((cy - h0 / 2) * ih / size)
        w_box = int(w0 * iw / size)
        h_box = int(h0 * ih / size)
        boxes.append([max(0, x), max(0, y), max(1, w_box), max(1, h_box)])
        scores.append(conf)
        class_ids.append(class_id)
    if not boxes:
        return []
    try:
        idxs = cv2.dnn.NMSBoxes(boxes, scores, conf_threshold, nms_threshold)  # type: ignore[attr-defined]
    except Exception:
        idxs = list(range(len(boxes)))
    detections: list[Detection] = []
    if isinstance(idxs, tuple) or isinstance(idxs, np.ndarray):
        idxs = list(np.array(idxs).reshape(-1))
    elif isinstance(idxs, list):
        idxs = [int(i) for i in idxs]
    for i in idxs or []:
        x, y, w_box, h_box = boxes[i]
        label = labels[class_ids[i]] if 0 <= class_ids[i] < len(labels) else str(class_ids[i])
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


class HailoDetector(Detector):
    """Detector that executes a compiled HEF on the Hailo accelerator."""

    def __init__(
        self,
        hef_path: str,
        labels: Sequence[str] | None,
        conf_threshold: float,
        nms_threshold: float,
        interval: int = 1,
        postprocess: str | None = None,
    ) -> None:
        if not _HAILO_AVAILABLE:
            raise RuntimeError("Hailo runtime libraries not available; install hailort on the device")
        if not _CV_AVAILABLE:
            raise RuntimeError("OpenCV is required for HailoDetector preprocessing")
        super().__init__(interval=interval)
        self._lock = threading.Lock()
        self._labels = list(labels or [])
        self._conf = float(conf_threshold)
        self._nms = float(nms_threshold)
        self._postprocess = _resolve_postprocess(postprocess) or _default_postprocess

        model_path = Path(hef_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"HEF model not found: {model_path}")

        self._hef = hailo.Hef(str(model_path))  # type: ignore[attr-defined]
        self._device = hailo.VDevice()  # type: ignore[attr-defined]
        net_groups = self._device.configure(self._hef)  # type: ignore[attr-defined]
        if not net_groups:
            raise RuntimeError("HEF configuration returned no network groups")
        self._network_group = net_groups[0]

        self._activation_cm = self._network_group.activate()  # type: ignore[attr-defined]
        try:
            self._activated_network = self._activation_cm.__enter__()
        except Exception as exc:
            self._activation_cm = None
            self._activated_network = None
            raise RuntimeError(f"Failed to activate Hailo network group: {exc}") from exc

        self._input_infos = list(self._network_group.get_input_vstream_infos())  # type: ignore[attr-defined]
        if not self._input_infos:
            raise RuntimeError("HEF does not define any input vstreams")
        if len(self._input_infos) > 1:
            log.warning("HailoDetector: multiple input vstreams detected; using the first one (%s)", self._input_infos[0].name)
        self._input_info = self._input_infos[0]
        shape = getattr(self._input_info, "shape", None) or (640, 640, 3)
        if isinstance(shape, tuple) and len(shape) >= 2:
            self._input_height = int(shape[0])
            self._input_width = int(shape[1])
        else:
            self._input_height = 640
            self._input_width = 640
        self._input_format = getattr(self._input_info, "format", None)

        self._input_params = hailo.InputVStreamParams.make(self._network_group)  # type: ignore[attr-defined]
        self._output_params = hailo.OutputVStreamParams.make(self._network_group)  # type: ignore[attr-defined]

        self._last_raw_output: dict[str, object] | None = None
        self._last_raw_boxes: list[tuple[int, float, float, float, float, float]] | None = None


        self._output_infos = list(self._network_group.get_output_vstream_infos())  # type: ignore[attr-defined]
        if not self._output_infos:
            raise RuntimeError("HEF does not define any output vstreams")

    def close(self) -> None:
        try:
            if getattr(self, "_activation_cm", None) is not None:
                self._activation_cm.__exit__(None, None, None)  # type: ignore[attr-defined]
        except Exception:
            pass
        try:
            if hasattr(self, "_network_group"):
                self._network_group.release()  # type: ignore[attr-defined]
        except Exception:
            pass
        try:
            if hasattr(self, "_device"):
                self._device.release()  # type: ignore[attr-defined]
        except Exception:
            pass

    def __del__(self) -> None:  # pragma: no cover - defensive cleanup
        try:
            self.close()
        except Exception:
            pass

    def _preprocess(self, frame_bgr: "np.ndarray") -> tuple[np.ndarray, dict[str, object]]:  # type: ignore[name-defined]
        orig_h, orig_w = frame_bgr.shape[:2]
        img = frame_bgr
        if getattr(img, "ndim", 0) == 3 and img.shape[2] > 3:
            try:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            except Exception:
                img = img[:, :, :3]

        target_w = int(self._input_width)
        target_h = int(self._input_height)
        scale = min(target_w / max(1, orig_w), target_h / max(1, orig_h))
        new_w = max(1, int(round(orig_w * scale)))
        new_h = max(1, int(round(orig_h * scale)))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_w = target_w - new_w
        pad_h = target_h - new_h
        pad_left = max(0, pad_w // 2)
        pad_right = max(0, pad_w - pad_left)
        pad_top = max(0, pad_h // 2)
        pad_bottom = max(0, pad_h - pad_top)
        if pad_w > 0 or pad_h > 0:
            resized = cv2.copyMakeBorder(
                resized,
                pad_top,
                pad_bottom,
                pad_left,
                pad_right,
                borderType=cv2.BORDER_CONSTANT,
                value=(114, 114, 114),
            )

        fmt = getattr(self._input_format, "order", None)
        tensor = resized
        if fmt is not None and hasattr(hailo, "FormatOrder"):
            if fmt == hailo.FormatOrder.NCHW:
                tensor = np.transpose(tensor, (2, 0, 1))
            elif fmt == hailo.FormatOrder.NHWC:
                pass
            else:
                tensor = resized
        else:
            tensor = resized

        dtype = np.uint8
        fmt_type = getattr(getattr(self._input_format, "type", None), "name", "UINT8")
        if fmt_type.upper() in {"FLOAT32", "FP32"}:
            dtype = np.float32

        if dtype is np.float32:
            tensor = tensor.astype(np.float32, copy=False) / 255.0
        else:
            tensor = tensor.astype(np.uint8, copy=False)

        tensor = np.ascontiguousarray(tensor)
        if tensor.ndim == 3:
            tensor = np.expand_dims(tensor, axis=0)

        meta = {
            "orig_hw": (orig_h, orig_w),
            "input_hw": (target_h, target_w),
            "scale": scale,
            "pad": (pad_top, pad_left),
        }
        return tensor, meta

    def detect(self, frame_bgr: "np.ndarray") -> Sequence[Detection]:  # type: ignore[name-defined]
        with self._lock:
            input_tensor, meta = self._preprocess(frame_bgr)
            inputs = {self._input_info.name: input_tensor}
            try:
                with hailo.InferVStreams(self._network_group, self._input_params, self._output_params) as pipeline:  # type: ignore[attr-defined]
                    raw_outputs = pipeline.infer(inputs)
            except Exception as exc:
                log.warning("HailoDetector inference failed: %s", exc)
                return []

            self._last_raw_output = raw_outputs
            result = self._postprocess(
                raw_outputs,
                meta,
                self._labels,
                self._conf,
                self._nms,
            )
            raw_boxes = None
            if isinstance(result, tuple):
                detections, raw_boxes = result
            else:
                detections = result
            self._last_raw_boxes = raw_boxes if raw_boxes is not None else []
            return list(detections)


__all__ = ["HailoDetector"]
