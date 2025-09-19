#!/usr/bin/env python
"""Quick Hailo detector test that captures a frame via Picamera2."""

import argparse
import contextlib
import os
import time
from collections import Counter
import numpy as np
import cv2

from app.services.detectors.base import Detection
from app.services.detectors.cpu import CpuDetector
from app.services.detectors.hailo import HailoDetector

COCO_LABELS = [
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
    from picamera2 import Picamera2
except Exception as exc:  # pragma: no cover - hardware dependency
    raise SystemExit(f"Picamera2 not available: {exc}")


def draw_detections(frame: np.ndarray, detections) -> np.ndarray:
    annotated = frame.copy()
    h, w = annotated.shape[:2]
    for det in detections:
        x1 = max(0, min(w - 1, int(round(det.x))))
        y1 = max(0, min(h - 1, int(round(det.y))))
        x2 = max(0, min(w - 1, int(round(det.x + det.width))))
        y2 = max(0, min(h - 1, int(round(det.y + det.height))))
        if x2 <= x1 or y2 <= y1:
            continue
        label_text = det.label
        if label_text.startswith("class"):
            try:
                idx = int(label_text[5:])
                if 0 <= idx < len(COCO_LABELS):
                    label_text = COCO_LABELS[idx]
            except Exception:
                pass
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 140, 255), 2)
        cv2.putText(
            annotated,
            f"{label_text} {det.confidence:.2f}",
            (x1, max(0, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 140, 255),
            2,
            cv2.LINE_AA,
        )
    return annotated


def _project_hailo_boxes(
    frame: np.ndarray,
    raw_boxes,
    labels: list[str],
    input_size: tuple[int, int],
) -> list[tuple[Detection, tuple[int, float, float, float, float, float]]]:
    """Map Hailo's normalized boxes back to frame pixel coordinates."""

    frame_h, frame_w = frame.shape[:2]
    in_h, in_w = input_size
    scale = min(in_w / max(1, frame_w), in_h / max(1, frame_h))
    scaled_w = frame_w * scale
    scaled_h = frame_h * scale
    pad_x = (in_w - scaled_w) / 2.0
    pad_y = (in_h - scaled_h) / 2.0
    detections: list[tuple[Detection, tuple[int, float, float, float, float, float]]] = []
    for raw in raw_boxes or []:
        if not raw:
            continue
        cls_idx, x1, y1, x2, y2, conf = raw
        x1_net = x1 * in_w
        y1_net = y1 * in_h
        x2_net = x2 * in_w
        y2_net = y2 * in_h
        x1_img = (x1_net - pad_x) / max(scale, 1e-6)
        y1_img = (y1_net - pad_y) / max(scale, 1e-6)
        x2_img = (x2_net - pad_x) / max(scale, 1e-6)
        y2_img = (y2_net - pad_y) / max(scale, 1e-6)

        x1_img = max(0.0, min(frame_w - 1.0, x1_img))
        y1_img = max(0.0, min(frame_h - 1.0, y1_img))
        x2_img = max(0.0, min(frame_w - 1.0, x2_img))
        y2_img = max(0.0, min(frame_h - 1.0, y2_img))

        if x2_img <= x1_img or y2_img <= y1_img:
            continue

        det = Detection(
            x=int(round(x1_img)),
            y=int(round(y1_img)),
            width=max(1, int(round(x2_img - x1_img))),
            height=max(1, int(round(y2_img - y1_img))),
            label=labels[int(cls_idx)] if 0 <= int(cls_idx) < len(labels) else f"class{cls_idx}",
            confidence=float(conf),
        )
        detections.append((det, raw))
    return detections


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Hailo vs CPU detection comparison")
    parser.add_argument(
        "--cpu-dialect",
        choices=["yolov5", "yolov8"],
        help="Override YOLO_ONNX_DIALECT for the CPU detector.",
    )
    args = parser.parse_args(argv)

    hef_path = "models/yolov8s.hef"
    onnx_path = "models/yolov8s.onnx"
    cpu_dialect = args.cpu_dialect or os.getenv("YOLO_ONNX_DIALECT", "yolov5")
    if args.cpu_dialect:
        os.environ["YOLO_ONNX_DIALECT"] = cpu_dialect
    print(f"Using HEF: {os.path.abspath(hef_path)}")
    print(f"Using ONNX: {os.path.abspath(onnx_path)}")
    print(f"CPU dialect: {cpu_dialect}")

    picam = Picamera2()
    det: HailoDetector | None = None
    cpu_det: CpuDetector | None = None

    try:
        picam.start()
        time.sleep(0.5)
        det = HailoDetector(
            hef_path=hef_path,
            labels=COCO_LABELS,
            conf_threshold=0.7,
            nms_threshold=0.5,
            interval=1,
            postprocess="app.services.detectors.postprocess:hailo_yolov8_nms_postprocess",
        )
        cpu_det = CpuDetector(
            onnx_path=onnx_path,
            labels=COCO_LABELS,
            conf_threshold=0.35,
            nms_threshold=0.45,
            input_size=640,
            interval=1,
            dialect=cpu_dialect,
        )

        frame = picam.capture_array()
        if getattr(frame, "ndim", 0) == 3 and frame.shape[2] > 3:
            try:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception:
                frame = frame[:, :, :3]

        detections = det.detect(frame)
        raw_boxes = getattr(det, "_last_raw_boxes", []) or []
        projected_pairs = _project_hailo_boxes(
            frame,
            raw_boxes,
            COCO_LABELS,
            (det._input_height, det._input_width),  # type: ignore[attr-defined]
        )
        projected_hailo = [det for det, _ in projected_pairs]

        cpu_detections = cpu_det.detect(frame) if cpu_det is not None else []

        print("OBJECTS DETECTED (Hailo)")
        hailo_to_print = projected_hailo if projected_hailo else list(detections)
        if hailo_to_print:
            if projected_pairs:
                hailo_iter = projected_pairs
            else:
                hailo_iter = list(zip(hailo_to_print, raw_boxes)) if raw_boxes else [(det, None) for det in hailo_to_print]

            for det, raw in hailo_iter:
                label = det.label
                class_id = None
                if label.startswith("class"):
                    try:
                        class_id = int(label[5:])
                        class_name = COCO_LABELS[class_id] if 0 <= class_id < len(COCO_LABELS) else label
                    except Exception:
                        class_name = label
                else:
                    class_name = label
                id_text = f"Class {class_id} ({class_name.upper()})" if class_id is not None else class_name.upper()
                print(f"\n{id_text}")
                print(f"PROCESSED (x,y,w,h): {det.x:.1f}, {det.y:.1f}, {det.width:.1f}, {det.height:.1f}")

                if raw:
                    class_idx, rx1, ry1, rx2, ry2, rconf = raw
                    print(f"RAW: ({rx1:.3f}, {ry1:.3f}, {rx2:.3f}, {ry2:.3f}) conf={rconf:.3f}")
                else:
                    print("RAW: (no data)")
            hailo_counts = Counter(det.label for det in hailo_to_print)
            print(f"TOTALS: {dict(hailo_counts)}")
        else:
            print("  none")

        print("\nOBJECTS DETECTED (CPU)")
        if cpu_detections:
            for det in cpu_detections:
                class_name = det.label
                print(f"\n{class_name.upper()}")
                print(f"PROCESSED (x,y,w,h): {det.x:.1f}, {det.y:.1f}, {det.width:.1f}, {det.height:.1f}")
                print(f"CONFIDENCE: {det.confidence:.2f}")
            cpu_counts = Counter(det.label for det in cpu_detections)
            print(f"TOTALS: {dict(cpu_counts)}")
        else:
            print("  none")

        base_path = os.path.dirname(__file__)
        raw_path = os.path.join(base_path, "test_cap.jpg")
        hailo_ann_path = os.path.join(base_path, "test_cap_hailo_annotated.jpg")
        cpu_ann_path = os.path.join(base_path, "test_cap_cpu_annotated.jpg")
        ann_path = os.path.join(base_path, "test_cap_annotated.jpg")
        cv2.imwrite(raw_path, frame)
        hailo_detections = hailo_to_print
        annotated = draw_detections(frame, hailo_detections)
        cpu_annotated = draw_detections(frame, cpu_detections)
        cv2.imwrite(hailo_ann_path, annotated)
        cv2.imwrite(cpu_ann_path, cpu_annotated)
        cv2.imwrite(ann_path, annotated)
        print(f"Saved raw frame to {raw_path}")
        print(f"Saved Hailo annotated frame to {hailo_ann_path}")
        print(f"Saved CPU annotated frame to {cpu_ann_path}")
        print(f"Saved legacy annotated frame to {ann_path}")

    finally:
        if det is not None:
            with contextlib.suppress(Exception):
                det.close()
        with contextlib.suppress(Exception):
            picam.stop()


if __name__ == "__main__":
    main()
