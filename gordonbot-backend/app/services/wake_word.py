from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Deque, Optional

log = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency in dev
    import pvporcupine  # type: ignore
except Exception:  # pragma: no cover - handled at runtime
    pvporcupine = None  # type: ignore

try:  # pragma: no cover - optional dependency in dev
    from pvrecorder import PvRecorder  # type: ignore
except Exception:  # pragma: no cover - handled at runtime
    PvRecorder = None  # type: ignore


DetectionCallback = Callable[[datetime], None]


@dataclass
class WakeWordStats:
    last_detected_at: Optional[datetime] = None
    detections: int = 0


class WakeWordService:
    """Background Porcupine wake-word listener."""

    def __init__(
        self,
        access_key: str,
        keyword_path: str,
        sensitivity: float = 0.6,
        device_index: int | None = None,
        *,
        callback: DetectionCallback | None = None,
        allow_missing_deps: bool = True,
    ) -> None:
        self._access_key = access_key
        self._keyword_path = keyword_path
        self._sensitivity = max(0.0, min(1.0, sensitivity))
        self._device_index = device_index
        self._callback = callback
        self._allow_missing_deps = allow_missing_deps

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._stats_lock = threading.Lock()
        self._stats = WakeWordStats()
        self._recent_detections: Deque[datetime] = deque(maxlen=10)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        if not self._access_key or not self._keyword_path:
            log.warning("WakeWordService start requested without access key or keyword path")
            return
        if pvporcupine is None or PvRecorder is None:
            msg = "Porcupine dependencies not available (pvporcupine/pvrecorder)"
            if self._allow_missing_deps:
                log.warning("%s; wake word disabled", msg)
                return
            raise RuntimeError(msg)

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, name="wakeword", daemon=True)
        self._thread.start()

    def stop(self, *, wait: bool = True) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread and wait:
            thread.join(timeout=2.0)
        self._thread = None

    def get_stats(self) -> WakeWordStats:
        with self._stats_lock:
            return WakeWordStats(
                last_detected_at=self._stats.last_detected_at,
                detections=self._stats.detections,
            )

    def recent_detections(self) -> list[datetime]:
        with self._stats_lock:
            return list(self._recent_detections)

    # ------------------------------------------------------------------
    def _run_loop(self) -> None:  # pragma: no cover - thread
        recorder = None
        porcupine = None
        try:
            porcupine = pvporcupine.create(
                access_key=self._access_key,
                keyword_paths=[self._keyword_path],
                sensitivities=[self._sensitivity],
            )
        except Exception as exc:
            log.error("Failed to initialise Porcupine: %s", exc)
            return

        frame_length = porcupine.frame_length
        sample_rate = porcupine.sample_rate
        log.info(
            "Wake word listener starting (device=%s, frame=%d, rate=%d)",
            self._device_index,
            frame_length,
            sample_rate,
        )

        try:
            recorder = PvRecorder(
                device_index=self._device_index if self._device_index is not None else -1,
                frame_length=frame_length,
            )
            recorder.start()
        except Exception as exc:
            log.error("Failed to start PvRecorder audio capture: %s", exc)
            porcupine.delete()
            return

        try:
            while not self._stop_event.is_set():
                try:
                    pcm = recorder.read()
                except Exception as exc:
                    log.warning("Wake word audio read failed: %s", exc)
                    time.sleep(0.2)
                    continue

                if porcupine.process(pcm) >= 0:
                    when = datetime.utcnow()
                    with self._stats_lock:
                        self._stats.detections += 1
                        self._stats.last_detected_at = when
                        self._recent_detections.append(when)
                    log.info("Wake word detected at %s", when.isoformat())
                    if self._callback:
                        try:
                            self._callback(when)
                        except Exception:  # pragma: no cover - callback safety
                            log.exception("Wake word callback raised an exception")

        finally:
            try:
                if recorder is not None:
                    recorder.stop()
            except Exception:
                pass
            try:
                if recorder is not None:
                    recorder.delete()
            except Exception:
                pass
            try:
                if porcupine is not None:
                    porcupine.delete()
            except Exception:
                pass
            log.info("Wake word listener stopped")
