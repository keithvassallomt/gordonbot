from __future__ import annotations

"""Detector abstractions used by the camera annotated publisher."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Sequence
import logging

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Detection:
    """Single detection result in image pixel space."""

    x: int
    y: int
    width: int
    height: int
    label: str
    confidence: float

    def bottom_right(self) -> tuple[int, int]:
        return (self.x + self.width, self.y + self.height)


class Detector(ABC):
    """Base class for object detectors.

    Handles frame interval throttling and persistence of the last
    detections so the caller can draw stale boxes between inference frames.
    """

    def __init__(self, interval: int = 1) -> None:
        self.interval = max(1, int(interval))
        self._frame_index = 0
        self._last: list[Detection] = []

    def process(self, frame_bgr: "np.ndarray") -> Sequence[Detection]:  # type: ignore[name-defined]
        """Run the detector at the configured interval.

        Returns the detections from the latest inference, or the cached
        detections when skipping frames.
        """
        self._frame_index += 1
        should_run = (self._frame_index % self.interval) == 0
        if should_run:
            try:
                detections = self.detect(frame_bgr)
                self._last = list(detections)
            except Exception as exc:  # pragma: no cover - defensive
                log.debug("detector detect() raised: %s", exc)
                self._last = []
        return list(self._last)

    @abstractmethod
    def detect(self, frame_bgr: "np.ndarray") -> Sequence[Detection]:  # type: ignore[name-defined]
        """Run inference for a single frame (BGR ndarray)."""


__all__ = ["Detection", "Detector"]
