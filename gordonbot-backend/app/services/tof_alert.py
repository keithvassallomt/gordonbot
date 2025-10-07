from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from app.core.config import settings
from app.services.audio_playback import AudioCuePlayer
from app.services.tof import read_distance_mm

log = logging.getLogger(__name__)

_MONITOR_LOCK = threading.Lock()
_MONITOR: Optional["_ToFAlertMonitor"] = None
_PLAYER: Optional[AudioCuePlayer] = None


class _ToFAlertMonitor(threading.Thread):
    def __init__(
        self,
        *,
        player: AudioCuePlayer,
        threshold_mm: int,
        retrigger_delta_mm: int,
        poll_interval: float,
    ) -> None:
        super().__init__(name="tof-alert-monitor", daemon=True)
        self._player = player
        self._threshold_mm = max(1, threshold_mm)
        self._retrigger_delta_mm = max(1, retrigger_delta_mm)
        self._poll_interval = max(0.02, poll_interval)
        self._stop_event = threading.Event()
        self._last_alert_distance: Optional[int] = None

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            distance_mm = read_distance_mm()
            try:
                self._process_distance(distance_mm)
            except Exception:
                log.exception("Unexpected error while processing ToF distance")
            elapsed = time.monotonic() - started
            wait = self._poll_interval - elapsed
            if wait <= 0.0:
                wait = self._poll_interval
            self._stop_event.wait(wait)

    def _process_distance(self, distance_mm: Optional[int]) -> None:
        if distance_mm is None:
            return

        if distance_mm <= self._threshold_mm:
            if self._should_trigger(distance_mm):
                log.info("ToF alert triggered at %d mm", distance_mm)
                try:
                    self._player.play()
                except Exception:
                    log.exception("Failed to play ToF alert audio")
                self._last_alert_distance = distance_mm
            return

        if (
            self._last_alert_distance is not None
            and distance_mm >= self._threshold_mm + self._retrigger_delta_mm
        ):
            self._last_alert_distance = None

    def _should_trigger(self, distance_mm: int) -> bool:
        if self._last_alert_distance is None:
            return True
        return distance_mm <= self._last_alert_distance - self._retrigger_delta_mm


def start_tof_alert_monitor() -> None:
    global _MONITOR, _PLAYER

    if not settings.tof_alert_enabled:
        log.info("ToF alert monitor disabled via settings")
        return

    with _MONITOR_LOCK:
        if _MONITOR is not None and _MONITOR.is_alive():
            return

        try:
            player = AudioCuePlayer(
                path=settings.tof_alert_audio_path,
                command_template=settings.tof_alert_audio_command,
                enabled=True,
            )
        except Exception as exc:
            log.error("Failed to initialise ToF alert audio player: %s", exc)
            _PLAYER = None
            return

        monitor = _ToFAlertMonitor(
            player=player,
            threshold_mm=int(settings.tof_alert_threshold_mm),
            retrigger_delta_mm=int(settings.tof_alert_retrigger_delta_mm),
            poll_interval=float(settings.tof_alert_poll_interval),
        )
        monitor.start()
        _PLAYER = player
        _MONITOR = monitor
        log.info(
            "Started ToF alert monitor (threshold=%dmm, retrigger_delta=%dmm, interval=%.3fs)",
            settings.tof_alert_threshold_mm,
            settings.tof_alert_retrigger_delta_mm,
            settings.tof_alert_poll_interval,
        )


def stop_tof_alert_monitor(timeout: float = 1.0) -> None:
    global _MONITOR, _PLAYER

    with _MONITOR_LOCK:
        monitor = _MONITOR
        if monitor is None:
            _PLAYER = None
            return
        monitor.stop()

    monitor.join(timeout=timeout)
    if monitor.is_alive():
        log.warning("ToF alert monitor did not stop cleanly; continuing shutdown")

    with _MONITOR_LOCK:
        _MONITOR = None
        _PLAYER = None
