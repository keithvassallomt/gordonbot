"""
Movement tracker service.

Monitors encoder speed to detect movement start/stop and tracks distance
traveled during the last completed movement.
"""

import threading
import time
from typing import Optional

from app.core.config import settings
from app.services.encoder import get_left_encoder, get_right_encoder

# Movement detection threshold (same as dashboard's "Moving" badge)
MOVEMENT_THRESHOLD_MM_S = 10.0  # 10mm/s = 1cm/s


class MovementTracker:
    """Tracks last completed movement distance from encoders."""

    def __init__(self):
        self._lock = threading.Lock()
        self._is_moving = False
        self._movement_start_left: Optional[float] = None
        self._movement_start_right: Optional[float] = None
        self._last_movement_left_mm: Optional[float] = None
        self._last_movement_right_mm: Optional[float] = None
        self._last_movement_avg_mm: Optional[float] = None

        # Start monitoring thread
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def _monitor_loop(self):
        """Background thread that monitors encoder speed."""
        while self._running:
            try:
                self._update()
                time.sleep(0.05)  # 20Hz polling (same as dashboard)
            except Exception as e:
                # Log errors but keep running
                import logging
                logging.getLogger(__name__).error(f"Movement tracker error: {e}")

    def _update(self):
        """Check encoder speeds and update movement state."""
        left_enc = get_left_encoder(
            settings.encoder_left_pa,
            settings.encoder_left_pb,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        left_enc.set_scales(settings.encoder_scale_fwd, settings.encoder_scale_rev)

        right_enc = get_right_encoder(
            settings.encoder_right_pa,
            settings.encoder_right_pb,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        right_enc.set_scales(settings.encoder_scale_fwd_right, settings.encoder_scale_rev_right)

        # Get current speeds
        left_speed = abs(left_enc.speed_mm_s() or 0)
        right_speed = abs(right_enc.speed_mm_s() or 0)
        max_speed = max(left_speed, right_speed)

        # Check if moving
        is_currently_moving = max_speed >= MOVEMENT_THRESHOLD_MM_S

        with self._lock:
            if is_currently_moving and not self._is_moving:
                # Movement started - record starting positions
                self._movement_start_left = left_enc.distance_m()
                self._movement_start_right = right_enc.distance_m()
                self._is_moving = True

            elif not is_currently_moving and self._is_moving:
                # Movement stopped - calculate distance traveled
                current_left = left_enc.distance_m()
                current_right = right_enc.distance_m()

                if (
                    self._movement_start_left is not None
                    and self._movement_start_right is not None
                    and current_left is not None
                    and current_right is not None
                ):
                    # Calculate absolute distance traveled (unsigned)
                    left_distance_mm = abs(current_left - self._movement_start_left) * 1000
                    right_distance_mm = abs(current_right - self._movement_start_right) * 1000
                    avg_distance_mm = (left_distance_mm + right_distance_mm) / 2.0

                    self._last_movement_left_mm = left_distance_mm
                    self._last_movement_right_mm = right_distance_mm
                    self._last_movement_avg_mm = avg_distance_mm

                self._is_moving = False

    def get_last_movement(self) -> dict:
        """Get the last completed movement distances."""
        with self._lock:
            return {
                "left_mm": self._last_movement_left_mm,
                "right_mm": self._last_movement_right_mm,
                "avg_mm": self._last_movement_avg_mm,
            }

    def shutdown(self):
        """Stop the monitoring thread."""
        self._running = False


# Singleton instance
_tracker: Optional[MovementTracker] = None
_tracker_lock = threading.Lock()


def get_movement_tracker() -> MovementTracker:
    """Get the singleton MovementTracker instance."""
    global _tracker
    if _tracker is None:
        with _tracker_lock:
            if _tracker is None:
                _tracker = MovementTracker()
    return _tracker
