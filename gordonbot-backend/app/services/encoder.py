from __future__ import annotations

"""
Quadrature encoder service (no gpiozero) — RPi.GPIO only.

Exposes factory functions `get_left_encoder` and `get_right_encoder` that return
singleton Encoder instances per side. Each Encoder uses interrupt-driven edge
callbacks (x1 or x4, like sandbox2.py) with software debounce and provides:

- connected(): whether hardware callbacks are active
- ticks(): absolute step count (monotonic, forward+reverse combined)
- distance_m(): cumulative distance (meters), scaled per direction (fwd/rev)
- rpm(): output shaft RPM (absolute), based on a short moving window
- speed_mm_s(): linear speed in mm/s (absolute), based on a short window
- set_scales(scale_fwd, scale_rev): empirical calibration multipliers

Counts-per-rev is interpreted as x4 counts (e.g., Pololu 12 CPR @ x4 × gear).
We adjust internally by the chosen decoding factor: steps_per_rev =
counts_per_rev_output × (decode_factor / 4).

If RPi.GPIO is unavailable (dev machine), returns a stub encoder that reports
connected=False and None metrics.
"""

import os
import math
import time
import threading
from collections import deque
from typing import Deque, Optional, Tuple

from app.services.drive_state import get_sign_left, get_sign_right


try:
    import RPi.GPIO as GPIO  # type: ignore
    _GPIO_OK = True
except Exception:
    GPIO = None  # type: ignore
    _GPIO_OK = False


# Defaults aligned with sandbox2.py behavior
_DEFAULT_DECODING = os.getenv("ENCODER_DECODING", "x1").lower()  # "x1" or "x4"
_DEFAULT_DEBOUNCE_US = int(os.getenv("ENCODER_DEBOUNCE_US", "1200"))


class Encoder:
    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        counts_per_rev_output: int,
        wheel_diameter_m: float,
        side: str,
        decoding: str = _DEFAULT_DECODING,
        debounce_us: int = _DEFAULT_DEBOUNCE_US,
    ) -> None:
        self.pin_a = pin_a
        self.pin_b = pin_b
        self._decoding = decoding if decoding in ("x1", "x4") else "x1"
        self._debounce_ns = max(0, int(debounce_us) * 1000)
        self._decoding_factor = 4 if self._decoding == "x4" else 1
        self._counts_per_rev_x4 = max(1, int(counts_per_rev_output))
        # Interpret provided counts as x4 counts/rev and adjust to chosen decoding
        self._steps_per_rev = max(
            1, int(round(self._counts_per_rev_x4 * (self._decoding_factor / 4.0)))
        )
        self._circum_mm = float(math.pi * max(0.0, wheel_diameter_m) * 1000.0)
        self._mm_per_step_base = self._circum_mm / float(self._steps_per_rev)
        # Which side this encoder is on ("left" or "right")
        self._side = "left" if str(side).lower().startswith("l") else "right"
        self._sign_fn = get_sign_left if self._side == "left" else get_sign_right
        # Empirical calibration scales (set via set_scales)
        self._scale_fwd = 1.0
        self._scale_rev = 1.0

        # State
        self._ok = False
        self._lock = threading.Lock()
        self._steps_fwd = 0  # absolute forward steps
        self._steps_rev = 0  # absolute reverse steps
        # Signed step events for windowed speed (tuple[ts_ns, +1|-1])
        self._events: Deque[Tuple[int, int]] = deque(maxlen=5000)

        # x4 state machine helpers
        self._prev_state: Optional[int] = None
        self._last_change_ns_a = 0
        self._last_change_ns_b = 0

        if not _GPIO_OK:
            # Dev/stub mode
            self._ok = False
            return

        try:
            # Configure pins (BCM numbering)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            if self._decoding_factor >= 4:
                # Initialize state from current pin levels
                a0 = GPIO.input(self.pin_a)
                b0 = GPIO.input(self.pin_b)
                self._prev_state = ((a0 & 1) << 1) | (b0 & 1)
                # Both edges on both channels
                GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._on_edge_x4)
                GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._on_edge_x4)
            else:
                # x1: rising edges on A only; determine direction by B level
                GPIO.add_event_detect(self.pin_a, GPIO.RISING, callback=self._on_edge_x1)
            self._ok = True
        except Exception:
            # If setup fails, remain in stub mode (no raise)
            try:
                # Best-effort detach if partially attached
                if _GPIO_OK:
                    with self._suppress():
                        GPIO.remove_event_detect(self.pin_a)
                        GPIO.remove_event_detect(self.pin_b)
            except Exception:
                pass
            self._ok = False

    # --- Public API ---
    def set_scales(self, scale_fwd: float, scale_rev: float) -> None:
        with self._lock:
            self._scale_fwd = float(scale_fwd)
            self._scale_rev = float(scale_rev)

    def connected(self) -> bool:
        return bool(self._ok)

    def ticks(self) -> int:
        with self._lock:
            return int(self._steps_fwd + self._steps_rev)

    def distance_m(self) -> Optional[float]:
        # Return cumulative distance (meters) with direction-specific scaling
        if not self._ok:
            return None
        with self._lock:
            # Forward distance increases; reverse distance decreases
            mm = (self._steps_fwd * self._mm_per_step_base * self._scale_fwd) - (
                self._steps_rev * self._mm_per_step_base * self._scale_rev
            )
        return mm / 1000.0

    def rpm(self, window_s: float = 0.5) -> Optional[float]:
        if not self._ok or window_s <= 0:
            return None
        steps_rate = self._steps_per_second(window_s)
        if steps_rate is None:
            return None
        rev_per_s = abs(steps_rate) / float(self._steps_per_rev)
        return rev_per_s * 60.0

    def speed_mm_s(self, window_s: float = 0.5) -> Optional[float]:
        if not self._ok or window_s <= 0:
            return None
        steps_rate = self._steps_per_second(window_s)
        if steps_rate is None:
            return None
        # Use base mm/step; directionless speed is absolute
        return abs(steps_rate) * self._mm_per_step_base

    # --- Internal helpers ---
    def _steps_per_second(self, window_s: float) -> Optional[float]:
        now_ns = time.monotonic_ns()
        window_ns = int(window_s * 1e9)
        cutoff = now_ns - window_ns
        with self._lock:
            # Prune old events
            while self._events and self._events[0][0] < cutoff:
                self._events.popleft()
            if not self._events:
                return 0.0
            # Sum signed steps over window
            signed_steps = sum(delta for _, delta in self._events)
        return float(signed_steps) / window_s

    # Direction inference on A rising: if B is high at A rising, one direction; else the other.
    # The mapping can vary with wiring; choose a consistent convention.
    def _on_edge_x1(self, channel: int) -> None:  # pragma: no cover - hardware callback
        try:
            now_ns = time.monotonic_ns()
            if self._debounce_ns > 0 and (now_ns - self._last_change_ns_a) < self._debounce_ns:
                return
            a = GPIO.input(self.pin_a)  # type: ignore[attr-defined]
            if a != GPIO.HIGH:  # only handle rising edges robustly
                return
            # Use commanded direction like sandbox2 instead of inferring from B
            direction = self._sign_fn()
            if direction == 0:
                return
            with self._lock:
                if direction > 0:
                    self._steps_fwd += 1
                else:
                    self._steps_rev += 1
                self._events.append((now_ns, direction))
            self._last_change_ns_a = now_ns
        except Exception:
            pass

    def _on_edge_x4(self, channel: int) -> None:  # pragma: no cover - hardware callback
        try:
            now_ns = time.monotonic_ns()
            # Per-channel debounce
            if channel == self.pin_a:
                if self._debounce_ns > 0 and (now_ns - self._last_change_ns_a) < self._debounce_ns:
                    return
                self._last_change_ns_a = now_ns
            else:
                if self._debounce_ns > 0 and (now_ns - self._last_change_ns_b) < self._debounce_ns:
                    return
                self._last_change_ns_b = now_ns

            a = GPIO.input(self.pin_a)  # type: ignore[attr-defined]
            b = GPIO.input(self.pin_b)  # type: ignore[attr-defined]
            state = ((a & 1) << 1) | (b & 1)
            prev = self._prev_state
            if prev is None or state == prev:
                self._prev_state = state
                return
            # Valid transition? If not, ignore. Direction comes from drive state.
            next_map = {0: 1, 1: 3, 3: 2, 2: 0}
            prev_map = {0: 2, 2: 3, 3: 1, 1: 0}
            if not (next_map.get(prev) == state or prev_map.get(prev) == state):
                self._prev_state = state
                return
            self._prev_state = state
            # Commanded direction like sandbox2
            direction = self._sign_fn()
            if direction == 0:
                return
            
            with self._lock:
                if direction > 0:
                    self._steps_fwd += 1
                else:
                    self._steps_rev += 1
                self._events.append((now_ns, direction))
        except Exception:
            pass

    # Context manager to suppress exceptions during cleanup
    class _suppress:
        def __enter__(self):  # noqa: D401
            return self

        def __exit__(self, exc_type, exc, tb):  # noqa: D401
            return True


class _StubEncoder:
    """Fallback when RPi.GPIO is unavailable: reports no hardware."""

    def __init__(self, *args, **kwargs) -> None:  # noqa: D401
        self._scale_fwd = 1.0
        self._scale_rev = 1.0

    def set_scales(self, scale_fwd: float, scale_rev: float) -> None:
        self._scale_fwd = float(scale_fwd)
        self._scale_rev = float(scale_rev)

    def connected(self) -> bool:
        return False

    def ticks(self) -> int:
        return 0

    def distance_m(self) -> Optional[float]:
        return None

    def rpm(self, window_s: float = 0.5) -> Optional[float]:  # noqa: ARG002
        return None

    def speed_mm_s(self, window_s: float = 0.5) -> Optional[float]:  # noqa: ARG002
        return None


# Module-level singletons so counts persist across requests
_left: Optional[Encoder] = None
_right: Optional[Encoder] = None


def _make_encoder(pin_a: int, pin_b: int, counts_per_rev_output: int, wheel_diameter_m: float, side: str) -> Encoder:
    if _GPIO_OK:
        return Encoder(pin_a, pin_b, counts_per_rev_output, wheel_diameter_m, side)
    return _StubEncoder()  # type: ignore[return-value]


def get_left_encoder(
    pin_a: int,
    pin_b: int,
    counts_per_rev_output: int,
    wheel_diameter_m: float,
):
    global _left
    if _left is None:
        _left = _make_encoder(pin_a, pin_b, counts_per_rev_output, wheel_diameter_m, side="left")
    return _left


def get_right_encoder(
    pin_a: int,
    pin_b: int,
    counts_per_rev_output: int,
    wheel_diameter_m: float,
):
    global _right
    if _right is None:
        _right = _make_encoder(pin_a, pin_b, counts_per_rev_output, wheel_diameter_m, side="right")
    return _right
