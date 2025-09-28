from __future__ import annotations

"""
Quadrature encoder service (no gpiozero) using lgpio or RPi.GPIO.

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

If neither lgpio nor RPi.GPIO is available (dev machine), returns a stub encoder
that reports connected=False and None metrics.
"""

import os
import math
import time
import threading
import logging
from glob import glob
from collections import deque
from typing import Deque, Optional, Tuple

from app.services.drive_state import get_sign_left, get_sign_right

log = logging.getLogger(__name__)


try:  # Prefer lgpio on newer Raspberry Pi platforms
    import lgpio  # type: ignore
except Exception:
    lgpio = None  # type: ignore

try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:
    GPIO = None  # type: ignore

if lgpio is not None:
    _GPIO_BACKEND = "lgpio"
elif GPIO is not None:
    _GPIO_BACKEND = "rpi"
else:
    _GPIO_BACKEND = None


def _detect_gpiochips() -> list[int]:
    try:
        paths = glob("/dev/gpiochip*")
        chips = sorted({int(path.rsplit("gpiochip", 1)[1]) for path in paths})
        return chips or [0]
    except Exception:
        return [0]


_GPIOCHIP_CANDIDATES = _detect_gpiochips()


def _env_flag(name: str, default: bool) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    val = val.strip().lower()
    if val in {"1", "true", "yes", "on"}:
        return True
    if val in {"0", "false", "no", "off"}:
        return False
    return default


_LEFT_ENABLED = _env_flag("ENCODER_LEFT_ENABLED", True)
_RIGHT_ENABLED = _env_flag("ENCODER_RIGHT_ENABLED", True)


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
        self._debounce_us = max(0, int(debounce_us))
        self._debounce_ns = self._debounce_us * 1000
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
        self._backend_pref = _GPIO_BACKEND
        self._backend_active: Optional[str] = None
        self._lgpio_handle: Optional[int] = None
        self._lgpio_callbacks = []
        self._lgpio_claimed: set[int] = set()

        ok = False

        if lgpio is not None:
            if self._init_lgpio():
                self._backend_active = "lgpio"
                ok = True

        if not ok and GPIO is not None:
            if self._init_rpi():
                self._backend_active = "rpi"
                ok = True

        if not ok:
            # Dev/stub mode (no usable backend)
            self._ok = False
            return

        self._ok = True

    def _init_lgpio(self) -> bool:
        if lgpio is None:
            return False

        handle: Optional[int] = None
        last_error: Optional[Exception] = None
        for chip in _GPIOCHIP_CANDIDATES:
            try:
                handle = lgpio.gpiochip_open(chip)
                # Ensure the lines exist on this chip
                lgpio.gpio_get_line_info(handle, self.pin_a)
                lgpio.gpio_get_line_info(handle, self.pin_b)
                break
            except Exception as exc:
                last_error = exc
                if handle is not None:
                    with self._suppress():
                        lgpio.gpiochip_close(handle)
                handle = None
        if handle is None:
            if last_error is not None:
                log.debug("lgpio init: unable to open gpiochip for %s encoder: %s", self._side, last_error)
            return False

        self._lgpio_handle = handle

        pull_flag = 0
        if hasattr(lgpio, "SET_BIAS_PULL_UP"):
            pull_flag = lgpio.SET_BIAS_PULL_UP  # type: ignore[attr-defined]
        elif hasattr(lgpio, "SET_PULL_UP"):
            pull_flag = lgpio.SET_PULL_UP  # type: ignore[attr-defined]

        def claim_alert(pin: int, edge: int) -> None:
            lgpio.gpio_claim_alert(handle, pin, edge, pull_flag)
            self._lgpio_claimed.add(pin)

        def claim_input(pin: int) -> None:
            lgpio.gpio_claim_input(handle, pin, pull_flag)
            self._lgpio_claimed.add(pin)

        try:
            if self._decoding_factor >= 4:
                claim_alert(self.pin_a, lgpio.BOTH_EDGES)
                claim_alert(self.pin_b, lgpio.BOTH_EDGES)
                if self._debounce_us > 0:
                    lgpio.gpio_set_debounce_micros(handle, self.pin_a, self._debounce_us)
                    lgpio.gpio_set_debounce_micros(handle, self.pin_b, self._debounce_us)
                a0 = lgpio.gpio_read(handle, self.pin_a)
                b0 = lgpio.gpio_read(handle, self.pin_b)
                self._prev_state = ((a0 & 1) << 1) | (b0 & 1)
                self._lgpio_callbacks.append(
                    lgpio.callback(handle, self.pin_a, lgpio.BOTH_EDGES, self._on_edge_x4_lgpio)
                )
                self._lgpio_callbacks.append(
                    lgpio.callback(handle, self.pin_b, lgpio.BOTH_EDGES, self._on_edge_x4_lgpio)
                )
            else:
                claim_alert(self.pin_a, lgpio.RISING_EDGE)
                claim_input(self.pin_b)
                if self._debounce_us > 0:
                    lgpio.gpio_set_debounce_micros(handle, self.pin_a, self._debounce_us)
                self._lgpio_callbacks.append(
                    lgpio.callback(handle, self.pin_a, lgpio.RISING_EDGE, self._on_edge_x1_lgpio)
                )
            log.info(
                "Encoder %s initialised via lgpio (pins A=%s, B=%s, decoding=%s)",
                self._side,
                self.pin_a,
                self.pin_b,
                self._decoding,
            )
            return True
        except Exception as exc:
            log.warning(
                "lgpio init failed for %s encoder on pins (%s,%s): %s",
                self._side,
                self.pin_a,
                self.pin_b,
                exc,
            )
            self._cleanup_lgpio()
            return False

    def _cleanup_lgpio(self) -> None:
        for cb in self._lgpio_callbacks:
            try:
                cb.cancel()
            except Exception:
                pass
        self._lgpio_callbacks = []
        handle = self._lgpio_handle
        if handle is not None:
            for pin in list(self._lgpio_claimed):
                try:
                    lgpio.gpio_free(handle, pin)
                except Exception:
                    pass
            try:
                lgpio.gpiochip_close(handle)
            except Exception:
                pass
        self._lgpio_claimed.clear()
        self._lgpio_handle = None
        if self._backend_active == "lgpio":
            self._backend_active = None

    def _init_rpi(self) -> bool:
        if GPIO is None:
            return False
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            if self._decoding_factor >= 4:
                a0 = GPIO.input(self.pin_a)
                b0 = GPIO.input(self.pin_b)
                self._prev_state = ((a0 & 1) << 1) | (b0 & 1)
                GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._on_edge_x4_rpi)
                GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._on_edge_x4_rpi)
            else:
                GPIO.add_event_detect(self.pin_a, GPIO.RISING, callback=self._on_edge_x1_rpi)
            log.info(
                "Encoder %s initialised via RPi.GPIO (pins A=%s, B=%s, decoding=%s)",
                self._side,
                self.pin_a,
                self.pin_b,
                self._decoding,
            )
            return True
        except Exception as exc:
            try:
                with self._suppress():
                    if GPIO is not None:
                        GPIO.remove_event_detect(self.pin_a)
                        GPIO.remove_event_detect(self.pin_b)
            except Exception:
                pass
            log.warning(
                "RPi.GPIO init failed for %s encoder on pins (%s,%s): %s",
                self._side,
                self.pin_a,
                self.pin_b,
                exc,
            )
            return False

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
    def _on_edge_x1_rpi(self, channel: int) -> None:  # pragma: no cover - hardware callback
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

    def _on_edge_x4_rpi(self, channel: int) -> None:  # pragma: no cover - hardware callback
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

    def _on_edge_x1_lgpio(self, chip: int, gpio: int, level: int, tick: int) -> None:  # pragma: no cover - hardware callback
        if level != 1:
            return
        try:
            now_ns = time.monotonic_ns()
            if self._debounce_ns > 0 and (now_ns - self._last_change_ns_a) < self._debounce_ns:
                return
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

    def _on_edge_x4_lgpio(self, chip: int, gpio: int, level: int, tick: int) -> None:  # pragma: no cover - hardware callback
        if level > 1:
            return
        handle = self._lgpio_handle
        if handle is None:
            return
        try:
            now_ns = time.monotonic_ns()
            if gpio == self.pin_a:
                if self._debounce_ns > 0 and (now_ns - self._last_change_ns_a) < self._debounce_ns:
                    return
                self._last_change_ns_a = now_ns
            else:
                if self._debounce_ns > 0 and (now_ns - self._last_change_ns_b) < self._debounce_ns:
                    return
                self._last_change_ns_b = now_ns

            a = lgpio.gpio_read(handle, self.pin_a)
            b = lgpio.gpio_read(handle, self.pin_b)
            state = ((a & 1) << 1) | (b & 1)
            prev = self._prev_state
            if prev is None or state == prev:
                self._prev_state = state
                return
            next_map = {0: 1, 1: 3, 3: 2, 2: 0}
            prev_map = {0: 2, 2: 3, 3: 1, 1: 0}
            if not (next_map.get(prev) == state or prev_map.get(prev) == state):
                self._prev_state = state
                return
            self._prev_state = state
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
    """Fallback when no GPIO backend is available: reports no hardware."""

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
    side_key = "left" if str(side).lower().startswith("l") else "right"
    if side_key == "left" and not _LEFT_ENABLED:
        log.info("Encoder %s disabled via ENCODER_LEFT_ENABLED=0", side_key)
        return _StubEncoder()  # type: ignore[return-value]
    if side_key == "right" and not _RIGHT_ENABLED:
        log.info("Encoder %s disabled via ENCODER_RIGHT_ENABLED=0", side_key)
        return _StubEncoder()  # type: ignore[return-value]
    if _GPIO_BACKEND is not None:
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
