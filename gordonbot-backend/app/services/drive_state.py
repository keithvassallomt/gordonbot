from __future__ import annotations

"""Shared drive state for encoders to infer direction like sandbox2.

Encoders in sandbox2 treat direction as the commanded motor direction during
measurement, not inferred from A/B phases. This module provides a tiny, thread-
safe store of the last commanded left/right values so encoder callbacks can
attribute each counted step as forward or reverse for that side.
"""

import threading
import time
from typing import Tuple

_lock = threading.Lock()
_left = 0.0
_right = 0.0
_ts_ns = 0


def set_drive(left: float, right: float) -> None:
    global _left, _right, _ts_ns
    with _lock:
        _left = float(left)
        _right = float(right)
        _ts_ns = time.monotonic_ns()


def _sign(x: float, deadband: float = 1e-3) -> int:
    if x > deadband:
        return 1
    if x < -deadband:
        return -1
    return 0


def get_sign_left() -> int:
    with _lock:
        return _sign(_left)


def get_sign_right() -> int:
    with _lock:
        return _sign(_right)


def last_command() -> Tuple[float, float, int]:
    with _lock:
        return _left, _right, _ts_ns

