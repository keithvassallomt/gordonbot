from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, Optional

# Ensure the backend package is importable when running from sandbox_code/
BACKEND_SRC = Path(__file__).resolve().parents[1] / "gordonbot-backend"
if str(BACKEND_SRC) not in sys.path:
    sys.path.insert(0, str(BACKEND_SRC))

from app.core.config import settings
from app.services.drive_state import set_drive as _set_drive_state
from app.services.encoder import get_left_encoder, get_right_encoder


@dataclass
class EncoderWrapper:
    name: str
    set_scales: Callable[[float, float], None]
    connected: Callable[[], bool]
    ticks: Callable[[], int]
    distance_m: Callable[[], Optional[float]]
    rpm: Callable[[], Optional[float]]
    speed_mm_s: Callable[[], Optional[float]]


def _init_encoders(include_left: bool, include_right: bool) -> Dict[str, EncoderWrapper]:
    encoders: Dict[str, EncoderWrapper] = {}

    if include_left:
        left = get_left_encoder(
            settings.encoder_left_pa,
            settings.encoder_left_pb,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        left.set_scales(settings.encoder_scale_fwd, settings.encoder_scale_rev)
        encoders["left"] = EncoderWrapper(
            name="left",
            set_scales=left.set_scales,
            connected=left.connected,
            ticks=left.ticks,
            distance_m=left.distance_m,
            rpm=left.rpm,
            speed_mm_s=left.speed_mm_s,
        )

    if include_right:
        right = get_right_encoder(
            settings.encoder_right_pa,
            settings.encoder_right_pb,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        right.set_scales(settings.encoder_scale_fwd_right, settings.encoder_scale_rev_right)
        encoders["right"] = EncoderWrapper(
            name="right",
            set_scales=right.set_scales,
            connected=right.connected,
            ticks=right.ticks,
            distance_m=right.distance_m,
            rpm=right.rpm,
            speed_mm_s=right.speed_mm_s,
        )

    if not encoders:
        raise RuntimeError("No encoders selected; choose --left, --right, or --both")

    return encoders


def _format_float(value: Optional[float], unit: str = "", precision: int = 3) -> str:
    if value is None:
        return "—"
    formatted = f"{value:.{precision}f}"
    return f"{formatted}{unit}"


def run(args: argparse.Namespace) -> None:
    encoders = _init_encoders(args.left or args.both, args.right or args.both)

    forced = False
    if args.force_direction is not None:
        forced = True
        left_val = args.force_direction if ("left" in encoders) else 0.0
        right_val = args.force_direction if ("right" in encoders) else 0.0
        _set_drive_state(left_val, right_val)

    stop_time = None if args.duration <= 0 else (time.time() + args.duration)
    start_time = time.time()

    def _handle_signal(signum, frame):  # noqa: ANN001
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while True:
            now = time.time()
            if stop_time is not None and now >= stop_time:
                break

            header = f"t={now - start_time:6.2f}s"
            lines = [header]
            for key in sorted(encoders):
                enc = encoders[key]
                connected = enc.connected()
                ticks = enc.ticks() if connected else "—"
                dist_m = enc.distance_m() if connected else None
                rpm = enc.rpm() if connected else None
                speed = enc.speed_mm_s() if connected else None
                display = (
                    f"{enc.name}: connected={connected} "
                    f"ticks={ticks} "
                    f"dist={_format_float(dist_m, 'm', 4)} "
                    f"rpm={_format_float(rpm)} "
                    f"speed={_format_float(speed, 'mm/s')}"
                )
                lines.append(display)

            print(" | ".join(lines), flush=True)
            time.sleep(max(args.interval, 0.05))
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
    finally:
        if forced:
            _set_drive_state(0.0, 0.0)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simple encoder inspection utility. Requires backend venv/environment."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--left", action="store_true", help="Inspect only the left encoder")
    group.add_argument("--right", action="store_true", help="Inspect only the right encoder")
    group.add_argument("--both", action="store_true", help="Inspect both encoders (default)")

    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Seconds between samples (default: 0.5)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Total seconds to run before exiting (default: run until Ctrl+C)",
    )
    parser.add_argument(
        "--force-direction",
        type=float,
        choices=(-1.0, 0.0, 1.0),
        help=(
            "Override drive state direction for selected encoders. "
            "Useful when spinning wheels by hand; values: -1, 0, or 1."
        ),
    )

    parsed = parser.parse_args(argv)
    if not (parsed.left or parsed.right or parsed.both):
        parsed.both = True
    return parsed


if __name__ == "__main__":
    run(parse_args())
