#!/usr/bin/env python3
"""Retired motor entry; use closed_loop_motion.py. Read-only helpers remain available."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from core.types import Twist2D  # noqa: E402

MAX_DURATION_S = 3.0
MIN_DURATION_S = 0.5
MAX_LINEAR_M_S = 0.05
MAX_ANGULAR_RAD_S = 0.25
ACTIONS = ("forward", "backward", "rotate-left", "rotate-right")


def _backend_diagnostics(drive) -> dict:
    backend = drive.backend
    encoded = getattr(backend, "last_encoded_wheel_speeds_m_s", None)
    chassis = getattr(backend, "last_c10b_chassis_command", None)
    return {
        "canonical_limited_twist": {
            "vx_m_s": drive.last_limited_twist.linear_x_m_s,
            "omega_rad_s": drive.last_limited_twist.angular_z_rad_s,
        },
        "canonical_wheel_targets_m_s": (
            None if encoded is None else list(encoded)
        ),
        "c10b_quantized": (
            None if chassis is None else {
                "vx_mm_s": chassis.linear_mm_s,
                "vz_mrad_s": chassis.angular_mrad_s,
                "requested_left_mm_s": chassis.requested.left_mm_s,
                "requested_right_mm_s": chassis.requested.right_mm_s,
            }
        ),
    }


def _write_event(path: Path, event: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="explicit vehicle TOML profile")
    parser.add_argument("--duration-s", type=float, default=2.0,
                        help="duration for this one action, 0.5..3.0 seconds")
    parser.add_argument("--output", required=True, type=Path,
                        help="new JSONL path outside the repository")
    parser.add_argument("--confirm-wheels-off-ground", action="store_true", required=True)
    parser.add_argument("--confirm-estop-ready", action="store_true", required=True)
    parser.add_argument("--confirm-area-clear", action="store_true", required=True)
    parser.add_argument("action", choices=ACTIONS)
    return parser


def main(argv=None) -> int:
    print("Unlocalized actuation probe retired; use tools/closed_loop_motion.py",file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
