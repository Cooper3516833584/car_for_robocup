#!/usr/bin/env python3
"""Retired motor entry; use closed_loop_motion.py. Read-only helpers remain available."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "code"))

from tools.motion_test_common import (  # noqa: E402
    T265PoseReader, trace_row, unwrap_delta, wait_for_settle,
)

SEND_HZ = 50.0
MAX_LINEAR_MM_S = 200
MAX_ANGULAR_MRAD_S = 800


def segment_timeout_s(target: float, speed_per_s: float) -> float:
    """Allow for acceleration and settling without an unbounded motor run."""
    if not math.isfinite(target) or not math.isfinite(speed_per_s) or target <= 0 or speed_per_s <= 0:
        raise ValueError("segment target and speed must be finite and positive")
    return min(2.0 * target / speed_per_s + 2.0, 25.0)


def segment_progress(start_pose, pose, mode: str) -> float:
    """Signed progress in the requested forward or CCW direction."""
    if mode == "distance":
        dx = pose[0] - start_pose[0]
        dy = pose[1] - start_pose[1]
        return (dx * math.cos(start_pose[2]) + dy * math.sin(start_pose[2])) * 100.0
    if mode == "angle":
        return math.degrees(unwrap_delta(start_pose[2], pose[2]))
    raise ValueError("unknown segment mode")


class Runner:
    def __init__(self, drive, reader: T265PoseReader, send_hz: float) -> None:
        self.drive = drive
        self.reader = reader
        self.period = 1.0 / send_hz
        self.aborted = False
        self.t0 = time.monotonic()
        self.ramp_cm = 12.0
        self.ramp_deg = 12.0
        self.trace: list[tuple[float, float, float, float, float, float, str]] = []

    def _record(self, pose, phase: str) -> None:
        self.trace.append(trace_row(
            self.drive, pose, phase, elapsed_s=time.monotonic() - self.t0,
        ))

    def _settle(self, *, timeout: float = 3.0):
        return wait_for_settle(
            self.drive, self.reader, aborted=lambda: self.aborted,
            record=self._record, timeout=timeout,
        )

    def hold(self, linear_mm_s: int, angular_mrad_s: int, seconds: float,
             *, start_pose, mode: str, target: float) -> tuple[bool, float, float]:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")

    def run_square(self, *, segment_cm, linear_mm_s, turn_mrad_s,
                   segments) -> None:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")


def main(argv=None) -> int:
    print("Retired motion entry; use tools/closed_loop_motion.py or run_center_target_route.py", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
