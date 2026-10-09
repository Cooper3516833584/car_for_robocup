#!/usr/bin/env python3
"""Retired motor entry; use closed_loop_motion.py. Read-only helpers remain available."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "code"))

from core.types import Twist2D  # noqa: E402
from tools.motion_test_common import (  # noqa: E402
    T265PoseReader, trace_row, unwrap_delta, wait_for_settle,
)

MIN_DISTANCE_CM = 10.0
MAX_DISTANCE_CM = 1000.0
MAX_ANGLE_DEG = 360.0
MAX_JOG_SECONDS = 3.0
MAX_MOTION_SECONDS = 600.0
MAX_LINEAR_M_S = 0.10
MAX_JOG_M_S = 0.05
MAX_ANGULAR_RAD_S = 0.35
RAMP_DISTANCE_CM = 15.0
RAMP_ANGLE_DEG = 15.0
MAX_POSE_STEP_M = 0.10
MAX_YAW_STEP_RAD = math.radians(20.0)
MAX_ROTATION_CENTER_DRIFT_M = 0.10
NO_PROGRESS_TIMEOUT_S = 3.0


def validate_distance_cm(value: float) -> float:
    distance = float(value)
    if not math.isfinite(distance) or not MIN_DISTANCE_CM <= abs(distance) <= MAX_DISTANCE_CM:
        raise ValueError("distance must be a signed 10..1000 cm")
    return distance


def validate_angle_deg(value: float) -> float:
    angle = float(value)
    if not math.isfinite(angle) or abs(angle) > MAX_ANGLE_DEG:
        raise ValueError("relative angle must be in [-360, +360] degrees")
    return angle


def motion_timeout_s(target: float, speed_per_s: float) -> float:
    """Target-scaled, finite motor deadline; 1000 cm at 0.1 m/s gets 205 s."""
    target = float(target)
    speed_per_s = float(speed_per_s)
    if not all(math.isfinite(v) and v > 0.0 for v in (target, speed_per_s)):
        raise ValueError("target and speed must be finite and positive")
    budget = 2.0 * target / speed_per_s + 5.0
    if budget > MAX_MOTION_SECONDS:
        raise ValueError("target exceeds the 600-second test budget at this profile speed")
    return budget


def forward_progress_cm(start_pose, pose) -> float:
    """Signed motion along the car's heading at the beginning of the action."""
    dx = pose[0] - start_pose[0]
    dy = pose[1] - start_pose[1]
    return 100.0 * (dx * math.cos(start_pose[2]) + dy * math.sin(start_pose[2]))


class YawAccumulator:
    """Accumulate relative CCW-positive yaw, including wrap and a full turn."""

    def __init__(self, initial_yaw_rad: float) -> None:
        self.previous_rad = float(initial_yaw_rad)
        self.total_rad = 0.0

    def add(self, yaw_rad: float) -> float:
        delta = unwrap_delta(self.previous_rad, float(yaw_rad))
        self.total_rad += delta
        self.previous_rad = float(yaw_rad)
        return self.total_rad


@dataclass(frozen=True)
class MotionResult:
    target: float
    achieved: float
    center_shift_cm: float
    elapsed_s: float


class BasicMotionRunner:
    def __init__(self, drive, reader=None, *, period_s: float = 0.02,
                 clock=time.monotonic, sleep=time.sleep) -> None:
        if not 0.0 < period_s < drive.command_timeout_s:
            raise ValueError("command period must be below the drive watchdog timeout")
        self.drive = drive
        self.reader = reader
        self.period_s = period_s
        self.clock = clock
        self.sleep = sleep
        self.aborted = False
        self.t0 = clock()
        self.trace = []
        self._last_pose = None
        self._yaw = None
        self._raw_origin = None

    def _record(self, pose, phase: str) -> None:
        row = trace_row(
            self.drive, pose, phase, elapsed_s=self.clock() - self.t0,
        )
        raw_getter = getattr(self.reader, "latest_raw_xy", None)
        raw_xy = raw_getter() if raw_getter is not None else None
        if raw_xy is None:
            raw_dx_cm = raw_dy_cm = float("nan")
        else:
            if self._raw_origin is None:
                self._raw_origin = raw_xy
            raw_dx_cm = 100.0 * (raw_xy[0] - self._raw_origin[0])
            raw_dy_cm = 100.0 * (raw_xy[1] - self._raw_origin[1])
        self.trace.append(row[:-1] + (raw_dx_cm, raw_dy_cm, row[-1]))

    def _observe(self, pose) -> None:
        if self._last_pose is not None:
            distance = math.hypot(
                pose[0] - self._last_pose[0], pose[1] - self._last_pose[1],
            )
            yaw_step = abs(unwrap_delta(self._last_pose[2], pose[2]))
            if distance > MAX_POSE_STEP_M or yaw_step > MAX_YAW_STEP_RAD:
                raise RuntimeError("T265 pose jumped during motion")
        self._last_pose = pose
        if self._yaw is not None:
            self._yaw.add(pose[2])

    def jog(self, direction: str, seconds: float, linear_speed_m_s: float,
            angular_speed_rad_s: float) -> None:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")

    def distance(self, signed_cm: float, speed_m_s: float) -> MotionResult:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")

    def rotate(self, signed_deg: float, speed_rad_s: float) -> MotionResult:
        # This is RELATIVE to the initial car heading: +CCW/left, -CW/right.
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")

    def _closed_loop(self, kind: str, target: float, speed: float) -> MotionResult:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="explicit, measured schema-v2 vehicle profile")
    parser.add_argument("--confirm-motor-test", action="store_true",
                        help="confirm a supervised motor test with accessible emergency stop")
    parser.add_argument("--confirm-t265-mount-measured", action="store_true",
                        help="confirm that base_link -> T265 mount was measured for this car")
    parser.add_argument("--trace", help="CSV trace path for distance/rotate (default: unique temp file)")
    actions = parser.add_subparsers(dest="action", required=True)
    jog = actions.add_parser("jog", help="brief timed forward/backward/left/right motor check")
    jog.add_argument("--direction", choices=("forward", "backward", "left", "right"), required=True)
    jog.add_argument("--seconds", type=float, default=1.0)
    distance = actions.add_parser("distance", help="T265-controlled signed distance, 10..1000 cm")
    distance.add_argument("--cm", type=float, required=True,
                          help="signed cm: +forward, -backward; magnitude 10..1000")
    rotate = actions.add_parser("rotate", help="T265-controlled relative turn, +/-360 degrees")
    rotate.add_argument("--deg", type=float, required=True,
                        help="RELATIVE to start: +CCW/left, -CW/right")
    return parser


def _write_trace(rows, path: str | None) -> Path | None:
    if not rows:
        return None
    if path is None:
        filename = "basic_motion_%s_%d.csv" % (time.strftime("%Y%m%d_%H%M%S"), os.getpid())
        output = Path(tempfile.gettempdir()) / filename
    else:
        output = Path(path)
    with output.open("w", encoding="utf-8", newline="") as handle:
        handle.write("t_s,base_x_cm,base_y_cm,yaw_ccw_deg,vx_mm_s,vz_mrad_s,raw_camera_dx_cm,raw_camera_dy_cm,phase\n")
        for row in rows:
            handle.write("%.3f,%.2f,%.2f,%.3f,%.1f,%.1f,%.2f,%.2f,%s\n" % row)
    return output


def main(argv=None) -> int:
    print("Retired motion entry; use tools/closed_loop_motion.py or run_center_target_route.py", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
