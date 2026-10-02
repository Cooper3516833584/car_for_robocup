#!/usr/bin/env python3
"""Supervised, single-action differential-car motion checks.

Examples (only on a prepared car with an accessible physical emergency stop):

    python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml \
        --confirm-motor-test jog --direction forward --seconds 1
    python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml \
        --confirm-motor-test --confirm-t265-mount-measured distance --cm -50
    python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml \
        --confirm-motor-test --confirm-t265-mount-measured rotate --deg +37

``distance --cm`` accepts signed 10..1000 cm. ``rotate --deg`` is a RELATIVE
angle from the initial heading: positive is counter-clockwise, negative is
clockwise, and either direction may make at most one complete turn. All pose
feedback is mount-corrected ``base_link`` data, never the raw T265 camera center.
No competition map or D500 localization is used by this tool.
"""

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
        """Timed, low-speed wheel-direction smoke test; T265 is not needed."""
        seconds = float(seconds)
        if direction not in {"forward", "backward", "left", "right"} or not math.isfinite(seconds) or not 0.1 <= seconds <= MAX_JOG_SECONDS:
            raise ValueError("jog needs forward/backward/left/right and 0.1..3 seconds")
        if not all(math.isfinite(v) and v > 0.0 for v in (linear_speed_m_s, angular_speed_rad_s)):
            raise ValueError("jog speeds must be positive and finite")
        if direction in {"forward", "backward"}:
            target = Twist2D(linear_speed_m_s if direction == "forward" else -linear_speed_m_s, 0.0)
        else:
            target = Twist2D(0.0, angular_speed_rad_s if direction == "left" else -angular_speed_rad_s)
        deadline = self.clock() + seconds
        next_send = self.clock()
        try:
            while self.clock() < deadline:
                if self.aborted:
                    raise RuntimeError("operator aborted jog")
                now = self.clock()
                if now >= next_send:
                    self.drive.command(target)
                    next_send = now + self.period_s
                self.sleep(0.005)
        finally:
            self.drive.stop()

    def distance(self, signed_cm: float, speed_m_s: float) -> MotionResult:
        target = validate_distance_cm(signed_cm)
        return self._closed_loop("distance", target, speed_m_s)

    def rotate(self, signed_deg: float, speed_rad_s: float) -> MotionResult:
        # This is RELATIVE to the initial car heading: +CCW/left, -CW/right.
        target = validate_angle_deg(signed_deg)
        if target == 0.0:
            self.drive.stop()
            return MotionResult(0.0, 0.0, 0.0, 0.0)
        return self._closed_loop("rotate", target, speed_rad_s)

    def _closed_loop(self, kind: str, target: float, speed: float) -> MotionResult:
        if self.reader is None:
            raise RuntimeError("T265 reader is required for closed-loop motion")
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("motion speed must be positive and finite")
        start_sample = self.reader.latest_sample()
        if start_sample is None:
            raise RuntimeError("no fresh T265 pose before motor command")
        start_pose = start_sample[1]
        self._last_pose = start_pose
        self._yaw = YawAccumulator(start_pose[2]) if kind == "rotate" else None
        direction = 1.0 if target > 0.0 else -1.0
        target_abs = abs(target)
        rate = speed * 100.0 if kind == "distance" else math.degrees(speed)
        deadline = self.clock() + motion_timeout_s(target_abs, rate)
        started = self.clock()
        self.t0 = started
        next_send = started
        last_progress_s = started
        best_progress = 0.0
        try:
            while True:
                now = self.clock()
                if self.aborted:
                    raise RuntimeError("operator aborted motion")
                if now >= deadline:
                    raise TimeoutError("motion exceeded its target-scaled time budget")
                sample = self.reader.latest_sample()
                if sample is None:
                    raise RuntimeError("T265 pose stale, invalid, or low-confidence during motion")
                pose = sample[1]
                self._observe(pose)
                if kind == "distance":
                    progress = direction * forward_progress_cm(start_pose, pose)
                    ramp = RAMP_DISTANCE_CM
                else:
                    progress = direction * math.degrees(self._yaw.total_rad)
                    ramp = RAMP_ANGLE_DEG
                    if math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1]) > MAX_ROTATION_CENTER_DRIFT_M:
                        raise RuntimeError("base_link moved over 10 cm during an in-place turn")
                if progress < -5.0:
                    raise RuntimeError("measured motion opposes the requested direction")
                if progress >= best_progress + 0.5:
                    best_progress = progress
                    last_progress_s = now
                elif now - last_progress_s >= NO_PROGRESS_TIMEOUT_S:
                    raise TimeoutError("no measured progress for 3 seconds")
                remaining = target_abs - progress
                if remaining <= 1e-6:
                    break
                if now >= next_send:
                    scale = min(1.0, max(0.20, remaining / ramp))
                    if kind == "distance":
                        self.drive.command(Twist2D(direction * speed * scale, 0.0))
                    else:
                        self.drive.command(Twist2D(0.0, direction * speed * scale))
                    self._record(pose, kind)
                    next_send = now + self.period_s
                self.sleep(0.005)
        finally:
            self.drive.stop()

        final_pose = wait_for_settle(
            self.drive, self.reader, aborted=lambda: self.aborted,
            record=self._record, on_pose=self._observe,
            clock=self.clock, sleep=self.sleep,
        )
        if kind == "distance":
            achieved = forward_progress_cm(start_pose, final_pose)
        else:
            achieved = math.degrees(self._yaw.total_rad)
        center_shift_cm = 100.0 * math.hypot(
            final_pose[0] - start_pose[0], final_pose[1] - start_pose[1],
        )
        return MotionResult(target, achieved, center_shift_cm, self.clock() - started)


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
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.confirm_motor_test:
        parser.error("--confirm-motor-test is required for this actuator tool")
    if args.action == "jog":
        if not math.isfinite(args.seconds) or not 0.1 <= args.seconds <= MAX_JOG_SECONDS:
            parser.error("--seconds must be in [0.1, 3.0]")
    elif args.action == "distance":
        try:
            validate_distance_cm(args.cm)
        except ValueError as exc:
            parser.error(str(exc))
    else:
        try:
            validate_angle_deg(args.deg)
        except ValueError as exc:
            parser.error(str(exc))
    if args.action != "jog" and not args.confirm_t265_mount_measured:
        parser.error("distance/rotate require --confirm-t265-mount-measured")

    from config.v2_factory import build_differential_drive
    from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
    from config.v2_runtime import RuntimeMode, runtime_constraints

    if Path(args.config).resolve() == DEFAULT_V2_CONFIG.resolve():
        parser.error("example profile contains unmeasured placeholders")
    config = load_v2_config(args.config)
    if config.drive.protocol_mode == "differential_vx_vz" and not config.calibration.c10b_diff_firmware_verified:
        parser.error("differential C10B firmware mode is not verified")
    if (args.action == "rotate" or (args.action == "jog" and args.direction in {"left", "right"})) and not config.drive.allow_in_place_rotation:
        parser.error("this profile disables in-place rotation")
    if args.action != "jog" and not config.t265.enabled:
        parser.error("this profile disables T265")
    if args.action != "jog" and math.hypot(config.t265_mount.x_m, config.t265_mount.y_m) < 0.001:
        parser.error("T265 is off the rotation axis: measure and enter its nonzero x/y mount")

    limits = runtime_constraints(config, RuntimeMode.HARDWARE_PROBE)
    linear_speed = min(MAX_LINEAR_M_S, limits.max_linear_speed_m_s)
    jog_speed = min(MAX_JOG_M_S, limits.max_linear_speed_m_s)
    angular_speed = min(MAX_ANGULAR_RAD_S, limits.max_angular_speed_rad_s)
    if args.action == "distance":
        try:
            motion_timeout_s(abs(args.cm), linear_speed * 100.0)
        except ValueError as exc:
            parser.error(str(exc))
    if args.action == "rotate" and args.deg != 0.0:
        try:
            motion_timeout_s(abs(args.deg), math.degrees(angular_speed))
        except ValueError as exc:
            parser.error(str(exc))

    if args.action == "rotate" and args.deg == 0.0:
        print("0-degree relative turn: no motor command")
        return 0

    drive = build_differential_drive(config)
    period_s = min(0.02, drive.command_timeout_s / 3.0)
    reader = None
    if args.action != "jog":
        reader = T265PoseReader(
            config.t265_mount, serial=config.t265.serial,
            max_age_s=config.fusion.t265_max_age_s,
            min_tracker_confidence=config.fusion.t265_min_tracker_confidence,
        )
    runner = BasicMotionRunner(drive, reader, period_s=period_s)

    def on_signal(signum, _frame):
        runner.aborted = True
        print("\nSIGNAL %d: stopping" % signum)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, on_signal)
    print("Supervised low-speed test: clear the path; keep the physical emergency stop within reach.")
    status = 0
    try:
        if reader is not None:
            reader.start()
            if not reader.wait_ready() or reader.error:
                raise RuntimeError("T265 %s" % (reader.error or "not ready"))
            if reader.wait_pose() is None or runner.aborted:
                raise RuntimeError("no fresh T265 pose before motor start")
        if runner.aborted:
            raise RuntimeError("operator aborted before motor start")
        with drive:
            if args.action == "jog":
                runner.jog(args.direction, args.seconds, jog_speed, angular_speed)
                print("jog completed: %s for %.2f s; distance not assessed" % (args.direction, args.seconds))
            elif args.action == "distance":
                result = runner.distance(args.cm, linear_speed)
                print("distance: target=%+.1f cm, T265 base_link=%+.1f cm, center shift=%.1f cm, time=%.1f s" % (
                    result.target, result.achieved, result.center_shift_cm, result.elapsed_s,
                ))
            else:
                result = runner.rotate(args.deg, angular_speed)
                print("relative turn: target=%+.1f deg, T265 base_link=%+.1f deg, center shift=%.1f cm, time=%.1f s" % (
                    result.target, result.achieved, result.center_shift_cm, result.elapsed_s,
                ))
    except Exception as exc:  # noqa: BLE001 - stop/close in finally before reporting
        print("ERROR: %s: %s" % (type(exc).__name__, exc))
        status = 1
    finally:
        try:
            drive.close()
        finally:
            if reader is not None:
                reader.stop()
                if reader.is_alive():
                    reader.join(timeout=2.0)
            print("stopped")

    if runner.trace:
        try:
            output = _write_trace(runner.trace, args.trace)
            print("trace written to %s (%d rows)" % (output, len(runner.trace)))
        except OSError as exc:
            print("could not write trace: %s" % exc)
            status = 1
    return status


if __name__ == "__main__":
    raise SystemExit(main())
