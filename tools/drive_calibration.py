#!/usr/bin/env python3
"""One supervised, low-speed drive pulse measured by the runtime's fused pose.

Start the separate SLAM sidecar first. Each invocation sends exactly one bounded
straight or in-place command, stops the drive, then records the settled result.
The requested displacement is predicted by integrating DifferentialDrive's
limited Twist2D output; pose feedback does not correct the motor command.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from components.pose_fusion import PoseFusionState  # noqa: E402
from config.relative_slam_profile import accepted_relative_slam_profile  # noqa: E402
from config.v2_factory import build_differential_drive  # noqa: E402
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config  # noqa: E402
from config.v2_runtime import RuntimeMode, runtime_constraints  # noqa: E402
from core.types import Twist2D  # noqa: E402
from robocup_runtime import RobocupMissionState, build_runtime  # noqa: E402
from tools.motion_test_common import unwrap_delta  # noqa: E402


SEND_PERIOD_S = 0.02
MAX_PULSE_S = 45.0
SETTLE_TIMEOUT_S = 3.0
NO_PROGRESS_TIMEOUT_S = 3.0
MAX_POSE_STEP_M = 0.10
MAX_YAW_STEP_RAD = math.radians(20.0)
MAX_ROTATION_CENTER_DRIFT_M = 0.10
ZERO = Twist2D(0.0, 0.0)


@dataclass(frozen=True)
class PoseSample:
    received_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    t265_age_s: float
    slam_age_s: float
    t265_confidence: float
    source_flags: tuple[str, ...]


@dataclass(frozen=True)
class PulseResult:
    target: float
    predicted: float
    measured: float
    lateral_m: float
    center_shift_m: float
    elapsed_s: float


def accepted_fused_sample(estimate, config, now_s: float) -> PoseSample | None:
    """Require the same fresh T265+SLAM chain used by relative task actions."""

    if (estimate.pose is None or estimate.state is not PoseFusionState.OK
            or not {"t265", "slam", "fused"}.issubset(estimate.source_flags)
            or not estimate.anchor_initialized
            or estimate.t265_age_s is None or estimate.d500_age_s is None
            or estimate.t265_confidence is None
            or not all(math.isfinite(value) for value in (
                now_s, estimate.pose.x_m, estimate.pose.y_m, estimate.pose.yaw_rad,
                estimate.t265_age_s, estimate.d500_age_s, estimate.t265_confidence,
            ))
            or estimate.t265_age_s < 0.0 or estimate.d500_age_s < 0.0
            or estimate.t265_age_s > config.fusion.t265_max_age_s
            or estimate.d500_age_s > 0.5
            or estimate.t265_confidence < config.fusion.t265_min_tracker_confidence / 3.0):
        return None
    pose = estimate.pose
    return PoseSample(
        now_s, pose.x_m, pose.y_m, pose.yaw_rad,
        estimate.t265_age_s, estimate.d500_age_s,
        estimate.t265_confidence, estimate.source_flags,
    )


class FusedPoseReader(threading.Thread):
    """Run the existing sensor-only Runtime and keep its latest accepted pose."""

    def __init__(self, config, *, clock=time.monotonic, sleep=time.sleep) -> None:
        super().__init__(daemon=True, name="drive-calibration-fused-pose")
        self.config = accepted_relative_slam_profile(config)
        self.clock = clock
        self.sleep = sleep
        self.runtime = build_runtime(self.config, RuntimeMode.HARDWARE_PROBE, sensor_only=True, clock=clock)
        self.error: str | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._sample: PoseSample | None = None

    def run(self) -> None:
        try:
            self.runtime.start()
            while not self._stop_event.is_set():
                result = self.runtime.step()
                if result.error or result.mission_state in {RobocupMissionState.ERROR, RobocupMissionState.SAFE_STOP}:
                    raise RuntimeError(result.error or result.mission_state.value)
                sample = accepted_fused_sample(result.estimate, self.config, self.clock())
                with self._lock:
                    self._sample = sample
                self.sleep(SEND_PERIOD_S)
        except Exception as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)
            with self._lock:
                self._sample = None
        finally:
            self.runtime.close()

    def latest(self) -> PoseSample | None:
        with self._lock:
            sample = self._sample
        if sample is None or self.error:
            return None
        age = self.clock() - sample.received_s
        return sample if 0.0 <= age <= self.config.fusion.t265_max_age_s else None

    def wait_ready(self, timeout_s: float = 30.0) -> bool:
        deadline = self.clock() + timeout_s
        while self.clock() < deadline and not self.error:
            if self.latest() is not None:
                return True
            self.sleep(0.05)
        return False

    def stop(self) -> None:
        self._stop_event.set()


def _progress(start: PoseSample, current: PoseSample, kind: str, yaw_total: float) -> tuple[float, float, float]:
    dx = current.x_m - start.x_m
    dy = current.y_m - start.y_m
    along = dx * math.cos(start.yaw_rad) + dy * math.sin(start.yaw_rad)
    lateral = -dx * math.sin(start.yaw_rad) + dy * math.cos(start.yaw_rad)
    center = math.hypot(dx, dy)
    return (along if kind == "distance" else yaw_total), lateral, center


class PulseRunner:
    def __init__(self, drive, reader, *, clock=time.monotonic, sleep=time.sleep) -> None:
        self.drive = drive
        self.reader = reader
        self.clock = clock
        self.sleep = sleep
        self.aborted = False
        self.trace: list[dict] = []

    def _sample(self) -> PoseSample:
        sample = self.reader.latest()
        if sample is None:
            raise RuntimeError("fresh fused T265+SLAM pose unavailable")
        return sample

    def run(self, kind: str, target: float, speed: float) -> PulseResult:
        if kind not in {"distance", "rotate"} or not math.isfinite(target) or target == 0.0:
            raise ValueError("a nonzero distance or rotation is required")
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed must be positive and finite")
        start = self._sample()
        previous = start
        yaw_total = 0.0
        predicted = 0.0
        direction = 1.0 if target > 0.0 else -1.0
        requested = Twist2D(direction * speed, 0.0) if kind == "distance" else Twist2D(0.0, direction * speed)
        started = self.clock()
        deadline = started + min(MAX_PULSE_S, 2.0 * abs(target) / speed + 5.0)
        last_send = started
        last_limited = ZERO
        last_progress_s = started
        best_progress = 0.0
        next_send = started
        try:
            while True:
                now = self.clock()
                if self.aborted:
                    raise RuntimeError("operator aborted motion")
                if now >= deadline:
                    raise TimeoutError("drive pulse exceeded its finite deadline")
                current = self._sample()
                delta_yaw = unwrap_delta(previous.yaw_rad, current.yaw_rad)
                if (math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) > MAX_POSE_STEP_M
                        or abs(delta_yaw) > MAX_YAW_STEP_RAD):
                    raise RuntimeError("fused pose jumped during motion")
                yaw_total += delta_yaw
                previous = current
                measured, lateral, center = _progress(start, current, kind, yaw_total)
                directed = direction * measured
                if directed < (-0.03 if kind == "distance" else -math.radians(5.0)):
                    raise RuntimeError("measured motion opposes the request")
                if kind == "rotate" and center > MAX_ROTATION_CENTER_DRIFT_M:
                    raise RuntimeError("rotation center drift exceeded 10 cm")
                if kind == "distance" and abs(lateral) > 0.10:
                    raise RuntimeError("straight pulse strayed over 10 cm laterally")
                if directed > abs(target) + (0.15 if kind == "distance" else math.radians(20.0)):
                    raise RuntimeError("measured motion exceeded the pulse guard")
                step_progress = 0.005 if kind == "distance" else math.radians(0.5)
                if directed >= best_progress + step_progress:
                    best_progress = directed
                    last_progress_s = now
                elif now - last_progress_s >= NO_PROGRESS_TIMEOUT_S:
                    raise TimeoutError("no measured progress for 3 seconds")

                predicted += (last_limited.linear_x_m_s if kind == "distance" else last_limited.angular_z_rad_s) * (now - last_send)
                last_send = now
                if direction * predicted >= abs(target):
                    break
                if now >= next_send:
                    self.drive.command(requested)
                    last_limited = self.drive.last_limited_twist
                    next_send = now + SEND_PERIOD_S
                    self.trace.append({
                        "t_s": now - started, "phase": "pulse", "kind": kind,
                        "target": target, "predicted": predicted,
                        "measured": measured, "lateral_m": lateral, "yaw_rad": yaw_total,
                        "base_x_m": current.x_m, "base_y_m": current.y_m,
                        "center_shift_m": center,
                        "requested_v_m_s": requested.linear_x_m_s,
                        "requested_omega_rad_s": requested.angular_z_rad_s,
                        "limited_v_m_s": last_limited.linear_x_m_s,
                        "limited_omega_rad_s": last_limited.angular_z_rad_s,
                        "t265_age_s": current.t265_age_s, "slam_age_s": current.slam_age_s,
                        "t265_confidence": current.t265_confidence,
                        "source_flags": "+".join(current.source_flags),
                    })
                self.sleep(0.005)
        finally:
            self.drive.stop()

        final, final_yaw = self._wait_for_settle(previous, yaw_total)
        measured, lateral, center = _progress(start, final, kind, final_yaw)
        self.trace.append({
            "t_s": self.clock() - started, "phase": "settled", "kind": kind,
            "target": target, "predicted": predicted,
            "measured": measured, "lateral_m": lateral, "yaw_rad": final_yaw,
            "base_x_m": final.x_m, "base_y_m": final.y_m,
            "center_shift_m": center,
            "requested_v_m_s": 0.0, "requested_omega_rad_s": 0.0,
            "limited_v_m_s": 0.0, "limited_omega_rad_s": 0.0,
            "t265_age_s": final.t265_age_s, "slam_age_s": final.slam_age_s,
            "t265_confidence": final.t265_confidence,
            "source_flags": "+".join(final.source_flags),
        })
        return PulseResult(target, predicted, measured, lateral, center, self.clock() - started)

    def _wait_for_settle(self, previous: PoseSample, yaw_total: float) -> tuple[PoseSample, float]:
        deadline = self.clock() + SETTLE_TIMEOUT_S
        still_since = None
        while self.clock() < deadline:
            if self.aborted:
                raise RuntimeError("operator aborted during settle")
            self.sleep(0.04)
            current = self._sample()
            distance = math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m)
            turn = unwrap_delta(previous.yaw_rad, current.yaw_rad)
            if distance > MAX_POSE_STEP_M or abs(turn) > MAX_YAW_STEP_RAD:
                raise RuntimeError("fused pose jumped while settling")
            yaw_total += turn
            if distance < 0.003 and abs(turn) < math.radians(0.3):
                if still_since is None:
                    still_since = self.clock()
                elif self.clock() - still_since >= 0.25:
                    return current, yaw_total
            else:
                still_since = None
            previous = current
        raise TimeoutError("vehicle did not settle within 3 seconds")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="this car's local schema-v2 profile")
    parser.add_argument("--trace", required=True, help="CSV file outside the repository for this one action")
    parser.add_argument("--confirm-motor-test", action="store_true")
    parser.add_argument("--confirm-mount-checked", action="store_true")
    actions = parser.add_subparsers(dest="action", required=True)
    distance = actions.add_parser("distance")
    distance.add_argument("--cm", type=float, required=True, help="signed 10..50 cm")
    distance.add_argument("--speed-m-s", type=float, default=0.05)
    rotate = actions.add_parser("rotate")
    rotate.add_argument("--deg", type=float, required=True, help="signed 10..90 degrees, CCW positive")
    rotate.add_argument("--speed-rad-s", type=float, default=0.20)
    return parser


def _write_trace(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.confirm_motor_test or not args.confirm_mount_checked:
        parser.error("both --confirm-motor-test and --confirm-mount-checked are required")
    if sys.platform != "linux":
        parser.error("real drive calibration must run on the car's Linux host")
    config_path = Path(args.config).resolve()
    if config_path == DEFAULT_V2_CONFIG.resolve():
        parser.error("the example profile is not a measured car profile")
    trace_path = Path(args.trace).resolve()
    if trace_path == ROOT or ROOT in trace_path.parents:
        parser.error("write calibration traces outside the repository")
    if trace_path.exists():
        parser.error("trace path already exists; use a unique file for each run")
    if not trace_path.parent.is_dir():
        parser.error("trace directory does not exist")
    config = load_v2_config(config_path)
    if config.relay.enabled:
        parser.error("disable the payload relay for drive calibration")
    if config.drive.protocol_mode == "differential_vx_vz" and not config.calibration.c10b_diff_firmware_verified:
        parser.error("C10B differential firmware mode is not verified")
    if args.action == "distance":
        target = float(args.cm) / 100.0
        speed = float(args.speed_m_s)
        if not math.isfinite(target) or not 0.10 <= abs(target) <= 0.50:
            parser.error("distance must be signed 10..50 cm")
    else:
        target = math.radians(float(args.deg))
        speed = float(args.speed_rad_s)
        if not math.isfinite(target) or not math.radians(10.0) <= abs(target) <= math.pi / 2.0:
            parser.error("rotation must be signed 10..90 degrees")
        if not config.drive.allow_in_place_rotation:
            parser.error("this profile disables in-place rotation")
    limits = runtime_constraints(config, RuntimeMode.HARDWARE_PROBE)
    speed_limit = limits.max_linear_speed_m_s if args.action == "distance" else limits.max_angular_speed_rad_s
    if not math.isfinite(speed) or not 0.0 < speed <= speed_limit:
        parser.error("speed exceeds the hardware-probe limit or is invalid")
    try:
        with trace_path.open("x", encoding="utf-8"):
            pass
    except OSError as exc:
        parser.error("cannot reserve trace path: %s" % exc)

    reader = FusedPoseReader(config)
    drive = build_differential_drive(config)
    runner = PulseRunner(drive, reader)

    def on_signal(_signum, _frame):
        runner.aborted = True

    for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if signum is not None:
            signal.signal(signum, on_signal)
    status = 0
    try:
        reader.start()
        if not reader.wait_ready():
            raise RuntimeError(reader.error or "fresh fused T265+SLAM pose not ready within 30 seconds")
        if runner.aborted:
            raise RuntimeError("operator aborted before motor start")
        with drive:
            result = runner.run(args.action, target, speed)
        unit = "m" if args.action == "distance" else "deg"
        scale = 1.0 if args.action == "distance" else 180.0 / math.pi
        print("target=%+.3f %s predicted=%+.3f %s measured=%+.3f %s error=%+.3f %s lateral=%.3f m center_shift=%.3f m" % (
            result.target * scale, unit, result.predicted * scale, unit,
            result.measured * scale, unit, (result.measured - result.target) * scale, unit,
            result.lateral_m, result.center_shift_m,
        ))
    except Exception as exc:
        print("ERROR: %s: %s" % (type(exc).__name__, exc))
        status = 1
    finally:
        try:
            drive.close()
        except Exception as exc:
            print("drive close failed: %s" % exc)
            status = 1
        finally:
            reader.stop()
            if reader.is_alive():
                reader.join(timeout=2.0)
                if reader.is_alive():
                    print("sensor worker did not stop within 2 seconds")
                    status = 1
        try:
            _write_trace(trace_path, runner.trace)
            if runner.trace:
                print("trace: %s" % trace_path)
        except OSError as exc:
            print("trace write failed: %s" % exc)
            status = 1
        if drive.is_running:
            print("WARNING: drive still reports running; use the physical emergency stop")
            status = 1
        else:
            print("drive closed")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
