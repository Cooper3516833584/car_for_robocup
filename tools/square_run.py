#!/usr/bin/env python3
"""EXPERIMENTAL: supervised square-path calibration for the differential car.

This tool has no hardware acceptance result.  Its previous direct-serial runs
did not close; the figures below are historical handoff measurements, not
verified performance of the production DifferentialDrive path.

Measured on the lab field, 4 x (forward 100 cm + 90 deg CCW), slow speed
(Vx=220 mm/s, Vz=400 mrad/s):

  run 1 (no ramping)         closure error 55.7 cm, heading error +43.4 deg
  run 2 (ramped + settle)    closure error 58.2 cm, heading error +47.3 deg

The old waypoint trace omitted the zero-command and settle interval, so it
cannot isolate coast from motion during the last commanded sample.  The new
trace records the drive facade's limited command and the settle interval.
Do not choose coast compensation from the historical numbers alone.

Sign conventions (verified, not assumed):

  * ``angular_z_rad_s > 0`` commands a *left* turn: ``left = linear - w*track/2``,
    ``right = linear + w*track/2`` (``differential_kinematics``), so the right
    wheel runs faster and the car turns counter-clockwise.
  * the radar's own ``yaw_cw_deg`` is clockwise-positive, which is the opposite
    sign.

Safety: this tool requires an explicit vehicle profile and operator
confirmation; it uses DifferentialDrive's speed/acceleration limits, hardware
lock, watchdog, and safe stop.  Fresh T265 pose is required throughout motion.
The operator must still provide a clear test area and an accessible emergency
stop.  This is not a replacement for production mission acceptance.

Usage:
    python3 tools/square_run.py --config configs/robocup_diffdrive.toml \
        --confirm-motor-test --segment-cm 100 --segments 4 \
        --linear-mm-s 100 --turn-mrad-s 400
"""

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
        """Drive one segment, ramping down before the target, then stopping.

        The production drive facade applies its configured acceleration limits
        and watchdog.  This extra distance/angle ramp is only for the approach.
        """

        deadline = time.monotonic() + seconds
        next_send = time.monotonic()
        ramp_cm = self.ramp_cm
        ramp_deg = self.ramp_deg
        min_scale = 0.35
        from core.types import Twist2D

        try:
            while True:
                now = time.monotonic()
                if self.aborted:
                    raise RuntimeError("operator aborted segment")
                if now >= deadline:
                    raise TimeoutError("segment exceeded its bounded time budget")
                pose = self.reader.latest()
                if pose is None:
                    raise RuntimeError("T265 pose stale or invalid during motion")
                done = segment_progress(start_pose, pose, mode)
                if done < -5.0:
                    raise RuntimeError("measured motion opposes the requested direction")
                if mode == "distance":
                    remaining = target - done
                    ramp = ramp_cm
                elif mode == "angle":
                    remaining = target - done
                    ramp = ramp_deg
                else:
                    raise ValueError("unknown segment mode")
                if remaining <= 0.0:
                    break
                scale = max(min_scale, remaining / ramp) if remaining < ramp else 1.0
                if now >= next_send:
                    self.drive.command(Twist2D(
                        linear_mm_s * scale / 1000.0,
                        angular_mrad_s * scale / 1000.0,
                    ))
                    self._record(pose, mode)
                    next_send = now + self.period
                time.sleep(0.005)
        finally:
            self.drive.stop()

        pose = self._settle()
        achieved = segment_progress(start_pose, pose, mode)
        return True, achieved, math.degrees(pose[2])

    def run_square(self, *, segment_cm, linear_mm_s, turn_mrad_s,
                   segments) -> None:
        pose = self.reader.latest()
        if pose is None:
            raise RuntimeError("no T265 pose at start")
        self.t0 = time.monotonic()
        origin = pose
        print("start pose: x=%.1f cm y=%.1f cm yaw=%.1f deg"
              % (pose[0] * 100.0, pose[1] * 100.0, math.degrees(pose[2])))
        print()
        print("%-4s %-9s %10s %10s %8s" %
              ("seg", "action", "target", "achieved", "err%"))
        print("-" * 48)
        for index in range(segments):
            before = self.reader.latest()
            if before is None:
                raise RuntimeError("T265 pose lost before forward segment")
            budget = segment_timeout_s(segment_cm, linear_mm_s / 10.0)
            _, achieved, _ = self.hold(
                linear_mm_s, 0, budget,
                start_pose=before, mode="distance", target=segment_cm,
            )
            err = (achieved - segment_cm) / segment_cm * 100.0
            print("%-4d %-9s %9.1fcm %9.1fcm %7.1f%%"
                  % (index + 1, "forward", segment_cm, achieved, err))
            before = self.reader.latest()
            if before is None:
                raise RuntimeError("T265 pose lost before turn segment")
            budget = segment_timeout_s(
                90.0, math.degrees(turn_mrad_s / 1000.0)
            )
            _, achieved, _ = self.hold(
                0, turn_mrad_s, budget,
                start_pose=before, mode="angle", target=90.0,
            )
            err = (achieved - 90.0) / 90.0 * 100.0
            print("%-4d %-9s %9.1fdeg %9.1fdeg %7.1f%%"
                  % (index + 1, "turn CCW", 90.0, achieved, err))
        final = self.reader.latest()
        if final is None:
            raise RuntimeError("T265 pose lost before closure measurement")
        dx = (final[0] - origin[0]) * 100.0
        dy = (final[1] - origin[1]) * 100.0
        dyaw = math.degrees(unwrap_delta(origin[2], final[2]))
        print()
        print("=== closure ===")
        print("  position error: dx=%+.1f cm dy=%+.1f cm (%.1f cm)"
              % (dx, dy, math.hypot(dx, dy)))
        print("  heading error : %+.1f deg" % dyaw)
        print("  note: T265 yaw is CCW-positive here; the radar convention is")
        print("        the opposite sign.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="explicit schema-v2 vehicle profile")
    parser.add_argument("--confirm-motor-test", action="store_true",
                        help="confirm a supervised low-speed motor test")
    parser.add_argument("--segment-cm", type=float, default=100.0)
    parser.add_argument("--segments", type=int, default=4)
    parser.add_argument("--linear-mm-s", type=int, default=100)
    parser.add_argument("--turn-mrad-s", type=int, default=400)
    parser.add_argument("--send-hz", type=float, default=SEND_HZ)
    parser.add_argument("--trace", help="CSV output path (default: unique file in /tmp)")
    args = parser.parse_args()

    if not args.confirm_motor_test:
        parser.error("--confirm-motor-test is required for this actuator tool")
    if not 0.0 < args.segment_cm <= 100.0:
        parser.error("segment length must be in (0, 100] cm")
    if not 0 < args.turn_mrad_s <= MAX_ANGULAR_MRAD_S:
        parser.error("CCW turn rate must be in (0, 800] mrad/s")
    if not 0 < args.linear_mm_s <= MAX_LINEAR_MM_S:
        parser.error("linear speed must be in (0, 200] mm/s")
    if not 20.0 <= args.send_hz <= 50.0:
        parser.error("send rate must be in [20, 50] Hz")
    if args.segments != 4:
        parser.error("this acceptance run is defined as exactly 4 segments")

    from config.v2_factory import build_differential_drive
    from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config

    if Path(args.config).resolve() == DEFAULT_V2_CONFIG.resolve():
        parser.error("the example profile contains unmeasured placeholders")
    config = load_v2_config(args.config)
    if not config.t265.enabled or not config.drive.allow_in_place_rotation:
        parser.error("profile must enable T265 and in-place rotation")
    if args.linear_mm_s > config.drive.max_linear_speed_m_s * 1000.0:
        parser.error("linear speed exceeds the vehicle profile limit")
    if args.turn_mrad_s > config.drive.max_angular_speed_rad_s * 1000.0:
        parser.error("turn rate exceeds the vehicle profile limit")
    if (config.drive.protocol_mode == "differential_vx_vz"
            and not config.calibration.c10b_diff_firmware_verified):
        parser.error("differential firmware mode is not verified")

    reader = T265PoseReader(
        config.t265_mount, serial=config.t265.serial,
        max_age_s=config.fusion.t265_max_age_s,
        min_tracker_confidence=config.fusion.t265_min_tracker_confidence,
    )
    drive = build_differential_drive(config)
    runner = Runner(drive, reader, args.send_hz)

    def on_signal(signum, frame):
        runner.aborted = True
        print("\nSIGNAL %d: stopping" % signum)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    print("square run: %d x %.0f cm forward + 90 deg CCW, Vx=%d mm/s Vz=%d mrad/s"
          % (args.segments, args.segment_cm, args.linear_mm_s, args.turn_mrad_s))
    print("estimate duration: ~%.0f s" % (
        args.segments * (
            args.segment_cm / (args.linear_mm_s / 10.0)
            + 90.0 / math.degrees(args.turn_mrad_s / 1000.0)
        )
    ))
    print()

    status = 0
    try:
        reader.start()
        if not reader.wait_ready() or reader.error:
            raise RuntimeError("T265 %s" % (reader.error or "not ready"))
        start_pose = reader.wait_pose(15.0)
        if start_pose is None or runner.aborted:
            raise RuntimeError("no fresh T265 pose before motor start")
        print("T265 ready, start x=%.1f cm y=%.1f cm yaw=%.1f deg"
              % (start_pose[0] * 100.0, start_pose[1] * 100.0,
                 math.degrees(start_pose[2])))
        with drive:
            runner.run_square(
                segment_cm=args.segment_cm, linear_mm_s=args.linear_mm_s,
                turn_mrad_s=args.turn_mrad_s, segments=args.segments,
            )
    except Exception as exc:  # noqa: BLE001
        print("ERROR: %s: %s" % (type(exc).__name__, exc))
        status = 1
    finally:
        try:
            drive.close()
        finally:
            reader.stop()
            if reader.is_alive():
                reader.join(timeout=2.0)
            print("stopped")

    if runner.trace:
        out = args.trace or "/tmp/square_trace_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
        try:
            with open(out, "w", encoding="utf-8") as handle:
                handle.write("t_s,x_cm,y_cm,yaw_ccw_deg,vx_mm_s,vz_mrad_s,phase\n")
                for row in runner.trace:
                    handle.write("%.3f,%.2f,%.2f,%.3f,%.1f,%.1f,%s\n" % row)
            print("trace written to %s (%d rows)" % (out, len(runner.trace)))
        except OSError as exc:
            print("could not write trace: %s" % exc)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
