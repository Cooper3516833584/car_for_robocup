#!/usr/bin/env python3
"""EXPERIMENTAL: square-path run for the differential car. Does NOT yet close.

STATUS: kept as a record of the test harness and of the measurements it produced,
not as a working acceptance tool.

Measured on the lab field, 4 x (forward 100 cm + 90 deg CCW), slow speed
(Vx=220 mm/s, Vz=400 mrad/s):

  run 1 (no ramping)         closure error 55.7 cm, heading error +43.4 deg
  run 2 (ramped + settle)    closure error 58.2 cm, heading error +47.3 deg

Ramping and waiting for the measured pose to stop improved the per-segment
figures (forward overshoot 4.7-5.7% -> 1.5-2.6%) but did not improve closure at
all, so per-segment accuracy is not the limiting factor.  Waypoint analysis of
the trace shows the actual mechanism: heading keeps increasing at each corner
*after* the last non-zero command.

    wp6  -> wp7  : +79.5 deg while commanded, then +8.0 deg more after the stop
    wp11 -> wp12 : +5.8 deg more after the stop
    wp38 -> wp39 : +10.4 deg more after the stop
    wp45 -> wp46 : +78.6 deg while commanded, then more after the stop

Four corners accumulate about +44 deg of extra heading, and each straight leg
also comes up 10-12 cm short, so the vehicle finishes near (55, -17) cm and
+47 deg instead of returning to the origin and initial heading.  Remaining work
is to compensate the post-stop coast (release earlier by the measured coast,
which the partially applied ``lead_deg`` / ``lead_cm`` were for) and to correct
the left/right wheel mismatch that skews each straight leg by 7-11 deg.

Sign conventions (verified, not assumed):

  * ``angular_z_rad_s > 0`` commands a *left* turn: ``left = linear - w*track/2``,
    ``right = linear + w*track/2`` (``differential_kinematics``), so the right
    wheel runs faster and the car turns counter-clockwise.
  * the radar's own ``yaw_cw_deg`` is clockwise-positive, which is the opposite
    sign.

Safety: bounded segment duration, a stop frame on every exit path including
exceptions and SIGINT, and a hard cap on commanded speed and rotation rate.

Usage:
    python3 tools/square_run.py --device /dev/ttyACM0 \
        --segment-cm 100 --segments 4 --linear-mm-s 220 --turn-mrad-s 400
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

try:
    import termios
except ModuleNotFoundError:
    termios = None

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
SEND_HZ = 50.0
FIRMWARE_MAX_ANGULAR_MRAD_S = 1047


def build_frame(linear_mm_s: int = 0, angular_mrad_s: int = 0) -> bytes:
    frame = bytearray(11)
    frame[0] = FRAME_HEADER
    frame[1] = 0x7C
    frame[2] = 0x7C
    linear = linear_mm_s & 0xFFFF
    frame[3] = (linear >> 8) & 0xFF
    frame[4] = linear & 0xFF
    frame[5] = 0x5A
    frame[6] = 0x5A
    angular = angular_mrad_s & 0xFFFF
    frame[7] = (angular >> 8) & 0xFF
    frame[8] = angular & 0xFF
    checksum = 0
    for byte in frame[:9]:
        checksum ^= byte
    frame[9] = checksum
    frame[10] = FRAME_TAIL
    return bytes(frame)


STOP_FRAME = build_frame(0, 0)


def open_serial(device: str) -> int:
    if termios is None:
        raise RuntimeError("this tool must run on Linux")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = termios.IGNPAR
        settings[1] = 0
        settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except BaseException:
        os.close(fd)
        raise


class T265PoseReader(threading.Thread):
    """base_link pose in the car's own frame, used to close each segment."""

    def __init__(self, mount) -> None:
        super().__init__(daemon=True)
        self.mount = mount
        self.samples: list[tuple[float, tuple[float, float, float]]] = []
        self.error: str | None = None
        self._ready = threading.Event()
        self._stop = threading.Event()

    def run(self) -> None:
        try:
            from components.t265_driver import T265RawPose
            from components.t265_pose_adapter import T265PoseAdapter
            import pyrealsense2 as rs

            adapter = T265PoseAdapter(self.mount, min_tracker_confidence=2,
                                      max_age_s=10.0)
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.pose)
            pipeline.start(config)
            self._ready.set()
            try:
                while not self._stop.is_set():
                    frames = pipeline.wait_for_frames(1000)
                    pose = frames.get_pose_frame()
                    if not pose:
                        continue
                    data = pose.get_pose_data()
                    now = time.monotonic()
                    raw = T265RawPose(
                        translation_xyz=(float(data.translation.x),
                                         float(data.translation.y),
                                         float(data.translation.z)),
                        quaternion_xyzw=(float(data.rotation.x),
                                         float(data.rotation.y),
                                         float(data.rotation.z),
                                         float(data.rotation.w)),
                        velocity_xyz=None, angular_velocity_xyz=None,
                        tracker_confidence=int(data.tracker_confidence),
                        mapper_confidence=None,
                        device_timestamp_ms=float(pose.get_timestamp()),
                        received_monotonic_s=now,
                    )
                    update = adapter.adapt(raw, now_s=now)
                    if update.pose is not None:
                        self.samples.append(
                            (now, (update.pose.x_m, update.pose.y_m,
                                   update.pose.yaw_rad))
                        )
            finally:
                pipeline.stop()
        except Exception as exc:  # noqa: BLE001
            self.error = "%s: %s" % (type(exc).__name__, exc)
            self._ready.set()

    def wait_ready(self, timeout: float = 45.0) -> bool:
        return self._ready.wait(timeout)

    def wait_pose(self, timeout: float = 15.0):
        """Block until at least one usable base_link pose has arrived.

        ``wait_ready`` only means the pipeline opened.  The adapter can still be
        withholding poses (confidence or freshness), and starting the run before
        any pose exists means there is no reference to close the first segment
        against.
        """

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error:
                return None
            if self.samples:
                return self.samples[-1][1]
            time.sleep(0.05)
        return None

    def latest(self):
        return self.samples[-1][1] if self.samples else None

    def stop(self) -> None:
        self._stop.set()


def unwrap_delta(start_rad: float, end_rad: float) -> float:
    delta = end_rad - start_rad
    while delta > math.pi:
        delta -= 2 * math.pi
    while delta < -math.pi:
        delta += 2 * math.pi
    return delta


class Runner:
    def __init__(self, fd: int, reader: T265PoseReader, send_hz: float) -> None:
        self.fd = fd
        self.reader = reader
        self.period = 1.0 / send_hz
        self.aborted = False
        self.t0 = time.monotonic()
        self.ramp_cm = 12.0
        self.ramp_deg = 12.0
        self.trace: list[tuple[float, float, float, float, float, float]] = []

    def _send(self, frame: bytes) -> None:
        os.write(self.fd, frame)

    def _settle(self, *, mode: str, start_pose, timeout: float = 3.0):
        """Stop, then wait until the measured pose actually stops changing.

        A fixed sleep was wrong: the base still coasts after the first zero
        frame, so the next segment started from a moving vehicle and every
        segment overshot by roughly 5%.  This waits for the measurement itself
        to settle instead of guessing a duration.
        """

        deadline = time.monotonic() + timeout
        last = None
        still_since = None
        while time.monotonic() < deadline:
            self._send(STOP_FRAME)
            time.sleep(0.04)
            pose = self.reader.latest()
            if pose is None:
                continue
            if last is not None:
                moved = math.hypot(pose[0] - last[0], pose[1] - last[1]) * 100.0
                turned = abs(math.degrees(unwrap_delta(last[2], pose[2])))
                if moved < 0.3 and turned < 0.3:
                    if still_since is None:
                        still_since = time.monotonic()
                    elif time.monotonic() - still_since >= 0.25:
                        break
                else:
                    still_since = None
            last = pose
        for _ in range(4):
            self._send(STOP_FRAME)
            time.sleep(0.03)
        return self.reader.latest()

    def hold(self, linear_mm_s: int, angular_mrad_s: int, seconds: float,
             *, start_pose, mode: str, target: float) -> tuple[bool, float, float]:
        """Drive one segment, ramping down before the target, then stopping.

        The base has no acceleration limiting of its own, so releasing a full
        speed command at the target let the vehicle coast past it.  The command is
        ramped down over the last ``RAMP_CM`` / ``RAMP_DEG`` so the target is
        approached at low speed.
        """

        deadline = time.monotonic() + seconds
        next_send = time.monotonic()
        trace_tick = 0.0
        ramp_cm = getattr(self, "ramp_cm", 12.0)
        ramp_deg = getattr(self, "ramp_deg", 12.0)
        min_scale = 0.35

        while True:
            now = time.monotonic()
            if self.aborted or now >= deadline:
                break
            pose = self.reader.latest()
            scale = 1.0
            done = 0.0
            if pose is not None:
                if mode == "distance":
                    done = math.hypot(pose[0] - start_pose[0],
                                      pose[1] - start_pose[1]) * 100.0
                    if target - done < ramp_cm:
                        scale = max(min_scale, (target - done) / ramp_cm)
                else:
                    done = abs(math.degrees(
                        unwrap_delta(start_pose[2], pose[2])))
                    if target - done < ramp_deg:
                        scale = max(min_scale, (target - done) / ramp_deg)

            if now >= next_send:
                self._send(build_frame(
                    int(linear_mm_s * scale), int(angular_mrad_s * scale)
                ))
                next_send = now + self.period

            if pose is not None:
                if now - trace_tick >= 0.1:
                    trace_tick = now
                    self.trace.append((now - self.t0, pose[0] * 100.0,
                                       pose[1] * 100.0,
                                       math.degrees(pose[2]),
                                       int(linear_mm_s * scale),
                                       int(angular_mrad_s * scale)))
                if done >= target:
                    break
            time.sleep(0.001)

        pose = self._settle(mode=mode, start_pose=start_pose)
        if pose is None:
            return False, 0.0, 0.0
        if mode == "distance":
            achieved = math.hypot(pose[0] - start_pose[0],
                                  pose[1] - start_pose[1]) * 100.0
        else:
            achieved = abs(math.degrees(unwrap_delta(start_pose[2], pose[2])))
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
            budget = segment_cm / max(1e-6, linear_mm_s / 1000.0) * 1.6 + 2.0
            ok, achieved, _ = self.hold(
                linear_mm_s, 0, budget,
                start_pose=before, mode="distance", target=segment_cm,
            )
            err = (achieved - segment_cm) / segment_cm * 100.0
            print("%-4d %-9s %9.1fcm %9.1fcm %7.1f%%"
                  % (index + 1, "forward", segment_cm, achieved, err))
            if not ok:
                print("  ABORT: no pose during forward segment")
                return

            before = self.reader.latest()
            budget = 90.0 / max(1e-6, turn_mrad_s / 1000.0) * 1.6 + 2.0
            ok, achieved, _ = self.hold(
                0, turn_mrad_s, budget,
                start_pose=before, mode="angle", target=90.0,
            )
            err = (achieved - 90.0) / 90.0 * 100.0
            print("%-4d %-9s %9.1fdeg %9.1fdeg %7.1f%%"
                  % (index + 1, "turn CCW", 90.0, achieved, err))
            if not ok:
                print("  ABORT: no pose during turn segment")
                return

        final = self.reader.latest()
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
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--segment-cm", type=float, default=100.0)
    parser.add_argument("--segments", type=int, default=4)
    parser.add_argument("--linear-mm-s", type=int, default=220)
    parser.add_argument("--turn-mrad-s", type=int, default=400)
    parser.add_argument("--send-hz", type=float, default=SEND_HZ)
    parser.add_argument("--mount-x-m", type=float, default=-0.258)
    parser.add_argument("--mount-z-m", type=float, default=0.076)
    parser.add_argument("--mount-yaw-deg", type=float, default=180.0)
    args = parser.parse_args()

    if abs(args.turn_mrad_s) > FIRMWARE_MAX_ANGULAR_MRAD_S:
        parser.error("turn rate exceeds the firmware clamp of +/-1047 mrad/s")
    if args.linear_mm_s <= 0 or args.linear_mm_s > 600:
        parser.error("linear speed must be in (0, 600] mm/s")
    if args.segments != 4:
        parser.error("this acceptance run is defined as exactly 4 segments")

    from config.v2_models import SensorMount3DConfig

    mount = SensorMount3DConfig(
        x_m=args.mount_x_m, y_m=0.0, z_m=args.mount_z_m,
        roll_rad=0.0, pitch_rad=0.0, yaw_rad=math.radians(args.mount_yaw_deg),
    )
    reader = T265PoseReader(mount)
    reader.start()
    if not reader.wait_ready() or reader.error:
        print("FAIL: T265 %s" % (reader.error or "not ready"))
        return 2
    start_pose = reader.wait_pose(15.0)
    if start_pose is None:
        print("FAIL: no usable T265 pose within 15 s (%s)"
              % (reader.error or "stream opened but produced no pose"))
        reader.stop()
        return 3
    print("T265 ready, start x=%.1f cm y=%.1f cm yaw=%.1f deg"
          % (start_pose[0] * 100.0, start_pose[1] * 100.0,
             math.degrees(start_pose[2])))

    fd = open_serial(args.device)
    runner = Runner(fd, reader, args.send_hz)

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
        runner.run_square(
            segment_cm=args.segment_cm, linear_mm_s=args.linear_mm_s,
            turn_mrad_s=args.turn_mrad_s, segments=args.segments,
        )
    except Exception as exc:  # noqa: BLE001
        print("ERROR: %s: %s" % (type(exc).__name__, exc))
        status = 1
    finally:
        for _ in range(20):
            try:
                os.write(fd, STOP_FRAME)
            except OSError:
                break
            time.sleep(0.03)
        os.close(fd)
        reader.stop()
        print("stopped")

    if runner.trace:
        out = "/tmp/square_trace.csv"
        try:
            with open(out, "w", encoding="utf-8") as handle:
                handle.write("t_s,x_cm,y_cm,yaw_ccw_deg,vx_mm_s,vz_mrad_s\n")
                for row in runner.trace:
                    handle.write("%.3f,%.2f,%.2f,%.3f,%d,%d\n" % row)
            print("trace written to %s (%d rows)" % (out, len(runner.trace)))
        except OSError as exc:
            print("could not write trace: %s" % exc)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
