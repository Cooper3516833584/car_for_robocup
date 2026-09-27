#!/usr/bin/env python3
"""D500 <-> T265 yaw consistency through absolute wall geometry.

Why not scan-to-scan ICP
------------------------
With this field, a point-to-point nearest-neighbour ICP, and no motion prior or
deskew, an in-place rotation often leaves the along-wall direction weakly
constrained: the matcher falls into a local minimum that trades a smaller
rotation for a slide along the wall.  Measured on this car that appears as a
large spurious translation tripping ``RadarOdometry.max_step_cm`` (8-30%
acceptance, roughly independent of rotation rate, and worse at lower speed).

That is a property of this configuration, not a proof that no implementation
could ever resolve rotation.  The absolute boundary path below does not depend
on ICP at all, so it can be evaluated even while ICP rejects.

Coordinates
-----------
Two different ``Pose2D`` types exist in this repository:

* ``core.types.Pose2D``   -- metres, CCW-positive radians
* ``radar_driver.Pose2D`` -- centimetres, CW-positive degrees

Mixing them raises ``AttributeError: 'Pose2D' object has no attribute
'x_cm'``.  Every wall-frame quantity here stays in the radar type, and the
conversion lives in ``components/radar_pose_adapter.py``.

The T265 adapter's local origin is wherever its pose stream started, which is
NOT automatically the field origin or the field centre.  The car's starting pose
in the wall frame is therefore supplied explicitly with ``--start-*`` and the
T265 local delta is composed onto that prior.

Read-only on the radar; the only motor writes are bounded stop-guarded frames.

Usage:
    python3 tools/yaw_wall_crosscheck.py \
        --start-x-m 2.0 --start-y-m 2.0 --start-yaw-deg 0
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import threading
import time

try:
    import termios
except ModuleNotFoundError:
    termios = None

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.radar_driver import (  # noqa: E402
    D500SerialDriver,
    DroneGlobalAlignment,
    Pose2D,
    RadarMount,
    RadarScanAssembler,
    RectangularWallReference,
    WallLineConfig,
    WallLineLocalizer,
)
from components.radar_pose_adapter import (  # noqa: E402
    compose_pose_with_delta_prior,
)
from components.t265_pose_adapter import T265PoseAdapter  # noqa: E402
from config.v2_models import SensorMount3DConfig  # noqa: E402

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
FIRMWARE_MAX_ANGULAR_MRAD_S = 1047
SEND_HZ = 50.0


def build_frame(linear_mm_s: int = 0, angular_mrad_s: int = 0) -> bytes:
    frame = bytearray(11)
    frame[0] = FRAME_HEADER
    frame[1] = 0x7C
    frame[2] = 0x7C
    lin = linear_mm_s & 0xFFFF
    frame[3] = (lin >> 8) & 0xFF
    frame[4] = lin & 0xFF
    frame[5] = 0x5A
    frame[6] = 0x5A
    ang = angular_mrad_s & 0xFFFF
    frame[7] = (ang >> 8) & 0xFF
    frame[8] = ang & 0xFF
    checksum = 0
    for byte in frame[:9]:
        checksum ^= byte
    frame[9] = checksum
    frame[10] = FRAME_TAIL
    return bytes(frame)


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


class T265Reader(threading.Thread):
    """Streams base_link pose from the adapter, used as the wall predictor."""

    def __init__(self, adapter: T265PoseAdapter) -> None:
        super().__init__(daemon=True)
        self.adapter = adapter
        self.poses: list[tuple[float, float, float, float]] = []
        self.error: str | None = None
        self._started = threading.Event()
        self._stop = threading.Event()

    def run(self) -> None:
        try:
            import pyrealsense2 as rs
            from components.t265_driver import T265RawPose

            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.pose)
            pipeline.start(config)
            self._started.set()
            try:
                while not self._stop.is_set():
                    frames = pipeline.wait_for_frames(1000)
                    pose = frames.get_pose_frame()
                    if not pose:
                        continue
                    d = pose.get_pose_data()
                    t = time.monotonic()
                    raw = T265RawPose(
                        translation_xyz=(float(d.translation.x),
                                         float(d.translation.y),
                                         float(d.translation.z)),
                        quaternion_xyzw=(float(d.rotation.x), float(d.rotation.y),
                                         float(d.rotation.z), float(d.rotation.w)),
                        velocity_xyz=None, angular_velocity_xyz=None,
                        tracker_confidence=int(d.tracker_confidence),
                        mapper_confidence=None,
                        device_timestamp_ms=float(pose.get_timestamp()),
                        received_monotonic_s=t,
                    )
                    upd = self.adapter.adapt(raw, now_s=t)
                    if upd.pose is not None:
                        self.poses.append(
                            (t, upd.pose.x_m, upd.pose.y_m, upd.pose.yaw_rad)
                        )
                    if len(self.poses) > 100000:
                        del self.poses[:50000]
            finally:
                pipeline.stop()
        except Exception as exc:  # noqa: BLE001
            self.error = "%s: %s" % (type(exc).__name__, exc)
            self._started.set()

    def latest(self):
        return self.poses[-1] if self.poses else None

    def wait_started(self, timeout: float = 45.0) -> bool:
        return self._started.wait(timeout)

    def stop(self) -> None:
        self._stop.set()


def unwrap_total(values):
    total = 0.0
    for a, b in zip(values, values[1:]):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total += d
    return total


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--radar-port", default="/dev/ttyS6")
    parser.add_argument("--linear-mm-s", type=int, default=0)
    parser.add_argument("--angular-mrad-s", type=int, default=250)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--field-width-m", type=float, default=5.0)
    parser.add_argument("--field-height-m", type=float, default=5.0)
    parser.add_argument("--start-x-m", type=float, default=2.5)
    parser.add_argument("--start-y-m", type=float, default=2.5)
    parser.add_argument("--start-yaw-deg", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if abs(args.angular_mrad_s) > FIRMWARE_MAX_ANGULAR_MRAD_S:
        parser.error("rate exceeds the firmware clamp of +/-1047 mrad/s")

    width_cm = args.field_width_m * 100.0
    height_cm = args.field_height_m * 100.0

    print("=== D500 <-> T265 yaw consistency (absolute wall geometry) ===")
    print("  command Vx=%+d mm/s Vz=%+d mrad/s for %.1f s"
          % (args.linear_mm_s, args.angular_mrad_s, args.seconds))
    print("  field   : x in [0, %.0f] cm, y in [0, %.0f] cm" % (width_cm, height_cm))
    print("  start   : (%.2f, %.2f) m, yaw %.1f deg  [explicit prior, not assumed]"
          % (args.start_x_m, args.start_y_m, args.start_yaw_deg))
    if args.dry_run:
        print("DRY RUN")
        return 0

    reference = RectangularWallReference(
        wall_to_global=DroneGlobalAlignment(0.0, 0.0, 0.0),
        back_wall_x_cm=0.0,
        right_wall_y_cm=0.0,
        front_wall_x_cm=width_cm,
        left_wall_y_cm=height_cm,
    )
    localizer = WallLineLocalizer(reference, mount=RadarMount(),
                                  config=WallLineConfig())
    start_prior = Pose2D(args.start_x_m * 100.0, args.start_y_m * 100.0,
                         -args.start_yaw_deg)

    adapter = T265PoseAdapter(
        SensorMount3DConfig(x_m=-0.258, y_m=0.0, z_m=0.076,
                            roll_rad=0.0, pitch_rad=0.0, yaw_rad=math.pi),
        min_tracker_confidence=2, max_age_s=10.0,
    )
    reader = T265Reader(adapter)
    reader.start()
    if not reader.wait_started() or reader.error:
        print("FAIL: T265 %s" % (reader.error or "not ready"))
        return 2
    print("  T265 ready")

    time.sleep(0.5)
    origin = reader.latest()
    if origin is None:
        print("FAIL: no T265 pose at start")
        reader.stop()
        return 2

    rows = []          # (t, predicted, observation)
    diag = {"scans": 0, "observed": 0, "no_obs": 0, "two_edge_yaw": 0,
            "one_edge_yaw": 0, "reasons": {}}
    assembler = RadarScanAssembler()

    def on_packet(packet):
        for scan in assembler.feed(packet):
            diag["scans"] += 1
            latest = reader.latest()
            if latest is None:
                diag["reasons"]["no_t265_pose"] = \
                    diag["reasons"].get("no_t265_pose", 0) + 1
                return
            _, px, py, pyaw = latest
            predicted = compose_pose_with_delta_prior(
                start_prior, px - origin[1], py - origin[2], pyaw - origin[3]
            )
            try:
                obs = localizer.observe(scan, predicted)
            except Exception as exc:  # noqa: BLE001
                key = "%s: %s" % (type(exc).__name__, exc)
                diag["reasons"][key] = diag["reasons"].get(key, 0) + 1
                diag["no_obs"] += 1
                continue
            if obs.x_cm is None and obs.y_cm is None:
                diag["no_obs"] += 1
                diag["reasons"]["no_axis_observed"] = \
                    diag["reasons"].get("no_axis_observed", 0) + 1
                return
            diag["observed"] += 1
            if obs.yaw_cw_deg is not None:
                axes = int(obs.back_wall_points > 0) + int(obs.right_wall_points > 0)
                if axes >= 2:
                    diag["two_edge_yaw"] += 1
                else:
                    diag["one_edge_yaw"] += 1
            rows.append((time.monotonic(), predicted, obs))

    driver = D500SerialDriver(on_packet=on_packet, port=args.radar_port)
    driver.start()
    if not driver.wait_connected(5.0):
        print("FAIL: D500 not connected")
        reader.stop()
        return 2
    print("  D500 ready on %s" % args.radar_port)

    fd = open_serial(args.device)
    frame = build_frame(args.linear_mm_s, args.angular_mrad_s)
    stop = build_frame(0, 0)
    try:
        time.sleep(1.0)
        print("  driving %.1f s ..." % args.seconds)
        t0 = time.monotonic()
        os.write(fd, frame)
        period = 1.0 / SEND_HZ
        nxt = time.monotonic()
        deadline = t0 + args.seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= nxt:
                os.write(fd, frame)
                nxt = now + period
            time.sleep(0.002)
        t1 = time.monotonic()
        for _ in range(10):
            os.write(fd, stop)
            time.sleep(0.03)
        time.sleep(0.5)
    finally:
        for _ in range(8):
            try:
                os.write(fd, stop)
            except OSError:
                pass
            time.sleep(0.03)
        os.close(fd)
        reader.stop()
        time.sleep(0.3)
        driver.close()
        print("  stopped")

    print()
    print("=== diagnostics ===")
    print("  scans %d, boundary observed %d, no observation %d"
          % (diag["scans"], diag["observed"], diag["no_obs"]))
    print("  yaw from 2 non-parallel edges: %d" % diag["two_edge_yaw"])
    print("  yaw from 1 edge:              %d" % diag["one_edge_yaw"])
    for reason, count in sorted(diag["reasons"].items(), key=lambda kv: -kv[1])[:6]:
        print("    %5d x %s" % (count, reason[:110]))

    t265 = [p for p in reader.poses if t0 <= p[0] <= t1]
    wins = [r for r in rows if t0 <= r[0] <= t1]
    if len(t265) < 20 or len(wins) < 5:
        print()
        print("FAIL: not enough samples (t265=%d wall=%d)" % (len(t265), len(wins)))
        return 4

    w0, w1 = wins[0][0], wins[-1][0]
    seg = [p for p in t265 if w0 <= p[0] <= w1]
    t265_total = unwrap_total([p[3] for p in seg])
    wall_ccw_deg = [-r[2].yaw_cw_deg for r in wins if r[2].yaw_cw_deg is not None]
    wall_total = math.radians(
        unwrap_total([math.radians(v) for v in wall_ccw_deg])
    )

    print()
    print("=== comparison over %.1f s ===" % (w1 - w0))
    print("  T265 accumulated yaw      = %+.2f deg" % math.degrees(t265_total))
    print("  D500 wall accumulated yaw = %+.2f deg" % math.degrees(wall_total))
    if abs(t265_total) > math.radians(15):
        ratio = wall_total / t265_total
        print("  ratio D500/T265 = %.4f (error %.1f%%)"
              % (ratio, abs(ratio - 1.0) * 100.0))
    else:
        print("  T265 rotation too small for a meaningful ratio")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
