#!/usr/bin/env python3
"""Retired motor entry; use closed_loop_motion.py. Read-only helpers remain available."""

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
    print("Direct serial motion retired; use closed_loop_motion.py and analyze its sensor logs")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
