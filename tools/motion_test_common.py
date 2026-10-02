"""Shared, supervised motion-test helpers; importing this module opens no hardware."""

from __future__ import annotations

import math
import threading
import time


class T265PoseReader(threading.Thread):
    """Keep the latest quality-gated, mount-corrected ``base_link`` pose."""

    def __init__(self, mount, *, serial: str, max_age_s: float,
                 min_tracker_confidence: int) -> None:
        super().__init__(daemon=True)
        self.mount = mount
        self.serial = serial
        self.max_age_s = max_age_s
        self.min_tracker_confidence = min_tracker_confidence
        self.error: str | None = None
        self._ready = threading.Event()
        self._stop_event = threading.Event()
        self._sample_lock = threading.Lock()
        self._sample = None

    def run(self) -> None:
        try:
            from components.t265_driver import RealSenseT265PoseSource
            from components.t265_pose_adapter import T265PoseAdapter

            adapter = T265PoseAdapter(
                self.mount, min_tracker_confidence=self.min_tracker_confidence,
                max_age_s=self.max_age_s,
            )
            source = RealSenseT265PoseSource(self.serial, timeout_ms=250)
            source.start()
            self._ready.set()
            try:
                while not self._stop_event.is_set():
                    raw = source.read()
                    if raw is None:
                        continue
                    update = adapter.adapt(raw, now_s=time.monotonic())
                    with self._sample_lock:
                        self._sample = (
                            (raw.received_monotonic_s,
                             (update.pose.x_m, update.pose.y_m, update.pose.yaw_rad),
                             (-raw.translation_xyz[2], -raw.translation_xyz[0]))
                            if update.pose is not None else None
                        )
            finally:
                source.stop()
        except Exception as exc:  # noqa: BLE001 - report worker failure to controller
            self.error = "%s: %s" % (type(exc).__name__, exc)
            self._ready.set()

    def wait_ready(self, timeout: float = 45.0) -> bool:
        return self._ready.wait(timeout)

    def wait_pose(self, timeout: float = 15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error:
                return None
            pose = self.latest()
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def latest_sample(self):
        """Return time, base pose and raw camera x/y only while still fresh."""
        with self._sample_lock:
            sample = self._sample
        if self.error or sample is None:
            return None
        age_s = time.monotonic() - sample[0]
        return sample if 0.0 <= age_s <= self.max_age_s else None

    def latest(self):
        sample = self.latest_sample()
        return None if sample is None else sample[1]

    def latest_raw_xy(self):
        """Raw camera position in robotics axes, for comparison with base_link."""
        sample = self.latest_sample()
        return None if sample is None or len(sample) < 3 else sample[2]

    def stop(self) -> None:
        self._stop_event.set()


def unwrap_delta(start_rad: float, end_rad: float) -> float:
    """Return the shortest signed yaw change from one adjacent sample to the next."""
    delta = end_rad - start_rad
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def trace_row(drive, pose, phase: str, *, elapsed_s: float):
    command = drive.last_limited_twist
    return (
        elapsed_s, pose[0] * 100.0, pose[1] * 100.0,
        math.degrees(pose[2]), command.linear_x_m_s * 1000.0,
        command.angular_z_rad_s * 1000.0, phase,
    )


def wait_for_settle(drive, reader, *, aborted, record, timeout: float = 3.0,
                    on_pose=None, clock=time.monotonic, sleep=time.sleep):
    """Stop first; return only after fresh ``base_link`` pose stays still."""
    drive.stop()
    pose = reader.latest()
    if pose is None:
        raise RuntimeError("T265 pose lost at stop")
    record(pose, "stop")
    deadline = clock() + timeout
    last = None
    still_since = None
    while clock() < deadline:
        if aborted():
            raise RuntimeError("operator aborted during settle")
        sleep(0.04)
        pose = reader.latest()
        if pose is None:
            raise RuntimeError("T265 pose stale or invalid during settle")
        if on_pose is not None:
            on_pose(pose)
        record(pose, "settle")
        if last is not None:
            moved = math.hypot(pose[0] - last[0], pose[1] - last[1]) * 100.0
            turned = abs(math.degrees(unwrap_delta(last[2], pose[2])))
            if moved < 0.3 and turned < 0.3:
                if still_since is None:
                    still_since = clock()
                elif clock() - still_since >= 0.25:
                    return pose
            else:
                still_since = None
        last = pose
    raise TimeoutError("vehicle did not settle within %.1f seconds" % timeout)
