"""Optional RealSense T265 pose acquisition with lazy SDK loading."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Protocol


class T265UnavailableError(RuntimeError):
    """The RealSense SDK or requested T265 device is unavailable."""


@dataclass(frozen=True, slots=True)
class T265RawPose:
    """One device pose sample, retaining native coordinates and metadata."""

    translation_xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    velocity_xyz: tuple[float, float, float] | None
    angular_velocity_xyz: tuple[float, float, float] | None
    tracker_confidence: int
    mapper_confidence: int | None
    device_timestamp_ms: float | None
    host_monotonic_s: float


class T265PoseSource(Protocol):
    def start(self) -> None: ...

    def read(self) -> T265RawPose | None: ...

    def stop(self) -> None: ...


def _import_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise T265UnavailableError(
            "pyrealsense2 is unavailable; install a compatible librealsense SDK "
            "on the target or use FakeT265PoseSource"
        ) from exc
    return rs


class RealSenseT265PoseSource:
    """Read pose frames from a T265; importing this class needs no SDK."""

    def __init__(self, serial: str = "", *, timeout_ms: int = 1000) -> None:
        if timeout_ms <= 0:
            raise ValueError("timeout_ms must be positive")
        self.serial = str(serial)
        self.timeout_ms = int(timeout_ms)
        self._rs = None
        self._pipeline = None

    def start(self) -> None:
        if self._pipeline is not None:
            raise RuntimeError("T265 source is already running")
        rs = _import_realsense()
        pipeline = rs.pipeline()
        configuration = rs.config()
        if self.serial:
            configuration.enable_device(self.serial)
        configuration.enable_stream(rs.stream.pose)
        try:
            profile = pipeline.start(configuration)
        except Exception as exc:
            raise T265UnavailableError(f"cannot start T265 pose stream: {exc}") from exc
        try:
            device = profile.get_device()
            if device.supports(rs.camera_info.serial_number):
                self.serial = device.get_info(rs.camera_info.serial_number)
        except Exception:
            # Some SDK builds do not expose device metadata through profiles.
            pass
        self._rs = rs
        self._pipeline = pipeline

    def read(self) -> T265RawPose | None:
        if self._pipeline is None:
            raise RuntimeError("T265 source is not running")
        try:
            frames = self._pipeline.wait_for_frames(self.timeout_ms)
        except Exception as exc:
            raise T265UnavailableError(f"T265 pose read failed: {exc}") from exc
        pose_frame = frames.get_pose_frame()
        if not pose_frame:
            return None
        data = pose_frame.get_pose_data()
        velocity = getattr(data, "velocity", None)
        angular_velocity = getattr(data, "angular_velocity", None)
        return T265RawPose(
            translation_xyz=(float(data.translation.x), float(data.translation.y), float(data.translation.z)),
            quaternion_xyzw=(float(data.rotation.x), float(data.rotation.y), float(data.rotation.z), float(data.rotation.w)),
            velocity_xyz=None if velocity is None else (float(velocity.x), float(velocity.y), float(velocity.z)),
            angular_velocity_xyz=None if angular_velocity is None else (
                float(angular_velocity.x), float(angular_velocity.y), float(angular_velocity.z)
            ),
            tracker_confidence=int(data.tracker_confidence),
            mapper_confidence=getattr(data, "mapper_confidence", None),
            device_timestamp_ms=float(pose_frame.get_timestamp()),
            host_monotonic_s=time.monotonic(),
        )

    def stop(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            finally:
                self._pipeline = None
                self._rs = None


class FakeT265PoseSource:
    """Deterministic source for unit tests and short replay sequences."""

    def __init__(
        self,
        samples: list[T265RawPose | None] | tuple[T265RawPose | None, ...] = (),
        *,
        repeat_last: bool = False,
    ) -> None:
        self.samples = tuple(samples)
        self.repeat_last = bool(repeat_last)
        self.read_count = 0
        self.started = False
        self.stopped = False

    def start(self) -> None:
        if self.started:
            raise RuntimeError("fake T265 source is already running")
        self.started = True
        self.stopped = False

    def read(self) -> T265RawPose | None:
        if not self.started:
            raise RuntimeError("fake T265 source is not running")
        if self.read_count < len(self.samples):
            sample = self.samples[self.read_count]
        elif self.repeat_last and self.samples:
            sample = self.samples[-1]
        else:
            sample = None
        self.read_count += 1
        return sample

    def stop(self) -> None:
        self.started = False
        self.stopped = True
