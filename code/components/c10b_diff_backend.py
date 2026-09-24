"""SI differential-drive interface over the existing C10B motor driver."""

from __future__ import annotations

from enum import Enum
import math
import time
from typing import Protocol

from .rear_motor import RearMotorDriver, UnsupportedWheelCommand
from core.types import WheelSpeeds


class DriveBackend(Protocol):
    def start(self) -> None: ...

    def command_wheel_speeds(self, wheel_speeds: WheelSpeeds) -> None: ...

    def stop(self) -> None: ...

    def close(self) -> None: ...


class C10BProtocolMode(str, Enum):
    ACKERMANN_FIRMWARE_COMPAT = "ackermann_firmware_compat"
    DIFFERENTIAL_VX_VZ = "differential_vx_vz"


class UnsupportedFirmwareMotion(RuntimeError):
    """The installed C10B firmware cannot represent the requested motion."""


def _positive(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


class C10BDifferentialBackend:
    """Translate canonical m/s wheel targets to the preserved C10B driver.

    Compatibility mode checks the old firmware's minimum radius before
    forwarding. Differential mode skips only that legacy radius check; the
    frame encoder, command range checks, periodic sender, and watchdog remain
    owned by ``RearMotorDriver``.
    """

    def __init__(
        self,
        rear_driver: RearMotorDriver,
        *,
        protocol_mode: C10BProtocolMode | str,
        firmware_track_width_m: float,
        firmware_min_turn_radius_m: float,
        allow_in_place_rotation: bool,
        max_wheel_speed_m_s: float,
    ) -> None:
        self.rear_driver = rear_driver
        self.protocol_mode = C10BProtocolMode(protocol_mode)
        self.firmware_track_width_m = _positive("firmware_track_width_m", firmware_track_width_m)
        self.firmware_min_turn_radius_m = _positive(
            "firmware_min_turn_radius_m", firmware_min_turn_radius_m
        )
        self.allow_in_place_rotation = bool(allow_in_place_rotation)
        self.max_wheel_speed_m_s = _positive("max_wheel_speed_m_s", max_wheel_speed_m_s)
        driver_track_m = getattr(rear_driver, "track_width_mm", None)
        if driver_track_m is not None and not math.isclose(
            float(driver_track_m) / 1000.0, self.firmware_track_width_m, rel_tol=1e-6
        ):
            raise ValueError("firmware_track_width_m does not match RearMotorDriver")

    def command_wheel_speeds(self, wheel_speeds: WheelSpeeds) -> None:
        left = float(wheel_speeds.left_m_s)
        right = float(wheel_speeds.right_m_s)
        if not math.isfinite(left) or not math.isfinite(right):
            raise ValueError("wheel speeds must be finite")
        if abs(left) > self.max_wheel_speed_m_s or abs(right) > self.max_wheel_speed_m_s:
            raise ValueError(
                f"wheel speed exceeds configured limit +/-{self.max_wheel_speed_m_s:g} m/s"
            )

        linear = (left + right) / 2.0
        angular = (right - left) / self.firmware_track_width_m
        is_pure_rotation = math.isclose(linear, 0.0, abs_tol=1e-12) and not math.isclose(
            angular, 0.0, abs_tol=1e-12
        )
        enforce_min_radius = self.protocol_mode is C10BProtocolMode.ACKERMANN_FIRMWARE_COMPAT
        if is_pure_rotation and not self.allow_in_place_rotation:
            raise UnsupportedFirmwareMotion("in-place rotation is disabled by configuration")
        if enforce_min_radius and not math.isclose(linear, 0.0, abs_tol=1e-12) and not math.isclose(
            angular, 0.0, abs_tol=1e-12
        ):
            radius_m = abs(linear / angular)
            if radius_m + 1e-12 < self.firmware_min_turn_radius_m:
                raise UnsupportedFirmwareMotion(
                    f"requested turn radius {radius_m:.3f} m is below legacy firmware "
                    f"minimum {self.firmware_min_turn_radius_m:.3f} m"
                )

        try:
            self.rear_driver.set_wheels(
                left * 1000.0,
                right * 1000.0,
                enforce_min_turn_radius=not (
                    self.protocol_mode is C10BProtocolMode.DIFFERENTIAL_VX_VZ
                ),
            )
        except UnsupportedWheelCommand as exc:
            raise UnsupportedFirmwareMotion(str(exc)) from exc

    def start(self) -> None:
        self.rear_driver.start()

    def stop(self) -> None:
        self.rear_driver.stop()

    def close(self) -> None:
        self.rear_driver.close()


class FakeDriveBackend:
    """In-memory backend for dry-runs and Windows tests."""

    def __init__(
        self,
        *,
        max_wheel_speed_m_s: float | None = None,
        clock=time.monotonic,
        fail_with: Exception | None = None,
    ) -> None:
        self.max_wheel_speed_m_s = (
            None if max_wheel_speed_m_s is None else _positive("max_wheel_speed_m_s", max_wheel_speed_m_s)
        )
        self.clock = clock
        self.fail_with = fail_with
        self.commands: list[WheelSpeeds] = []
        self.command_times_s: list[float] = []
        self.stopped = False
        self.stop_count = 0
        self.close_count = 0

    def command_wheel_speeds(self, wheel_speeds: WheelSpeeds) -> None:
        if self.fail_with is not None:
            raise self.fail_with
        if self.max_wheel_speed_m_s is not None and max(
            abs(wheel_speeds.left_m_s), abs(wheel_speeds.right_m_s)
        ) > self.max_wheel_speed_m_s:
            raise ValueError("wheel speed exceeds fake backend limit")
        self.commands.append(wheel_speeds)
        self.command_times_s.append(float(self.clock()))
        self.stopped = False

    def start(self) -> None:
        self.stopped = True

    def stop(self) -> None:
        self.stop_count += 1
        self.stopped = True

    def close(self) -> None:
        self.close_count += 1
        self.stopped = True
