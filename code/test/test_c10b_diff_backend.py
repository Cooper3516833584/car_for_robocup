from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import (
    C10BDifferentialBackend,
    C10BProtocolMode,
    FakeDriveBackend,
    UnsupportedFirmwareMotion,
)
from components.rear_motor import RearMotorDriver, UnsupportedWheelCommand, wheel_speeds_to_chassis
from core import WheelSpeeds


class C10BDifferentialBackendTests(unittest.TestCase):
    def make_backend(
        self,
        mode: C10BProtocolMode = C10BProtocolMode.ACKERMANN_FIRMWARE_COMPAT,
        *,
        allow_in_place: bool = False,
    ):
        rear = Mock(spec=RearMotorDriver)
        rear.track_width_mm = 164.0
        backend = C10BDifferentialBackend(
            rear,
            protocol_mode=mode,
            firmware_track_width_m=0.164,
            firmware_min_turn_radius_m=0.350,
            allow_in_place_rotation=allow_in_place,
            max_wheel_speed_m_s=1.0,
        )
        return rear, backend

    def test_si_wheel_speed_is_converted_to_mm_per_second(self) -> None:
        rear, backend = self.make_backend()

        backend.command_wheel_speeds(WheelSpeeds(1.0, 1.0))

        rear.set_wheels.assert_called_once_with(1000.0, 1000.0, enforce_min_turn_radius=True)

    def test_equal_signed_wheel_speeds_are_allowed_in_compat_mode(self) -> None:
        rear, backend = self.make_backend()

        backend.command_wheel_speeds(WheelSpeeds(0.2, 0.2))

        rear.set_wheels.assert_called_once()

    def test_in_place_rotation_requires_explicit_enable(self) -> None:
        rear, backend = self.make_backend(allow_in_place=True)

        backend.command_wheel_speeds(WheelSpeeds(-0.2, 0.2))

        rear.set_wheels.assert_called_once()

    def test_small_radius_continuous_turn_is_rejected_by_compat_mode(self) -> None:
        rear, backend = self.make_backend()

        with self.assertRaises(UnsupportedFirmwareMotion):
            backend.command_wheel_speeds(WheelSpeeds(0.1, 0.3))
        rear.set_wheels.assert_not_called()

    def test_verified_differential_mode_skips_legacy_radius_gate(self) -> None:
        rear, backend = self.make_backend(C10BProtocolMode.DIFFERENTIAL_VX_VZ)

        backend.command_wheel_speeds(WheelSpeeds(0.1, 0.3))

        rear.set_wheels.assert_called_once_with(100.0, 300.0, enforce_min_turn_radius=False)

    def test_low_level_radius_gate_only_disables_when_explicit(self) -> None:
        with self.assertRaises(UnsupportedWheelCommand):
            wheel_speeds_to_chassis(100.0, 300.0)
        command = wheel_speeds_to_chassis(
            100.0, 300.0, enforce_min_turn_radius=False
        )
        self.assertEqual(command.linear_mm_s, 200)

    def test_stop_and_close_are_forwarded(self) -> None:
        rear, backend = self.make_backend()

        backend.stop()
        backend.close()

        rear.stop.assert_called_once_with()
        rear.close.assert_called_once_with()

    def test_fake_backend_records_command_time_and_stops(self) -> None:
        times = iter((10.0, 10.1))
        fake = FakeDriveBackend(clock=lambda: next(times), max_wheel_speed_m_s=0.5)
        expected = WheelSpeeds(0.2, 0.3)

        fake.command_wheel_speeds(expected)
        fake.stop()

        self.assertEqual(fake.commands, [expected])
        self.assertEqual(fake.command_times_s, [10.0])
        self.assertEqual(fake.stop_count, 1)
        self.assertTrue(fake.stopped)
        with self.assertRaises(ValueError):
            fake.command_wheel_speeds(WheelSpeeds(0.6, 0.0))

    def test_fake_backend_can_raise_configured_failure(self) -> None:
        failure = RuntimeError("backend offline")
        fake = FakeDriveBackend(fail_with=failure)

        with self.assertRaisesRegex(RuntimeError, "backend offline"):
            fake.command_wheel_speeds(WheelSpeeds(0.1, 0.1))


if __name__ == "__main__":
    unittest.main()
