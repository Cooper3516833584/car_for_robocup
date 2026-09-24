from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend, UnsupportedFirmwareMotion
from components.differential_drive import DifferentialDrive
from components.differential_kinematics import DifferentialGeometry
from core import Twist2D


class FakeClock:
    now_s = 0.0

    def __call__(self) -> float:
        return self.now_s


class RaisingBackend(FakeDriveBackend):
    def __init__(self, failure: Exception):
        super().__init__()
        self.failure = failure

    def command_wheel_speeds(self, wheel_speeds) -> None:
        raise self.failure


class SafetyWatchdogTests(unittest.TestCase):
    def make_drive(self, backend=None, *, start=True):
        clock = FakeClock()
        chosen_backend = backend or FakeDriveBackend(clock=clock)
        drive = DifferentialDrive(
            chosen_backend,
            geometry=DifferentialGeometry(0.2),
            max_wheel_speed_m_s=1.0,
            max_linear_speed_m_s=0.5,
            max_angular_speed_rad_s=1.0,
            max_linear_accel_m_s2=10.0,
            max_angular_accel_rad_s2=10.0,
            command_timeout_s=0.25,
            clock=clock,
            hardware_lock_path=None,
        )
        if start:
            drive.start()
        return drive, chosen_backend, clock

    def test_stop_is_immediate_and_repeated_stop_is_safe(self) -> None:
        drive, backend, clock = self.make_drive()
        clock.now_s = 0.1
        drive.command(Twist2D(0.4, 0.0), now_s=clock.now_s)

        drive.stop()
        drive.stop()

        self.assertTrue(backend.stopped)
        self.assertEqual(backend.stop_count, 2)
        drive.close()

    def test_backend_error_stops_and_reraises(self) -> None:
        failure = RuntimeError("backend offline")
        drive, backend, clock = self.make_drive(RaisingBackend(failure))
        clock.now_s = 0.1

        with self.assertRaisesRegex(RuntimeError, "backend offline"):
            drive.command(Twist2D(0.1, 0.0), now_s=clock.now_s)
        self.assertTrue(backend.stopped)
        self.assertEqual(backend.stop_count, 1)
        drive.close()

    def test_unsupported_firmware_motion_stops_and_reraises(self) -> None:
        drive, backend, clock = self.make_drive(
            RaisingBackend(UnsupportedFirmwareMotion("radius rejected"))
        )
        clock.now_s = 0.1

        with self.assertRaisesRegex(UnsupportedFirmwareMotion, "radius rejected"):
            drive.command(Twist2D(0.1, 1.0), now_s=clock.now_s)
        self.assertTrue(backend.stopped)
        drive.close()

    def test_context_exit_stops_and_closes_after_exception(self) -> None:
        drive, backend, clock = self.make_drive(start=False)
        with self.assertRaisesRegex(RuntimeError, "mission failed"):
            with drive:
                clock.now_s = 0.1
                drive.command(Twist2D(0.1, 0.0), now_s=clock.now_s)
                raise RuntimeError("mission failed")
        self.assertTrue(backend.stopped)
        self.assertEqual(backend.close_count, 1)
        self.assertFalse(drive.is_running)

    def test_watchdog_stops_at_timeout_without_sleeping(self) -> None:
        drive, backend, clock = self.make_drive()
        clock.now_s = 0.1
        drive.command(Twist2D(0.1, 0.0), now_s=clock.now_s)
        clock.now_s = 0.36

        self.assertTrue(drive.check_watchdog(now_s=clock.now_s))
        self.assertTrue(backend.stopped)
        self.assertFalse(drive.check_watchdog(now_s=clock.now_s))
        drive.close()

    def test_nan_and_infinity_are_rejected(self) -> None:
        drive, backend, clock = self.make_drive()

        with self.assertRaisesRegex(ValueError, "linear_x_m_s"):
            drive.command(Twist2D(float("nan"), 0.0), now_s=clock.now_s)
        with self.assertRaisesRegex(ValueError, "angular_z_rad_s"):
            drive.command(Twist2D(0.0, float("inf")), now_s=clock.now_s)
        self.assertEqual(backend.commands, [])
        drive.close()

    def test_monotonic_time_cannot_go_backwards(self) -> None:
        drive, _, clock = self.make_drive()
        clock.now_s = 0.1
        drive.command(Twist2D(0.1, 0.0), now_s=clock.now_s)
        with self.assertRaisesRegex(ValueError, "move backwards"):
            drive.command(Twist2D(0.1, 0.0), now_s=0.05)
        drive.close()


if __name__ == "__main__":
    unittest.main()
