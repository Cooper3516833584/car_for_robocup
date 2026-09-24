from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.differential_drive import DifferentialDrive
from components.differential_kinematics import DifferentialGeometry
from config.v2_factory import build_differential_drive
from config.v2_loader import load_v2_config
from core import Twist2D, WheelSpeeds


class FakeClock:
    def __init__(self, now_s: float = 0.0) -> None:
        self.now_s = now_s

    def __call__(self) -> float:
        return self.now_s


class DifferentialDriveTests(unittest.TestCase):
    def make_drive(self, **overrides):
        clock = FakeClock()
        backend = FakeDriveBackend(max_wheel_speed_m_s=overrides.pop("backend_max", 10.0), clock=clock)
        start = overrides.pop("start", True)
        settings = dict(
            geometry=DifferentialGeometry(0.20),
            max_wheel_speed_m_s=1.0,
            max_linear_speed_m_s=1.0,
            max_angular_speed_rad_s=2.0,
            max_linear_accel_m_s2=100.0,
            max_angular_accel_rad_s2=100.0,
            command_timeout_s=0.25,
            clock=clock,
            hardware_lock_path=None,
        )
        settings.update(overrides)
        drive = DifferentialDrive(backend, **settings)
        if start:
            drive.start()
        return drive, backend, clock

    def test_forward_and_rotation_map_to_expected_wheels(self) -> None:
        drive, backend, clock = self.make_drive()
        clock.now_s = 0.1
        drive.command(Twist2D(0.5, 0.0), now_s=clock.now_s)
        self.assertEqual(backend.commands[-1], WheelSpeeds(0.5, 0.5))

        clock.now_s = 0.2
        drive.command(Twist2D(0.0, 1.0), now_s=clock.now_s)
        self.assertEqual(backend.commands[-1], WheelSpeeds(-0.1, 0.1))
        drive.close()

    def test_linear_and_angular_commands_are_clamped(self) -> None:
        drive, backend, clock = self.make_drive(
            max_linear_speed_m_s=0.4,
            max_angular_speed_rad_s=0.5,
        )
        clock.now_s = 0.1
        drive.command(Twist2D(2.0, 0.0), now_s=clock.now_s)
        self.assertEqual(backend.commands[-1], WheelSpeeds(0.4, 0.4))
        clock.now_s = 0.2
        drive.command(Twist2D(0.0, 2.0), now_s=clock.now_s)
        self.assertAlmostEqual(backend.commands[-1].left_m_s, -0.05)
        self.assertAlmostEqual(backend.commands[-1].right_m_s, 0.05)
        drive.close()

    def test_wheel_saturation_scales_proportionally(self) -> None:
        drive, backend, clock = self.make_drive(
            max_wheel_speed_m_s=0.5,
            max_linear_speed_m_s=2.0,
            max_angular_speed_rad_s=10.0,
        )
        clock.now_s = 0.1
        drive.command(Twist2D(0.75, 2.5), now_s=clock.now_s)
        self.assertAlmostEqual(backend.commands[-1].left_m_s, 0.25)
        self.assertAlmostEqual(backend.commands[-1].right_m_s, 0.5)
        drive.close()

    def test_acceleration_limiter_uses_elapsed_time(self) -> None:
        drive, backend, clock = self.make_drive(
            max_linear_accel_m_s2=0.5,
            command_timeout_s=3.0,
        )
        clock.now_s = 0.2
        drive.command(Twist2D(1.0, 0.0), now_s=clock.now_s)
        self.assertAlmostEqual(backend.commands[-1].left_m_s, 0.1)
        clock.now_s = 2.0
        drive.command(Twist2D(1.0, 0.0), now_s=clock.now_s)
        self.assertAlmostEqual(backend.commands[-1].left_m_s, 0.35)
        drive.close()

    def test_factory_fake_mode_uses_only_fake_backend(self) -> None:
        from components.c10b_diff_backend import FakeDriveBackend as FakeBackendType

        drive = build_differential_drive(load_v2_config(), fake=True)
        self.assertIsInstance(drive.backend, FakeBackendType)
        with drive:
            drive.forward(0.05)
            self.assertEqual(len(drive.backend.commands), 1)


if __name__ == "__main__":
    unittest.main()
