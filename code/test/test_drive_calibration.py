"""Hardware-free checks for the supervised open-loop calibration pulse."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.pose_fusion import FusedPoseEstimate, PoseFusionState
from config.v2_loader import load_v2_config
from core.types import Pose2D, Twist2D
from tools.drive_calibration import PulseRunner, PoseSample, accepted_fused_sample


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeDrive:
    def __init__(self):
        self.last_limited_twist = Twist2D(0.0, 0.0)
        self.commands = []
        self.stop_count = 0

    def command(self, twist):
        self.last_limited_twist = twist
        self.commands.append(twist)

    def stop(self):
        self.last_limited_twist = Twist2D(0.0, 0.0)
        self.stop_count += 1


class FakeReader:
    def __init__(self, clock, drive, *, scale=1.0, lost_at=None):
        self.clock, self.drive, self.scale, self.lost_at = clock, drive, scale, lost_at
        self.last_t = clock()
        self.x = self.yaw = 0.0

    def latest(self):
        now = self.clock()
        if self.lost_at is not None and now >= self.lost_at:
            return None
        dt = now - self.last_t
        self.x += self.drive.last_limited_twist.linear_x_m_s * dt * self.scale
        self.yaw += self.drive.last_limited_twist.angular_z_rad_s * dt * self.scale
        self.last_t = now
        return PoseSample(now, self.x, 0.0, self.yaw, 0.01, 0.02, 1.0, ("t265", "slam", "fused"))


class DriveCalibrationTests(unittest.TestCase):
    def test_fused_sample_requires_fresh_accepted_anchor(self):
        config = load_v2_config()
        valid = FusedPoseEstimate(
            Pose2D(0.0, 0.0, 0.0, 1.0), PoseFusionState.OK, 0.01,
            ("t265", "slam", "fused"), 0.01, 0.02, None, None, True,
            anchor_initialized=True, t265_confidence=1.0,
        )
        self.assertIsNotNone(accepted_fused_sample(valid, config, 1.0))
        self.assertIsNone(accepted_fused_sample(replace(valid, state=PoseFusionState.D500_DEGRADED), config, 1.0))
        self.assertIsNone(accepted_fused_sample(replace(valid, anchor_initialized=False), config, 1.0))
        self.assertIsNone(accepted_fused_sample(replace(valid, d500_age_s=0.6), config, 1.0))
        self.assertIsNone(accepted_fused_sample(replace(valid, source_flags=("t265",)), config, 1.0))

    def test_linear_pulse_exposes_scale_error_and_stops(self):
        clock, drive = FakeClock(), FakeDrive()
        reader = FakeReader(clock, drive, scale=0.9)
        result = PulseRunner(drive, reader, clock=clock, sleep=clock.sleep).run("distance", 0.10, 0.05)
        self.assertGreaterEqual(drive.stop_count, 1)
        self.assertAlmostEqual(result.predicted, 0.10, delta=0.001)
        self.assertAlmostEqual(result.measured, 0.09, delta=0.002)
        self.assertTrue(drive.commands)

    def test_rotation_accumulates_yaw_and_stops(self):
        clock, drive = FakeClock(), FakeDrive()
        reader = FakeReader(clock, drive, scale=0.95)
        result = PulseRunner(drive, reader, clock=clock, sleep=clock.sleep).run("rotate", math.pi / 2.0, 0.20)
        self.assertAlmostEqual(result.predicted, math.pi / 2.0, delta=0.001)
        self.assertAlmostEqual(result.measured, 0.95 * math.pi / 2.0, delta=0.002)
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_reverse_pulse_keeps_signed_prediction_and_measurement(self):
        clock, drive = FakeClock(), FakeDrive()
        reader = FakeReader(clock, drive, scale=1.1)
        result = PulseRunner(drive, reader, clock=clock, sleep=clock.sleep).run("distance", -0.10, 0.05)
        self.assertAlmostEqual(result.predicted, -0.10, delta=0.001)
        self.assertAlmostEqual(result.measured, -0.11, delta=0.002)
        self.assertTrue(all(command.linear_x_m_s < 0.0 for command in drive.commands))
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_pose_loss_aborts_and_stops(self):
        clock, drive = FakeClock(), FakeDrive()
        reader = FakeReader(clock, drive, lost_at=0.25)
        with self.assertRaisesRegex(RuntimeError, "fresh fused"):
            PulseRunner(drive, reader, clock=clock, sleep=clock.sleep).run("distance", 0.10, 0.05)
        self.assertGreaterEqual(drive.stop_count, 1)


if __name__ == "__main__":
    unittest.main()
