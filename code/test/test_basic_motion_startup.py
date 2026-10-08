"""Safety and time contract for opt-in post-turn straight startup compensation."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.basic_motion_controller import MotionActionState
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import build_runtime, load_runtime_config


class StraightStartupTests(unittest.TestCase):
    def setUp(self):
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN)
        self.motion = self.runtime.motion
        self.addCleanup(self.runtime.close)

    def start(self, bias=-0.15, duration=1.0):
        self.motion.drive_distance(0.2, lateral_tolerance_m=0.03, heading_yaw_rad=0.0,
                                   startup_yaw_bias_rad_s=bias, startup_duration_s=duration)

    def test_bias_is_mirrored_fades_and_does_not_repeat(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                self.motion.stop()
                self.start(bias=sign * 0.15)
                for stamp, expected in ((10.0, sign * 0.15), (10.5, sign * 0.075),
                                        (11.0, 0.0), (12.0, 0.0)):
                    output = self.motion.step(Pose2D(0, 0, 0, stamp), now_s=stamp)
                    self.assertAlmostEqual(output.command.angular_z_rad_s, expected)
                    self.assertGreater(output.command.linear_x_m_s, 0.0)

    def test_completion_safety_loss_and_cancel_always_zero(self):
        self.start()
        self.motion.step(Pose2D(0, 0, 0, 10), now_s=10)
        done = self.motion.step(Pose2D(0.2, 0, 0, 10.1), now_s=10.1)
        self.assertEqual(done.state, MotionActionState.SUCCEEDED)
        self.assertEqual(done.command.angular_z_rad_s, 0.0)
        self.motion.stop()
        self.start()
        lost = self.motion.step(None, now_s=20)
        self.assertEqual(lost.command.linear_x_m_s, 0.0)
        self.assertEqual(lost.command.angular_z_rad_s, 0.0)
        self.motion.step(Pose2D(0, 0, 0, 20.1), now_s=20.1)
        blocked = self.motion.step(Pose2D(0, 0, math.radians(21), 20.2), now_s=20.2)
        self.assertEqual(blocked.state, MotionActionState.BLOCKED)
        self.assertEqual(blocked.command.angular_z_rad_s, 0.0)
        self.motion.stop()
        cancelled = self.motion.step(Pose2D(0, 0, 0, 20.3), now_s=20.3)
        self.assertEqual(cancelled.command.linear_x_m_s, 0.0)
        self.assertEqual(cancelled.command.angular_z_rad_s, 0.0)

    def test_compensation_never_requests_a_backward_wheel(self):
        self.start(bias=-self.motion.drive.max_angular_speed_rad_s)
        output = self.motion.step(Pose2D(0, 0, math.radians(15), 10), now_s=10)
        self.assertEqual(output.state, MotionActionState.RUNNING)
        v, w = output.command.linear_x_m_s, output.command.angular_z_rad_s
        half_track = self.motion.track_width_m / 2
        self.assertGreaterEqual(v - half_track * w, -1e-12)
        self.assertGreaterEqual(v + half_track * w, -1e-12)

    def test_invalid_or_unbounded_bias_is_rejected_before_motion(self):
        for bias, duration in ((float("nan"), 1), (float("inf"), 1), (0.1, 0), (0.1, 2.01)):
            with self.subTest(bias=bias, duration=duration), self.assertRaises(ValueError):
                self.start(bias, duration)
        with self.assertRaises(ValueError):
            self.motion.drive_distance(-0.2, lateral_tolerance_m=0.03, heading_yaw_rad=0,
                                       startup_yaw_bias_rad_s=0.1, startup_duration_s=1)
        with self.assertRaises(ValueError):
            self.motion.drive_distance(0.2, startup_yaw_bias_rad_s=0.1, startup_duration_s=1)


if __name__ == "__main__":
    unittest.main()
