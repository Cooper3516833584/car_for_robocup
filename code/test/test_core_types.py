from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core import HealthState, Pose2D, PoseQuality, SensorHealth, Twist2D, WheelSpeeds


class CoreTypeTests(unittest.TestCase):
    def test_motion_types_keep_canonical_si_fields(self) -> None:
        pose = Pose2D(1.0, 2.0, 0.5, 3.0)
        twist = Twist2D(0.4, -0.2)
        wheels = WheelSpeeds(0.3, 0.5)

        self.assertEqual((pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s), (1.0, 2.0, 0.5, 3.0))
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0.4, -0.2))
        self.assertEqual((wheels.left_m_s, wheels.right_m_s), (0.3, 0.5))
        with self.assertRaises(FrozenInstanceError):
            pose.x_m = 4.0  # type: ignore[misc]

    def test_pose_quality_and_sensor_health_are_shared_types(self) -> None:
        quality = PoseQuality("t265", True, False, 0.9, 0.8, 0.02)
        health = SensorHealth("t265", HealthState.OK, 12.5)

        self.assertTrue(quality.valid)
        self.assertEqual(quality.source, health.component)
        self.assertIs(health.state, HealthState.OK)


if __name__ == "__main__":
    unittest.main()
