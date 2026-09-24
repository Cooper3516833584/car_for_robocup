from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core import (
    Pose2D,
    compose_pose2d,
    inverse_pose2d,
    normalize_angle_rad,
    transform_point2d,
    transform_pose2d,
)


class FrameMathTests(unittest.TestCase):
    def assertPoseAlmostEqual(self, actual: Pose2D, expected: Pose2D) -> None:
        self.assertAlmostEqual(actual.x_m, expected.x_m, places=12)
        self.assertAlmostEqual(actual.y_m, expected.y_m, places=12)
        self.assertAlmostEqual(actual.yaw_rad, expected.yaw_rad, places=12)
        self.assertAlmostEqual(actual.timestamp_s, expected.timestamp_s, places=12)

    def test_normalize_angle_pi_boundaries(self) -> None:
        self.assertEqual(normalize_angle_rad(0.0), 0.0)
        self.assertEqual(normalize_angle_rad(math.pi), -math.pi)
        self.assertEqual(normalize_angle_rad(-math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle_rad(3.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle_rad(-3.0 * math.pi), -math.pi)

    def test_ninety_degree_rotation_and_translation(self) -> None:
        parent_T_child = Pose2D(1.0, 2.0, math.pi / 2.0, 1.0)
        child_pose = Pose2D(1.0, 0.0, 0.0, 2.0)

        result = transform_pose2d(parent_T_child, child_pose)
        self.assertPoseAlmostEqual(result, Pose2D(1.0, 3.0, math.pi / 2.0, 2.0))
        self.assertEqual(transform_point2d(parent_T_child, (1.0, 0.0)), (1.0, 3.0))

    def test_pose_composed_with_inverse_is_identity(self) -> None:
        pose = Pose2D(2.5, -1.25, -0.7, 7.0)
        identity = compose_pose2d(pose, inverse_pose2d(pose))

        self.assertPoseAlmostEqual(identity, Pose2D(0.0, 0.0, 0.0, 7.0))

    def test_composition_order_is_parent_then_child(self) -> None:
        parent_T_child = Pose2D(10.0, 0.0, math.pi / 2.0, 1.0)
        child_T_sensor = Pose2D(2.0, 0.0, 0.0, 2.0)
        parent_T_sensor = compose_pose2d(parent_T_child, child_T_sensor)
        reversed_order = compose_pose2d(child_T_sensor, parent_T_child)

        self.assertPoseAlmostEqual(parent_T_sensor, Pose2D(10.0, 2.0, math.pi / 2.0, 2.0))
        self.assertNotAlmostEqual(parent_T_sensor.x_m, reversed_order.x_m)
        self.assertNotAlmostEqual(parent_T_sensor.y_m, reversed_order.y_m)


if __name__ == "__main__":
    unittest.main()
