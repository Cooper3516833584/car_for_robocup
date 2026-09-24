from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import Pose2D as LegacyPose2D
from components.radar_pose_adapter import LegacyRadarPoseSpec, RadarPoseAdapter
from core import Pose2D


class RadarPoseAdapterTests(unittest.TestCase):
    def test_centimetres_and_degrees_convert_to_si(self) -> None:
        pose = RadarPoseAdapter().to_map_base_pose(LegacyPose2D(100.0, 0.0, 0.0), timestamp_s=12.5)

        self.assertEqual(pose, Pose2D(1.0, 0.0, 0.0, 12.5))

    def test_clockwise_and_counter_clockwise_yaw_signs(self) -> None:
        cw = RadarPoseAdapter(LegacyRadarPoseSpec(yaw_positive="cw"))
        ccw = RadarPoseAdapter(LegacyRadarPoseSpec(yaw_positive="ccw"))
        legacy = LegacyPose2D(0.0, 0.0, 90.0)

        self.assertAlmostEqual(cw.to_map_base_pose(legacy, timestamp_s=1.0).yaw_rad, -math.pi / 2.0)
        self.assertAlmostEqual(ccw.to_map_base_pose(legacy, timestamp_s=1.0).yaw_rad, math.pi / 2.0)

    def test_reference_offset_is_rotated_with_legacy_pose(self) -> None:
        adapter = RadarPoseAdapter(
            legacy_T_base_link=Pose2D(0.2, 0.0, 0.0, 0.0)
        )
        legacy = LegacyPose2D(100.0, 50.0, -90.0)

        pose = adapter.to_map_base_pose(legacy, timestamp_s=3.0)

        self.assertAlmostEqual(pose.x_m, 1.0, places=12)
        self.assertAlmostEqual(pose.y_m, 0.7, places=12)
        self.assertAlmostEqual(pose.yaw_rad, math.pi / 2.0, places=12)
        self.assertEqual(pose.timestamp_s, 3.0)

    def test_radian_and_metre_legacy_spec(self) -> None:
        class Pose:
            x_m = 0.4
            y_m = -0.2
            yaw_rad = -0.3

        adapter = RadarPoseAdapter(
            LegacyRadarPoseSpec(distance_unit="m", yaw_unit="rad", yaw_positive="ccw")
        )
        pose = adapter.to_map_base_pose(Pose(), timestamp_s=4.0)
        self.assertAlmostEqual(pose.x_m, 0.4)
        self.assertAlmostEqual(pose.y_m, -0.2)
        self.assertAlmostEqual(pose.yaw_rad, -0.3)
        self.assertEqual(pose.timestamp_s, 4.0)

    def test_non_finite_legacy_pose_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            RadarPoseAdapter().to_map_base_pose(
                LegacyPose2D(float("nan"), 0.0, 0.0), timestamp_s=0.0
            )


if __name__ == "__main__":
    unittest.main()
