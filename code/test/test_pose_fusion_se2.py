from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.pose_fusion import PoseFusion
from config.v2_loader import load_v2_config
from core.frames import compose_pose2d
from core.types import Pose2D, PoseQuality


def quality(source: str = "d500") -> PoseQuality:
    return PoseQuality(source, True, False, 1.0, 1.0)


class PoseFusionSE2Tests(unittest.TestCase):
    def test_far_from_origin_pure_yaw_correction_does_not_shift_position(self) -> None:
        config = replace(load_v2_config().fusion, position_correction_gain=0.1, yaw_correction_gain=0.1)
        fusion = PoseFusion(config)
        fusion.update_t265(Pose2D(5.0, 0.0, 0.0, 1.0), quality("t265"))
        fusion.update_d500(Pose2D(5.0, 0.0, 0.0, 1.0), quality())
        fusion.update_d500(Pose2D(5.0, 0.0, 0.2, 1.1), quality())

        result = fusion.estimate(1.11).pose
        self.assertAlmostEqual(result.x_m, 5.0, delta=1e-6)
        self.assertLess(abs(result.y_m), 1e-6)
        self.assertAlmostEqual(result.yaw_rad, 0.02, delta=1e-6)

    def test_corrected_map_pose_matches_recomputed_map_to_odom(self) -> None:
        config = replace(
            load_v2_config().fusion,
            position_correction_gain=0.25,
            yaw_correction_gain=0.5,
            max_single_position_correction_m=1.0,
        )
        fusion = PoseFusion(config)
        odom_pose = Pose2D(5.0, 2.0, 0.3, 1.0)
        fusion.update_t265(odom_pose, quality("t265"))
        fusion.update_d500(Pose2D(5.0, 2.0, 0.3, 1.0), quality())
        fusion.update_d500(Pose2D(5.4, 1.8, 0.5, 1.1), quality())

        desired = Pose2D(5.1, 1.95, 0.4, 1.1)
        recomposed = compose_pose2d(fusion.map_T_t265_odom, odom_pose)
        self.assertAlmostEqual(recomposed.x_m, desired.x_m, places=9)
        self.assertAlmostEqual(recomposed.y_m, desired.y_m, places=9)
        self.assertAlmostEqual(recomposed.yaw_rad, desired.yaw_rad, places=9)

    def test_repeated_absolute_corrections_converge_without_nan_or_wrap_jumps(self) -> None:
        config = replace(
            load_v2_config().fusion,
            position_correction_gain=0.2,
            yaw_correction_gain=0.2,
            max_single_position_correction_m=1.0,
            max_single_yaw_correction_rad=0.5,
        )
        fusion = PoseFusion(config)
        target = Pose2D(5.2, 0.1, -math.pi + 0.02, 1.0)
        initial = Pose2D(5.0, 0.0, math.pi - 0.02, 1.0)
        fusion.update_t265(initial, quality("t265"))
        fusion.update_d500(initial, quality())
        for index in range(1, 101):
            stamp = 1.0 + index * 0.01
            odom = Pose2D(5.0, 0.0, math.pi - 0.02, stamp)
            absolute = Pose2D(target.x_m, target.y_m, target.yaw_rad, stamp)
            fusion.update_t265(odom, quality("t265"))
            fusion.update_d500(absolute, quality())
            estimate = fusion.estimate(stamp)
            self.assertTrue(all(math.isfinite(value) for value in (estimate.pose.x_m, estimate.pose.y_m, estimate.pose.yaw_rad)))
        final = fusion.estimate(2.0).pose
        self.assertAlmostEqual(final.x_m, target.x_m, delta=0.01)
        self.assertAlmostEqual(final.y_m, target.y_m, delta=0.01)
        self.assertLess(abs((final.yaw_rad - target.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi), 0.01)


if __name__ == "__main__":
    unittest.main()
