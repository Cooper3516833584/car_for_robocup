from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.pose_fusion import PoseFusion, PoseFusionState
from config.v2_loader import load_v2_config
from core import Pose2D, PoseQuality


def quality(source: str, age: float = 0.0) -> PoseQuality:
    return PoseQuality(source, True, False, 1.0, 1.0, age)


class PoseFusionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config().fusion
        self.fusion = PoseFusion(self.config)

    def test_t265_first_is_unanchored_until_d500_arrives(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        unanchored = self.fusion.estimate(1.01)
        self.assertIs(unanchored.state, PoseFusionState.UNANCHORED)
        self.assertIsNone(unanchored.pose)

        self.fusion.update_d500(Pose2D(4.0, 3.0, 0.5, 1.02), quality("d500"))
        estimate = self.fusion.estimate(1.03)
        self.assertIs(estimate.state, PoseFusionState.OK)
        self.assertAlmostEqual(estimate.pose.x_m, 4.0)
        self.assertAlmostEqual(estimate.pose.y_m, 3.0)
        self.assertAlmostEqual(estimate.pose.yaw_rad, 0.5)

    def test_d500_first_provides_degraded_absolute_pose_then_anchors(self) -> None:
        self.fusion.update_d500(Pose2D(2.0, 1.0, -0.2, 1.0), quality("d500"))
        estimate = self.fusion.estimate(1.01)
        self.assertIs(estimate.state, PoseFusionState.T265_DEGRADED)
        self.assertEqual(estimate.pose, Pose2D(2.0, 1.0, -0.2, 1.0))

        self.fusion.update_t265(Pose2D(5.0, -2.0, 0.4, 1.05), quality("t265"))
        estimate = self.fusion.estimate(1.06)
        self.assertIs(estimate.state, PoseFusionState.OK)
        self.assertAlmostEqual(estimate.pose.x_m, 2.0)
        self.assertAlmostEqual(estimate.pose.y_m, 1.0)

    def test_t265_forward_motion_is_continuous_in_map(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(3.0, 2.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_t265(Pose2D(0.2, 0.0, 0.0, 1.1), quality("t265"))

        estimate = self.fusion.estimate(1.11)

        self.assertAlmostEqual(estimate.pose.x_m, 3.2)
        self.assertAlmostEqual(estimate.pose.y_m, 2.0)

    def test_d500_small_correction_is_applied_by_gain_and_single_step_cap(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_d500(Pose2D(0.6, 0.0, 0.2, 1.1), quality("d500"))

        estimate = self.fusion.estimate(1.11)

        self.assertAlmostEqual(estimate.pose.x_m, self.config.max_single_position_correction_m)
        self.assertAlmostEqual(estimate.pose.yaw_rad, 0.02)
        self.assertTrue(estimate.d500_accepted)
        self.assertAlmostEqual(estimate.last_d500_innovation_m, 0.6)

    def test_large_d500_outlier_is_rejected_without_pose_jump(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        before = self.fusion.estimate(1.0).pose
        self.fusion.update_d500(Pose2D(5.0, 0.0, 0.0, 1.1), quality("d500"))

        estimate = self.fusion.estimate(1.11)

        self.assertFalse(estimate.d500_accepted)
        self.assertEqual(estimate.rejection_reason, "position_innovation_gate")
        self.assertEqual(estimate.pose.x_m, before.x_m)

    def test_t265_stale_falls_back_to_recent_d500(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(1.0, 2.0, 0.3, 1.0), quality("d500"))

        estimate = self.fusion.estimate(1.2)
        self.assertIs(estimate.state, PoseFusionState.T265_DEGRADED)
        self.assertEqual(estimate.pose, Pose2D(1.0, 2.0, 0.3, 1.0))

    def test_d500_stale_uses_t265_dead_reckoning_after_anchor(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_t265(Pose2D(0.2, 0.0, 0.0, 1.5), quality("t265"))

        estimate = self.fusion.estimate(1.51)
        self.assertIs(estimate.state, PoseFusionState.D500_DEGRADED)
        self.assertAlmostEqual(estimate.pose.x_m, 0.2)

    def test_both_stale_means_lost(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))

        estimate = self.fusion.estimate(2.0)
        self.assertIs(estimate.state, PoseFusionState.LOST)
        self.assertIsNone(estimate.pose)

    def test_yaw_innovation_wraps_across_pi_boundary(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 3.13, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 3.13, 1.0), quality("d500"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, -3.13, 1.1), quality("d500"))

        estimate = self.fusion.estimate(1.11)
        self.assertTrue(estimate.d500_accepted)
        self.assertLess(abs(estimate.last_d500_innovation_yaw_rad), 0.03)

    def test_reset_requires_a_new_anchor(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(1.0, 0.0, 0.0, 1.0), quality("d500"))
        self.fusion.reset()

        self.assertIs(self.fusion.estimate(1.1).state, PoseFusionState.INITIALIZING)
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.2), quality("t265"))
        self.assertIs(self.fusion.estimate(1.21).state, PoseFusionState.UNANCHORED)

    def test_fusion_accepts_only_base_pose_not_sensor_mount(self) -> None:
        base_pose = Pose2D(1.0, 0.0, 0.0, 1.0)
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(base_pose, quality("d500"))
        self.assertEqual(self.fusion.estimate(1.0).pose, base_pose)

    def test_v2_factory_builds_fusion_from_fusion_config(self) -> None:
        from config.v2_factory import build_pose_fusion

        fusion = build_pose_fusion(load_v2_config())
        self.assertIsInstance(fusion, PoseFusion)


if __name__ == "__main__":
    unittest.main()
