from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import math
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

        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.02), quality("t265"))
        self.fusion.update_d500(Pose2D(4.0, 3.0, 0.5, 1.02), quality("d500"))
        estimate = self.fusion.estimate(1.03)
        self.assertIs(estimate.state, PoseFusionState.OK)
        self.assertAlmostEqual(estimate.pose.x_m, 4.0)
        self.assertAlmostEqual(estimate.pose.y_m, 3.0)
        self.assertAlmostEqual(estimate.pose.yaw_rad, 0.5)

    def test_d500_without_aligned_t265_cannot_anchor(self) -> None:
        self.fusion.update_d500(Pose2D(2.0, 1.0, -0.2, 1.0), quality("d500"))
        estimate = self.fusion.estimate(1.01)
        self.assertIs(estimate.state, PoseFusionState.LOST)
        self.assertIsNone(estimate.pose)
        self.assertFalse(self.fusion.global_anchor_established)

        self.fusion.update_t265(Pose2D(5.0, -2.0, 0.4, 1.0), quality("t265"))
        estimate = self.fusion.estimate(1.06)
        self.assertIs(estimate.state, PoseFusionState.UNANCHORED)
        self.assertIsNone(estimate.pose)

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
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.1), quality("t265"))
        self.fusion.update_d500(Pose2D(0.2, 0.0, 0.1, 1.1), quality("d500"))

        estimate = self.fusion.estimate(1.11)

        self.assertAlmostEqual(estimate.pose.x_m, 0.07)
        self.assertAlmostEqual(estimate.pose.yaw_rad, 0.035)
        self.assertTrue(estimate.d500_accepted)
        self.assertAlmostEqual(estimate.last_d500_innovation_m, 0.2)

    def test_large_d500_outlier_is_rejected_without_pose_jump(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        before = self.fusion.estimate(1.0).pose
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.1), quality("t265"))
        self.fusion.update_d500(Pose2D(5.0, 0.0, 0.0, 1.1), quality("d500"))

        estimate = self.fusion.estimate(1.11)

        self.assertFalse(estimate.d500_accepted)
        self.assertEqual(estimate.rejection_reason, "position_innovation_gate")
        self.assertEqual(estimate.pose.x_m, before.x_m)

    def test_t265_stale_does_not_reuse_old_absolute_as_propagated_fallback(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(1.0, 2.0, 0.3, 1.0), quality("d500"))

        estimate = self.fusion.estimate(1.2)
        self.assertIs(estimate.state, PoseFusionState.LOST)
        self.assertIsNone(estimate.pose)

    def test_d500_stale_uses_t265_dead_reckoning_after_anchor(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        for index in range(1, 6):
            stamp = 1.0 + index * 0.1
            self.fusion.update_t265(Pose2D(0.2, 0.0, 0.0, stamp), quality("t265"))

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
        self.fusion.update_t265(Pose2D(0.0, 0.0, 3.13, 1.1), quality("t265"))
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

    def test_history_interpolates_yaw_across_pi_boundary(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, math.radians(179), 1.0), quality("t265"))
        self.fusion.update_t265(Pose2D(0.1, 0.0, math.radians(-179), 1.1), quality("t265"))
        aligned = self.fusion._sample_t265(1.05)
        self.assertIsNotNone(aligned)
        self.assertAlmostEqual(aligned.pose.x_m, 0.05)
        self.assertAlmostEqual(abs(aligned.pose.yaw_rad), math.pi, places=5)

    def test_delayed_d500_uses_measurement_time_history(self) -> None:
        immediate = PoseFusion(self.config)
        delayed = PoseFusion(self.config)
        for stamp in (1.0, 1.05, 1.1):
            pose = Pose2D(stamp - 1.0, 0.0, 0.0, stamp)
            immediate.update_t265(pose, quality("t265"))
            delayed.update_t265(pose, quality("t265"))
        measurement = Pose2D(0.075, 0.0, 0.0, 1.075)
        immediate.update_d500_absolute(measurement, quality("d500"))
        for stamp in (1.15, 1.2):
            delayed.update_t265(Pose2D(stamp - 1.0, 0.0, 0.0, stamp), quality("t265"))
        delayed.update_d500_absolute(measurement, quality("d500"))
        self.assertAlmostEqual(immediate.map_T_t265_odom.x_m, delayed.map_T_t265_odom.x_m)
        self.assertAlmostEqual(immediate.map_T_t265_odom.yaw_rad, delayed.map_T_t265_odom.yaw_rad)
        self.assertAlmostEqual(delayed.estimate(1.2).t265_time_alignment_ms, 25.0)

    def test_d500_requires_bracketing_t265_history(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_t265(Pose2D(0.1, 0.0, 0.0, 1.1), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(2.0, 0.0, 0.0, 1.2), quality("d500"))
        self.assertFalse(self.fusion.global_anchor_established)
        self.assertEqual(self.fusion.estimate(1.2).rejection_reason, "t265_time_alignment_unavailable")

    def test_t265_relocalization_rebases_without_map_pose_jump(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(3.0, 2.0, 0.0, 1.0), quality("d500"))
        before = self.fusion.estimate(1.0).pose
        self.fusion.update_t265(Pose2D(2.0, 0.0, math.radians(70), 1.1), quality("t265"))
        after = self.fusion.estimate(1.1).pose
        self.assertAlmostEqual(after.x_m, before.x_m)
        self.assertAlmostEqual(after.y_m, before.y_m)
        self.assertAlmostEqual(after.yaw_rad, before.yaw_rad)

    def test_tracking_gap_and_relocalization_without_fallback_stays_lost(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(3.0, 2.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_t265(Pose2D(0.05, 0.0, 0.0, 1.05), quality("t265"))
        low = PoseQuality("t265", False, True, 1.0 / 3.0, 1.0 / 3.0)
        self.fusion.update_t265(None, low)
        self.fusion.update_t265(None, PoseQuality("t265", False, True))
        self.fusion.update_t265(None, PoseQuality("t265", False, True))
        self.fusion.update_t265(
            Pose2D(2.0, -1.0, math.radians(70), 1.6), quality("t265")
        )
        estimate = self.fusion.estimate(1.6)
        self.assertIsNone(estimate.pose)
        self.assertTrue(estimate.t265_continuity_broken)
        self.assertEqual(len(self.fusion._t265_history), 0)

    def test_normal_twenty_hz_motion_does_not_break_continuity(self) -> None:
        for step in range(21):
            stamp = 1.0 + step * 0.05
            self.fusion.update_t265(
                Pose2D(step * 0.03, 0.0, math.radians(step * 3), stamp),
                quality("t265"),
            )
        estimate = self.fusion.estimate(2.0)
        self.assertFalse(estimate.t265_continuity_broken)
        self.assertIs(estimate.state, PoseFusionState.UNANCHORED)

    def test_local_only_d500_cannot_change_anchor(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(1.0, 2.0, 0.0, 1.0), quality("d500"))
        anchor = self.fusion.map_T_t265_odom
        self.fusion.update_d500_absolute(
            Pose2D(9.0, 8.0, 1.0, 1.0), PoseQuality("d500", True, True)
        )
        self.assertEqual(self.fusion.map_T_t265_odom, anchor)

    def test_t265_dead_reckons_through_two_second_d500_dropout_and_recovers_smoothly(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        before_dropout_end = None
        for step in range(1, 41):
            stamp = 1.0 + step * 0.05
            local_x = step * 0.05
            self.fusion.update_t265(Pose2D(local_x, 0.0, 0.0, stamp), quality("t265"))
            before_dropout_end = self.fusion.estimate(stamp).pose
        self.assertIs(self.fusion.estimate(3.0).state, PoseFusionState.D500_DEGRADED)
        self.fusion.update_d500_absolute(Pose2D(1.95, 0.0, 0.0, 3.0), quality("d500"))
        after_recovery = self.fusion.estimate(3.0)
        self.assertTrue(after_recovery.d500_accepted)
        self.assertLess(abs(after_recovery.pose.x_m - before_dropout_end.x_m), 0.05)
        self.assertIs(after_recovery.state, PoseFusionState.OK)

    def test_fresh_global_fallback_is_used_when_absolute_and_t265_are_stale(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(1.0, 2.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_d500_global_fallback(
            Pose2D(1.3, 2.1, 0.05, 4.0), quality("d500"), map_alignment_valid=True
        )
        estimate = self.fusion.estimate(4.1)
        self.assertIs(estimate.state, PoseFusionState.T265_DEGRADED)
        self.assertEqual(estimate.source_flags, ("d500_global_fallback",))
        self.assertEqual(estimate.pose, Pose2D(1.3, 2.1, 0.05, 4.0))

    def test_global_fallback_never_changes_anchor_and_never_preempts_healthy_t265(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(1.0, 0.0, 0.0, 1.0), quality("d500"))
        anchor = self.fusion.map_T_t265_odom
        self.fusion.update_d500_global_fallback(
            Pose2D(9.0, 8.0, 1.0, 1.1), quality("d500"), map_alignment_valid=True
        )
        self.assertEqual(self.fusion.map_T_t265_odom, anchor)
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.1), quality("t265"))
        estimate = self.fusion.estimate(1.1)
        self.assertEqual(estimate.pose.x_m, 1.0)
        self.assertNotIn("d500_global_fallback", estimate.source_flags)

    def test_t265_recovery_uses_fresh_fallback_to_rebuild_map_anchor(self) -> None:
        self.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), quality("t265"))
        self.fusion.update_d500_absolute(Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"))
        self.fusion.update_d500_global_fallback(
            Pose2D(0.3, 0.0, 0.0, 1.6), quality("d500"), map_alignment_valid=True
        )
        self.fusion.update_t265(None, PoseQuality("t265", False, True, 1.0 / 3.0, 1.0 / 3.0))
        self.fusion.update_t265(
            Pose2D(2.0, -1.0, math.radians(70), 1.6), quality("t265")
        )
        estimate = self.fusion.estimate(1.6)
        self.assertFalse(estimate.t265_continuity_broken)
        self.assertAlmostEqual(estimate.pose.x_m, 0.3)
        self.assertAlmostEqual(estimate.pose.y_m, 0.0)
        self.assertAlmostEqual(estimate.pose.yaw_rad, 0.0)

    def test_fusion_confidence_threshold_comes_from_fusion_config(self) -> None:
        config = replace(self.config, t265_min_tracker_confidence=3)
        fusion = PoseFusion(config)
        confidence_two = PoseQuality("t265", True, False, 2.0 / 3.0, 2.0 / 3.0)
        fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), confidence_two)
        fusion.update_d500_absolute(Pose2D(1.0, 0.0, 0.0, 1.0), quality("d500"))
        self.assertFalse(fusion.global_anchor_established)
        self.assertEqual(fusion.estimate(1.0).rejection_reason, "t265_time_alignment_unavailable")

        confidence_three = PoseQuality("t265", True, False, 1.0, 1.0)
        recovered = PoseFusion(config)
        recovered.update_t265(Pose2D(0.0, 0.0, 0.0, 1.1), confidence_three)
        recovered.update_d500_absolute(Pose2D(1.0, 0.0, 0.0, 1.1), quality("d500"))
        self.assertTrue(recovered.global_anchor_established)

    def test_disabled_t265_can_localize_from_aligned_d500_global_stream(self) -> None:
        self.fusion.update_d500_global_fallback(
            Pose2D(4.0, 1.0, 0.2, 2.0), quality("d500"), map_alignment_valid=True
        )
        estimate = self.fusion.estimate(2.1)
        self.assertIs(estimate.state, PoseFusionState.T265_DEGRADED)
        self.assertIsNotNone(estimate.pose)
        self.assertEqual(estimate.pose.x_m, 4.0)

    def test_identity_or_untrusted_map_alignment_cannot_enable_fallback(self) -> None:
        self.fusion.update_d500_global_fallback(
            Pose2D(0.0, 0.0, 0.0, 1.0), quality("d500"), map_alignment_valid=False
        )
        estimate = self.fusion.estimate(1.1)
        self.assertIsNone(estimate.pose)
        self.assertIs(estimate.state, PoseFusionState.INITIALIZING)

    def test_v2_factory_builds_fusion_from_fusion_config(self) -> None:
        from config.v2_factory import build_pose_fusion

        fusion = build_pose_fusion(load_v2_config())
        self.assertIsInstance(fusion, PoseFusion)


if __name__ == "__main__":
    unittest.main()
