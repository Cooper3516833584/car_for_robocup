from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.pose_fusion import PoseFusion, PoseFusionState
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode, validate_runtime_readiness
from core.types import Pose2D, PoseQuality


GOOD = PoseQuality("t265", True, False, 1.0, 1.0)
WALL = PoseQuality("fixed_wall", True, False, 0.9, 0.9)


class SlamAnchorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fusion = PoseFusion(load_v2_config().fusion, backend="slam_toolbox")

    def add_t265(self, timestamp_s: float, x_m: float = 0.0) -> None:
        self.fusion.update_t265(Pose2D(x_m, 0.0, 0.0, timestamp_s), GOOD)

    def test_high_rate_t265_and_stale_slam(self) -> None:
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(2.0, 3.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.add_t265(1.05, 0.1)
        estimate = self.fusion.estimate(1.05)
        self.assertIs(estimate.state, PoseFusionState.OK)
        self.assertAlmostEqual(estimate.pose.x_m, 2.1)
        self.add_t265(1.10, 0.2)
        self.assertIs(self.fusion.estimate(1.55).state, PoseFusionState.LOST)
        self.add_t265(1.55, 0.2)
        self.assertIs(self.fusion.estimate(1.56).state, PoseFusionState.D500_DEGRADED)

    def test_outlier_does_not_refresh_slam_health(self) -> None:
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(0.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.add_t265(1.1)
        self.fusion.update_slam_anchor(Pose2D(5.0, 0.0, 0.0, 1.1), valid=True, timestamp_s=1.1)
        self.assertEqual(self.fusion.estimate(1.11).rejection_reason, "slam_innovation_gate")
        self.assertAlmostEqual(self.fusion._slam_received_s, 1.0)

    def test_persistent_anchor_disagreement_migrates_instead_of_latching(self) -> None:
        # Regression for the 2026-10-05 on-car latch: slam_toolbox re-solved its
        # pose graph and stepped map->odom by 0.331 m, then stayed there.  The
        # single-sample innovation gate rejected it forever, so fusion sat in
        # d500_degraded for the rest of the run and only a process restart could
        # recover.  A repeatable, coherent disagreement is a real re-solve and
        # must be ramped in instead.
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(0.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.assertIs(self.fusion.estimate(1.0).state, PoseFusionState.OK)

        for stamp in (1.05, 1.10):
            self.add_t265(stamp)
            self.fusion.update_slam_anchor(Pose2D(0.331, 0.0, 0.0, stamp),
                                           valid=True, timestamp_s=stamp)
            self.assertEqual(self.fusion.estimate(stamp).rejection_reason, "slam_innovation_gate")
            self.assertAlmostEqual(self.fusion.map_T_t265_odom.x_m, 0.0)

        # The third coherent sample starts a bounded ramp, never a jump.
        observed: list[float] = []
        for step in range(0, 12):
            stamp = 1.15 + step * 0.05
            self.add_t265(stamp)
            self.fusion.update_slam_anchor(Pose2D(0.331, 0.0, 0.0, stamp),
                                           valid=True, timestamp_s=stamp)
            estimate = self.fusion.estimate(stamp)
            self.assertIs(estimate.state, PoseFusionState.OK)
            observed.append(estimate.pose.x_m)

        self.assertAlmostEqual(observed[-1], 0.331, places=3)
        steps = [abs(b - a) for a, b in zip([0.0, *observed], observed)]
        self.assertLess(max(steps), 0.10)

    def test_isolated_large_anchor_outlier_does_not_start_a_migration(self) -> None:
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(0.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.add_t265(1.05)
        self.fusion.update_slam_anchor(Pose2D(0.297, 0.0, 0.0, 1.05), valid=True, timestamp_s=1.05)
        self.assertEqual(self.fusion.estimate(1.05).rejection_reason, "slam_innovation_gate")

        for stamp in (1.10, 1.15, 1.20):
            self.add_t265(stamp)
            self.fusion.update_slam_anchor(Pose2D(0.01, 0.0, 0.0, stamp),
                                           valid=True, timestamp_s=stamp)

        self.assertIsNone(self.fusion._slam_migration_target)
        self.assertLess(self.fusion.map_T_t265_odom.x_m, 0.02)

    def test_anchor_far_beyond_the_migration_bound_stays_gated(self) -> None:
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(0.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        for stamp in (1.05, 1.10, 1.15, 1.20):
            self.add_t265(stamp)
            self.fusion.update_slam_anchor(Pose2D(3.0, 0.0, 0.0, stamp),
                                           valid=True, timestamp_s=stamp)

        self.assertIsNone(self.fusion._slam_migration_target)
        self.assertAlmostEqual(self.fusion.map_T_t265_odom.x_m, 0.0)
        self.assertEqual(self.fusion.estimate(1.20).rejection_reason, "slam_innovation_gate")

    def test_loop_closure_requires_three_consistent_anchors_and_blends(self) -> None:
        self.add_t265(1.0)
        self.fusion.update_slam_anchor(Pose2D(0.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        for index in range(1, 4):
            stamp = 1.0 + index * 0.05
            self.add_t265(stamp)
            self.fusion.update_slam_anchor(
                Pose2D(0.8, 0.0, math.radians(20), stamp),
                valid=True, timestamp_s=stamp, loop_closure=True,
            )
            if index < 3:
                self.assertAlmostEqual(self.fusion.map_T_t265_odom.x_m, 0.0)
        self.assertAlmostEqual(self.fusion.estimate(1.15).pose.x_m, 0.0)
        for stamp in (1.2, 1.25, 1.3, 1.35, 1.4):
            self.add_t265(stamp)
        self.assertAlmostEqual(self.fusion.estimate(1.4).pose.x_m, 0.4, places=2)
        for stamp in (1.45, 1.5, 1.55, 1.6, 1.65):
            self.add_t265(stamp)
        self.assertAlmostEqual(self.fusion.estimate(1.65).pose.x_m, 0.8, places=2)

    def test_field_alignment_uses_fixed_wall_and_requires_it_when_configured(self) -> None:
        fusion = PoseFusion(load_v2_config().fusion, backend="slam_toolbox", require_field_anchor=True)
        fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), GOOD)
        fusion.update_slam_anchor(Pose2D(1.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.assertIs(fusion.estimate(1.0).state, PoseFusionState.UNANCHORED)
        self.assertTrue(fusion.update_field_wall(Pose2D(4.0, 2.0, 0.0, 1.0), WALL))
        estimate = fusion.estimate(1.0)
        self.assertIs(estimate.state, PoseFusionState.OK)
        self.assertAlmostEqual(estimate.pose.x_m, 4.0)
        self.assertAlmostEqual(estimate.pose.y_m, 2.0)

    def test_config_requires_explicit_slam_enable(self) -> None:
        config = load_v2_config()
        with self.assertRaises(ValueError):
            replace(config.localization, backend="slam_toolbox")

    def test_hardware_mission_is_blocked_before_slam_validation(self) -> None:
        config = load_v2_config()
        slam = replace(config.localization.slam, enabled=True)
        configured = replace(config, localization=replace(
            config.localization, backend="slam_toolbox", slam=slam,
        ))
        self.assertTrue(any("SLAM hardware mission" in error for error in
                            validate_runtime_readiness(configured, RuntimeMode.HARDWARE_MISSION)))

    def test_t265_recovery_rebases_without_pose_jump(self) -> None:
        self.add_t265(1.0, 0.2)
        self.fusion.update_slam_anchor(Pose2D(1.0, 0.0, 0.0, 1.0), valid=True, timestamp_s=1.0)
        self.fusion.update_t265(None, PoseQuality("t265", False, True))
        self.assertIsNone(self.fusion.continuous_t265_pose)
        self.add_t265(1.05, 5.0)
        self.assertAlmostEqual(self.fusion.continuous_t265_pose.x_m, 0.2)


if __name__ == "__main__":
    unittest.main()
