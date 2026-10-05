"""Hardware-free checks for the task motion layer."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.basic_motion_controller import (
    BasicMotionController, MotionActionState, MotionBusyError, MotionPhase,
)
from components.differential_navigation import DifferentialNavigator
from config.v2_loader import load_v2_config
from core.frames import normalize_angle_rad
from core.types import Pose2D


class BasicMotionTests(unittest.TestCase):
    def setUp(self) -> None:
        config = load_v2_config()
        navigator = DifferentialNavigator(config.drive, config.navigation)
        self.motion = BasicMotionController(navigator, config.navigation, config.drive)
        self.speed = config.drive.max_linear_speed_m_s

    @staticmethod
    def pose(x=0.0, y=0.0, yaw=0.0, t=1.0):
        return Pose2D(x, y, yaw, t)

    def step(self, pose=None, state="ok"):
        return self.motion.step(self.pose() if pose is None else pose, now_s=1.0, pose_state=state)

    def test_busy_cancel_safe_stop_and_zero_distance(self) -> None:
        self.motion.drive_distance(1.0)
        with self.assertRaises(MotionBusyError):
            self.motion.rotate(1.0)
        self.motion.stop()
        self.assertIsNone(self.motion.navigator.goal)
        self.assertIs(self.step().state, MotionActionState.CANCELLED)
        self.motion.drive_distance(0.0)
        self.assertIs(self.step().state, MotionActionState.SUCCEEDED)
        self.motion.safe_stop("operator")
        self.assertIs(self.step().state, MotionActionState.SAFE_STOPPED)
        with self.assertRaises(RuntimeError):
            self.motion.rotate(1.0)

    def test_relative_rotation_accumulates_across_wrap_and_full_turn(self) -> None:
        for requested in (math.pi / 2, -math.pi / 2, math.pi, 2 * math.pi):
            motion = self.motion
            motion.rotate(requested)
            start = math.radians(170)
            self.assertIs(self.step(self.pose(yaw=start)).state, MotionActionState.RUNNING)
            increments = 8
            for index in range(1, increments + 1):
                yaw = ((start + requested * index / increments + math.pi) % (2 * math.pi)) - math.pi
                output = self.step(self.pose(yaw=yaw))
            self.assertIs(output.state, MotionActionState.SUCCEEDED)

    def test_absolute_rotation_and_face_point(self) -> None:
        self.motion.rotate_to(math.radians(-179))
        result = self.step(self.pose(yaw=math.radians(175)))
        self.assertGreater(result.command.angular_z_rad_s, 0.0)
        self.motion.stop()
        self.motion.face_point(0.0, 1.0)
        result = self.step()
        self.assertEqual(result.command.linear_x_m_s, 0.0)
        self.assertGreater(result.command.angular_z_rad_s, 0.0)

    def test_distance_forward_and_reverse_keep_start_heading(self) -> None:
        self.motion.drive_distance(1.0)
        self.assertGreater(self.step().command.linear_x_m_s, 0.0)
        self.assertIs(self.step(self.pose(x=1.0)).state, MotionActionState.SUCCEEDED)
        self.motion.drive_distance(-0.5)
        self.assertLess(self.step(self.pose(x=1.0)).command.linear_x_m_s, 0.0)
        self.assertIs(self.step(self.pose(x=0.5)).state, MotionActionState.SUCCEEDED)

    def _tight_motion(self) -> BasicMotionController:
        """The car's measured endgame tolerance; the shipped example profile is looser."""
        config = load_v2_config()
        navigation = replace(config.navigation, position_tolerance_m=0.01)
        return BasicMotionController(
            DifferentialNavigator(config.drive, navigation), navigation, config.drive)

    def test_reverse_endgame_does_not_pivot_toward_lateral_residual(self) -> None:
        """A straight reverse action cannot turn sideways to chase a missed point."""
        motion = self._tight_motion()
        motion.drive_distance(-0.20)
        self.assertIs(motion.step(self.pose(), now_s=1.0).state, MotionActionState.RUNNING)
        # 2 cm past the target and 3 cm to the left would require a large turn.
        output = motion.step(self.pose(x=-0.22, y=0.03), now_s=1.0)
        self.assertIs(output.state, MotionActionState.BLOCKED)
        self.assertAlmostEqual(output.diagnostics["remaining_m"], 0.02, places=9)
        self.assertGreater(output.diagnostics["goal_distance_m"],
                           motion.navigation.position_tolerance_m)
        self.assertEqual(output.diagnostics["reason"], "reverse_goal_requires_turn")
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertEqual(output.command.angular_z_rad_s, 0.0)

    def test_drive_distance_overshoot_drives_back_to_the_goal(self) -> None:
        motion = self._tight_motion()
        motion.drive_distance(0.20)
        motion.step(self.pose(), now_s=1.0)
        output = motion.step(self.pose(x=0.25), now_s=1.0)
        self.assertAlmostEqual(output.diagnostics["remaining_m"], -0.05, places=9)
        self.assertLess(output.diagnostics["goal_distance_m"],
                        motion.navigation.position_tolerance_m * 10.0)
        self.assertLess(output.command.linear_x_m_s, 0.0)

    def test_reverse_stops_on_heading_divergence_with_a_weak_wheel(self) -> None:
        """Do not let a weak drive channel turn a straight reverse into a spin."""
        config = load_v2_config()
        navigation = replace(config.navigation, position_tolerance_m=0.01,
                             slowdown_distance_m=0.15)
        motion = BasicMotionController(
            DifferentialNavigator(config.drive, navigation), navigation, config.drive)
        motion.drive_distance(-0.20)
        track = config.geometry.drive_track_width_m
        x = y = yaw = 0.0
        t = 0.0
        for _ in range(600):
            output = motion.step(Pose2D(x, y, yaw, t), now_s=t, pose_state="ok")
            if output.state is MotionActionState.BLOCKED:
                break
            v, w = output.command.linear_x_m_s, output.command.angular_z_rad_s
            left = (v - w * track / 2.0) * 1.0
            right = (v + w * track / 2.0) * 0.65
            x += (left + right) / 2.0 * math.cos(yaw) * 0.05
            y += (left + right) / 2.0 * math.sin(yaw) * 0.05
            yaw = normalize_angle_rad(yaw + (right - left) / track * 0.05)
            t += 0.05
        else:
            self.fail("backward drive did not stop after its heading diverged")
        self.assertIn(output.diagnostics["reason"],
                      {"reverse_heading_diverged", "reverse_goal_requires_turn"})
        self.assertLess(abs(yaw), math.radians(25.0))
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertEqual(output.command.angular_z_rad_s, 0.0)

    def test_reverse_correction_with_small_heading_drift_stays_in_reverse(self) -> None:
        motion = self._tight_motion()
        motion.drive_distance(-0.20)
        motion.step(self.pose(), now_s=1.0)
        output = motion.step(self.pose(x=-0.05, y=0.005, yaw=math.radians(-5)), now_s=1.0)
        self.assertIs(output.state, MotionActionState.RUNNING)
        self.assertLess(output.command.linear_x_m_s, 0.0)
        self.assertGreater(output.command.angular_z_rad_s, 0.0)

    def test_reverse_heading_divergence_blocks_before_pivot(self) -> None:
        motion = self._tight_motion()
        motion.drive_distance(-0.20)
        motion.step(self.pose(), now_s=1.0)
        output = motion.step(self.pose(x=-0.08, yaw=math.radians(-21)), now_s=1.0)
        self.assertIs(output.state, MotionActionState.BLOCKED)
        self.assertEqual(output.diagnostics["reason"], "reverse_heading_diverged")
        self.assertAlmostEqual(output.diagnostics["heading_drift_rad"],
                               math.radians(-21))
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertEqual(output.command.angular_z_rad_s, 0.0)

    def test_drive_to_is_straight_and_requires_no_map(self) -> None:
        self.motion.drive_to(1.0, 0.0)
        self.assertGreater(self.step().command.linear_x_m_s, 0.0)
        self.assertIs(self.step(self.pose(x=1.0)).state, MotionActionState.SUCCEEDED)

    def test_segment_thresholds_and_correction_direction(self) -> None:
        for y, phase in ((0.05, MotionPhase.TRACKING), (0.10, MotionPhase.STRONG_CORRECTION),
                         (0.15, MotionPhase.STRONG_CORRECTION), (0.20, MotionPhase.STRONG_CORRECTION),
                         (-0.15, MotionPhase.STRONG_CORRECTION)):
            self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
            result = self.step(self.pose(y=y))
            self.assertIs(result.phase, phase)
            self.assertIs(result.state, MotionActionState.RUNNING)
            self.assertGreater(result.command.linear_x_m_s, 0.0)
            self.assertLessEqual(result.command.linear_x_m_s, self.speed * (0.35 if abs(y) >= 0.10 else 1.0))
            self.assertLess(result.command.angular_z_rad_s * y, 0.0)
            self.motion.stop()

    def test_segment_recovery_hysteresis_and_finite_target(self) -> None:
        self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
        first = self.step(self.pose(x=0.5, y=0.201))
        self.assertIs(first.state, MotionActionState.RUNNING)
        self.assertIs(first.phase, MotionPhase.RECOVERY_ALIGN)
        self.assertEqual(first.command.linear_x_m_s, 0.0)
        self.assertAlmostEqual(first.diagnostics["recovery_target_x_m"], 0.5)
        turning = self.step(self.pose(x=0.5, y=0.25, yaw=-math.pi / 2))
        self.assertIs(turning.phase, MotionPhase.RECOVERY_RETURN)
        self.assertGreater(turning.command.linear_x_m_s, 0.0)
        self.assertLessEqual(turning.command.linear_x_m_s, 0.10)
        still = self.step(self.pose(x=0.5, y=0.18, yaw=-math.pi / 2))
        self.assertTrue(still.diagnostics["recovery_active"])
        exit_result = self.step(self.pose(x=0.5, y=0.15))
        self.assertIs(exit_result.phase, MotionPhase.STRONG_CORRECTION)
        self.assertIs(self.step(self.pose(x=0.5, y=0.09)).phase, MotionPhase.TRACKING)
        self.motion.stop()
        self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
        beyond = self.step(self.pose(x=1.5, y=0.25))
        self.assertIs(beyond.state, MotionActionState.RUNNING)
        self.assertAlmostEqual(beyond.diagnostics["recovery_target_x_m"], 1.0)
        self.assertAlmostEqual(beyond.diagnostics["recovery_target_y_m"], 0.0)
        self.motion.stop()
        self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
        end_plane = self.step(self.pose(x=1.0, y=0.15))
        self.assertIs(end_plane.phase, MotionPhase.RECOVERY_ALIGN)
        self.assertIs(end_plane.state, MotionActionState.RUNNING)
        self.motion.stop()
        self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
        before_start = self.step(self.pose(x=-0.5, y=0.25))
        self.assertAlmostEqual(before_start.diagnostics["recovery_target_x_m"], 0.0)
        self.assertAlmostEqual(before_start.diagnostics["recovery_target_y_m"], 0.0)

    def test_large_segment_error_does_not_safety_stop(self) -> None:
        for y in (0.25, 0.50):
            self.motion.follow_segment((0.0, 0.0), (1.0, 0.0))
            for _ in range(3):
                output = self.step(self.pose(x=0.5, y=y))
                self.assertIs(output.state, MotionActionState.RUNNING)
                self.assertIn(output.phase, {MotionPhase.RECOVERY_ALIGN, MotionPhase.RECOVERY_RETURN})
            self.motion.stop()

    def test_pose_loss_and_degraded_scale(self) -> None:
        self.motion.drive_distance(1.0)
        normal = self.step().command
        degraded = self.step(state="t265_degraded").command
        scale = self.motion.navigation.degraded_speed_scale
        self.assertAlmostEqual(degraded.linear_x_m_s, normal.linear_x_m_s * scale)
        lost = self.motion.step(None, now_s=1.0)
        self.assertIs(lost.state, MotionActionState.POSE_LOST)
        self.assertEqual(lost.command.linear_x_m_s, 0.0)

    def test_navigation_delegates_without_second_degraded_scale(self) -> None:
        self.motion.navigate_to(2.0, 0.0)
        normal = self.motion.step(self.pose(), now_s=1.0)
        degraded = self.motion.step(self.pose(), now_s=1.0, pose_state="d500_degraded")
        self.assertGreater(normal.command.linear_x_m_s, 0.0)
        self.assertAlmostEqual(degraded.command.linear_x_m_s,
                               normal.command.linear_x_m_s * self.motion.navigation.degraded_speed_scale)


if __name__ == "__main__":
    unittest.main()
