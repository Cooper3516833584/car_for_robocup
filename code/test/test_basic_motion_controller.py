"""Hardware-free checks for the task motion layer."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.basic_motion_controller import (
    LINE_MINIMUM_SPEED_M_S,
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

    def test_straight_distance_stops_at_observed_forward_residual_without_pivot(self):
        # 2026-10-07: 47cm already reached, 2.001cm sideways; the old point
        # closure requested omega=0.8 rad/s and changed the car heading by 46deg.
        motion = self._tight_motion()
        motion.navigation = replace(motion.navigation, position_tolerance_m=.005)
        motion.drive_distance(.47, lateral_tolerance_m=.03)
        motion.step(self.pose(), now_s=1.)
        output = motion.step(self.pose(x=.4723193827, y=.0200137066, yaw=-.0415743644), now_s=2.)
        self.assertIs(output.state, MotionActionState.SUCCEEDED)
        self.assertEqual(output.command.linear_x_m_s, 0.)
        self.assertEqual(output.command.angular_z_rad_s, 0.)
        self.assertEqual(output.diagnostics["distance_policy"], "straight")

    def test_straight_reverse_stops_at_observed_residual_instead_of_requiring_turn(self):
        # Last recorded failure: along-track remaining 0.41mm, lateral 13.83mm.
        motion = self._tight_motion()
        motion.navigation = replace(motion.navigation, position_tolerance_m=.005)
        motion.drive_distance(-.47, lateral_tolerance_m=.03)
        motion.step(self.pose(), now_s=1.)
        output = motion.step(self.pose(x=-.4704098771, y=-.0138318772, yaw=-.0285950052), now_s=2.)
        self.assertIs(output.state, MotionActionState.SUCCEEDED)
        self.assertEqual(output.command.linear_x_m_s, 0.)
        self.assertEqual(output.command.angular_z_rad_s, 0.)

    def test_saved_straight_heading_is_reused_despite_pose_yaw_change(self):
        motion = self._tight_motion()
        motion.drive_distance(-.47, lateral_tolerance_m=.03, heading_yaw_rad=1.5)
        output = motion.step(self.pose(yaw=1.55), now_s=1.)
        self.assertAlmostEqual(output.diagnostics["heading_reference_rad"], 1.5)
        self.assertAlmostEqual(output.diagnostics["heading_drift_rad"], .05)
        self.assertLess(output.command.linear_x_m_s, 0.)
        self.assertLess(output.command.angular_z_rad_s, 0.)

    def test_straight_distance_fails_closed_on_large_residual_or_heading(self):
        for distance in (.47, -.47):
            for x, y, yaw, reason in (
                (distance, .031, 0., "straight_distance_lateral_error_exceeded"),
                (0., 0., math.radians(21), "straight_distance_heading_diverged" if distance > 0 else "reverse_heading_diverged"),
                (distance * 1.2, 0., 0., "straight_distance_overshoot"),
            ):
                with self.subTest(distance=distance, reason=reason):
                    motion = self._tight_motion()
                    motion.drive_distance(distance, lateral_tolerance_m=.03)
                    motion.step(self.pose(), now_s=1.)
                    output = motion.step(self.pose(x=x, y=y, yaw=yaw), now_s=2.)
                    self.assertIs(output.state, MotionActionState.BLOCKED)
                    self.assertEqual(output.diagnostics["reason"], reason)
                    self.assertEqual(output.command.linear_x_m_s, 0.)
                    self.assertEqual(output.command.angular_z_rad_s, 0.)

    def test_invalid_straight_options_do_not_start_an_action(self):
        for kwargs in ({"lateral_tolerance_m": 0}, {"lateral_tolerance_m": float("nan")},
                       {"heading_yaw_rad": 1.}, {"lateral_tolerance_m": .03, "heading_yaw_rad": float("inf")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.motion.drive_distance(.47, **kwargs)
            self.assertIs(self.motion.state, MotionActionState.IDLE)

    def _tight_motion(self) -> BasicMotionController:
        """The car's measured endgame tolerance; the shipped example profile is looser."""
        config = load_v2_config()
        navigation = replace(config.navigation, position_tolerance_m=0.01)
        return BasicMotionController(
            DifferentialNavigator(config.drive, navigation), navigation, config.drive)

    def _run_against_a_deadband(self, motion, track_m, deadband_m_s,
                                max_steps=600, dt=0.05):
        """Integrate the commanded twist through a plant that ignores slow wheels.

        The 2026-10-05 session measured that the drivetrain delivers nothing below
        roughly 0.03 m/s, which is what stalled the reverse endgame.
        """
        x = y = yaw = 0.0
        t = 0.0
        travel = 0.0
        output = None
        for _ in range(max_steps):
            output = motion.step(Pose2D(x, y, yaw, t), now_s=t, pose_state="ok")
            if output.state is not MotionActionState.RUNNING:
                return output, (x, y, yaw), travel
            v, w = output.command.linear_x_m_s, output.command.angular_z_rad_s
            left = v - w * track_m / 2.0
            right = v + w * track_m / 2.0
            left = 0.0 if abs(left) < deadband_m_s else left
            right = 0.0 if abs(right) < deadband_m_s else right
            x += (left + right) / 2.0 * math.cos(yaw) * dt
            y += (left + right) / 2.0 * math.sin(yaw) * dt
            yaw = normalize_angle_rad(yaw + (right - left) / track_m * dt)
            travel += abs((right - left) / track_m) * dt
            t += dt
        return None, (x, y, yaw), travel

    def test_stop_at_target_commands_at_least_the_drive_minimum(self) -> None:
        """The slow endgame must stay above the drivetrain's effective minimum."""
        motion = self._tight_motion()
        motion.drive_distance(0.50)
        motion.step(self.pose(), now_s=1.0)
        output = motion.step(self.pose(x=0.48), now_s=1.0)
        self.assertIs(output.state, MotionActionState.RUNNING)
        self.assertLess(output.diagnostics["goal_distance_m"],
                        motion.navigation.slowdown_distance_m)
        floor = min(motion.drive.max_linear_speed_m_s, LINE_MINIMUM_SPEED_M_S)
        self.assertGreaterEqual(output.command.linear_x_m_s, floor)

    def test_drive_distance_finishes_against_a_deadband_plant(self) -> None:
        """Every acceptance distance must still land on target when slow commands do nothing."""
        config = load_v2_config()
        track = config.geometry.drive_track_width_m
        for distance in (-0.50, -0.20, 0.20, 0.50):
            navigation = replace(config.navigation, position_tolerance_m=0.01)
            motion = BasicMotionController(
                DifferentialNavigator(config.drive, navigation), navigation, config.drive)
            motion.drive_distance(distance)
            output, (x, _y, _yaw), _travel = self._run_against_a_deadband(motion, track, 0.030)
            self.assertIsNotNone(output, "action never finished for %+.2f m" % distance)
            self.assertIs(output.state, MotionActionState.SUCCEEDED,
                          "action did not succeed for %+.2f m" % distance)
            self.assertLessEqual(abs(x - distance), navigation.position_tolerance_m + 1e-9,
                                 "landed %.4f m short/over for %+.2f m" % (x - distance, distance))

    def test_drive_to_finishes_against_a_deadband_plant(self) -> None:
        config = load_v2_config()
        track = config.geometry.drive_track_width_m
        navigation = replace(config.navigation, position_tolerance_m=0.01)
        motion = BasicMotionController(
            DifferentialNavigator(config.drive, navigation), navigation, config.drive)
        motion.drive_to(0.30, 0.20)
        output, (x, y, _yaw), _travel = self._run_against_a_deadband(motion, track, 0.030)
        self.assertIsNotNone(output, "drive_to never finished against the deadband plant")
        self.assertIs(output.state, MotionActionState.SUCCEEDED)
        self.assertLessEqual(math.hypot(x - 0.30, y - 0.20),
                             navigation.position_tolerance_m + 1e-9)

    def test_drive_to_endgame_keeps_translating_instead_of_pivoting(self) -> None:
        """Past the end point the residual must be closed by driving, not pivoting.

        Regression: the branch used ``atan2(end - pose)`` for its heading, which at
        centimetre range is noise-dominated, and the in-place turn that followed
        spun the vehicle -- 177 deg was observed on the car on 2026-10-05 even
        though the goal distance itself stayed inside tolerance.
        """
        motion = self._tight_motion()
        motion.drive_to(0.30, 0.20)
        motion.step(self.pose(), now_s=1.0)
        # 5 cm past the end point, abeam of it: the goal sits ~90 deg to the side.
        output = motion.step(self.pose(x=0.30, y=0.25, yaw=math.pi / 2), now_s=1.0)
        self.assertIs(output.state, MotionActionState.RUNNING)
        self.assertNotEqual(output.command.linear_x_m_s, 0.0)

    def test_follow_segment_finishes_against_a_deadband_plant(self) -> None:
        config = load_v2_config()
        track = config.geometry.drive_track_width_m
        navigation = replace(config.navigation, position_tolerance_m=0.01)
        motion = BasicMotionController(
            DifferentialNavigator(config.drive, navigation), navigation, config.drive)
        motion.follow_segment((0.0, 0.0), (0.50, 0.0))
        output, (x, y, _yaw), travel = self._run_against_a_deadband(
            motion, track, 0.030, max_steps=1200)
        self.assertIsNotNone(output, "follow_segment never finished against the deadband plant")
        self.assertIs(output.state, MotionActionState.SUCCEEDED)
        self.assertLessEqual(math.hypot(x - 0.50, y), navigation.position_tolerance_m + 1e-9)
        self.assertLess(travel, math.radians(60.0),
                        "follow_segment turned %.1f deg to finish" % math.degrees(travel))

    def test_follow_segment_past_the_end_hands_over_to_the_endpoint_approach(self) -> None:
        """Recovery can never exit past the segment end, so it must hand over.

        Regression for the 138 deg spin observed on the car on 2026-10-05: the
        recovery exit test requires ``remaining_m > 0``, which is never true once
        the vehicle has passed the end of the segment.
        """
        motion = self._tight_motion()
        motion.follow_segment((0.0, 0.0), (0.50, 0.0))
        motion.step(self.pose(), now_s=1.0)
        self.assertIs(motion.step(self.pose(x=0.45, y=0.25), now_s=1.0).phase,
                      MotionPhase.RECOVERY_ALIGN)
        output = motion.step(self.pose(x=0.55, y=0.22), now_s=1.0)
        self.assertIs(output.state, MotionActionState.RUNNING)
        self.assertIsNot(output.phase, MotionPhase.RECOVERY_ALIGN)
        self.assertFalse(output.diagnostics["recovery_active"])

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
