"""Isolated math/behavior checks for the reference Pure Pursuit core."""

import math
import unittest

from pathlib import Path
import sys
from dataclasses import replace
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.differential_navigation import DifferentialPathController
from config.v2_loader import load_v2_config
from core.types import Pose2D

def line_step(pose, start, end, previous_progress_m, **kwargs):
    config = load_v2_config()
    nav = replace(config.navigation, lookahead_m=kwargs.pop("lookahead_m", .20),
                  position_tolerance_m=kwargs.pop("stop_tolerance_m", .005))
    drive = replace(config.drive, max_linear_speed_m_s=kwargs.pop("speed_limit_m_s", .15),
                    max_angular_speed_rad_s=kwargs.pop("max_omega_rad_s", .80),
                    max_linear_accel_m_s2=kwargs.pop("decel_m_s2", .30))
    controller = DifferentialPathController(nav, drive, track_width_m=kwargs.pop("track_width_m", .198))
    output = controller.compute_line(Pose2D(*pose, 1.), start, end, previous_progress_m,
                                     terminal_lateral_m=kwargs.pop("terminal_lateral_m", .03), **kwargs)
    return SimpleNamespace(v_m_s=output.command.linear_x_m_s, omega_rad_s=output.command.angular_z_rad_s,
                           progress_m=output.diagnostics["progress_m"], status=output.diagnostics["line_state"],
                           details=output.diagnostics)


class PurePursuitReferenceTests(unittest.TestCase):
    def test_near_endpoint_virtual_carrot_keeps_small_forward_correction(self):
        out = line_step((.463, .020, 0), (0, 0), (.47, 0), 0)
        self.assertEqual(out.status, "tracking")
        self.assertGreater(out.v_m_s, 0)
        self.assertLess(abs(out.details["heading_error_rad"]), math.radians(15))
        self.assertLess(out.omega_rad_s, 0)
        self.assertAlmostEqual(out.details["carrot_x_m"], .663)
        self.assertAlmostEqual(out.details["remaining_m"], .007)
        self.assertAlmostEqual(out.details["segment_length_m"], .47)

    def test_endpoint_cross_track_checks_stop_without_turning(self):
        for cross, status in ((.020, "arrived"), (.040, "terminal_lateral_error")):
            with self.subTest(cross=cross):
                out = line_step((.470, cross, 0), (0, 0), (.47, 0), 0)
                self.assertEqual(out.status, status)
                self.assertEqual((out.v_m_s, out.omega_rad_s), (0, 0))

    def test_real_overshoot_is_blocked_even_with_large_yaw_error(self):
        for x in (.570, .476):
            for yaw in (0, math.pi):
                with self.subTest(x=x, yaw=yaw):
                    out = line_step((x, 0, yaw), (0, 0), (.47, 0), .47)
                    self.assertEqual(out.status, "line_overshoot")
                    self.assertEqual(out.details["reason"], "line_overshoot")
                    self.assertEqual((out.v_m_s, out.omega_rad_s), (0, 0))
                    self.assertAlmostEqual(out.details["raw_progress_m"], x)
                    self.assertAlmostEqual(out.details["remaining_m"], .47-x)

    def test_symmetric_arrival_tolerance_uses_actual_projection(self):
        for x, status in ((.466, "arrived"), (.474, "arrived"), (.464, "tracking")):
            with self.subTest(x=x):
                out = line_step((x, 0, 0), (0, 0), (.47, 0), .47)
                self.assertEqual(out.status, status)
                self.assertAlmostEqual(out.progress_m, .47)
                if status == "tracking":
                    self.assertGreater(out.v_m_s, 0)

    def test_reverse_near_endpoint_uses_virtual_carrot_without_forward_motion(self):
        out = line_step((.007, .020, 0), (.47, 0), (0, 0), 0, reverse=True)
        self.assertEqual(out.status, "tracking")
        self.assertLess(out.v_m_s, 0)
        self.assertLess(abs(out.details["heading_error_rad"]), math.radians(15))
        self.assertGreater(out.omega_rad_s, 0)
        self.assertAlmostEqual(out.details["carrot_x_m"], -.193)
        self.assertAlmostEqual(out.details["remaining_m"], .007)

    def controller(self):
        config = load_v2_config()
        return DifferentialPathController(replace(config.navigation, lookahead_m=.20), config.drive)

    def test_alignment_waits_below_60_degrees_and_during_measured_coast(self):
        controller = self.controller()
        def step(t, yaw, local_yaw):
            return controller.compute_line(Pose2D(0, 0, yaw, t), (0, 0), (2, 0),
                t265_pose=Pose2D(0, 0, local_yaw, t), now_s=t)
        out = step(1., math.pi/2, math.pi/2)
        self.assertEqual(out.diagnostics["line_state"], "align_forward")
        self.assertEqual(out.command.linear_x_m_s, 0)
        out = step(1.05, math.radians(55), math.radians(55))
        self.assertEqual(out.command.linear_x_m_s, 0)
        self.assertEqual(out.diagnostics["stable_frames"], 0)
        # Fused yaw is in tolerance while real local yaw is still moving.
        for i in range(4):
            out = step(1.10 + .05*i, 0, .01*i)
            self.assertEqual(out.command.linear_x_m_s, 0)
            self.assertEqual(out.diagnostics["stable_frames"], 0)
        for i in range(1, 5):
            out = step(1.25 + .05*i, 0, .03)
            self.assertEqual(out.command.linear_x_m_s, 0)
            if i < 4:
                self.assertEqual(out.diagnostics["line_state"], "align_forward")
        self.assertEqual(out.diagnostics["line_state"], "align_settled")
        self.assertEqual(out.command.linear_x_m_s, 0)
        self.assertEqual(out.command.angular_z_rad_s, 0)
        self.assertEqual(step(1.50, 0, .03).diagnostics["line_state"], "tracking")

    def test_alignment_repeated_samples_do_not_count_and_missing_stale_stop(self):
        controller = self.controller()
        def step(t, yaw, local):
            return controller.compute_line(Pose2D(0, 0, yaw, t), (0, 0), (2, 0),
                                           t265_pose=local, now_s=t)
        step(1., math.pi/2, Pose2D(0, 0, math.pi/2, 1.))
        local = Pose2D(0, 0, 0, 1.05)
        step(1.05, 0, local)
        for t in (1.06, 1.07, 1.08, 1.09):
            out = step(t, 0, local)
            self.assertEqual(out.diagnostics["stable_frames"], 0)
            self.assertEqual(out.command.linear_x_m_s, 0)
        for t, local in ((1.10, None), (1.50, Pose2D(0,0,0,1.05))):
            out = step(t, 0, local)
            self.assertEqual(out.state.value, "pose_lost")
            self.assertEqual((out.command.linear_x_m_s, out.command.angular_z_rad_s), (0,0))

    def test_alignment_blocked_preserves_turn_failure_reason(self):
        from dataclasses import replace
        from components.differential_navigation import NavigationState
        controller = self.controller()
        controller.drive = replace(controller.drive, allow_in_place_rotation=False)
        p = Pose2D(0, 0, math.pi/2, 1.)
        out = controller.compute_line(p, (0,0), (2,0), t265_pose=p, now_s=1.)
        self.assertIs(out.state, NavigationState.BLOCKED)
        self.assertEqual(out.diagnostics["reason"], "in_place_rotation_unavailable")
        self.assertEqual((out.command.linear_x_m_s,out.command.angular_z_rad_s),(0,0))

    def test_alignment_locks_carrot_bearing_instead_of_line_tangent(self):
        controller = self.controller()
        pose = Pose2D(0, 1, 0, 1.)
        out = controller.compute_line(pose, (0,0), (2,0), t265_pose=pose, now_s=1.)
        self.assertAlmostEqual(out.diagnostics["target_yaw_rad"], math.atan2(-1,.2))
        changed = Pose2D(.1, 1, -.2, 1.05)
        out = controller.compute_line(changed, (0,0), (2,0), t265_pose=changed, now_s=1.05)
        self.assertAlmostEqual(out.diagnostics["target_yaw_rad"], math.atan2(-1,.2))

    def test_reverse_alignment_settles_then_drives_only_backward(self):
        controller = self.controller()
        def step(t, yaw):
            p = Pose2D(0,0,yaw,t)
            return controller.compute_line(p, (0,0), (1,0), reverse=True, t265_pose=p, now_s=t)
        out = step(1., math.pi/2)
        self.assertAlmostEqual(abs(out.diagnostics["target_yaw_rad"]), math.pi)
        self.assertEqual(out.command.linear_x_m_s, 0)
        step(1.05, math.pi)
        for i in range(1,5):
            out = step(1.05+.05*i, math.pi)
            self.assertEqual(out.command.linear_x_m_s, 0)
        self.assertEqual(out.diagnostics["line_state"], "align_settled")
        self.assertLess(step(1.30, math.pi).command.linear_x_m_s, 0)

    def test_endpoint_preempts_an_active_alignment(self):
        from components.differential_navigation import NavigationState
        for x, state in ((.47, NavigationState.GOAL_REACHED), (.57, NavigationState.BLOCKED)):
            controller = self.controller()
            p = Pose2D(0,0,math.pi/2,1.)
            controller.compute_line(p, (0,0), (.47,0), t265_pose=p, now_s=1.)
            out = controller.compute_line(Pose2D(x,0,math.pi/2,1.05),(0,0),(.47,0))
            self.assertIs(out.state, state)
            self.assertEqual((out.command.linear_x_m_s,out.command.angular_z_rad_s),(0,0))

    def test_forward_drift_to_left_turns_right_without_reversing(self):
        out = line_step((0.05, 0.06, 0.0), (0.0, 0.0), (2.0, 0.0), 0.0)
        self.assertEqual(out.status, "tracking")
        self.assertGreater(out.v_m_s, 0.0)
        self.assertLess(out.omega_rad_s, 0.0)

    def test_forward_drift_to_right_turns_left(self):
        out = line_step((0.05, -0.06, 0.0), (0.0, 0.0), (2.0, 0.0), 0.0)
        self.assertGreater(out.v_m_s, 0.0)
        self.assertGreater(out.omega_rad_s, 0.0)

    def test_true_reverse_track_has_negative_velocity_and_correct_yaw(self):
        out = line_step((0.45, 0.05, 0.0), (0.47, 0.0), (0.0, 0.0), 0.0,
                        reverse=True)
        self.assertEqual(out.status, "tracking")
        self.assertLess(out.v_m_s, 0.0)
        self.assertGreater(out.omega_rad_s, 0.0)

    def test_progress_does_not_regress_when_pose_moves_backward(self):
        out = line_step((0.3, 0.0, 0.0), (0.0, 0.0), (2.0, 0.0), 0.8)
        self.assertAlmostEqual(out.progress_m, 0.8)
        self.assertAlmostEqual(out.details["carrot_progress_m"], 1.0)
        self.assertGreater(out.v_m_s, 0.0)

    def test_terminal_lateral_error_never_spins_to_chase_endpoint(self):
        out = line_step((0.469, 0.04, 0.0), (0.0, 0.0), (0.47, 0.0), 0.0)
        self.assertEqual(out.status, "terminal_lateral_error")
        self.assertEqual((out.v_m_s, out.omega_rad_s), (0.0, 0.0))

    def test_arrival_requires_actual_projection(self):
        out = line_step((0.466, 0.001, 0.0), (0.0, 0.0), (0.47, 0.0), 0.46)
        self.assertEqual(out.status, "arrived")
        self.assertEqual((out.v_m_s, out.omega_rad_s), (0.0, 0.0))

    def test_large_heading_error_rotates_without_backward_translation(self):
        out = line_step((0.0, 0.0, math.pi / 2), (0.0, 0.0), (2.0, 0.0), 0.0,
                        t265_pose=Pose2D(0, 0, math.pi/2, 1.), now_s=1.)
        self.assertEqual(out.status, "align_forward")
        self.assertEqual(out.v_m_s, 0.0)
        self.assertLess(out.omega_rad_s, 0.0)

    def test_wheel_sign_preservation_for_forward_tracking(self):
        out = line_step((0.0, 0.15, 0.0), (0.0, 0.0), (2.0, 0.0), 0.0)
        self.assertGreater(out.v_m_s, 0.0)
        left = out.v_m_s - 0.5 * 0.198 * out.omega_rad_s
        right = out.v_m_s + 0.5 * 0.198 * out.omega_rad_s
        self.assertGreaterEqual(left, 0.0)
        self.assertGreaterEqual(right, 0.0)


    def test_closed_loop_sim_recovers_yaw_impulse_without_reverse(self):
        x, y, yaw, progress = 0.0, 0.0, 0.0, 0.0
        completed = False
        for i in range(500):
            out = line_step((x, y, yaw), (0, 0), (0.47, 0), progress)
            progress = out.progress_m
            if out.status == "arrived":
                completed = True
                self.assertLess(abs(y), 0.03)
                break
            self.assertIn(out.status, ("tracking", "align_forward"))
            self.assertGreaterEqual(out.v_m_s, 0.0)
            w = out.omega_rad_s + (0.35 if i < 10 else 0.0)
            x += 0.05 * out.v_m_s * math.cos(yaw + 0.025 * w)
            y += 0.05 * out.v_m_s * math.sin(yaw + 0.025 * w)
            yaw += 0.05 * w
        self.assertTrue(completed)

    def test_closed_loop_reverse_returns_to_original_local_line(self):
        x, y, yaw, progress = 0.47, 0.01, 0.0, 0.0
        completed = False
        for _ in range(500):
            out = line_step((x, y, yaw), (0.47, 0), (0, 0), progress,
                            reverse=True)
            progress = out.progress_m
            if out.status == "arrived":
                completed = True
                self.assertLess(math.hypot(x, y), 0.03)
                break
            self.assertIn(out.status, ("tracking", "align_forward"))
            self.assertLessEqual(out.v_m_s, 0.0)
            x += 0.05 * out.v_m_s * math.cos(yaw + 0.025 * out.omega_rad_s)
            y += 0.05 * out.v_m_s * math.sin(yaw + 0.025 * out.omega_rad_s)
            yaw += 0.05 * out.omega_rad_s
        self.assertTrue(completed)


if __name__ == "__main__":
    unittest.main()
