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
        out = line_step((0.0, 0.0, math.pi / 2), (0.0, 0.0), (2.0, 0.0), 0.0)
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
