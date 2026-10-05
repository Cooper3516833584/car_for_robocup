from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.differential_navigation import DifferentialNavigator, NavigationState
from components.navigation_common import NavigationGoal, NavigationGrid
from config.v2_factory import build_differential_navigator
from config.v2_loader import load_v2_config
from core import Pose2D, Twist2D


class DifferentialNavigationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config()
        self.grid = NavigationGrid(50, 50, 0.1)

    def make_navigator(self, *, compat: bool = False, drive_changes=None):
        drive = self.config.drive
        if compat:
            drive = replace(drive, protocol_mode="ackermann_firmware_compat")
        elif drive.protocol_mode == "ackermann_firmware_compat":
            drive = replace(drive, protocol_mode="differential_vx_vz")
        if drive_changes:
            drive = replace(drive, **drive_changes)
        return DifferentialNavigator(self.config.geometry, drive, self.config.navigation)

    def test_empty_map_straight_goal_outputs_forward_twist(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.5, 1.0))

        output = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0)

        self.assertIs(output.state, NavigationState.TRACKING)
        self.assertGreater(output.command.linear_x_m_s, 0.0)
        self.assertAlmostEqual(output.command.angular_z_rad_s, 0.0, places=9)

    def test_goal_to_left_requests_positive_yaw(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(0.8, 3.0))

        output = navigator.step(Pose2D(0.8, 1.0, 0.0, 1.0), self.grid, now_s=1.0)

        self.assertIs(output.state, NavigationState.ROTATING_TO_PATH)
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertGreater(output.command.angular_z_rad_s, 0.0)

    def test_final_yaw_alignment_is_separate_from_position_arrival(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.0, 2.0, 1.0))
        aligning = navigator.step(Pose2D(2.0, 2.0, 0.0, 1.0), self.grid, now_s=1.0)
        self.assertIs(aligning.state, NavigationState.FINAL_ALIGN)
        self.assertEqual(aligning.command.linear_x_m_s, 0.0)
        self.assertGreater(aligning.command.angular_z_rad_s, 0.0)

        navigator.set_goal(NavigationGoal(2.0, 2.0, 1.0))
        arrived = navigator.step(Pose2D(2.0, 2.0, 1.0, 1.0), self.grid, now_s=1.0)
        self.assertIs(arrived.state, NavigationState.GOAL_REACHED)
        self.assertEqual(arrived.command, Twist2D(0.0, 0.0))

    def test_astar_routes_around_inflated_obstacle(self) -> None:
        grid = NavigationGrid(
            50,
            50,
            0.1,
            occupied_cells={(25, y) for y in range(15, 35)},
        )
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(4.4, 2.5))

        output = navigator.step(Pose2D(0.6, 2.5, 0.0, 1.0), grid, now_s=1.0)

        self.assertIsNot(output.state, NavigationState.BLOCKED)
        self.assertGreater(len(output.path), 2)
        self.assertTrue(any(y > 3.8 or y < 1.2 for _, y in output.path))

    def test_no_path_returns_blocked_and_zero(self) -> None:
        grid = NavigationGrid(50, 50, 0.1, occupied_cells={(25, y) for y in range(50)})
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(4.4, 2.5))

        output = navigator.step(Pose2D(0.6, 2.5, 0.0, 1.0), grid, now_s=1.0)

        self.assertIs(output.state, NavigationState.BLOCKED)
        self.assertEqual(output.command, Twist2D(0.0, 0.0))

    def test_pose_lost_returns_zero_without_reusing_path(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0)

        output = navigator.step(
            None, self.grid, now_s=1.1, pose_state="lost"
        )

        self.assertIs(output.state, NavigationState.POSE_LOST)
        self.assertEqual(output.command, Twist2D(0.0, 0.0))

    def test_commands_respect_linear_and_angular_speed_limits(self) -> None:
        navigator = self.make_navigator(
            drive_changes={"max_linear_speed_m_s": 0.08, "max_angular_speed_rad_s": 0.12}
        )
        navigator.set_goal(NavigationGoal(2.0, 1.6))

        output = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0)

        self.assertLessEqual(output.command.linear_x_m_s, 0.08)
        self.assertLessEqual(abs(output.command.angular_z_rad_s), 0.12)

    def test_compatibility_mode_uses_rotate_then_straight_policy(self) -> None:
        navigator = self.make_navigator(compat=True)
        navigator.set_goal(NavigationGoal(0.8, 3.0))
        output = navigator.step(Pose2D(0.8, 1.0, 0.0, 1.0), self.grid, now_s=1.0)

        self.assertTrue(output.diagnostics["compatibility_rotate_first"])
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertGreater(output.command.angular_z_rad_s, 0.0)
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        straight = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0)
        self.assertEqual(straight.command.angular_z_rad_s, 0.0)

    def test_degraded_fusion_state_scales_command_down(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        normal = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0)
        degraded = navigator.step(
            Pose2D(0.6, 1.0, 0.0, 1.0), self.grid, now_s=1.0, pose_state="d500_degraded"
        )
        self.assertAlmostEqual(
            degraded.command.linear_x_m_s,
            normal.command.linear_x_m_s * self.config.navigation.degraded_speed_scale,
        )

    def test_factory_builds_navigator_without_hardware(self) -> None:
        navigator = build_differential_navigator(self.config)
        self.assertIsInstance(navigator, DifferentialNavigator)


if __name__ == "__main__":
    unittest.main()
