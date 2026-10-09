from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.differential_navigation import DifferentialNavigator, NavigationState
from components.navigation_common import NavigationGoal
from config.v2_factory import build_differential_navigator
from config.v2_loader import load_v2_config
from core import Pose2D, Twist2D


class DifferentialNavigationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config()

    def make_navigator(self, *, compat: bool = False, drive_changes=None):
        drive = self.config.drive
        if compat:
            drive = replace(drive, protocol_mode="ackermann_firmware_compat")
        elif drive.protocol_mode == "ackermann_firmware_compat":
            drive = replace(drive, protocol_mode="differential_vx_vz")
        if drive_changes:
            drive = replace(drive, **drive_changes)
        return DifferentialNavigator(drive, self.config.navigation)

    def test_direct_goal_from_arbitrary_coordinates(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(-102.0, 47.0))
        output = navigator.step(Pose2D(-104.0, 47.0, 0.0, 1.0), now_s=1.0)
        self.assertIs(output.state, NavigationState.TRACKING)
        self.assertEqual(output.path, ((-104.0, 47.0), (-102.0, 47.0)))
        self.assertGreater(output.command.linear_x_m_s, 0.0)
        self.assertAlmostEqual(output.command.angular_z_rad_s, 0.0)

    def test_goal_to_left_requests_positive_yaw(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(0.8, 3.0))
        output = navigator.step(Pose2D(0.8, 1.0, 0.0, 1.0), now_s=1.0,
                                t265_pose=Pose2D(.8,1.,0.,1.))
        self.assertIs(output.state, NavigationState.ROTATING_TO_PATH)
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertGreater(output.command.angular_z_rad_s, 0.0)

    def test_final_yaw_alignment_is_separate_from_position_arrival(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.0, 2.0, 1.0))
        aligning = navigator.step(Pose2D(2.0, 2.0, 0.0, 1.0), now_s=1.0, t265_pose=Pose2D(2.,2.,0.,1.))
        self.assertIs(aligning.state, NavigationState.FINAL_ALIGN)
        self.assertEqual(aligning.command.linear_x_m_s, 0.0)
        self.assertGreater(aligning.command.angular_z_rad_s, 0.0)
        for i in range(6):
            t=1.05+.05*i
            p=Pose2D(2.,2.,1.,t)
            arrived=navigator.step(p,now_s=t,t265_pose=p)
        self.assertIs(arrived.state, NavigationState.GOAL_REACHED)
        self.assertEqual(arrived.command, Twist2D(0.0, 0.0))

    def test_goal_change_rebuilds_direct_path(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(1.0, 0.0))
        navigator.step(Pose2D(0.0, 0.0, 0.0, 1.0), now_s=1.0)
        navigator.set_goal(NavigationGoal(6.0, -4.0))
        output = navigator.step(Pose2D(5.0, -4.0, 0.0, 1.1), now_s=1.1)
        self.assertEqual(output.path, ((5.0, -4.0), (6.0, -4.0)))

    def test_pose_lost_returns_zero_without_reusing_path(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), now_s=1.0)
        output = navigator.step(None, now_s=1.1, pose_state="lost")
        self.assertIs(output.state, NavigationState.POSE_LOST)
        self.assertEqual(output.command, Twist2D(0.0, 0.0))

    def test_commands_respect_linear_and_angular_speed_limits(self) -> None:
        navigator = self.make_navigator(
            drive_changes={"max_linear_speed_m_s": 0.08, "max_angular_speed_rad_s": 0.12}
        )
        navigator.set_goal(NavigationGoal(2.0, 1.6))
        output = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), now_s=1.0)
        self.assertLessEqual(output.command.linear_x_m_s, 0.08)
        self.assertLessEqual(abs(output.command.angular_z_rad_s), 0.12)

    def test_compatibility_mode_uses_rotate_then_straight_policy(self) -> None:
        navigator = self.make_navigator(compat=True)
        navigator.set_goal(NavigationGoal(0.8, 3.0))
        output = navigator.step(Pose2D(0.8, 1.0, 0.0, 1.0), now_s=1.0,
                                t265_pose=Pose2D(.8,1.,0.,1.))
        self.assertIs(output.state, NavigationState.ROTATING_TO_PATH)
        self.assertEqual(output.command.linear_x_m_s, 0.0)
        self.assertGreater(output.command.angular_z_rad_s, 0.0)
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        straight = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), now_s=1.0)
        self.assertEqual(straight.command.angular_z_rad_s, 0.0)

    def test_degraded_fusion_state_scales_command_down(self) -> None:
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(2.5, 1.0))
        normal = navigator.step(Pose2D(0.6, 1.0, 0.0, 1.0), now_s=1.0)
        degraded = navigator.step(
            Pose2D(0.6, 1.0, 0.0, 1.0), now_s=1.0, pose_state="d500_degraded"
        )
        self.assertAlmostEqual(
            degraded.command.linear_x_m_s,
            normal.command.linear_x_m_s * self.config.navigation.degraded_speed_scale,
        )

    def test_alignment_and_tracking_respect_angular_and_wheel_limits(self):
        navigator=self.make_navigator(drive_changes={"max_angular_speed_rad_s":.12,"max_wheel_speed_m_s":.05})
        p=Pose2D(0,0,3.,1.)
        out=navigator.controller.compute_line(p,(0,0),(1,0),t265_pose=p,now_s=1.)
        self.assertLessEqual(abs(out.command.angular_z_rad_s),.12)
        navigator.controller.reset_path()
        p=Pose2D(.1,.1,0,1.)
        out=navigator.controller.compute_line(p,(0,0),(1,0))
        v,w=out.command.linear_x_m_s,out.command.angular_z_rad_s
        for speed in (v-w*.198/2,v+w*.198/2):
            self.assertGreaterEqual(speed,0)
            self.assertLessEqual(speed,.05+1e-9)

    def test_navigation_line_alignment_settles_and_goal_change_resets_target(self):
        import math
        navigator = self.make_navigator()
        navigator.set_goal(NavigationGoal(1,0))
        def step(t,yaw):
            p=Pose2D(0,0,yaw,t)
            return navigator.step(p,now_s=t,t265_pose=p)
        out=step(1.,math.pi/2)
        self.assertIs(out.state,NavigationState.ROTATING_TO_PATH)
        self.assertLess(out.command.angular_z_rad_s,0)
        out=step(1.05,math.radians(55))
        self.assertEqual(out.command.linear_x_m_s,0)
        step(1.10,0)
        for i in range(1,5): out=step(1.10+i*.05,0)
        self.assertEqual(out.diagnostics["line_state"],"align_settled")
        self.assertIs(out.state,NavigationState.ROTATING_TO_PATH)
        self.assertGreater(step(1.35,0).command.linear_x_m_s,0)
        navigator.set_goal(NavigationGoal(0,1))
        out=step(1.40,0)
        self.assertGreater(out.command.angular_z_rad_s,0)
        navigator.clear_goal()
        self.assertIsNone(navigator.controller._line_align_yaw)

    def test_factory_builds_navigator_without_hardware(self) -> None:
        self.assertIsInstance(build_differential_navigator(self.config), DifferentialNavigator)


if __name__ == "__main__":
    unittest.main()
