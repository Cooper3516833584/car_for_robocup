from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.navigation_common import NavigationGoal, NavigationGrid
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import RobocupMissionState, RuntimeReadinessError, build_runtime


class RobocupRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config()
        self.now = [10.0]

    def clock(self) -> float:
        return self.now[0]

    def make_runtime(self, *, mode=RuntimeMode.DRY_RUN, count=4):
        return build_runtime(
            self.config,
            mode,
            clock=self.clock,
            fake_sample_count=count,
            world=NavigationGrid(100, 100, 0.1, origin_x_m=-5.0, origin_y_m=-5.0),
        )

    def test_dry_run_builds_full_fake_runtime_and_closes(self) -> None:
        runtime = self.make_runtime()
        self.assertIsInstance(runtime.drive.backend, FakeDriveBackend)
        results = runtime.run_steps(2, period_s=0.001)
        self.assertEqual(len(results), 2)
        self.assertTrue(any(result.navigation is not None for result in results))
        self.assertTrue(runtime.drive.backend.commands)
        self.assertTrue(runtime.t265_source.stopped)
        self.assertTrue(runtime.d500_source.stopped)
        self.assertEqual(runtime.drive.backend.close_count, 1)

    def test_unmeasured_hardware_mission_is_rejected_before_devices(self) -> None:
        with self.assertRaisesRegex(RuntimeReadinessError, "measured drive geometry"):
            self.make_runtime(mode=RuntimeMode.HARDWARE_MISSION)

    def test_lost_pose_during_navigation_stops_drive(self) -> None:
        runtime = self.make_runtime(count=1)
        runtime.start()
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        first = runtime.step(now_s=10.0)
        self.assertEqual(first.mission_state, RobocupMissionState.NAVIGATING)
        self.now[0] = 11.0
        lost = runtime.step(now_s=11.0)
        self.assertEqual(lost.mission_state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(lost.command.linear_x_m_s, 0.0)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()

    def test_backend_exception_enters_error_and_stops(self) -> None:
        runtime = self.make_runtime()
        runtime.drive.backend.fail_with = OSError("synthetic motor fault")
        runtime.start()
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        result = runtime.step(now_s=10.0)
        self.assertEqual(result.mission_state, RobocupMissionState.ERROR)
        self.assertIn("synthetic motor fault", result.error)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()

    def test_keyboard_interrupt_path_runs_sensor_and_drive_cleanup(self) -> None:
        runtime = self.make_runtime()
        with patch.object(runtime, "step", side_effect=KeyboardInterrupt):
            runtime.run()
        self.assertEqual(runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertTrue(runtime.t265_source.stopped)
        self.assertTrue(runtime.d500_source.stopped)
        self.assertEqual(runtime.drive.backend.close_count, 1)

    def test_finished_mission_keeps_zero_command(self) -> None:
        runtime = self.make_runtime()
        runtime.start()
        runtime.mission.finish()
        result = runtime.step(now_s=10.0)
        self.assertEqual(result.mission_state, RobocupMissionState.FINISHED)
        self.assertEqual(result.command.linear_x_m_s, 0.0)
        self.assertEqual(result.command.angular_z_rad_s, 0.0)
        runtime.close()

    def test_logging_failure_cannot_interrupt_safe_stop(self) -> None:
        class BrokenLogger:
            def emit(self, *_args, **_kwargs):
                raise OSError("disk unavailable")

            def close(self):
                raise OSError("disk unavailable")

        runtime = self.make_runtime()
        runtime.event_logger = BrokenLogger()
        runtime.start()
        runtime.mission.finish()
        result = runtime.step(now_s=10.0)
        self.assertEqual(result.mission_state, RobocupMissionState.FINISHED)
        self.assertEqual(result.command.linear_x_m_s, 0.0)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
