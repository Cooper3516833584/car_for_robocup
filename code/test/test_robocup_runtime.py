from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.navigation_common import NavigationGoal
from components.pose_fusion import PoseFusionState
from components.t265_driver import T265RawPose
from config.relative_slam_profile import accepted_relative_slam_profile
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

    def test_direct_motion_reaches_fake_drive_without_map_navigation(self) -> None:
        runtime = self.make_runtime()
        runtime.motion.drive_distance(1.0)
        runtime.start()
        try:
            result = runtime.step(now_s=10.0)
            self.assertIsNone(result.navigation)
            self.assertIsNotNone(result.motion)
            self.assertEqual(result.motion.action_type.value, "drive_distance")
            self.assertGreater(result.command.linear_x_m_s, 0.0)
            self.assertIsNone(runtime.navigator.goal)
        finally:
            runtime.close()

    def test_direct_motion_completion_advances_mission(self) -> None:
        runtime = self.make_runtime()
        runtime.motion.drive_distance(0.0)
        runtime.start()
        try:
            result = runtime.step(now_s=10.0)
            self.assertEqual(result.mission_state, RobocupMissionState.TARGET_OPERATION)
            self.assertEqual(result.command.linear_x_m_s, 0.0)
        finally:
            runtime.close()

    def test_legacy_absolute_hardware_mission_still_requires_d500_reference(self) -> None:
        with self.assertRaisesRegex(RuntimeReadinessError, "measured D500 field reference"):
            self.make_runtime(mode=RuntimeMode.HARDWARE_MISSION)

    def test_live_step_uses_clock_after_blocking_t265_read(self) -> None:
        runtime = self.make_runtime()
        runtime.d500_source = None
        runtime.start()

        def delayed_read():
            self.now[0] = 10.05
            return T265RawPose(
                translation_xyz=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                velocity_xyz=None,
                angular_velocity_xyz=None,
                tracker_confidence=3,
                mapper_confidence=0,
                device_timestamp_ms=10050.0,
                received_monotonic_s=10.05,
            )

        runtime.t265_source.read = delayed_read
        try:
            result = runtime.step()
            self.assertEqual(result.estimate.state, PoseFusionState.UNANCHORED)
            self.assertEqual(result.estimate.t265_age_s, 0.0)
        finally:
            runtime.close()

    def test_t265_log_preserves_raw_time_angular_velocity_and_causal_clamp(self) -> None:
        class CaptureLogger:
            def __init__(self):
                self.events = []

            def emit(self, event, **_kwargs):
                self.events.append(event)

            def close(self):
                pass

        config = accepted_relative_slam_profile(self.config)
        runtime = build_runtime(config, RuntimeMode.DRY_RUN, clock=self.clock, fake_sample_count=2)
        logger = CaptureLogger()
        runtime.event_logger = logger
        runtime.start()
        self.now[0] = 10.05
        runtime.t265_source.read = lambda: T265RawPose(
            translation_xyz=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            velocity_xyz=(0.1, 0.2, 0.3),
            angular_velocity_xyz=(0.4, 0.5, 0.6),
            tracker_confidence=3,
            mapper_confidence=2,
            device_timestamp_ms=10080.0,
            received_monotonic_s=10.05,
            measurement_monotonic_s=10.08,
        )
        try:
            runtime._consume_t265(10.05, live_time=True)
            event = next(item for item in logger.events if item["type"] == "t265_pose")
            self.assertEqual(event["raw_angular_velocity_xyz"], (0.4, 0.5, 0.6))
            self.assertEqual(event["t265_mapped_measurement_time_before_causal_clamp"], 10.08)
            self.assertEqual(event["t265_measurement_time_used"], 10.05)
            self.assertTrue(event["measurement_time_clamped_to_receive"])
            self.assertAlmostEqual(event["measurement_to_receive_ms"], -30.0)
        finally:
            runtime.close()

    def test_rejected_t265_log_keeps_raw_quaternion_and_angular_velocity(self) -> None:
        class CaptureLogger:
            def __init__(self):
                self.events = []

            def emit(self, event, **_kwargs):
                self.events.append(event)

            def close(self):
                pass

        runtime = self.make_runtime()
        logger = CaptureLogger()
        runtime.event_logger = logger
        runtime.start()
        runtime.t265_source.read = lambda: T265RawPose(
            translation_xyz=(1.0, 2.0, 3.0),
            quaternion_xyzw=(0.1, 0.2, 0.3, 0.9),
            velocity_xyz=(0.0, 0.0, 0.0),
            angular_velocity_xyz=(0.0, 0.0, 0.25),
            tracker_confidence=0,
            mapper_confidence=1,
            device_timestamp_ms=10000.0,
            received_monotonic_s=10.0,
            measurement_monotonic_s=10.0,
        )
        try:
            runtime._consume_t265(10.0, live_time=False)
            event = next(item for item in logger.events if item["type"] == "t265_rejected")
            self.assertEqual(event["raw_quaternion_xyzw"], (0.1, 0.2, 0.3, 0.9))
            self.assertEqual(event["raw_angular_velocity_xyz"], (0.0, 0.0, 0.25))
        finally:
            runtime.close()

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
