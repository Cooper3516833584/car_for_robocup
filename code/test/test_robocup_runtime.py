from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, PropertyMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.basic_motion_controller import MotionActionState, MotionActionType, MotionOutput, MotionPhase
from components.navigation_common import NavigationGoal
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.t265_driver import T265RawPose
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, Twist2D
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

    def make_distance_reference_runtime(self, *, fused=None, local=None):
        runtime = self.make_runtime(count=0)
        self.addCleanup(runtime.close)
        self.local = local or Pose2D(0., 0., 0., self.now[0])
        self.estimate = FusedPoseEstimate(
            fused or Pose2D(10., 20., 0., self.now[0]), PoseFusionState.OK,
            0., ("t265", "slam", "fused"), 0., 0., None, None, True,
            anchor_initialized=True, t265_confidence=1.0)
        local_patch = patch.object(PoseFusion, "continuous_t265_pose", new_callable=PropertyMock)
        local_property = local_patch.start()
        self.addCleanup(local_patch.stop)
        local_property.side_effect = lambda: self.local
        runtime._consume_t265 = Mock()
        runtime._consume_d500 = Mock()
        runtime.fusion.estimate = lambda _now: self.estimate
        runtime.event_logger = Mock()
        runtime.start()
        return runtime

    def distance_reference_step(self, runtime, local, fused):
        self.now[0] += .05
        self.local = None if local is None else Pose2D(*local, self.now[0])
        self.estimate = replace(self.estimate, pose=Pose2D(*fused, self.now[0]))
        return runtime.step(now_s=self.now[0])

    def last_motion_event(self, runtime):
        return [call.args[0] for call in runtime.event_logger.emit.call_args_list
                if call.args[0]["type"] == "motion_action"][-1]

    def test_local_line_ignores_slam_anchor_correction(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.track_local_line((0,0),(.47,0))
        runtime.step(now_s=self.now[0])
        result = self.distance_reference_step(runtime, (.06,0,0), (10.06,20.03,.1))
        self.assertAlmostEqual(result.motion.diagnostics["cross_track_m"],0)
        self.assertEqual(result.command.angular_z_rad_s,0)
        self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "t265_local")

    def test_real_local_drift_is_corrected_without_blocking(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.track_local_line((0,0),(.47,0))
        runtime.step(now_s=self.now[0])
        result = self.distance_reference_step(runtime, (.06,.05,0), (10.06,20.05,0))
        self.assertEqual(result.motion.state,MotionActionState.RUNNING)
        self.assertGreater(result.command.linear_x_m_s,0)
        self.assertLess(result.command.angular_z_rad_s,0)

    def test_local_missing_stale_confidence_and_continuity_fail_closed(self):
        for reason in ('missing','stale','confidence','continuity','future'):
            with self.subTest(reason=reason):
                runtime=self.make_distance_reference_runtime()
                runtime.motion.track_local_line((0,0),(.47,0))
                runtime.step(now_s=self.now[0])
                if reason == 'missing': self.local=None
                if reason == 'stale': self.local=replace(self.local,timestamp_s=self.now[0]-1)
                if reason == 'future': self.local=replace(self.local,timestamp_s=self.now[0]+1)
                if reason == 'confidence': self.estimate=replace(self.estimate,t265_confidence=0)
                if reason == 'continuity': self.estimate=replace(self.estimate,t265_continuity_broken=True)
                result=runtime.step(now_s=self.now[0])
                self.assertEqual(result.command,Twist2D(0,0))
                self.assertEqual(result.mission_state,RobocupMissionState.SAFE_STOP)

    def test_global_line_alignment_rejects_invalid_local_settle_input(self):
        for reason in ("missing", "stale", "confidence", "continuity"):
            with self.subTest(reason=reason):
                runtime = self.make_distance_reference_runtime(
                    fused=Pose2D(10,20,math.pi/2,self.now[0]),
                    local=Pose2D(0,0,math.pi/2,self.now[0]))
                runtime.motion.track_global_line((10,20),(11,20))
                initial = runtime.step(now_s=self.now[0])
                self.assertEqual(initial.motion.diagnostics["line_state"], "align_forward")
                if reason == "missing": self.local = None
                if reason == "stale": self.local = replace(self.local,timestamp_s=self.now[0]-1)
                if reason == "confidence": self.estimate = replace(self.estimate,t265_confidence=0)
                if reason == "continuity": self.estimate = replace(self.estimate,t265_continuity_broken=True)
                result = runtime.step(now_s=self.now[0])
                self.assertEqual(result.command,Twist2D(0,0))
                self.assertEqual(result.motion.state,MotionActionState.POSE_LOST)
                self.assertEqual(result.mission_state,RobocupMissionState.SAFE_STOP)

    def test_global_actions_use_fused_and_local_turns_use_t265(self):
        for name,args,local in [('track_global_line',((10,20),(11,20)),False),
                               ('navigate_to',(11,20),False),('rotate_to',(.5,),False),
                               ('rotate_local_to',(.5,),True),('rotate',(.5,),True)]:
            runtime=self.make_distance_reference_runtime()
            getattr(runtime.motion,name)(*args)
            runtime._current_step_s=self.now[0]
            self.assertIs(runtime._motion_control_pose(self.estimate),self.local if local else self.estimate.pose)

    def test_cancel_and_resume_reprojects_original_local_line(self):
        runtime=self.make_distance_reference_runtime()
        runtime.motion.track_local_line((0,0),(.47,0))
        runtime.step(now_s=self.now[0])
        runtime.motion.stop()
        runtime.motion.track_local_line((0,0),(.47,0))
        result=self.distance_reference_step(runtime,(.2,0,0),(12,23,.2))
        self.assertAlmostEqual(result.motion.diagnostics['progress_m'],.2)

    def test_terminal_lateral_error_stops_mission(self):
        runtime=self.make_distance_reference_runtime()
        runtime.motion.track_local_line((0,0),(.47,0))
        runtime.step(now_s=self.now[0])
        result=self.distance_reference_step(runtime,(.47,.04,0),(10.47,20.04,0))
        self.assertEqual(result.command,Twist2D(0,0))
        self.assertEqual(result.mission_state,RobocupMissionState.SAFE_STOP)

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
        runtime.motion.track_global_line((0,0),(1,0))
        runtime.start()
        try:
            result = runtime.step(now_s=10.0)
            self.assertIsNone(result.navigation)
            self.assertIsNotNone(result.motion)
            self.assertEqual(result.motion.action_type.value, "track_global_line")
            self.assertGreater(result.command.linear_x_m_s, 0.0)
            self.assertIsNone(runtime.navigator.goal)
        finally:
            runtime.close()

    def test_direct_motion_completion_advances_mission(self) -> None:
        runtime = self.make_runtime()
        runtime.motion.track_global_line((-.1,0),(0,0))
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
