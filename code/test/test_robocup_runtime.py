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
            anchor_initialized=True)
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

    def test_distance_slam_lateral_correction_does_not_steer_or_block(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            runtime.step(now_s=self.now[0])
            result = self.distance_reference_step(runtime, (.06, 0., 0.), (10.06, 20.03, 0.))
            control_pose = step.call_args.args[0]
        self.assertAlmostEqual(control_pose.x_m, 10.06)
        self.assertAlmostEqual(control_pose.y_m, 20.)
        self.assertEqual(result.motion.state, MotionActionState.RUNNING)
        self.assertAlmostEqual(result.motion.diagnostics["cross_track_error_m"], 0.)
        self.assertAlmostEqual(result.command.angular_z_rad_s, 0.)
        self.assertIs(result.estimate, self.estimate)  # Global logging/mission pose remains fused.
        self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "t265_local_fixed")

    def test_distance_real_local_lateral_motion_still_blocks(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        result = self.distance_reference_step(runtime, (.06, .031, 0.), (10.06, 20.031, 0.))
        self.assertAlmostEqual(result.motion.diagnostics["cross_track_error_m"], .031)
        self.assertEqual(result.motion.diagnostics["reason"], "straight_distance_lateral_error_exceeded")
        self.assertEqual(result.mission_state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(result.command, Twist2D(0., 0.))
        self.assertIsNone(runtime._distance_pose_mode)
        self.assertIsNone(runtime._distance_control_T_t265)
        self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "t265_local_fixed")

    def test_distance_gradual_slam_blend_never_changes_fixed_reference(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        fixed = runtime._distance_control_T_t265
        offsets = (0., .0107, .0177, .0223, .0253, .0273, .0305)
        for i, offset in enumerate(offsets, 1):
            result = self.distance_reference_step(runtime, (.01*i, 0., 0.), (10.+.01*i, 20.+offset, 0.))
            self.assertIs(runtime._distance_control_T_t265, fixed)
            self.assertAlmostEqual(result.motion.diagnostics["cross_track_error_m"], 0.)
            self.assertAlmostEqual(result.command.angular_z_rad_s, 0.)
            self.assertEqual(result.motion.state, MotionActionState.RUNNING)

    def test_distance_fixed_alignment_handles_map_rotation_and_saved_heading(self):
        start = Pose2D(10., 20., math.pi/2, 10.)
        runtime = self.make_distance_reference_runtime(fused=start, local=Pose2D(2., 3., 0., 10.))
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03, heading_yaw_rad=math.pi/2)
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            runtime.step(now_s=self.now[0])
            initial = step.call_args.args[0]
            self.assertAlmostEqual(initial.x_m, start.x_m)
            self.assertAlmostEqual(initial.y_m, start.y_m)
            self.assertAlmostEqual(initial.yaw_rad, start.yaw_rad)
            result = self.distance_reference_step(runtime, (2.06, 3., 0.), (10.03, 20.06, 1.8))
            control = step.call_args.args[0]
        self.assertAlmostEqual(control.x_m, 10.)
        self.assertAlmostEqual(control.y_m, 20.06)
        self.assertAlmostEqual(control.yaw_rad, math.pi/2)
        self.assertAlmostEqual(result.motion.diagnostics["cross_track_error_m"], 0.)
        self.assertAlmostEqual(result.command.angular_z_rad_s, 0.)

    def test_non_distance_actions_keep_fused_pose_and_clear_reference(self):
        actions = (("rotate", (.5,)), ("rotate_to", (.5,)),
                   ("follow_segment", ((10., 20.), (11., 20.))),
                   ("navigate_to", (11., 20.)), ("navigate_to_pose", (11., 20., .5)))
        for method, args in actions:
            with self.subTest(action=method):
                runtime = self.make_distance_reference_runtime()
                runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
                runtime.step(now_s=self.now[0])
                runtime.motion.stop()
                getattr(runtime.motion, method)(*args)
                with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
                    self.distance_reference_step(runtime, (.06, 0., 0.), (10.06, 20.03, .1))
                    self.assertIs(step.call_args.args[0], self.estimate.pose)
                self.assertIsNone(runtime._distance_pose_mode)
                self.assertIsNone(runtime._distance_control_T_t265)
                self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "fused")

    def test_consecutive_distances_realign_after_success(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.07, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        fixed = runtime._distance_control_T_t265
        done = self.distance_reference_step(runtime, (.07, 0., 0.), (10.07, 20.03, 0.))
        self.assertEqual(done.motion.state, MotionActionState.SUCCEEDED)
        self.assertIsNone(runtime._distance_pose_mode)
        self.assertIsNone(runtime._distance_control_T_t265)
        self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "t265_local_fixed")
        runtime.mission.on_payload_action_done()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            self.distance_reference_step(runtime, (.07, 0., 0.), (11.07, 21.03, .1))
            self.assertAlmostEqual(step.call_args.args[0].x_m, 11.07)
            self.assertAlmostEqual(step.call_args.args[0].y_m, 21.03)
        self.assertNotEqual(runtime._distance_control_T_t265, fixed)

    def test_cancel_and_replace_distance_before_runtime_tick_realigns(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.07, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        runtime.motion.stop()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            self.distance_reference_step(runtime, (.02, 0., 0.), (12.02, 23., .2))
            self.assertAlmostEqual(step.call_args.args[0].x_m, 12.02)
            self.assertAlmostEqual(step.call_args.args[0].y_m, 23.)

    def test_local_distance_pose_loss_stops_without_fused_fallback(self):
        runtime = self.make_distance_reference_runtime()
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            result = self.distance_reference_step(runtime, None, (10.06, 20.03, 0.))
            self.assertIsNone(step.call_args.args[0])
        self.assertEqual(result.motion.state, MotionActionState.POSE_LOST)
        self.assertEqual(result.command, Twist2D(0., 0.))
        self.assertEqual(result.mission_state, RobocupMissionState.SAFE_STOP)
        self.assertIsNone(runtime._distance_pose_mode)
        self.assertIsNone(runtime._distance_control_T_t265)

    def test_distance_fused_start_keeps_fused_when_t265_appears(self):
        runtime = self.make_distance_reference_runtime()
        self.local = None
        runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
        runtime.step(now_s=self.now[0])
        self.assertEqual(runtime._distance_pose_mode, "fused")
        with patch.object(runtime.motion, "step", wraps=runtime.motion.step) as step:
            self.distance_reference_step(runtime, (.06, 0., 0.), (10.06, 20.01, 0.))
            self.assertIs(step.call_args.args[0], self.estimate.pose)
        self.assertEqual(runtime._distance_pose_mode, "fused")
        self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "fused")

    def test_distance_forward_reverse_endpoint_residuals_still_stop_without_pivot(self):
        for distance in (.47, -.47):
            with self.subTest(distance=distance):
                runtime = self.make_distance_reference_runtime()
                runtime.motion.drive_distance(distance, lateral_tolerance_m=.03, heading_yaw_rad=0.)
                runtime.step(now_s=self.now[0])
                result = self.distance_reference_step(runtime, (distance, .02, 0.),
                                                      (10.+distance, 20.05, 0.))
                self.assertEqual(result.motion.state, MotionActionState.SUCCEEDED)
                self.assertEqual(result.command, Twist2D(0., 0.))

    def test_distance_terminal_outputs_clear_reference_and_log_used_frame(self):
        for state in (MotionActionState.SUCCEEDED, MotionActionState.BLOCKED,
                      MotionActionState.POSE_LOST, MotionActionState.SAFE_STOPPED, MotionActionState.ERROR):
            with self.subTest(state=state):
                runtime = self.make_distance_reference_runtime()
                runtime.motion.drive_distance(.47, lateral_tolerance_m=.03)
                output = MotionOutput(Twist2D(0., 0.), state, MotionActionType.DRIVE_DISTANCE,
                                      MotionPhase.DONE, {"reason": "test terminal"})
                with patch.object(runtime.motion, "step", return_value=output):
                    runtime.step(now_s=self.now[0])
                self.assertIsNone(runtime._distance_pose_mode)
                self.assertIsNone(runtime._distance_control_T_t265)
                self.assertEqual(self.last_motion_event(runtime)["pose_reference"], "t265_local_fixed")

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
