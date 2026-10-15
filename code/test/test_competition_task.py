"""Hardware-free route acceptance using the production motion/runtime loop."""

from contextlib import ExitStack
from pathlib import Path
import math
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch, PropertyMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
from components.basic_motion_controller import MotionActionState
from components.pose_fusion import FusedPoseEstimate, PoseFusionState, PoseFusion
from components.task_board_reader import TaskCounts, TaskBoardResult
from components.yellow_yolo_adapter import YellowDetection
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from main_robocup import main
from robocup_runtime import RobocupMissionState, build_runtime, load_runtime_config


class CompetitionTaskTests(unittest.TestCase):
    def test_assignment_removes_exactly_one_single_item_color_from_drone(self):
        for source,color,expected in (((2,1,1),'blue',(2,0,1)),
                                       ((1,2,1),'red',(0,2,1)),
                                       ((0,3,1),'green',(0,3,0))):
            with self.subTest(source=source):
                selected,counts=task.split_task_allocation(TaskCounts(*source))
                self.assertEqual(selected,color)
                self.assertEqual(tuple(counts.as_dict().values()),expected)
                self.assertEqual(sum(counts.as_dict().values()),3)
        for source in ((4,0,0),(2,2,0)):
            with self.assertRaisesRegex(ValueError,'single-item'):
                task.split_task_allocation(TaskCounts(*source))

    def test_unconfigured_payload_skips_both_releases_and_finishes_route(self):
        self.stack.enter_context(patch.object(task,'payload_configured',return_value=False))
        self.stack.enter_context(patch.object(task,'read_task_board',return_value=TaskCounts(2,1,1)))
        send=self.stack.enter_context(patch.object(task,'send_task_to_drone_once',return_value=True))
        search=self.stack.enter_context(patch.object(task,'search_centered_yellow_on_line',return_value=Mock()))
        task.run_full_mission(self.runtime,detector=Mock())
        self.assertEqual([c.kwargs['color'] for c in search.call_args_list],['yellow','blue'])
        send.assert_called_once_with(task.DroneAllocation(2,0,1),runtime=self.runtime)
        self.open.assert_not_called()
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)

    def setUp(self):
        self.now = 10.0
        self.xy_yaw = [0.0, 0.0, 0.0]
        self.lost = False
        self.sleep_count = 0
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN,
                                     clock=lambda: self.now)
        self.runtime.start()
        self.addCleanup(self.runtime.close)
        # Fake canonical pose observations, propagated from the fake drive.
        # All production fusion/motion/drive algorithms remain unmodified.
        self.runtime._consume_t265 = lambda *_args, **_kwargs: None
        self.runtime._consume_d500 = lambda *_args, **_kwargs: None
        self.runtime.fusion.estimate = self.estimate
        self.runtime.record_event = Mock()
        self.stack = ExitStack()
        self.stack.enter_context(patch.object(task,"payload_configured",return_value=True))
        camera_servo = Mock()
        camera_servo.is_running = False
        self.stack.enter_context(patch("config.v2_factory.build_servo", return_value=camera_servo))
        self.open = self.stack.enter_context(patch("payload_detour.open_payload"))
        self.stack.enter_context(patch("main_robocup.open_payload"))
        self.stack.enter_context(patch.object(task, "open_payload"))
        self.addCleanup(self.stack.close)
        # No test may open a real 0 deg camera: the fake frame reader is used by
        # the visual alignment fallback paths, which is the same behaviour as a
        # missing field camera.
        frame_reader = Mock()
        frame_reader.read.return_value = (False, None)
        self.stack.enter_context(patch.object(task, "_open_yellow_camera",
                                              return_value=(frame_reader, False)))
        local = self.stack.enter_context(patch.object(PoseFusion, "continuous_t265_pose", new_callable=PropertyMock))
        local.side_effect = lambda: None if self.lost else Pose2D(*self.xy_yaw, self.now)
        self.stack.enter_context(patch.object(task.time, "sleep", self.advance))
        # Keep the shortened search endpoint inside the shortened corner segment.
        # Otherwise the fixture physically overshoots corner 2 before rejoining.
        for name, value in {"LANE_ENTRY_X": 0.10, "TASK_BOARD_X": 0.20,
                            "CORNER_1_X": 0.30, "YELLOW_SEARCH_START_X": 0.35,
                            "YELLOW_SEARCH_END_X": 0.40,
                            "CORNER_2_X": 0.45, "FINISH_X": 0.55}.items():
            self.stack.enter_context(patch.object(task, name, value))

    def estimate(self, now):
        pose = None if self.lost else Pose2D(*self.xy_yaw, now)
        return FusedPoseEstimate(pose, PoseFusionState.LOST if self.lost else PoseFusionState.OK,
                                 0.0, ("test",), None if self.lost else 0.0,
                                 None if self.lost else 0.0, None, None, True,
                                 anchor_initialized=True, t265_confidence=1.)

    def advance(self, dt):
        self.sleep_count += 1
        if self.sleep_count > 10000:
            self.fail("simulation did not terminate")
        twist = self.runtime.drive.last_limited_twist
        yaw = self.xy_yaw[2] + twist.angular_z_rad_s * dt / 2
        self.xy_yaw[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.xy_yaw[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.xy_yaw[2] += twist.angular_z_rad_s * dt
        self.now += dt

    def test_consecutive_real_actions_resume_ready_and_stop(self):
        task.move_local_distance(self.runtime, 0.12)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.TARGET_OPERATION)
        first_x = self.xy_yaw[0]
        task.move_local_distance(self.runtime, -0.04)
        self.assertLess(self.xy_yaw[0], first_x - 0.025)
        self.assertEqual(self.runtime.motion.state, MotionActionState.SUCCEEDED)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)

    def test_two_centimetre_move_is_not_swallowed_by_route_tolerance(self):
        original = self.runtime.motion.navigation
        task.move_local_distance(self.runtime, 0.02)
        self.assertGreater(self.xy_yaw[0], 0.015)
        self.assertIs(self.runtime.motion.navigation, original)

    def test_fixed_route_uses_existing_rotate_actions(self):
        self.stack.enter_context(patch.object(task, "FIXED_DROP_ROUTE", [
            ("drive", 0.10), ("rotate", 20), ("rotate_to", 0)]))
        task.run_fixed_drop_route(self.runtime)
        self.assertGreater(self.xy_yaw[0], 0.08)
        self.assertLess(abs(self.xy_yaw[2]), math.radians(3))

    def test_transient_pose_loss_recovers_during_motion(self):
        initial_advance = self.advance
        elapsed = [0.0]

        def advance(dt):
            initial_advance(dt)
            elapsed[0] += dt
            self.lost = 0.1 < elapsed[0] < 0.3

        self.stack.enter_context(patch.object(task.time, "sleep", advance))
        task.move_local_distance(self.runtime, 0.10)
        self.assertEqual(self.runtime.motion.state, MotionActionState.SUCCEEDED)

    def test_complete_pose_loss_stops_route(self):
        initial_advance = self.advance

        def advance(dt):
            initial_advance(dt)
            self.lost = True

        self.stack.enter_context(patch.object(task.time, "sleep", advance))
        with self.assertRaises(task.LocalizationLostError):
            task.move_local_distance(self.runtime, 0.20)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)

    def test_startup_localization_timeout(self):
        self.lost = True
        self.stack.enter_context(patch.object(task, "LOCALIZATION_WAIT_S", 0.15))
        with self.assertRaises(task.LocalizationLostError):
            task.wait_for_fused_localization(self.runtime)

    def test_reader_consensus_and_fallback(self):
        counts = TaskCounts(2, 1, 1)
        reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
        reader.recognize_camera.return_value = TaskBoardResult(counts, 0.9, votes=3)
        self.assertEqual(task.read_task_board(reader=reader).key, (2, 1, 1))
        reader.recognize_camera.side_effect = OSError("camera missing")
        self.assertEqual(task.read_task_board(reader=reader).key, (1, 2, 1))
        self.assertNotEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_reader_retries_three_times_then_records_fallback_without_stopping(self):
        reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
        reader.recognize_camera.side_effect = RuntimeError("camera unavailable")
        with patch.object(task, "TASK_BOARD_RETRY_DELAY_S", 0), \
             self.assertLogs("competition_task", level="WARNING") as logs:
            counts = task.read_task_board(reader=reader, runtime=self.runtime)
        self.assertEqual(counts.key, (1, 2, 1))
        self.assertEqual(reader.recognize_camera.call_count, 3)
        self.runtime.record_event.assert_called_with(
            "task_board_counts", red=1, blue=2, green=1, fallback=True,
            reason="camera unavailable")
        self.assertIn("using fallback task", "\n".join(logs.output))
        self.assertNotEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_recognized_task_counts_keep_real_values_and_allocation(self):
        expected = (((2, 1, 1), "blue", (2, 0, 1)),
                    ((1, 2, 1), "red", (0, 2, 1)),
                    ((1, 1, 2), "red", (0, 1, 2)))
        for source, color, drone_quota in expected:
            with self.subTest(source=source):
                counts = TaskCounts(*source)
                reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
                reader.recognize_camera.return_value = TaskBoardResult(
                    counts, 0.95, votes=3)
                actual = task.read_task_board(reader=reader, runtime=self.runtime)
                car_color, drone_counts = task.split_task_allocation(actual)
                self.assertEqual(actual.key, source)
                self.assertEqual(car_color, color)
                self.assertEqual(tuple(drone_counts.as_dict().values()), drone_quota)
                self.runtime.record_event.assert_called_with(
                    "task_board_counts", red=source[0], blue=source[1],
                    green=source[2], fallback=False, votes=3, confidence=0.95)

    def test_search_clips_last_step_and_needs_consecutive_frames(self):
        self.stack.enter_context(patch.object(task, "YELLOW_SEARCH_END_X", 0.54))
        detection = YellowDetection(640, 360, 80, 80, 0.9)
        reads = [detection, None, None, None, None]
        detect = self.stack.enter_context(patch.object(task, "_detect_stopped",
            side_effect=lambda *_: reads.pop(0) if reads else None))
        with patch.object(task, "move_local_distance", wraps=task.move_local_distance) as move:
            self.assertIsNone(task.search_yellow_drop_zone(self.runtime, Mock(), Mock()))
        self.assertGreaterEqual(detect.call_count, 4)
        distances = [c.args[1] for c in move.call_args_list]
        self.assertLessEqual(sum(distances), 0.19 + 1e-9)
        self.assertLessEqual(distances[-1], task.YELLOW_SEARCH_STEP_M)
        self.assertNotEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_alignment_forward_reverse_then_within_tolerance(self):
        detections = iter([YellowDetection(260, 1000, 80, 80, 0.9),
                           YellowDetection(321, 1000, 80, 80, 0.9)])
        self.stack.enter_context(patch.object(task, "_detect_stopped", side_effect=lambda *_: next(detections)))
        with patch.object(task, "move_local_distance", wraps=task.move_local_distance) as move:
            aligned = task.align_yellow_drop_zone(self.runtime, Mock(), Mock(),
                initial_detection=YellowDetection(380, 1000, 80, 80, 0.9))
        self.assertEqual([c.args[1] for c in move.call_args_list], [0.02, -0.02])
        self.assertIsNotNone(aligned)  # cy is not a gate.

    def test_native_frame_pixels_are_normalized_before_alignment(self):
        for shape, native in [((480, 640, 3), YellowDetection(320, 240, 80, 80, .9)),
                              ((720, 1280, 3), YellowDetection(640, 360, 160, 120, .9))]:
            camera, detector = Mock(), Mock()
            frame = SimpleNamespace(shape=shape)
            camera.read.return_value = True, frame
            with patch.object(task, "select_yellow", return_value=native):
                detection = task.detect_yellow_once(camera, detector)
            self.assertEqual(detection, YellowDetection(320, 240, 80, 80, .9))
            with patch.object(task, "move_local_distance") as drive:
                self.assertIs(task.align_yellow_drop_zone(self.runtime, camera, detector,
                                                          initial_detection=detection), detection)
            drive.assert_not_called()

    def test_inference_failure_returns_no_target(self):
        camera, detector = Mock(), Mock()
        camera.read.return_value = True, SimpleNamespace(shape=(480, 640, 3))
        detector.predict.side_effect = RuntimeError("backend failed")
        with self.assertLogs("competition_task", level="ERROR"):
            self.assertIsNone(task.detect_yellow_once(camera, detector))

    def test_alignment_sign_and_max_step_limit(self):
        self.stack.enter_context(patch.object(task, "ALIGN_PIXEL_TO_DRIVE_SIGN", -1))
        self.stack.enter_context(patch.object(task, "ALIGN_MAX_STEPS", 2))
        detection = YellowDetection(800, 360, 80, 80, 0.9)
        self.stack.enter_context(patch.object(task, "_detect_stopped", return_value=detection))
        with patch.object(task, "move_local_distance", wraps=task.move_local_distance) as move:
            self.assertIsNone(task.align_yellow_drop_zone(self.runtime, Mock(), Mock()))
        self.assertEqual([c.args[1] for c in move.call_args_list], [-0.02, -0.02])

    def test_alignment_loss_retries_then_skips(self):
        detect = self.stack.enter_context(patch.object(task, "_detect_stopped", return_value=None))
        self.assertIsNone(task.align_yellow_drop_zone(self.runtime, Mock(), Mock()))
        self.assertEqual(detect.call_count, 3)
        self.assertNotEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_full_route_stops_on_payload_failure_after_confirmed_handoff(self):
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        send = self.stack.enter_context(patch.object(task, "send_task_to_drone_once", return_value=True))
        self.stack.enter_context(patch.object(task, "search_centered_yellow_on_line",
            return_value=YellowDetection(320, 240, 80, 80, 0.9)))
        drop = self.stack.enter_context(patch("payload_detour.open_payload", side_effect=RuntimeError("release failed")))
        with patch.object(task, "go_to_second_corner") as corner:
            with self.assertRaisesRegex(RuntimeError, "release"):
                task.run_full_mission(self.runtime, yellow_camera=Mock(), detector=Mock())
            corner.assert_not_called()
        send.assert_called_once()
        self.assertGreater(drop.call_count, 0)
        self.assertTrue(any(c.args == (0,) for c in drop.call_args_list))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)

    def test_finish_at_xy_does_not_align_yaw(self):
        self.xy_yaw[:] = [task.FINISH_X, task.FINISH_Y, math.pi / 2]
        self.stack.enter_context(patch.object(task, "FINISH_YAW_DEG", -135))
        with patch.object(self.runtime.motion, "rotate_to", wraps=self.runtime.motion.rotate_to) as rotate, \
             patch.object(self.runtime.motion, "navigate_to_pose", wraps=self.runtime.motion.navigate_to_pose) as pose:
            task.go_to_finish(self.runtime)
        rotate.assert_not_called()
        pose.assert_not_called()
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)
        self.assertEqual(self.xy_yaw[2], math.pi / 2)
        self.assertEqual(self.sleep_count, 0)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)

    def test_full_route_uses_measured_lane_segments(self):
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        self.stack.enter_context(patch.object(task, "send_task_to_drone_once", return_value=True))
        with patch.object(task, "track_lane_line", wraps=task.track_lane_line) as follow:
            task.run_full_mission(self.runtime, detector=None)
        self.assertEqual([(c.args[1], c.args[2]) for c in follow.call_args_list], [
            ((0.10, 0.0), (0.30, 0.0)),
            ((0.30, 0.0), (0.35, 0.0)), ((0.35, 0.0), (0.45, 0.0)),
            ((0.45, 0.0), (0.55, 0.0))])
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)

    def test_full_route_can_use_shared_task_sender(self):
        counts = TaskCounts(1, 2, 1)
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=counts))
        old_sender = self.stack.enter_context(patch.object(
            task, "send_task_to_drone_once", side_effect=AssertionError("reopened HC-15")))
        send = Mock(return_value=True)
        task.run_full_mission(self.runtime, detector=None, send_drone_task=send)
        send.assert_called_once_with(task.DroneAllocation(0,2,1))
        old_sender.assert_not_called()
        self.runtime.record_event.assert_any_call("hc_task_send", red=0, blue=2, green=1,
            success=True, protocol="FleetBus", start_confirmed=True, bridge_envelope=True)

    def test_full_route_uses_verified_horizontal_centre_without_reopening_camera(self):
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        self.stack.enter_context(patch.object(task, "send_task_to_drone_once", return_value=True))
        self.stack.enter_context(patch.object(task, "search_centered_yellow_on_line",
            return_value=YellowDetection(320, 360, 80, 80, .9)))
        with patch.object(task,"align_yellow_drop_zone",side_effect=AssertionError("duplicate alignment")), \
             patch.object(task,"perform_payload_detour") as drop:
            task.run_full_mission(self.runtime,detector=Mock())
        self.assertEqual(drop.call_count,2)
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.FINISHED)

    def test_move_to_xy_does_not_set_final_yaw(self):
        with patch.object(self.runtime.motion, "navigate_to", wraps=self.runtime.motion.navigate_to) as navigate:
            task.move_to_xy(self.runtime, 0.10, 0.0)
        navigate.assert_called_once_with(0.10, 0.0)

    def test_second_corner_recovers_to_fixed_lane_after_offset_drop(self):
        self.xy_yaw[:] = [0.38, 0.15, 0.2]
        # Rejoin geometry must remain the measured lane, not current-pose -> goal.
        with patch.object(task, "track_lane_line", return_value=Mock()) as follow:
            task.go_to_second_corner(self.runtime)
        follow.assert_called_once_with(self.runtime, (0.35, 0.0), (0.45, 0.0))

    def test_full_route_without_yellow_skips_drop_and_finishes(self):
        self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        self.stack.enter_context(patch.object(task, "send_task_to_drone_once", return_value=True))
        drop = self.stack.enter_context(patch.object(task, "perform_payload_detour"))
        task.run_full_mission(self.runtime, detector=None)
        drop.assert_not_called()
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)

    def test_standalone_drop_align_sets_zero_degrees_without_releasing(self):
        servo = Mock(is_running=True, pulse_us=1500)
        self.stack.enter_context(patch("config.v2_factory.build_servo", return_value=servo))
        from components import drop_target_vision as vision
        before=self.open.call_count
        camera=Mock()
        camera.read.return_value=(True,object())
        with patch.object(task,"_open_yellow_camera",return_value=(camera,False)), \
             patch.object(vision,"extract_target_arc",return_value=(300.,300.,400.)), \
             patch.object(self.runtime.motion,"rotate_to") as rotate, \
             patch.object(self.runtime.motion,"track_global_line") as move:
            result=task.run_competition_stage(self.runtime,"drop-align",yellow_camera="/dev/video9")
        self.assertEqual(result,(300.,300.,400.))
        rotate.assert_not_called()
        move.assert_not_called()
        servo.set_angle.assert_called_once_with(0,settle=True)
        servo.close.assert_called_once_with(hold=True)
        self.assertEqual(self.open.call_count,before)

    def test_main_stage_drop_align_uses_the_alignment_result(self):
        for result, expected in ((None, 1), (Mock(), 0)):
            with self.subTest(found=result is not None), \
                 patch("main_robocup.build_runtime", return_value=self.runtime), \
                 patch.object(task, "run_competition_stage", return_value=result) as run:
                self.assertEqual(main(["--mode", "hardware-mission",
                                       "--competition-stage", "drop-align"]), expected)
                self.assertEqual(run.call_args.args, (self.runtime, "drop-align"))

    def test_main_uses_dual_servo_without_magnet_slot(self):
        with patch("main_robocup.build_runtime", return_value=self.runtime), patch.object(task,"run_competition_stage",return_value=True) as run:
            main(["--mode","hardware-mission","--competition"])
        self.assertNotIn("slot",run.call_args.kwargs)

    def test_main_can_run_in_listener_worker_without_signal_registration(self):
        published = []
        results = []
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch.object(task, "run_competition_stage", return_value=True):
            worker = threading.Thread(target=lambda: results.append(main(
                ["--mode", "hardware-mission", "--competition-stage", "lane"],
                stop_event=threading.Event(), on_runtime_ready=published.append)))
            worker.start()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [0])
        self.assertEqual(published, [self.runtime, None])

    def test_standalone_drop_does_not_start_full_route(self):
        with patch.object(task,"open_payload") as drop, patch.object(task,"run_full_mission") as full:
            task.run_competition_stage(self.runtime,"drop")
        drop.assert_called_once_with()
        full.assert_not_called()

    def test_standalone_task_board_does_not_navigate(self):
        reader = self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        go = self.stack.enter_context(patch.object(task, "go_to_task_board"))
        task.run_competition_stage(self.runtime, "task-board", task_board_camera="/dev/camera")
        reader.assert_called_once_with("/dev/camera", runtime=self.runtime)
        go.assert_not_called()

    def test_main_motion_stage_routes_camera_without_legacy_turn_and_always_closes(self):
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch.object(task, "run_competition_stage", return_value=True) as run:
            self.assertEqual(main(["--mode", "hardware-mission", "--competition-stage", "lane",
                                   "--task-board-camera", "1"]), 0)
        self.assertEqual(run.call_args.args, (self.runtime, "lane"))
        self.assertEqual(run.call_args.kwargs["task_board_camera"], 1)
        self.assertFalse(self.runtime.is_running)

    def test_main_rejects_conflicting_goal_and_nonhardware_modes(self):
        with patch("main_robocup.build_runtime") as build:
            self.assertEqual(main(["--competition"]), 2)
            self.assertEqual(main(["--mode", "hardware-mission", "--competition",
                                   "--goal-x", "0", "--goal-y", "0"]), 2)
            build.assert_not_called()

    def test_main_standalone_search_and_align_codes_use_result(self):
        detection = YellowDetection(640, 360, 80, 80, 0.9)
        for stage in ("yellow-search", "yellow-align"):
            for result, expected in ((None, 1), (detection, 0)):
                with self.subTest(stage=stage, found=result is not None), \
                     patch("main_robocup.build_runtime", return_value=self.runtime), \
                     patch.object(task, "run_competition_stage", return_value=result):
                    self.assertEqual(main(["--mode", "hardware-mission", "--competition-stage", stage]), expected)

    def test_main_runtime_safety_and_route_exception_return_one(self):
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch.object(task, "run_competition_stage", side_effect=RuntimeError("motion failed")):
            self.assertEqual(main(["--mode", "hardware-mission", "--competition-stage", "drop-route"]), 1)
        self.assertFalse(self.runtime.is_running)
        self.runtime.mission.request_safe_stop("runtime safety")
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch.object(task, "run_competition_stage", return_value=YellowDetection(640, 360, 80, 80, 0.9)):
            self.assertEqual(main(["--mode", "hardware-mission", "--competition-stage", "yellow-search"]), 1)

    def _run_full_cli(self, *, hc_ok=True, found=True, align_ok=True, drop_ok=True, ocr_ok=True,
                      expected_code=0):
        self.stack.enter_context(patch.object(task, "load_detector", return_value=Mock()))
        self.stack.enter_context(patch.object(task, "_open_yellow_camera", return_value=(Mock(), False)))
        self.stack.enter_context(patch.object(task, "search_centered_yellow_on_line", return_value=(
            YellowDetection(320, 240, 80, 80, 0.9) if found else None)))
        self.handoff = self.stack.enter_context(
            patch.object(task, "send_task_and_start", return_value=hc_ok))
        def detour(runtime, **_kwargs):
            if not drop_ok:
                runtime.mission.request_safe_stop("payload release was not confirmed")
                raise RuntimeError("payload release was not confirmed")
            return self.estimate(self.now).pose
        drop = self.stack.enter_context(patch.object(task, "perform_payload_detour", side_effect=detour))
        if not align_ok:
            self.stack.enter_context(patch.object(task, "align_yellow_drop_zone", return_value=None))
        if ocr_ok:
            self.stack.enter_context(patch.object(task, "read_task_board", return_value=TaskCounts(1, 2, 1)))
        else:
            reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
            reader.recognize_camera.side_effect = OSError("OCR camera unavailable")
            self.stack.enter_context(patch("components.task_board_reader.TaskBoardReader", return_value=reader))
        with patch("main_robocup.build_runtime", return_value=self.runtime):
            code = main(["--mode", "hardware-mission", "--competition"])
        self.assertEqual(code, expected_code)
        self.assertEqual(self.runtime.mission.state,
                         RobocupMissionState.FINISHED if expected_code == 0 else RobocupMissionState.SAFE_STOP)
        self.assertFalse(self.runtime.is_running)
        return drop

    def test_full_cli_hc_failure_stops_before_next_stage(self):
        self._run_full_cli(hc_ok=False, expected_code=1).assert_not_called()

    def test_full_cli_yellow_missing_still_returns_success(self):
        self._run_full_cli(found=False).assert_not_called()

    def test_full_cli_does_not_repeat_legacy_alignment_after_search(self):
        self.assertEqual(self._run_full_cli(align_ok=False).call_count,2)

    def test_full_cli_drop_failure_stops_and_returns_failure(self):
        drop = self._run_full_cli(drop_ok=False, expected_code=1)
        # The competition default is the middle CH3 (slot 2), never CH2.
        self.assertEqual(drop.call_args.args, (self.runtime,))

    def test_full_mission_never_accesses_legacy_relay(self):
        self.runtime.relay = Mock()
        self.stack.enter_context(patch.object(task,"read_task_board",return_value=TaskCounts(1,2,1)))
        self.stack.enter_context(patch.object(task,"send_task_to_drone_once",return_value=True))
        self.stack.enter_context(patch.object(task,"search_centered_yellow_on_line",return_value=Mock()))
        task.run_competition_stage(self.runtime,"full")
        self.runtime.relay.assert_not_called()
        self.assertEqual(self.runtime.relay.method_calls,[])
        self.assertEqual([c.args for c in self.open.call_args_list],[(0,),(1,)])

    def test_shared_sender_failure_prevents_post_ocr_motion(self):
        self.stack.enter_context(patch.object(task,"read_task_board",return_value=TaskCounts(1,2,1)))
        self.stack.enter_context(patch.object(task,"search_centered_yellow_on_line",return_value=None))
        send=Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError,"drone task/start"):
            task.run_full_mission(self.runtime,send_drone_task=send)
        send.assert_called_once()
        self.assertEqual(self.runtime.mission.state,RobocupMissionState.SAFE_STOP)

    def test_full_cli_ocr_failure_uses_fallback_and_continues_handoff(self):
        with self.assertLogs("competition_task", level="WARNING") as logs:
            self._run_full_cli(ocr_ok=False).assert_called()
        self.assertEqual(self.runtime.mission.task_counts.key, (1, 2, 1))
        self.assertTrue(any("using fallback task" in message for message in logs.output))
        self.runtime.record_event.assert_any_call(
            "task_board_counts", red=1, blue=2, green=1, fallback=True,
            reason="OCR camera unavailable")
        self.runtime.record_event.assert_any_call(
            "payload_target_assignment", car_color="red", car_mandatory_yellow=1,
            car_target_required=1, drone_allocation={"red": 0, "blue": 2, "green": 1},
            drone_mandatory_yellow=1)
        self.assertEqual(self.handoff.call_args.args[0].as_dict(),
                         {"red": 0, "blue": 2, "green": 1})

    def test_full_cli_does_not_report_success_before_finish(self):
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch.object(task, "run_competition_stage", return_value=None):
            self.assertEqual(main(["--mode", "hardware-mission", "--competition"]), 1)


if __name__ == "__main__":
    unittest.main()
