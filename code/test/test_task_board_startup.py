from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch, PropertyMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.basic_motion_controller import MotionActionState
from components.pose_fusion import FusedPoseEstimate, PoseFusionState, PoseFusion
from components.task_board_reader import TaskBoardConfig, TaskBoardResult, TaskCounts
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from main_robocup import main
from robocup_runtime import RobocupMissionState, build_runtime
from task_board_startup import acquire_task_board


class TaskBoardStartupTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.runtime = build_runtime(load_v2_config(), RuntimeMode.REPLAY, clock=lambda: self.now)
        self.addCleanup(self.runtime.close)
        self.events = []
        self.runtime.event_logger = SimpleNamespace(
            emit=lambda event, **kwargs: self.events.append(event), close=lambda: None,
        )
        self.turn_steps = 0
        self.never_ready = False
        self.never_turn = False
        self.lose_pose = False
        self.pose_patch = patch.object(self.runtime.fusion, "estimate", side_effect=self.estimate)
        self.pose_patch.start()
        self.addCleanup(self.pose_patch.stop)
        local_patch=patch.object(PoseFusion,"continuous_t265_pose",new_callable=PropertyMock)
        local=local_patch.start()
        self.addCleanup(local_patch.stop)
        local.side_effect=lambda: None if self.never_ready or self.lose_pose else Pose2D(
            0,0,min(max(0,self.turn_steps-1)*math.pi/4,math.pi/2),self.now)
        self.reader = SimpleNamespace(
            config=TaskBoardConfig(), recognize_camera=Mock(side_effect=self.read),
        )

    def sleep(self, seconds):
        self.now += seconds

    def estimate(self, now):
        yaw = 0.0
        if self.runtime.motion.state is MotionActionState.RUNNING:
            yaw = min(self.turn_steps * math.pi / 4, math.pi / 2)
            if not self.never_turn:
                self.turn_steps += 1
        elif self.turn_steps:
            yaw = math.pi / 2
        lost = self.never_ready or self.lose_pose
        return FusedPoseEstimate(
            pose=None if lost else Pose2D(0, 0, yaw, now),
            state=PoseFusionState.LOST if lost else PoseFusionState.OK,
            age_s=0, source_flags=("fused",), t265_age_s=0, d500_age_s=0,
            last_d500_innovation_m=0, last_d500_innovation_yaw_rad=0, d500_accepted=True,
            anchor_initialized=True, t265_confidence=1.,
        )

    def assert_stopped(self):
        twist = self.runtime.drive.last_limited_twist
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0, 0))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.TARGET_OPERATION)

    def read(self, camera):
        self.assertEqual(camera, "/dev/v4l/by-id/task-camera")
        self.assert_stopped()
        return TaskBoardResult(TaskCounts(0, 3, 1), 0.94, ("红0蓝3绿1",), votes=3)

    def acquire(self, **kwargs):
        return acquire_task_board(self.runtime, self.reader, camera="/dev/v4l/by-id/task-camera",
                                  turn_rad=math.pi / 2, sleep=self.sleep, **kwargs)

    def test_turn_stop_read_store_and_log(self):
        result = self.acquire()
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(self.runtime.mission.task_counts, TaskCounts(0, 3, 1))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.READY)
        self.assertGreaterEqual(self.turn_steps, 3)
        self.reader.recognize_camera.assert_called_once()
        events = [event for event in self.events if event["type"] == "task_board_recognition"]
        self.assertEqual(len(events), 1)
        self.assertEqual((events[0]["red"], events[0]["blue"], events[0]["green"]), (0, 3, 1))
        self.assertEqual(events[0]["votes"], 3)
        self.assertGreater(events[0]["turn_time_ms"], 0)

    def assert_fallback(self, result, reason):
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.counts, TaskCounts(1, 2, 1))
        self.assertEqual(self.runtime.mission.task_counts, TaskCounts(1, 2, 1))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.READY)
        self.assertEqual(result.source, "fallback_1_2_1")
        self.assertEqual((result.confidence, result.votes), (0.0, 0))
        self.assertIn(reason, result.reason)
        twist = self.runtime.drive.last_limited_twist
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0, 0))
        events = [event for event in self.events if event["type"] == "task_board_recognition_fallback"]
        self.assertEqual(len(events), 1)
        self.assertTrue(events[0]["fallback"])
        self.assertEqual((events[0]["red"], events[0]["blue"], events[0]["green"]), (1, 2, 1))
        self.assertFalse(any(event["type"] in {"task_board_startup_fatal", "task_board_recognition"}
                             for event in self.events))
        return events[0]

    def assert_fatal(self, result):
        self.assertFalse(result.valid)
        self.assertIsNone(self.runtime.mission.task_counts)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)
        self.assertTrue(any(event["type"] == "task_board_startup_fatal" for event in self.events))
        self.assertFalse(any(event["type"] == "task_board_recognition_fallback" for event in self.events))

    def test_invalid_ocr_uses_fallback_and_continues(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(
            None, 0.37, ("original OCR",), board_quad=((1, 2), (3, 2), (3, 4), (1, 4)),
            source="rectified", reason="empty OCR", votes=2,
        )
        result = self.acquire()
        event = self.assert_fallback(result, "empty OCR")
        self.assertEqual(result.raw_lines, ("original OCR",))
        self.assertIsNotNone(result.board_quad)
        self.assertEqual(event["original_confidence"], 0.37)
        self.assertEqual(event["original_votes"], 2)
        self.assertEqual(event["original_raw_lines"], ("original OCR",))

    def test_ocr_exception_uses_fallback(self):
        self.reader.recognize_camera.side_effect = OSError("camera disconnected")
        result = self.acquire()
        event = self.assert_fallback(result, "OSError: camera disconnected")
        self.assertIsNone(event["original_confidence"])
        self.assertEqual(event["original_votes"], 0)

    def test_only_one_vote_uses_fallback(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(TaskCounts(2, 1, 1), 0.99, votes=1)
        event = self.assert_fallback(self.acquire(), "1/3")
        self.assertEqual(event["original_votes"], 1)

    def test_two_votes_use_fallback(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(TaskCounts(0, 3, 1), 0.99, votes=2)
        self.assert_fallback(self.acquire(), "2/3")

    def test_perception_failure_reasons_use_fallback(self):
        reasons = ("no board", "no usable OCR text", "camera cannot open", "no frames supplied",
                   "missing color counts", "ambiguous count", "conflicting OCR values", "consensus not reached")
        for index, reason in enumerate(reasons):
            with self.subTest(reason=reason):
                if index:
                    self.doCleanups()
                    self.setUp()
                self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(None, 0, reason=reason, votes=0)
                self.assert_fallback(self.acquire(), reason)

    def test_valid_result_never_uses_fallback(self):
        result = self.acquire()
        self.assertEqual(result.counts, TaskCounts(0, 3, 1))
        self.assertNotEqual(result.source, "fallback_1_2_1")
        self.assertFalse(any(event["type"] == "task_board_recognition_fallback" for event in self.events))

    def test_valid_synthetic_counts_are_real_recognition(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(
            TaskCounts(2, 1, 1), 0.95, source="rectified", votes=3,
        )
        result = self.acquire()
        self.assertEqual(result.counts, TaskCounts(2, 1, 1))
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.READY)
        self.assertEqual(result.source, "rectified")
        self.assertFalse(any(event["type"] == "task_board_recognition_fallback" for event in self.events))

    def test_localization_timeout_does_not_open_camera(self):
        self.never_ready = True
        result = self.acquire(timeout_s=0.2)
        self.assert_fatal(result)
        self.assertIn("timed out", result.reason)
        self.reader.recognize_camera.assert_not_called()

    def test_turn_timeout_does_not_open_camera(self):
        self.never_turn = True
        self.assert_fatal(self.acquire(timeout_s=0.25))
        self.reader.recognize_camera.assert_not_called()
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)

    def test_blocked_rotation_does_not_open_camera(self):
        self.runtime.motion.drive = replace(self.runtime.motion.drive, allow_in_place_rotation=False)
        self.assert_fatal(self.acquire())
        self.reader.recognize_camera.assert_not_called()

    def test_keyboard_interrupt_stops_before_propagating(self):
        self.reader.recognize_camera.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.acquire()
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)

    def test_localization_loss_during_ocr_rejects_result(self):
        def read_and_lose(camera):
            result = self.read(camera)
            self.lose_pose = True
            return result
        self.reader.recognize_camera.side_effect = read_and_lose
        self.assert_fatal(self.acquire())
        self.assertIsNone(self.runtime.mission.task_counts)

    def test_localization_loss_after_invalid_ocr_is_fatal(self):
        def read_and_lose(camera):
            self.lose_pose = True
            return TaskBoardResult(None, 0, reason="empty OCR")
        self.reader.recognize_camera.side_effect = read_and_lose
        self.assert_fatal(self.acquire())

    def test_localization_loss_after_ocr_exception_is_fatal(self):
        def read_and_lose(camera):
            self.lose_pose = True
            raise RuntimeError("model failed")
        self.reader.recognize_camera.side_effect = read_and_lose
        self.assert_fatal(self.acquire())

    def test_runtime_error_during_ocr_is_not_masked_by_fallback(self):
        def read_and_fail(camera):
            self.runtime.mission.request_error("backend failed")
            return TaskBoardResult(None, 0, reason="empty OCR")
        self.reader.recognize_camera.side_effect = read_and_fail
        self.assert_fatal(self.acquire())

    def test_localization_loss_during_rotation_does_not_open_camera(self):
        original_estimate = self.estimate
        def estimate_and_lose(now):
            if self.turn_steps >= 1:
                self.lose_pose = True
            return original_estimate(now)
        self.runtime.fusion.estimate.side_effect = estimate_and_lose
        self.assert_fatal(self.acquire(timeout_s=1.0))
        self.reader.recognize_camera.assert_not_called()

    def test_system_exit_stops_before_propagating(self):
        self.reader.recognize_camera.side_effect = SystemExit(3)
        with self.assertRaises(SystemExit):
            self.acquire()
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)

    def test_invalid_parameters_are_fatal(self):
        self.assert_fatal(self.acquire(timeout_s=-1))
        self.reader.recognize_camera.assert_not_called()

    def test_existing_motion_is_not_replaced(self):
        self.runtime.motion.track_global_line((0,0),(1,0))
        self.assert_fatal(self.acquire())
        self.reader.recognize_camera.assert_not_called()

    def test_refresh_failure_after_ocr_exception_is_fatal(self):
        with patch.object(self.runtime, "step", wraps=self.runtime.step) as stepper:
            def read_and_break_backend(camera):
                stepper.side_effect = RuntimeError("backend step failed")
                raise OSError("camera failed")
            self.reader.recognize_camera.side_effect = read_and_break_backend
            result = self.acquire()
        self.assert_fatal(result)
        self.assertIn("backend step failed", result.reason)

    def test_cli_fallback_warns_and_runs_the_remaining_mission(self):
        self.reader.recognize_camera.side_effect = OSError("camera disconnected")
        def startup(runtime, reader, **kwargs):
            return acquire_task_board(runtime, reader, sleep=self.sleep, **kwargs)
        with patch("main_robocup.build_runtime", return_value=self.runtime), \
             patch("main_robocup.JsonlEventLogger", return_value=self.runtime.event_logger), \
             patch("components.task_board_reader.TaskBoardReader", return_value=self.reader), \
             patch("task_board_startup.acquire_task_board", side_effect=startup), \
             patch.object(self.runtime, "run") as run, self.assertLogs(level="WARNING") as logged:
            code = main(["--mode", "hardware-mission", "--task-board-camera", "/dev/v4l/by-id/task-camera",
                         "--task-board-turn-deg", "90"])
        self.assertEqual(code, 0)
        run.assert_called_once()
        self.assertEqual(self.runtime.mission.task_counts, TaskCounts(1, 2, 1))
        self.assertTrue(any("using fallback" in line for line in logged.output))
        self.assertFalse(any("ERROR" in line for line in logged.output))

    def test_counts_can_only_be_set_once(self):
        self.runtime.mission.set_task_counts(TaskCounts(1, 3, 0))
        with self.assertRaisesRegex(RuntimeError, "already set"):
            self.runtime.mission.set_task_counts(TaskCounts(0, 4, 0))
        with self.assertRaises(TypeError):
            self.runtime.mission.set_task_counts({"red": 1, "blue": 3, "green": 0})

    def test_cli_rejects_missing_angle_or_wrong_mode_before_hardware_build(self):
        with patch("main_robocup.build_runtime") as builder:
            self.assertEqual(main(["--mode", "hardware-mission", "--task-board-camera", "0"]), 2)
            self.assertEqual(main(["--task-board-camera", "0", "--task-board-turn-deg", "90"]), 2)
            self.assertEqual(main(["--mode", "hardware-mission", "--task-board-camera", "0",
                                   "--task-board-turn-deg", "nan"]), 2)
        builder.assert_not_called()


if __name__ == "__main__":
    unittest.main()
