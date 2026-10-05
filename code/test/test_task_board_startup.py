from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.basic_motion_controller import MotionActionState
from components.pose_fusion import FusedPoseEstimate, PoseFusionState
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
            anchor_initialized=True,
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

    def test_invalid_ocr_stops_and_never_assigns_counts(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(None, 0, reason="empty OCR")
        result = self.acquire()
        self.assertIsNone(result.task)
        self.assertIsNone(self.runtime.mission.task_counts)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)
        self.assertTrue(any(event["type"] == "task_board_recognition_failed" for event in self.events))

    def test_ocr_exception_stops(self):
        self.reader.recognize_camera.side_effect = OSError("camera disconnected")
        result = self.acquire()
        self.assertIn("camera disconnected", result.reason)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)

    def test_only_one_vote_is_rejected(self):
        self.reader.recognize_camera.side_effect = lambda camera: TaskBoardResult(TaskCounts(2, 1, 1), 0.99, votes=1)
        self.assertFalse(self.acquire().valid)
        self.assertIsNone(self.runtime.mission.task_counts)

    def test_localization_timeout_does_not_open_camera(self):
        self.never_ready = True
        result = self.acquire(timeout_s=0.2)
        self.assertIn("timed out", result.reason)
        self.reader.recognize_camera.assert_not_called()

    def test_turn_timeout_does_not_open_camera(self):
        self.never_turn = True
        self.assertFalse(self.acquire(timeout_s=0.25).valid)
        self.reader.recognize_camera.assert_not_called()
        self.assertEqual(self.runtime.drive.last_limited_twist.angular_z_rad_s, 0)

    def test_blocked_rotation_does_not_open_camera(self):
        self.runtime.motion.drive = replace(self.runtime.motion.drive, allow_in_place_rotation=False)
        self.assertFalse(self.acquire().valid)
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
        self.assertFalse(self.acquire().valid)
        self.assertIsNone(self.runtime.mission.task_counts)

    def test_existing_motion_is_not_replaced(self):
        self.runtime.motion.drive_distance(1)
        self.assertFalse(self.acquire().valid)
        self.reader.recognize_camera.assert_not_called()

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
