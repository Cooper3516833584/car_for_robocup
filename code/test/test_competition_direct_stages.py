"""Non-motion CLI tests: no localization startup, real result codes, cleanup."""

from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
from components.task_board_reader import TaskBoardResult, TaskCounts
from components.yellow_yolo_adapter import YellowDetection
from robocup_runtime import load_runtime_config
from main_robocup import DIRECT_COMPETITION_STAGES, main


class DirectCompetitionStageTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.build = self.stack.enter_context(patch("main_robocup.build_runtime"))
        self.profile = self.stack.enter_context(patch("main_robocup.accepted_relative_slam_profile"))
        self.config = load_runtime_config()
        self.loader = self.stack.enter_context(patch("main_robocup.load_runtime_config", return_value=self.config))
        self.relay = Mock(connected=True)
        self.relay.all_off.return_value = True
        self.factory = self.stack.enter_context(patch("config.v2_factory.build_relay", return_value=self.relay))
        self.stack.enter_context(patch.object(task.time, "sleep"))
        self.addCleanup(self.build.assert_not_called)
        self.addCleanup(self.profile.assert_not_called)

    def test_only_four_stages_are_direct(self):
        self.assertEqual(DIRECT_COMPETITION_STAGES, {"task-board", "hc-send", "yellow-detect", "drop"})

    def test_task_board_without_mode_uses_reader_and_prints_counts(self):
        reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
        reader.recognize_camera.return_value = TaskBoardResult(TaskCounts(2, 1, 1), 0.9, votes=3)
        with patch("components.task_board_reader.TaskBoardReader", return_value=reader), \
             self.assertLogs(level="INFO") as logs:
            self.assertEqual(main(["--competition-stage", "task-board", "--task-board-camera", "2"]), 0)
        reader.recognize_camera.assert_called_once_with(2)
        self.assertTrue(any("task-board result: red=2 blue=1 green=1" in s for s in logs.output))
        self.loader.assert_not_called()
        self.factory.assert_not_called()

    def test_task_board_failure_warns_and_returns_zero_with_fallback(self):
        reader = Mock(config=SimpleNamespace(required_consensus_votes=3))
        reader.recognize_camera.side_effect = OSError("camera unavailable")
        with patch("components.task_board_reader.TaskBoardReader", return_value=reader), \
             self.assertLogs("competition_task", level="WARNING") as logs:
            self.assertEqual(main(["--competition-stage", "task-board"]), 0)
        self.assertEqual(reader.recognize_camera.call_count, task.TASK_BOARD_MAX_TRIES)
        self.assertTrue(any("using fallback task" in s for s in logs.output))

    def test_direct_hc_sends_once_and_reports_success_or_failure(self):
        for success in (True, False):
            with self.subTest(success=success), patch.object(task, "send_task_and_start", return_value=success) as send:
                self.assertEqual(main(["--competition-stage", "hc-send", "--task-red-count", "2",
                                       "--task-blue-count", "1", "--task-green-count", "1"]), 0 if success else 1)
                send.assert_called_once()
                self.assertEqual(tuple(send.call_args.args[0].as_dict().values()), (2, 0, 1))
                self.assertFalse(send.call_args.kwargs["start_drone"])
                self.assertEqual(send.call_args.kwargs["baudrate"], 115200)
        self.loader.assert_not_called()
        self.factory.assert_not_called()

    def test_direct_hc_missing_or_invalid_quota_never_sends(self):
        with patch.object(task, "send_task_and_start") as send, self.assertLogs(level="ERROR"):
            self.assertEqual(main(["--competition-stage", "hc-send"]), 1)
            self.assertEqual(main(["--competition-stage", "hc-send", "--task-red-count", "1",
                                   "--task-blue-count", "1", "--task-green-count", "1"]), 1)
            send.assert_not_called()

    def test_yellow_detection_loads_once_and_releases_camera(self):
        detection = YellowDetection(640, 360, 80, 80, 0.9)
        detector, capture = Mock(), Mock()
        with patch.object(task, "load_detector", return_value=detector) as load, \
             patch.object(task, "_open_yellow_camera", return_value=(capture, True)) as open_camera, \
             patch.object(task, "detect_yellow_once", return_value=detection) as detect, \
             self.assertLogs(level="INFO") as logs:
            self.assertEqual(main(["--competition-stage", "yellow-detect", "--yellow-camera", "/dev/camera",
                                   "--yellow-model", "local-model.pt"]), 0)
        load.assert_called_once_with(Path("local-model.pt"))
        open_camera.assert_called_once_with("/dev/camera")
        detect.assert_called_once_with(capture, detector)
        capture.release.assert_called_once()
        self.assertTrue(any("cx=640.0 cy=360.0 w=80.0 h=80.0 conf=0.900" in s for s in logs.output))
        self.loader.assert_not_called()

    def test_yellow_not_found_returns_one_and_releases_camera(self):
        capture = Mock()
        with patch.object(task, "load_detector"), \
             patch.object(task, "_open_yellow_camera", return_value=(capture, True)) as open_camera, \
             patch.object(task, "detect_yellow_once", return_value=None):
            self.assertEqual(main(["--competition-stage", "yellow-detect"]), 1)
        open_camera.assert_called_once_with(task.YELLOW_CAMERA)
        capture.release.assert_called_once()

    def test_yellow_model_and_camera_failures_return_one(self):
        with patch.object(task, "load_detector", side_effect=FileNotFoundError("weights missing")) as load, \
             patch.object(task, "_open_yellow_camera") as open_camera:
            self.assertEqual(main(["--competition-stage", "yellow-detect"]), 1)
            load.assert_called_once()
            open_camera.assert_not_called()
        with patch.object(task, "load_detector") as load, \
             patch.object(task, "_open_yellow_camera", side_effect=OSError("camera unavailable")):
            self.assertEqual(main(["--competition-stage", "yellow-detect"]), 1)
            load.assert_called_once()

    def test_yellow_inference_exception_releases_camera(self):
        capture = Mock()
        with patch.object(task, "load_detector"), \
             patch.object(task, "_open_yellow_camera", return_value=(capture, True)), \
             patch.object(task, "detect_yellow_once", side_effect=OSError("inference failed")):
            self.assertEqual(main(["--competition-stage", "yellow-detect"]), 1)
        capture.release.assert_called_once()

    def test_yellow_external_capture_is_not_closed(self):
        capture = Mock()
        with patch.object(task, "detect_yellow_once", return_value=None):
            self.assertIsNone(task.detect_yellow_from_camera(capture, Mock()))
        capture.release.assert_not_called()

    def test_drop_opens_dual_servo_without_runtime_or_relay(self):
        with patch('main_robocup.open_payload') as output:
            self.assertEqual(main(['--competition-stage','drop']),0)
        output.assert_called_once_with()
        self.build.assert_not_called()
        self.factory.assert_not_called()

    def test_failed_servo_write_returns_failure(self):
        with patch('main_robocup.open_payload',side_effect=OSError('PWM failed')):
            self.assertEqual(main(['--competition-stage','drop']),1)
        self.build.assert_not_called()
        self.factory.assert_not_called()

    def test_servo_interrupt_returns_130(self):
        with patch('main_robocup.open_payload',side_effect=KeyboardInterrupt):
            self.assertEqual(main(['--competition-stage','drop']),130)

    def test_retired_payload_flags_rejected_before_any_hardware(self):
        for flag,value in (('--relay-port','/dev/confirmed'),('--payload-slot','2'),('--release-mode','relay')):
            with self.subTest(flag=flag), self.assertRaises(SystemExit):
                main(['--competition-stage','drop',flag,value])
        self.build.assert_not_called()
        self.factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
