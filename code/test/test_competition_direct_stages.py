"""Non-motion CLI tests: no localization startup, real result codes, cleanup."""

from contextlib import ExitStack
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
from components.task_board_reader import TaskBoardResult, TaskCounts
from components.yellow_yolo_adapter import YellowDetection
from main_robocup import DIRECT_COMPETITION_STAGES, main


class DirectCompetitionStageTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.build = self.stack.enter_context(patch("main_robocup.build_runtime"))
        self.profile = self.stack.enter_context(patch("main_robocup.accepted_relative_slam_profile"))
        self.config = SimpleNamespace(relay=SimpleNamespace(
            verify_writes=True, disconnect_on_shutdown=True))
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
            with self.subTest(success=success), patch.object(task, "send_task_once", return_value=success) as send:
                self.assertEqual(main(["--competition-stage", "hc-send"]), 0 if success else 1)
                send.assert_called_once()
                self.assertEqual(send.call_args.args[0].key, (1, 2, 1))
                self.assertFalse(send.call_args.kwargs["bridge_envelope"])
                self.assertEqual(send.call_args.kwargs["baudrate"], 115200)
        self.loader.assert_not_called()
        self.factory.assert_not_called()

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

    def test_drop_only_builds_relay_and_cleans_up_in_order(self):
        order = Mock()
        order.attach_mock(self.relay, "relay")
        drop = Mock(return_value=True)
        order.attach_mock(drop, "drop")
        with patch.object(task, "drop_payload", drop):
            self.assertEqual(main(["--competition-stage", "drop", "--payload-slot", "3", "--config", "site.toml"]), 0)
        self.loader.assert_called_once_with("site.toml")
        self.factory.assert_called_once_with(self.config, fake=False)
        self.assertEqual(order.mock_calls, [call.relay.open(), call.drop(self.relay, 3),
                                          call.relay.all_off(verify=True), call.relay.close()])

    def test_failed_drop_returns_one_but_cleans_up(self):
        with patch.object(task, "drop_payload", return_value=False):
            self.assertEqual(main(["--competition-stage", "drop"]), 1)
        self.relay.all_off.assert_called_once_with(verify=True)
        self.relay.close.assert_called_once()

    def test_disabled_relay_returns_one_without_opening(self):
        self.factory.return_value = None
        with patch.object(task, "drop_payload") as drop:
            self.assertEqual(main(["--competition-stage", "drop"]), 1)
            drop.assert_not_called()
        self.relay.open.assert_not_called()

    def test_drop_exception_and_interrupt_still_clean_up(self):
        for exception, expected in ((OSError("drop failed"), 1), (KeyboardInterrupt(), 130)):
            with self.subTest(exception=type(exception).__name__), patch.object(task, "drop_payload", side_effect=exception):
                self.relay.reset_mock()
                self.assertEqual(main(["--competition-stage", "drop"]), expected)
                self.relay.all_off.assert_called_once()
                self.relay.close.assert_called_once()

    def test_relay_open_failure_closes_without_actuation(self):
        self.relay.connected = False
        self.relay.open.side_effect = OSError("port unavailable")
        with patch.object(task, "drop_payload") as drop:
            self.assertEqual(main(["--competition-stage", "drop"]), 1)
            drop.assert_not_called()
        self.relay.close.assert_called_once()

    def test_all_off_failure_returns_one_and_still_closes(self):
        for value in (False, OSError("cleanup failed")):
            with self.subTest(value=value), patch.object(task, "drop_payload", return_value=True):
                self.relay.reset_mock()
                self.relay.all_off.return_value = value
                self.relay.all_off.side_effect = value if isinstance(value, Exception) else None
                self.assertEqual(main(["--competition-stage", "drop"]), 1)
                self.relay.close.assert_called_once()

    def test_close_failure_returns_one(self):
        self.relay.close.side_effect = OSError("close failed")
        with patch.object(task, "drop_payload", return_value=True):
            self.assertEqual(main(["--competition-stage", "drop"]), 1)

    def test_direct_drop_always_requests_all_off(self):
        self.config.relay.disconnect_on_shutdown = False
        with patch.object(task, "drop_payload", return_value=True):
            self.assertEqual(main(["--competition-stage", "drop"]), 0)
        self.relay.all_off.assert_called_once()


if __name__ == "__main__":
    unittest.main()
