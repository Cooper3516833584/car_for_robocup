"""The standalone CH3 test wires +90 search, visual drop and return together."""

import math
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools import run_ch3_drop_test as entry


BOX = (290, 0, 350, 40, .9, 3)  # Horizontally centred detection.


def estimate(point):
    pose = Mock(x_m=point[0], y_m=point[1], yaw_rad=point[2])
    return Mock(estimate=Mock(pose=pose))


class Ch3DropTestEntryTests(unittest.TestCase):
    def fixture(self, *, box=BOX, search_error=None, relay_ok=True):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.weights = self.root / "weights.pt"
        self.weights.touch()
        self.log = self.root / "run"
        self.pose = [1.0, 2.0, math.pi / 2]
        self.runtime = Mock()
        self.events = []
        self.runtime.record_event = lambda event, **values: self.events.append((event, values))
        self.runtime.clock.return_value = 100.0
        self.runtime.relay.connected = True
        self.runtime.relay.all_off.return_value = True
        self.servo = Mock(is_running=True, pulse_us=2500)
        self.vision = Mock()
        self.resolve = Mock(side_effect=lambda runtime: estimate(self.pose))
        self.prepare = Mock(return_value=relay_ok)
        self.search = Mock(return_value=box, side_effect=search_error)
        self.fine = Mock(return_value=box)
        self.detour = Mock(return_value=SimpleNamespace(x_m=1.0, y_m=2.0, yaw_rad=0.0))
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        # Factories return the fake objects; the rest replace the function itself
        # so their side effects run when the entry point calls them.
        for name, value in (("build_runtime", self.runtime), ("build_servo", self.servo),
                            ("YoloVision", self.vision), ("JsonlEventLogger", Mock())):
            self.stack.enter_context(patch.object(entry, name, return_value=value))
        for name, value in (("wait_for_fused_localization", self.resolve),
                            ("prepare_payload", self.prepare),
                            ("find_centered_target", self.search),
                            ("perform_payload_detour", self.detour),
                            ("fine_center_on_road", self.fine)):
            self.stack.enter_context(patch.object(entry, name, new=value))
        self.stack.enter_context(patch.object(entry.signal, "signal"))

    def run_entry(self, *extra):
        return entry.main(["--confirm-motor-test", "--release-mode", "relay",
                           "--relay-port", "/dev/test-relay", "--weights", str(self.weights),
                           "--log-dir", str(self.log), *extra])

    def test_search_then_blind_drop_then_return_finishes_the_program(self):
        self.fixture()
        self.assertEqual(self.run_entry("--search-distance-m", "2.5"), 0)
        self.assertEqual((self.log / "result.txt").read_text(), "FINISHED\n")
        # The payload is held before the first motion, not after the target.
        self.prepare.assert_called_once()
        self.assertEqual(self.prepare.call_args.args[1], 2)
        self.assertTrue(self.prepare.call_args.kwargs["verify"])
        # Camera servo is parked on the search angle the YOLO worker uses.
        self.servo.start.assert_called_once_with(home=False)
        self.assertEqual([c.args[0] for c in self.servo.set_angle.call_args_list], [90])
        # The search line starts at the current fused pose and runs forward.
        start_xy, end_xy = self.search.call_args.args[2:4]
        self.assertEqual(start_xy, (1.0, 2.0))
        self.assertAlmostEqual(end_xy[0], 1.0, delta=1e-9)
        self.assertAlmostEqual(end_xy[1], 4.5, delta=1e-9)
        self.assertEqual(self.search.call_args.kwargs["abort"] is not None, True)
        # The drop uses the middle CH3 slot and the shared fused motion.
        self.detour.assert_called_once_with(self.runtime, 2, servo=self.servo, camera=0,
                                           color="yellow", safe_distance_m=.43)
        done = [values for event, values in self.events if event == "ch3_drop_test_drop_done"]
        self.assertEqual(done, [{"slot": 2, "pose": self.detour.return_value,
                                 "pose_reference": "fused"}])
        self.assertTrue(self.vision.close.called)
        self.fine.assert_called_once_with(self.runtime,self.vision,BOX)
        self.runtime.close.assert_called_once()
        self.runtime.relay.all_off.assert_called_once_with(verify=True)
        self.servo.close.assert_called_once_with(hold=True)

    def test_target_not_found_stops_without_dropping_and_releases_everything(self):
        self.fixture(box=None)
        self.assertEqual(self.run_entry(), 1)
        self.assertEqual((self.log / "result.txt").read_text(), "NOT_FOUND\n")
        self.detour.assert_not_called()
        self.runtime.relay.all_off.assert_called_once_with(verify=True)
        self.runtime.close.assert_called_once()
        self.servo.close.assert_called_once_with(hold=True)

    def test_holding_failure_stops_before_any_motion(self):
        self.fixture(relay_ok=False)
        self.assertEqual(self.run_entry(), 1)
        self.assertIn("holding", (self.log / "result.txt").read_text())
        self.search.assert_not_called()
        self.detour.assert_not_called()
        self.assertFalse(self.servo.start.called)

    def test_horizontal_fine_alignment_failure_never_starts_blind_drop(self):
        self.fixture()
        self.fine.return_value = None
        self.assertEqual(self.run_entry(), 1)
        self.assertEqual((self.log / "result.txt").read_text(), "NOT_FOUND\n")
        self.detour.assert_not_called()
        self.runtime.close.assert_called_once()

    def test_search_failure_keeps_the_failed_result_and_closes_every_resource(self):
        self.fixture(search_error=ValueError("camera/model failure"))
        self.assertEqual(self.run_entry(), 1)
        self.assertIn("FAILED: ValueError", (self.log / "result.txt").read_text())
        self.detour.assert_not_called()
        self.runtime.relay.all_off.assert_called_once_with(verify=True)
        self.runtime.close.assert_called_once()
        self.servo.close.assert_called_once_with(hold=True)

    def test_search_endpoints_follow_the_current_heading(self):
        runtime = Mock()
        with patch.object(entry, "wait_for_fused_localization",
                          side_effect=lambda _r: estimate((0.5, -0.25, -math.pi / 2))):
            start_xy, end_xy = entry.search_endpoints(runtime, 1.5)
        self.assertEqual(start_xy, (0.5, -0.25))
        self.assertAlmostEqual(end_xy[0], 0.5, delta=1e-9)
        self.assertAlmostEqual(end_xy[1], -1.75, delta=1e-9)

    def test_preview_saves_three_frames_without_runtime_relay_or_motor_confirmation(self):
        from components import drop_target_vision as vision
        self.fixture()
        camera=Mock()
        camera.read.return_value=(True,object())
        with patch.object(entry,'_open_yellow_camera',return_value=(camera,True)), \
             patch.object(vision,'save_observation',return_value=(300.,300.,400.)) as save, \
             patch.object(entry,'build_runtime') as build:
            result=entry.main(['--vision-preview','--log-dir',str(self.log)])
        self.assertEqual(result,0)
        self.assertEqual(save.call_count,3)
        build.assert_not_called()
        self.prepare.assert_not_called()
        self.search.assert_not_called()
        self.detour.assert_not_called()
        self.servo.set_angle.assert_called_once_with(0,settle=True)
        camera.release.assert_called_once()
        self.assertTrue((self.log/'preview.json').is_file())


if __name__ == "__main__":
    unittest.main()
