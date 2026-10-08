"""Exercise the complete filming route with real limiting and fake hardware."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.v2_factory import build_payload_demo_session
from payload_demo import PATROL_STAGES, run_payload_demo
from payload_detour import DetourSettings
from robocup_runtime import load_runtime_config
from tools import run_center_target_route as entry


class PayloadDemoTests(unittest.TestCase):
    def fixture(self, *, slot=2):
        self.now, self.stage = 10.0, None
        self.pose = [0.0, 0.0, 0.0]
        self.events, self.releases = [], []
        config = load_runtime_config()
        config = replace(config, relay=replace(config.relay, enabled=True, channel_count=4))
        with patch("config.v2_factory.build_t265_source", side_effect=AssertionError("position opened")):
            self.session = build_payload_demo_session(config, fake=True, clock=lambda: self.now)
        self.addCleanup(self.session.close)
        self.settings = DetourSettings(payload_slot=slot)
        self.session.record_event = self.record
        self.vision = Mock(ready=True)
        self.vision.observe.return_value = (self.now, None)
        self.vision.observe_drop.side_effect = self.observe_drop
        self.triggered = False

    def record(self, event, **data):
        self.events.append((event, data))
        if event == "demo_stage_start":
            self.stage = data["stage"]
        if event == "payload_release_start":
            self.assert_stopped()
            self.releases.append((data["channel"], tuple(self.pose)))

    def observe_drop(self, _now):
        box = (290, 0, 350, 40, .9, 0)
        # A target remains visible before it reaches the horizontal trigger band.
        if not self.triggered and self.pose[0] > .30:
            if self.pose[0] > .45:
                self.triggered = True
                return self.now, box, box
            return self.now, box, None
        return self.now, None, None

    def advance(self, dt):
        twist = self.session.drive.last_limited_twist
        yaw = self.pose[2] + twist.angular_z_rad_s * dt / 2
        self.pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.pose[2] += twist.angular_z_rad_s * dt
        self.now += dt

    def assert_stopped(self):
        twist = self.session.drive.last_limited_twist
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0.0, 0.0))

    def run_demo(self, **kwargs):
        return run_payload_demo(self.session, self.vision, self.settings,
                                clock=lambda: self.now, sleep=self.advance, **kwargs)

    def test_full_route_releases_each_selectable_magnet_and_returns_to_patrol(self):
        for slot in (1, 2, 3):
            with self.subTest(slot=slot):
                self.fixture(slot=slot)
                self.assertEqual(self.run_demo(), 1)
                completed = [data["stage"] for event, data in self.events if event == "demo_stage_done"]
                self.assertEqual(completed, ["advance_7cm", "payload_left_90deg", "forward_47cm",
                                            "reverse_47cm", "payload_right_90deg",
                                            "forward_280cm", "left_90deg_1", "forward_420cm",
                                            "left_90deg_2", "forward_250cm"])
                self.assertEqual(self.releases[0][0], slot + 1)
                self.assertAlmostEqual(self.pose[0], .30, delta=.025)
                self.assertAlmostEqual(self.pose[1], 4.20, delta=.025)
                self.assertAlmostEqual(self.pose[2], math.pi, delta=.015)
                self.assertFalse(any(self.session.relay.query_status().values()))
                self.assert_stopped()

    def test_stop_at_every_moving_stage_prevents_next_stage_and_clears_relays(self):
        stages = [label for label, _ in PATROL_STAGES] + ["advance_7cm", "payload_left_90deg",
                                                         "forward_47cm", "reverse_47cm",
                                                         "payload_right_90deg"]
        for stage in stages:
            with self.subTest(stage=stage):
                self.fixture()
                with self.assertRaisesRegex(RuntimeError, "interrupted"):
                    self.run_demo(abort=lambda: self.stage == stage)
                self.assert_stopped()
                self.assertFalse(any(self.session.relay.query_status().values()))
                self.assertFalse(any(event == "test_route_finished" for event, _ in self.events))

    def test_camera_loss_deadline_and_hold_failure_stop_without_completing_route(self):
        for problem in ("camera", "deadline", "release"):
            with self.subTest(problem=problem):
                self.fixture()
                if problem == "camera":
                    self.vision.observe.side_effect = lambda *_: (
                        (_ for _ in ()).throw(RuntimeError("camera stale")) if self.pose[0] > .1 else None)
                    kwargs = {}
                elif problem == "deadline":
                    kwargs = {"max_seconds": 2}
                else:
                    kwargs = {"abort": lambda: bool(self.releases)}
                with self.assertRaises(RuntimeError):
                    self.run_demo(**kwargs)
                self.assert_stopped()
                self.assertFalse(any(self.session.relay.query_status().values()))

    def test_unconfirmed_payload_hold_never_opens_motor(self):
        self.fixture()
        with patch.object(self.session.relay, "turn_on", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "holding state"):
                self.run_demo()
        self.assertFalse(self.session.drive.is_running)
        self.assert_stopped()

    def test_startup_preserves_preheld_load_until_the_release_stage(self):
        self.fixture()
        self.session.relay.open()
        self.session.relay.turn_on(3)
        self.session.relay.close()  # Closing the port leaves the contact latched.
        initial_commands = len(self.session.relay.commands)
        release_command_index = []
        original_record = self.record

        def record(event, **data):
            if event == "payload_release_start":
                release_command_index.append(len(self.session.relay.commands))
                self.assertTrue(self.session.relay.get_channel_state(3))
            original_record(event, **data)

        self.session.record_event = record
        self.assertEqual(self.run_demo(), 1)
        ch3_off = bytes.fromhex("A0 03 00 A3")
        self.assertNotIn(ch3_off, self.session.relay.commands[initial_commands:release_command_index[0]])

    def test_filming_cli_defaults_to_real_middle_magnet_without_starting_regular_runtime(self):
        self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            weights = Path(tmp) / "best.pt"
            weights.touch()
            with patch.object(entry, "build_runtime", side_effect=AssertionError("localization built")), \
                 patch.object(entry, "build_payload_demo_session", return_value=self.session) as builder, \
                 patch.object(entry, "build_servo", return_value=Mock()), \
                 patch.object(entry, "YoloVision", return_value=Mock()) as vision_builder, \
                 patch.object(entry, "run_payload_demo", return_value=1) as run:
                self.assertEqual(entry.main(["--demo-no-position-checks", "--confirm-motor-test",
                                             "--weights", str(weights), "--log-dir", str(Path(tmp) / "log")]), 0)
                config = builder.call_args.args[0]
                self.assertTrue(config.relay.enabled)
                self.assertEqual(config.relay.port, entry.DEFAULT_RELAY_PORT)
                self.assertEqual(run.call_args.args[2].payload_slot, 2)
                self.assertEqual(vision_builder.call_args.kwargs["target_class_name"], "yellow")


if __name__ == "__main__":
    unittest.main()
