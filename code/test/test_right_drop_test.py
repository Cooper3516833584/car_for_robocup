"""The timed right-drop entry needs no pose, wall or line: only the order and the
signs of its timed commands, the stopped release, and its refusal paths."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir
import run_right_drop_test as entry


class FakeDrive:
    """Records every commanded twist; motion exists only in the command stream."""

    def __init__(self):
        self.commands = []  # (now_s, linear_x_m_s, angular_z_rad_s)
        self.stops = 0

    def command(self, twist, *, now_s=None):
        self.commands.append((now_s, twist.linear_x_m_s, twist.angular_z_rad_s))

    def stop(self):
        self.stops += 1


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0.0, float(seconds))


def settings(**overrides):
    values = dict(forward_1_s=0.2, forward_2_s=0.2, side_time_s=0.2, turn_time_s=0.2,
                  release_hold_s=0.1, travel_speed_m_s=0.10, turn_rate_rad_s=0.50)
    values.update(overrides)
    return entry.RightDropSettings(**values)


def run(settings_value, *, release=None, guard=lambda: None, simulate_release=False):
    drive, clock = FakeDrive(), FakeClock()
    events = []  # (event_type, fields, commands sent so far)

    def on_event(event_type, **fields):
        events.append((event_type, fields, len(drive.commands)))

    completed = entry.run_right_drop_test(
        drive, settings_value, release=release, guard=guard, clock=clock, sleep=clock.sleep,
        on_event=on_event, simulate_release=simulate_release)
    return drive, events, completed


def commands_between(drive, events, start, stop_names):
    """Twists commanded from the first matching event until one of ``stop_names``."""
    for index, (name, fields, sent) in enumerate(events):
        if start(name, fields):
            for later_name, _later_fields, later_sent in events[index + 1:]:
                if later_name in stop_names:
                    return drive.commands[sent:later_sent]
            return drive.commands[sent:]
    raise AssertionError("start event never fired")


def step_commands(drive, events, step):
    return commands_between(
        drive, events, lambda name, fields: name == "right_drop_test_step" and fields["step"] == step,
        {"right_drop_test_step", "right_drop_test_step_done"})


def hold_commands(drive, events):
    return commands_between(
        drive, events, lambda name, _fields: name in {"right_drop_test_release_start",
                                                       "right_drop_test_release_skipped"},
        {"right_drop_test_step"})


def only_twist(drive, events, step):
    twists = {twist[1:] for twist in step_commands(drive, events, step)}
    if len(twists) != 1:
        raise AssertionError(f"{step} sent {len(twists)} different twists: {twists}")
    return twists.pop()


class SequenceTests(unittest.TestCase):
    def test_steps_run_in_the_documented_order_with_production_turn_signs(self):
        release = Mock(return_value=True)
        drive, events, completed = run(settings(), release=release)
        self.assertEqual(completed, ("forward_1", "turn_out_1", "side_out", "turn_out_2",
                                     "release", "turn_back_1", "side_back", "turn_back_2",
                                     "forward_2"))
        self.assertEqual([fields["step"] for name, fields, _ in events
                          if name == "right_drop_test_step"],
                         ["forward_1", "turn_out_1", "side_out", "turn_out_2",
                          "turn_back_1", "side_back", "turn_back_2", "forward_2"])
        release.assert_called_once_with()
        self.assertEqual(only_twist(drive, events, "forward_1"), (0.10, 0.0))
        self.assertEqual(only_twist(drive, events, "side_out"), (0.10, 0.0))
        self.assertEqual(only_twist(drive, events, "forward_2"), (0.10, 0.0))
        self.assertEqual(only_twist(drive, events, "side_back"), (-0.10, 0.0))
        # Production right_mount turns left twice, then right twice.
        self.assertAlmostEqual(only_twist(drive, events, "turn_out_1")[1], 0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_out_2")[1], 0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_back_1")[1], -0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_back_2")[1], -0.50)

    def test_detour_direction_flips_every_turn_and_keeps_the_return_axis(self):
        drive, events, _ = run(settings(detour_direction=-1), release=Mock(return_value=True))
        self.assertAlmostEqual(only_twist(drive, events, "turn_out_1")[1], -0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_out_2")[1], -0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_back_1")[1], 0.50)
        self.assertAlmostEqual(only_twist(drive, events, "turn_back_2")[1], 0.50)
        self.assertEqual(only_twist(drive, events, "side_out"), (0.10, 0.0))
        self.assertEqual(only_twist(drive, events, "side_back"), (-0.10, 0.0))

    def test_release_holds_the_chassis_stopped(self):
        drive, events, _ = run(settings(), release=Mock(return_value=True))
        hold = hold_commands(drive, events)
        self.assertTrue(hold, "the release hold sent no command at all")
        self.assertEqual({twist[1:] for twist in hold}, {(0.0, 0.0)})
        self.assertGreater(len(drive.commands), len(hold))

    def test_commands_are_resent_faster_than_the_drive_command_timeout(self):
        drive, events, _ = run(settings(forward_1_s=0.5), release=Mock(return_value=True))
        stamps = [twist[0] for twist in step_commands(drive, events, "forward_1")]
        self.assertGreaterEqual(len(stamps), 9)
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
        self.assertLess(max(b - a for a, b in zip(stamps, stamps[1:])), 0.25)

    def test_simulated_release_keeps_every_motion_step_and_writes_no_pwm(self):
        release = Mock(return_value=True)
        drive, events, completed = run(settings(), release=release, simulate_release=True)
        release.assert_not_called()
        self.assertNotIn("release", completed)
        self.assertEqual([name for name, _, _ in events if name.startswith("right_drop_test_release")],
                         ["right_drop_test_release_skipped"])
        self.assertEqual([fields["step"] for name, fields, _ in events
                          if name == "right_drop_test_step"],
                         ["forward_1", "turn_out_1", "side_out", "turn_out_2",
                          "turn_back_1", "side_back", "turn_back_2", "forward_2"])

    def test_release_without_a_callable_is_refused_before_any_motion(self):
        drive, clock = FakeDrive(), FakeClock()
        with self.assertRaises(ValueError):
            entry.run_right_drop_test(drive, settings(), clock=clock, sleep=clock.sleep)
        self.assertEqual(drive.commands, [])

    def test_incomplete_payload_command_stops_before_the_return(self):
        drive, clock = FakeDrive(), FakeClock()
        steps = []
        with self.assertRaises(RuntimeError):
            entry.run_right_drop_test(
                drive, settings(), release=Mock(return_value=False), clock=clock,
                sleep=clock.sleep,
                on_event=lambda event, **fields: steps.append(fields.get("step")))
        self.assertNotIn("turn_back_1", steps)
        self.assertGreater(drive.stops, 0, "a failed release must leave the base stopped")

    def test_guard_interrupt_stops_the_drive_and_skips_the_rest(self):
        drive, clock = FakeDrive(), FakeClock()
        calls = []

        def guard():
            calls.append(1)
            if len(calls) > 3:
                raise RuntimeError("interrupted")

        with self.assertRaises(RuntimeError):
            entry.run_right_drop_test(drive, settings(), release=Mock(return_value=True),
                                      guard=guard, clock=clock, sleep=clock.sleep)
        self.assertGreater(drive.stops, 0)

    def test_settings_reject_impossible_inputs(self):
        for bad in (dict(payload_slot=5), dict(payload_slot=True), dict(detour_direction=0),
                    dict(side_time_s=0.0), dict(turn_rate_rad_s=float('nan')),
                    dict(forward_1_s=-1.0), dict(release_hold_s=float('inf'))):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                settings(**bad)


class EntryTests(unittest.TestCase):
    def fixture(self):
        self.tmp = WorkspaceTempDir()
        self.addCleanup(self.tmp.cleanup)
        self.log_dir = Path(self.tmp.name) / "run"
        self.drive = Mock()
        self.config = SimpleNamespace(drive=SimpleNamespace(
            max_linear_speed_m_s=0.20, max_angular_speed_rad_s=0.80, command_timeout_s=0.25))
        for name, value in (("load_v2_config", Mock(return_value=self.config)),
                            ("build_differential_drive", Mock(return_value=self.drive)),
                            ("JsonlEventLogger", Mock()),
                            ("close_payload", Mock(return_value=True)),
                            ("open_payload", Mock(return_value=True))):
            patched = patch.object(entry, name, value)
            patched.start()
            self.addCleanup(patched.stop)

    def run_entry(self, *extra):
        return entry.main(["--log-dir", str(self.log_dir),
                           "--forward-1-s", "0.05", "--forward-2-s", "0.05",
                           "--side-time-s", "0.05", "--turn-time-s", "0.05",
                           "--release-hold-s", "0.05", *extra])

    def test_the_parser_requires_no_arguments_at_all(self):
        args = entry.build_parser().parse_args([])
        self.assertFalse(args.simulate_release)
        self.assertEqual(args.payload_slot, entry.PAYLOAD_SLOT)
        self.assertEqual(args.detour_direction, entry.DETOUR_DIRECTION)
        self.assertEqual(args.forward_1_s, entry.FORWARD_1_S)
        self.assertEqual(args.forward_2_s, entry.FORWARD_2_S)
        self.assertEqual(args.side_time_s, entry.SIDE_TIME_S)
        self.assertEqual(args.turn_time_s, entry.TURN_TIME_S)
        self.assertEqual(args.release_hold_s, entry.RELEASE_HOLD_S)
        self.assertAlmostEqual(args.travel_speed_cm_s, entry.TRAVEL_SPEED_M_S * 100.0)
        self.assertEqual(Path(args.config), entry.ROOT / "configs" / "robocup_diffdrive.toml")
        self.assertIsNone(args.log_dir)

    def test_entry_closes_then_releases_the_configured_slot_and_opens_all_at_the_end(self):
        self.fixture()
        self.assertEqual(self.run_entry(), 0)
        entry.close_payload.assert_called_once_with()
        self.assertEqual(entry.open_payload.call_args_list, [call(4), call()])
        self.drive.start.assert_called_once_with()
        stamps = [arguments.kwargs["now_s"] for arguments in self.drive.command.call_args_list]
        self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])),
                        "the drive requires strictly increasing command timestamps")
        self.assertGreaterEqual(self.drive.stop.call_count, 1)
        self.drive.close.assert_called_once_with()
        self.assertEqual((self.log_dir / "result.txt").read_text().strip(), "FINISHED")

    def test_simulated_run_neither_closes_nor_opens_the_payload(self):
        self.fixture()
        self.assertEqual(self.run_entry("--simulate-release"), 0)
        entry.close_payload.assert_not_called()
        entry.open_payload.assert_not_called()
        self.drive.close.assert_called_once_with()

    def test_payload_close_failure_stops_before_any_motion(self):
        self.fixture()
        entry.close_payload.return_value = False
        self.assertEqual(self.run_entry(), 1)
        self.drive.command.assert_not_called()
        entry.open_payload.assert_called_once_with()
        self.assertTrue((self.log_dir / "result.txt").read_text().startswith("FAILED"))
        self.drive.close.assert_called_once_with()

    def test_stop_file_written_during_the_run_interrupts_before_the_next_command(self):
        self.fixture()
        entry.close_payload.side_effect = lambda: (self.log_dir / "STOP").write_text("") or True
        self.assertEqual(self.run_entry(), 1)
        self.drive.command.assert_not_called()
        entry.open_payload.assert_called_once_with()

    def test_speed_above_the_profile_limit_is_refused_before_hardware(self):
        self.fixture()
        with self.assertRaises(SystemExit):
            self.run_entry("--travel-speed-cm-s", "50")
        entry.build_differential_drive.assert_not_called()


if __name__ == "__main__":
    unittest.main()
