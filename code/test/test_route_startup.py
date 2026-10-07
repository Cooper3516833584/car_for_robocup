"""Startup sequencing and cancellation without opening physical hardware."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.relay_lcus import FakeLCUSRelay
from tools.run_center_target_route import startup_countdown


class RouteStartupTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.relay = FakeLCUSRelay(4)
        self.alarm = SimpleNamespace(is_initialized=False, is_active=False)

        def initialize(*, active=False):
            self.assertEqual(self.relay.query_status(), {1: False, 2: False, 3: True, 4: False})
            self.alarm.is_initialized = True
            self.alarm.is_active = active

        self.alarm.initialize = Mock(side_effect=initialize)
        self.alarm.on = Mock(side_effect=lambda: setattr(self.alarm, "is_active", True))
        self.alarm.off = Mock(side_effect=lambda: setattr(self.alarm, "is_active", False))

    def sleep(self, dt):
        self.assertTrue(self.alarm.is_active)
        self.assertEqual(self.relay.query_status(), {1: False, 2: False, 3: True, 4: False})
        self.now += dt

    def run_countdown(self, **kwargs):
        startup_countdown(self.relay, self.alarm, 2, 10,
                          clock=lambda: self.now, sleep=self.sleep, **kwargs)

    def test_holds_only_middle_magnet_before_ten_second_alarm_and_leaves_it_held(self):
        self.relay.open()
        self.relay.all_on()
        self.run_countdown()
        self.assertAlmostEqual(self.now, 10)
        self.assertFalse(self.alarm.is_active)
        self.assertEqual(self.relay.query_status(), {1: False, 2: False, 3: True, 4: False})
        self.alarm.on.assert_called_once()
        self.alarm.off.assert_called_once()

    def test_stop_during_alarm_silences_and_releases_all(self):
        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            self.run_countdown(abort=lambda: self.now >= 2)
        self.assertFalse(self.alarm.is_active)
        self.assertFalse(any(self.relay.query_status().values()))

    def test_unconfirmed_holding_prevents_alarm_and_releases_all(self):
        self.relay.turn_on = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "relay"):
            self.run_countdown()
        self.alarm.on.assert_not_called()
        self.assertFalse(any(self.relay.query_status().values()))

    def test_alarm_failure_releases_magnet_and_silences(self):
        self.alarm.on.side_effect = OSError("GPIO failure")
        with self.assertRaisesRegex(OSError, "GPIO"):
            self.run_countdown()
        self.assertFalse(self.alarm.is_active)
        self.assertFalse(any(self.relay.query_status().values()))

    def test_invalid_countdown_does_not_open_relay(self):
        for duration in (-1, 0, 61, float("nan")):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                startup_countdown(self.relay, self.alarm, 2, duration)
            self.assertFalse(self.relay.connected)


if __name__ == "__main__":
    unittest.main()
