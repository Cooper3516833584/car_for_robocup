from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.payload_task import drop_payload, payload_channel, prepare_payload
from components.relay_lcus import FakeLCUSRelay


class PayloadTaskTests(unittest.TestCase):
    def test_all_slots_and_polarities(self):
        for slot in (1, 2, 3):
            for polarity in (True, False):
                with self.subTest(slot=slot, polarity=polarity):
                    relay, sleep = Mock(), Mock()
                    self.assertTrue(drop_payload(relay, slot, active_on=polarity,
                                                 hold_s=0.4, sleep=sleep))
                    self.assertEqual([c[0] for c in relay.method_calls],
                                     ["turn_on", "turn_off"] if polarity else ["turn_off", "turn_off"])
                    self.assertEqual([c[1] for c in relay.method_calls], [(slot + 1,), (slot + 1,)])
                    sleep.assert_called_once_with(0.4)

    def test_mapping_and_missing_relay(self):
        relay = Mock()
        self.assertTrue(drop_payload(relay, 2, slot_to_relay={2: 4}, sleep=Mock()))
        self.assertEqual(relay.turn_off.call_args_list[0].args, (4,))
        relay.turn_on.assert_not_called()
        self.assertFalse(drop_payload(None))
        for invalid in (0, 4, True):
            self.assertFalse(drop_payload(relay, invalid))

    def test_unconfirmed_activation_is_failure_and_restores(self):
        relay, sleep = Mock(), Mock()
        relay.turn_off.return_value = False
        self.assertFalse(drop_payload(relay, sleep=sleep))
        sleep.assert_not_called()
        relay.turn_on.assert_not_called()
        self.assertTrue(all(call.args == (2,) for call in relay.turn_off.call_args_list))

    def test_activation_exception_still_restores(self):
        relay = Mock()
        relay.turn_off.side_effect = [OSError("disconnected"), True]
        self.assertFalse(drop_payload(relay, sleep=Mock()))
        self.assertEqual(relay.turn_off.call_count, 2)
        relay.turn_on.assert_not_called()

    def test_restore_exception_attempts_recovery(self):
        relay = Mock()
        relay.turn_off.side_effect = [True, OSError("restore failed"), True]
        self.assertFalse(drop_payload(relay, sleep=Mock()))
        self.assertEqual(relay.turn_off.call_count, 3)

    def test_interrupt_restores_and_propagates(self):
        relay = Mock()
        with self.assertRaises(KeyboardInterrupt):
            drop_payload(relay, sleep=Mock(side_effect=KeyboardInterrupt))
        self.assertEqual(relay.turn_off.call_count, 2)
        relay.turn_on.assert_not_called()

    def test_confirmed_wiring_holds_then_releases_only_selected_magnet(self):
        for slot, channel in ((1, 2), (2, 3), (3, 4)):
            with self.subTest(slot=slot), FakeLCUSRelay(4) as relay:
                relay.all_on()
                relay.turn_off(1)  # CH1 has no electromagnet.
                self.assertEqual(payload_channel(slot), channel)
                self.assertTrue(prepare_payload(relay, slot))
                self.assertTrue(relay.get_channel_state(channel))
                self.assertTrue(drop_payload(relay, slot, verify=True, sleep=Mock()))
                self.assertFalse(relay.get_channel_state(channel))
                for other in (2, 3, 4):
                    if other != channel:
                        self.assertTrue(relay.get_channel_state(other))
                self.assertFalse(relay.get_channel_state(1))

    def test_preparation_failure_attempts_off_and_reports_failure(self):
        relay = Mock()
        relay.turn_on.return_value = False
        self.assertFalse(prepare_payload(relay, 3))
        relay.turn_on.assert_called_once_with(4, verify=True)
        relay.turn_off.assert_called_once_with(4, verify=True)

    def test_invalid_slot_never_actuates(self):
        for slot in (True, 1.0, 0, 4):
            relay = Mock()
            self.assertFalse(drop_payload(relay, slot))
            self.assertFalse(prepare_payload(relay, slot))
            self.assertFalse(relay.method_calls)


if __name__ == "__main__":
    unittest.main()
