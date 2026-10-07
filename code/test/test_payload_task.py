from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.payload_task import drop_payload


class PayloadTaskTests(unittest.TestCase):
    def test_all_slots_and_polarities(self):
        for slot in (1, 2, 3):
            for polarity in (True, False):
                with self.subTest(slot=slot, polarity=polarity):
                    relay, sleep = Mock(), Mock()
                    self.assertTrue(drop_payload(relay, slot, active_on=polarity,
                                                 hold_s=0.4, sleep=sleep))
                    self.assertEqual([c[0] for c in relay.method_calls],
                                     ["turn_on", "turn_off"] if polarity else ["turn_off", "turn_on"])
                    self.assertEqual([c[1] for c in relay.method_calls], [(slot,), (slot,)])
                    sleep.assert_called_once_with(0.4)

    def test_mapping_and_missing_relay(self):
        relay = Mock()
        self.assertTrue(drop_payload(relay, 2, slot_to_relay={2: 4}, sleep=Mock()))
        relay.turn_on.assert_called_once_with(4, verify=False)
        self.assertFalse(drop_payload(None))
        for invalid in (0, 4, True):
            self.assertFalse(drop_payload(relay, invalid))

    def test_unconfirmed_activation_is_failure_and_restores(self):
        relay, sleep = Mock(), Mock()
        relay.turn_on.return_value = False
        self.assertFalse(drop_payload(relay, sleep=sleep))
        sleep.assert_not_called()
        relay.turn_off.assert_called_once_with(1, verify=False)

    def test_activation_exception_still_restores(self):
        relay = Mock()
        relay.turn_on.side_effect = OSError("disconnected")
        self.assertFalse(drop_payload(relay, sleep=Mock()))
        relay.turn_off.assert_called_once()

    def test_restore_exception_attempts_recovery(self):
        relay = Mock()
        relay.turn_off.side_effect = [OSError("restore failed"), True]
        self.assertFalse(drop_payload(relay, sleep=Mock()))
        self.assertEqual(relay.turn_off.call_count, 2)

    def test_interrupt_restores_and_propagates(self):
        relay = Mock()
        with self.assertRaises(KeyboardInterrupt):
            drop_payload(relay, sleep=Mock(side_effect=KeyboardInterrupt))
        relay.turn_off.assert_called_once()


if __name__ == "__main__":
    unittest.main()
