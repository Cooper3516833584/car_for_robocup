"""Real ServoAxis conversions with fake PWM; never touch sysfs."""
from contextlib import ExitStack
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch, call
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components import payload_servos as payload

class PayloadServoTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.outputs = [Mock(is_running=False) for _ in range(5)]
        for output in self.outputs:
            output.start.side_effect = lambda output=output: setattr(output, 'is_running', True)
        self.stack.enter_context(patch.object(payload, '_axes', [None]*5))
        self.factory = self.stack.enter_context(patch.object(payload, 'LinuxSysfsPWMOutput', side_effect=self.outputs))

    def test_addresses_period_and_actual_open_close_pulses(self):
        self.assertTrue(payload.open_payload())
        self.assertTrue(payload.close_payload())
        self.assertEqual(self.factory.call_args_list,
                         [call(chip_device_match='fd8b0000.pwm', channel=0),
                          call(chip_device_match='fd8b0010.pwm', channel=0),
                          call(chip_device_match='febe0030.pwm', channel=0),
                          call(chip_device_match='febf0020.pwm', channel=0),
                          call(chip_device_match='febd0020.pwm', channel=0)])
        self.assertEqual(payload.SG90_TRAVEL.period_us, 20000)
        for output in self.outputs:
            self.assertEqual(output.set_pulse_us.call_args_list, [call(1000), call(1500)])
            output.disable.assert_not_called()
            output.close.assert_not_called()
            output.start.assert_called_once()

    def test_single_slot_and_repeated_open(self):
        payload.close_payload()
        for output in self.outputs: output.reset_mock()
        payload.open_payload(1)
        self.outputs[0].set_pulse_us.assert_not_called()
        self.outputs[1].set_pulse_us.assert_called_once_with(1000)
        payload.open_payload(0)
        payload.open_payload(0)
        self.assertEqual(self.outputs[0].set_pulse_us.call_args_list, [call(1000), call(1000)])
        payload.open_payload(2)
        payload.open_payload(3)
        payload.open_payload(4)
        for output in self.outputs[2:]:
            output.set_pulse_us.assert_called_once_with(1000)

    def test_first_write_failure_still_commands_second(self):
        self.outputs[0].set_pulse_us.side_effect = OSError('PWM write denied')
        with self.assertRaisesRegex(RuntimeError, 'servo 1'): payload.open_payload()
        self.outputs[1].set_pulse_us.assert_called_once_with(1000)
        for output in self.outputs[2:]:
            output.set_pulse_us.assert_called_once_with(1000)

    def test_slot4_only_writes_pwm6_and_never_constructs_camera_output(self):
        payload.open_payload(4)
        self.factory.assert_called_once_with(chip_device_match='febd0020.pwm', channel=0)
        self.outputs[0].set_pulse_us.assert_called_once_with(1000)
        for output in self.outputs[1:]:
            output.set_pulse_us.assert_not_called()
        with patch.object(payload, 'resolve_chip', return_value=Path('/fake/pwmchip6')) as resolve, \
             patch.object(payload.os, 'access', return_value=True):
            self.assertTrue(payload.payload_configured(4))
        resolve.assert_called_once_with(Path('/sys/class/pwm'), 'febd0020.pwm')

    def test_duplicate_channel_is_rejected_before_any_output(self):
        with patch.object(payload, 'SERVO_5_PWM_NODE', payload.SERVO_1_PWM_NODE):
            with self.assertRaisesRegex(RuntimeError, 'share a PWM channel'):
                payload.open_payload()
        self.factory.assert_not_called()

    def test_first_start_failure_still_commands_second_and_retries(self):
        self.outputs[0].start.side_effect = OSError('missing overlay')
        with self.assertRaisesRegex(RuntimeError, 'missing overlay'): payload.close_payload()
        self.outputs[1].set_pulse_us.assert_called_once_with(1500)
        recovered = Mock(is_running=True)
        self.factory.side_effect = [recovered]
        payload.open_payload()
        recovered.set_pulse_us.assert_called_once_with(1000)

    def test_closed_cached_axis_is_replaced_without_disabling_hold(self):
        payload.open_payload()
        payload._axes[0].close(hold=True)
        self.outputs[0].is_running = False
        replacement = Mock(is_running=True)
        self.factory.side_effect = [replacement]
        payload.close_payload()
        replacement.set_pulse_us.assert_called_once_with(1500)
        self.outputs[0].disable.assert_not_called()

    def test_invalid_slots_do_not_construct_pwm(self):
        for slot in (-1, 5, True, '0'):
            with self.subTest(slot=slot), self.assertRaises(ValueError): payload.open_payload(slot)
        self.factory.assert_not_called()

    def test_unconfigured_address_returns_false_but_commands_other_channel(self):
        with patch.object(payload, 'SERVO_1_PWM_NODE', None): self.assertFalse(payload.open_payload())
        self.outputs[0].set_pulse_us.assert_called_once_with(1000)

    def test_configuration_checks_real_nodes_and_write_access_without_output(self):
        with patch.object(payload, 'resolve_chip', return_value=Path('/fake/pwmchip9')) as resolve, patch.object(payload.os, 'access', return_value=True):
            self.assertTrue(payload.payload_configured())
            self.assertEqual([c.args[1] for c in resolve.call_args_list], ['fd8b0000.pwm', 'fd8b0010.pwm', 'febe0030.pwm', 'febf0020.pwm', 'febd0020.pwm'])
        with patch.object(payload, 'resolve_chip', side_effect=RuntimeError('overlay absent')):
            self.assertFalse(payload.payload_configured())
        with patch.object(payload, 'resolve_chip', return_value=Path('/fake/pwmchip9')), patch.object(payload.os, 'access', return_value=False):
            self.assertFalse(payload.payload_configured(1))
        self.factory.assert_not_called()

    def test_concurrent_commands_never_interleave_the_two_slots(self):
        payload.open_payload()
        calls = []
        for index, output in enumerate(self.outputs):
            output.set_pulse_us.side_effect = lambda pulse, index=index: calls.append((index, pulse))
        threads = [threading.Thread(target=payload.open_payload if i % 2 else payload.close_payload) for i in range(30)]
        for thread in threads: thread.start()
        for thread in threads: thread.join(2)
        self.assertEqual(len(calls), 150)
        for offset in range(0, len(calls), 5):
            group = calls[offset:offset+5]
            self.assertEqual([index for index, pulse in group], [0, 1, 2, 3, 4])
            self.assertEqual(len({pulse for index, pulse in group}), 1)

    def test_car_only_listener_lifecycle_writes_real_conversion_pulses(self):
        import competition_boot as boot
        driver = Mock()
        stop = Mock()
        stop.is_set.return_value = False
        stop.wait.return_value = False
        mission = Mock(side_effect=lambda *args, **kwargs:
                       (payload.open_payload(1), payload.open_payload(0), 0)[-1])
        listener = boot.CompetitionBoot(driver_factory=Mock(return_value=driver),
            alarm=Mock(), mission_main=mission, car_only_test=True, clock=lambda: 0.)
        with patch.object(boot, 'restart_car_slam_sidecar'):
            listener.start()
            listener._run_mission(stop, None, 7, 0.)
            listener._cancel_current()
            listener.close()
        for output in self.outputs:
            pulses = [c.args[0] for c in output.set_pulse_us.call_args_list]
            self.assertEqual(pulses[:4], [1000, 1500, 1000, 1000])
            self.assertTrue(all(p == 1000 for p in pulses[4:]))
            output.disable.assert_not_called()
            output.close.assert_not_called()
        driver.write.assert_not_called()

if __name__ == '__main__': unittest.main()
