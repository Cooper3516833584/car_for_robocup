from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch, call
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from components import payload_servos as payload


class PayloadServoTests(unittest.TestCase):
    def test_each_target_opens_only_its_own_cargo(self):
        axes=[Mock(is_running=True),Mock(is_running=True)]
        with patch.object(payload,'_axes',axes):
            payload.open_payload(0)
            axes[0].apply_angle.assert_called_once_with(0,force=True)
            axes[1].apply_angle.assert_not_called()
            payload.open_payload(1)
            axes[0].apply_angle.assert_called_once()
            axes[1].apply_angle.assert_called_once_with(0,force=True)

    def test_both_axes_and_forced_repeated_open(self):
        axes=[Mock(is_running=True),Mock(is_running=True)]
        with patch.object(payload,'_axes',axes):
            payload.close_payload(); payload.open_payload(); payload.open_payload()
        for axis in axes:
            self.assertEqual(axis.apply_angle.call_args_list,
                             [call(90,force=True),call(0,force=True),call(0,force=True)])

    def test_first_failure_still_attempts_second(self):
        axes=[Mock(is_running=True),Mock(is_running=True)]
        axes[0].apply_angle.side_effect=OSError('PWM')
        with patch.object(payload,'_axes',axes), self.assertRaisesRegex(RuntimeError,'servo 1'):
            payload.open_payload()
        axes[1].apply_angle.assert_called_once_with(0,force=True)

    def test_none_pins_skip_without_constructing_pwm(self):
        with patch.object(payload,'_axes',[None,None]), patch.object(payload,'LinuxSysfsPWMOutput') as pwm:
            self.assertFalse(payload.close_payload())
            self.assertFalse(payload.open_payload())
            pwm.assert_not_called()

    def test_lazy_axes_start_without_homing_and_are_reused(self):
        axes=[Mock(is_running=False),Mock(is_running=False)]
        for axis in axes:
            axis.start.side_effect=lambda axis=axis,**kw:setattr(axis,'is_running',True)
        with patch.object(payload,'_axes',[None,None]), \
             patch.object(payload,'SERVO_1_PWM_NODE','servo1.pwm'), \
             patch.object(payload,'SERVO_2_PWM_NODE','servo2.pwm'), \
             patch.object(payload,'SERVO_1_PWM_CHANNEL',0), \
             patch.object(payload,'SERVO_2_PWM_CHANNEL',1), \
             patch.object(payload,'LinuxSysfsPWMOutput') as pwm, \
             patch.object(payload,'ServoAxis',side_effect=axes) as constructor:
            payload.open_payload();payload.close_payload()
            self.assertEqual(constructor.call_count,2)
            self.assertEqual(pwm.call_args_list,[call(chip_device_match='servo1.pwm',channel=0),
                                               call(chip_device_match='servo2.pwm',channel=1)])
        for axis in axes:
            axis.start.assert_called_once_with(home=False)
        from components.servo_axis import angle_to_pulse_us
        self.assertEqual(angle_to_pulse_us(payload.SG90_TRAVEL,0),1500)
        self.assertEqual(angle_to_pulse_us(payload.SG90_TRAVEL,90),2000)
