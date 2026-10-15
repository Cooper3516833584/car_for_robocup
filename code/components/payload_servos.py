"""Two payload SG90 axes, separate from the calibrated camera axis."""
import threading
import logging
import os
from pathlib import Path

from .servo_axis import ServoAxis, ServoTravel
from hal.pwm import LinuxSysfsPWMOutput, resolve_chip

SERVO_1_PWM_NODE = "fd8b0000.pwm"
SERVO_1_PWM_CHANNEL = 0
SERVO_2_PWM_NODE = "fd8b0010.pwm"
SERVO_2_PWM_CHANNEL = 0
OPEN_ANGLE = 0
CLOSE_ANGLE = 90
SG90_TRAVEL = ServoTravel(half_range_deg=90, pulse_min_us=1000, pulse_max_us=2000)
_axes = [None, None]
_lock = threading.RLock()
LOG = logging.getLogger(__name__)
PAYLOAD_COUNT = 2
# Logical cargo slots: 0 = mandatory yellow, 1 = single-item task-board target.


def payload_configured(slot=None) -> bool:
    pins = ((SERVO_1_PWM_NODE, SERVO_1_PWM_CHANNEL), (SERVO_2_PWM_NODE, SERVO_2_PWM_CHANNEL))
    if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or slot not in (0, 1)):
        raise ValueError('payload slot must be 0 (yellow) or 1 (assigned target)')
    with _lock:
        for index, (node, channel) in enumerate(pins):
            if slot is not None and index != slot:
                continue
            try:
                if node is None or channel is None:
                    return False
                chip = resolve_chip(Path('/sys/class/pwm'), node)
                pwm = chip / ('pwm%d' % channel)
                if not all(os.access(pwm / name, os.W_OK) for name in
                           ('period', 'duty_cycle', 'enable', 'polarity')):
                    LOG.error('payload PWM unavailable or not writable: %s', pwm)
                    return False
            except (OSError, RuntimeError):
                LOG.exception('payload PWM unavailable slot=%d node=%s', index, node)
                return False
    return True


def _payload_axis_angle(payload_angle):
    return payload_angle - 90


def _command(angle, slot=None):
    errors = []
    complete = True
    with _lock:
        if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or slot not in (0, 1)):
            raise ValueError('payload slot must be 0 (yellow) or 1 (assigned target)')
        pins = ((SERVO_1_PWM_NODE, SERVO_1_PWM_CHANNEL),
                (SERVO_2_PWM_NODE, SERVO_2_PWM_CHANNEL))
        if all(value is not None for pin in pins for value in pin) and pins[0] == pins[1]:
            raise RuntimeError('payload servos cannot share a PWM channel')
        for index, (node, channel) in enumerate(pins):
            if slot is not None and slot != index:
                continue
            try:
                if _axes[index] is None or not _axes[index].is_running:
                    if node is None or channel is None:
                        LOG.error('payload PWM command incomplete slot=%d: address unconfigured', index)
                        complete = False
                        continue
                    _axes[index] = ServoAxis(SG90_TRAVEL, pwm=LinuxSysfsPWMOutput(
                        chip_device_match=node, channel=channel))
                axis = _axes[index]
                if not axis.is_running:
                    axis.start(home=False)
                axis.apply_angle(_payload_axis_angle(angle), force=True)
                LOG.info('payload PWM command written slot=%d angle=%d pulse_us=%d (no mechanical feedback)',
                         index, angle, axis.pulse_us)
            except Exception as exc:
                errors.append("servo %d: %s" % (index + 1, exc))
    if errors:
        raise RuntimeError("payload command failed: " + "; ".join(errors))
    return complete


def open_payload(slot=None) -> bool:
    """OPEN both for listening/cleanup, or just cargo slot 0/1 for release."""
    return _command(OPEN_ANGLE, slot)


def close_payload() -> bool:
    """CLOSE both after accepting RUN; keep the PWM pulse train running."""
    return _command(CLOSE_ANGLE)
