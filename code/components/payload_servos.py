"""Two payload SG90 axes, separate from the calibrated camera axis."""
import threading
import logging

from .servo_axis import ServoAxis, ServoTravel
from hal.pwm import LinuxSysfsPWMOutput

SERVO_1_PWM_NODE = None
SERVO_1_PWM_CHANNEL = None
SERVO_2_PWM_NODE = None
SERVO_2_PWM_CHANNEL = None
OPEN_ANGLE = 0
CLOSE_ANGLE = 90
SG90_TRAVEL = ServoTravel(half_range_deg=90, pulse_min_us=1000, pulse_max_us=2000)
_axes = [None, None]
_lock = threading.RLock()
LOG = logging.getLogger(__name__)
PAYLOAD_COUNT = 2
# Logical cargo slots: 0 = mandatory yellow, 1 = single-item task-board target.


def payload_configured(slot=None):
    pins = ((SERVO_1_PWM_NODE, SERVO_1_PWM_CHANNEL), (SERVO_2_PWM_NODE, SERVO_2_PWM_CHANNEL))
    if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or slot not in (0, 1)):
        raise ValueError('payload slot must be 0 (yellow) or 1 (assigned target)')
    return all(value is not None for index, pin in enumerate(pins)
               if slot is None or index == slot for value in pin)


def _command(angle, slot=None):
    errors = []
    complete = True
    with _lock:
        payload_configured(slot)  # Validate the optional logical cargo slot.
        pins = ((SERVO_1_PWM_NODE, SERVO_1_PWM_CHANNEL),
                (SERVO_2_PWM_NODE, SERVO_2_PWM_CHANNEL))
        if all(value is not None for pin in pins for value in pin) and pins[0] == pins[1]:
            raise RuntimeError('payload servos cannot share a PWM channel')
        for index, (node, channel) in enumerate(pins):
            if slot is not None and slot != index:
                continue
            try:
                if _axes[index] is None:
                    if node is None or channel is None:
                        LOG.warning('PAYLOAD SKIPPED slot=%d: PWM address unconfigured', index)
                        complete = False
                        continue
                    _axes[index] = ServoAxis(SG90_TRAVEL, pwm=LinuxSysfsPWMOutput(
                        chip_device_match=node, channel=channel))
                axis = _axes[index]
                if not axis.is_running:
                    axis.start(home=False)
                axis.apply_angle(angle, force=True)
            except Exception as exc:
                errors.append("servo %d: %s" % (index + 1, exc))
    if errors:
        raise RuntimeError("payload command failed: " + "; ".join(errors))
    return complete


def open_payload(slot=None):
    return _command(OPEN_ANGLE, slot)


def close_payload():
    return _command(CLOSE_ANGLE)
