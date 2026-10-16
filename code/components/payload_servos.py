"""Five payload SG90 axes, separate from the calibrated camera axis."""
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
SERVO_3_PWM_NODE = "febe0030.pwm"  # PWM11_IR_M1, physical pin 15.
SERVO_3_PWM_CHANNEL = 0
SERVO_4_PWM_NODE = "febf0020.pwm"  # PWM14_M1, physical pin 13.
SERVO_4_PWM_CHANNEL = 0
SERVO_5_PWM_NODE = "febd0020.pwm"  # PWM6_M0, physical pin 27; NOT camera PWM7.
SERVO_5_PWM_CHANNEL = 0
OPEN_ANGLE = 0
CLOSE_ANGLE = 90
SG90_TRAVEL = ServoTravel(half_range_deg=90, pulse_min_us=1000, pulse_max_us=2000)
PAYLOAD_COUNT = 5
_axes = [None] * PAYLOAD_COUNT
_lock = threading.RLock()
LOG = logging.getLogger(__name__)
# Competition slot usage: 4 = yellow mandatory drop (right_mount, 32 cm);
# 1 = first single-item colour and 0 = second single-item colour (standard,
# 43 cm each); 2/3 = the double-item colour released in one approach. The plan
# in competition_task.build_payload_plan() is the only place that decides this.


def _pins():
    return ((SERVO_1_PWM_NODE, SERVO_1_PWM_CHANNEL),
            (SERVO_2_PWM_NODE, SERVO_2_PWM_CHANNEL),
            (SERVO_3_PWM_NODE, SERVO_3_PWM_CHANNEL),
            (SERVO_4_PWM_NODE, SERVO_4_PWM_CHANNEL),
            (SERVO_5_PWM_NODE, SERVO_5_PWM_CHANNEL))


def payload_configured(slot=None) -> bool:
    pins = _pins()
    if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or slot not in range(PAYLOAD_COUNT)):
        raise ValueError('payload slot must be 0..4')
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


def _axis_address_present(node, channel) -> bool:
    """Whether this axis has a usable PWM address on this board right now.

    A pin whose device-tree overlay is not enabled cannot be commanded, and that
    is a property of the hardware build rather than a fault of the command. The
    broadcast paths (RUN CLOSE, STOP/cleanup OPEN) are what the whole competition
    depends on, so they must not be aborted by an axis that does not exist.
    """
    if node is None or channel is None:
        return False
    try:
        resolve_chip(Path('/sys/class/pwm'), node)
    except (OSError, RuntimeError):
        return False
    return True


def _command(angle, slot=None):
    errors = []
    complete = True
    with _lock:
        if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or slot not in range(PAYLOAD_COUNT)):
            raise ValueError('payload slot must be 0..4')
        pins = _pins()
        configured = [pin for pin in pins if all(value is not None for value in pin)]
        if len(set(configured)) != len(configured):
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
                if not _axis_address_present(node, channel):
                    # This pin has no overlay. The axis cannot exist, the
                    # remaining axes still get their command, and the RUN/STOP
                    # path must survive it.
                    LOG.error('payload axis unavailable slot=%d node=%s: %s', index, node, exc)
                    continue
                errors.append("servo %d: %s" % (index + 1, exc))
    if errors:
        raise RuntimeError("payload command failed: " + "; ".join(errors))
    return complete


def open_payload(slot=None) -> bool:
    """OPEN all for listening/cleanup, or just one cargo slot for release."""
    return _command(OPEN_ANGLE, slot)


def close_payload() -> bool:
    """CLOSE all after accepting RUN; keep the PWM pulse train running."""
    return _command(CLOSE_ANGLE)
