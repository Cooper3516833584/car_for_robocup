"""Shared calibrated camera positioning with bounded PWM export retry."""

import time
from hal.pwm import PWMBackendError


def park_servo(servo, angle, *, clock=time.monotonic, sleep=time.sleep):
    """Allow udev's group permissions to settle after the first PWM export."""
    deadline = clock() + 2.0
    while True:
        try:
            servo.start(home=False)
            break
        except PWMBackendError as exc:
            if not isinstance(exc.__cause__, PermissionError) or clock() >= deadline:
                raise
            sleep(0.05)
    servo.set_angle(angle, settle=True)
