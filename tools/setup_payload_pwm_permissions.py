#!/usr/bin/env python3
"""Export and chmod five payload and one camera channels; never set PWM."""
from pathlib import Path
import stat
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code'))
from hal.pwm import resolve_chip
from components.payload_servos import (SERVO_1_PWM_NODE, SERVO_2_PWM_NODE,
                                       SERVO_3_PWM_NODE, SERVO_4_PWM_NODE, SERVO_5_PWM_NODE)
from components.servo_axis import ROCK5A_PWM7_M0_CHIP

NODES = (SERVO_1_PWM_NODE, SERVO_2_PWM_NODE, SERVO_3_PWM_NODE, SERVO_4_PWM_NODE, SERVO_5_PWM_NODE,
         ROCK5A_PWM7_M0_CHIP)


def _settle_udev():
    try:
        subprocess.run(['/usr/bin/udevadm', 'settle', '--timeout=10'],
                       check=True, timeout=12)
    except subprocess.SubprocessError as exc:
        raise RuntimeError('PWM export udev events did not settle') from exc


def setup(root=Path('/sys/class/pwm'), *, sleep=time.sleep):
    errors = []
    channels = []
    for node in NODES:
        try:
            chip = resolve_chip(Path(root), node)
            pwm = chip / 'pwm0'
            if not pwm.exists():
                (chip / 'export').write_text('0\n', encoding='ascii')
                for _ in range(20):
                    if pwm.exists():
                        break
                    sleep(.05)
            if not pwm.exists():
                raise RuntimeError(f'export did not create {pwm}')
            channels.append((node, chip, pwm))
        except (OSError, RuntimeError) as exc:
            errors.append(f'{node}: {exc}')
    # Radxa's add/change rule asynchronously chmods all PWM nodes to 0775.
    # Export every project channel first and finish those events before chmod.
    _settle_udev()
    for node, chip, pwm in channels:
        try:
            paths = [chip / 'export', chip / 'unexport', pwm]
            paths.extend(pwm / name for name in ('period', 'duty_cycle', 'enable', 'polarity'))
            for path in paths:
                path.chmod(0o777)
                mode = stat.S_IMODE(path.stat().st_mode)
                if mode != 0o777:
                    raise RuntimeError(f'{path}: chmod 0777 did not take effect ({mode:04o})')
                print(f'{node}: {path} {mode:04o}')
        except (OSError, RuntimeError) as exc:
            errors.append(f'{node}: {exc}')
    if errors:
        raise RuntimeError('servo PWM permission setup failed; check overlays/export/permissions:\n' + '\n'.join(errors))


def main():
    try:
        setup()
        return 0
    except (OSError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
