#!/usr/bin/env python3
"""Export and chmod five payload and one camera channels; never set PWM.

A channel whose device-tree overlay is not enabled is *reported and skipped*
rather than treated as a failure. This script is the listener's ``ExecStartPre``,
and ``ExecStartPre`` failing means the whole competition program never starts:
one missing overlay would take down the ground-station listener, the RUN path and
the radio, which is a far worse outcome than one payload axis being unavailable.
The absent axis is still loud -- named on stderr, and the payload code refuses to
claim it opened -- and anything that is a real fault (a channel that exists but
cannot be exported or chmod'ed) still fails hard.
"""
from pathlib import Path
import stat
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code'))
from hal.pwm import PWMBackendError, resolve_chip
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


def setup(root=Path('/sys/class/pwm'), *, sleep=time.sleep, warn=None):
    """Prepare every channel that exists. Returns the names that do not.

    ``PWMBackendError`` from the chip lookup means the pin has no device-tree
    overlay, so the channel cannot exist on this board. It is not a fault of the
    preparation itself, so it is collected and returned.
    """
    errors = []
    unavailable = []
    channels = []
    for node in NODES:
        try:
            chip = resolve_chip(Path(root), node)
        except PWMBackendError as exc:
            # The pin is not described by the device tree. Nothing to prepare.
            unavailable.append(node)
            if warn is not None:
                warn('%s: channel not present; is its overlay enabled and has the '
                     'board rebooted since? (%s)' % (node, exc))
            continue
        try:
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
            # The chip exists but could not be prepared: a real fault.
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
    return tuple(unavailable)


def main():
    try:
        unavailable = setup(warn=lambda message: print('WARNING: ' + message,
                                                       file=sys.stderr))
        if unavailable:
            print('WARNING: %d of %d PWM channels are not present on this board: %s'
                  % (len(unavailable), len(NODES), ', '.join(unavailable)), file=sys.stderr)
            print('WARNING: those payload axes cannot be commanded; the listener '
                  'still starts so RUN/STOP and the remaining axes work.',
                  file=sys.stderr)
        return 0
    except (OSError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
