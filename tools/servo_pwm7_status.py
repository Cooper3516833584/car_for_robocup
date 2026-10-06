#!/usr/bin/env python3
"""Read-only check that the servo software path is intact on the board.

Confirms, without writing a single PWM register:

1. the deployed modules import (component, HAL, config model/loader/factory);
2. the live profile enables the servo and with which chip selector;
3. the angle -> pulse mapping the component would use (0 deg = mid pulse);
4. whether the PWM channel for the servo pin is actually available.

Exits non-zero when something is missing, so it can gate a deploy. Run it on the
board, or through ``tools/board_servo_deploy.py`` which invokes it after install:

    python3 tools/servo_pwm7_status.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.servo_axis import (  # noqa: E402
    MG90S_TRAVEL,
    ROCK5A_PWM7_M0_CHIP,
    ROCK5A_PWM7_M0_OVERLAY,
    ROCK5A_PWM7_M0_PHYSICAL_PIN,
    angle_to_pulse_us,
)
from hal.pwm import PWMBackendError, resolve_chip  # noqa: E402

DEFAULT_PROFILE = "/home/radxa/car/configs/robocup_diffdrive.toml"
SYSFS_ROOT = Path("/sys/class/pwm")


def report_channel_state(channel: Path) -> None:
    """Print the channel's latched state, because it outlives the process.

    The kernel keeps emitting the last duty cycle after whoever wrote it exits,
    so ``enable = 1`` here means a servo is still being actively held - possibly
    by a holder process that is long gone.
    """

    if not channel.is_dir():
        print(f"  channel {channel.name:<7} : not exported yet")
        return
    try:
        duty_ns = int((channel / "duty_cycle").read_text(encoding="ascii").strip())
        enable = (channel / "enable").read_text(encoding="ascii").strip()
        period_ns = int((channel / "period").read_text(encoding="ascii").strip())
    except (OSError, ValueError) as exc:
        print(f"  channel {channel.name:<7} : unreadable ({exc})")
        return
    duty_us = duty_ns / 1000.0
    state = "ENABLED (holding torque)" if enable == "1" else "disabled (axis free)"
    print(f"  channel {channel.name:<7} : {state}")
    print(f"    period    : {period_ns / 1000.0:.0f} us")
    print(f"    pulse     : {duty_us:.0f} us")
    half_range = MG90S_TRAVEL.half_range_deg
    if MG90S_TRAVEL.pulse_min_us <= duty_us <= MG90S_TRAVEL.pulse_max_us:
        angle = (duty_us - MG90S_TRAVEL.pulse_mid_us) / MG90S_TRAVEL.us_per_deg
        print(f"    angle     : {angle:+.1f} deg (MG90S calibration, +/-{half_range:g} deg)")
    else:
        print("    angle     : outside the calibrated pulse range")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default=DEFAULT_PROFILE,
                        help="schema-v2 profile to inspect")
    parser.add_argument("--require-chip", action="store_true",
                        help="fail when the servo PWM channel is not available yet")
    args = parser.parse_args()

    failures: list[str] = []

    print("imports            : OK (servo_axis, hal.pwm, config.v2_loader, config.v2_factory)")

    try:
        from config.v2_factory import build_servo
        from config.v2_loader import load_v2_config
    except Exception as exc:  # noqa: BLE001 - report any import breakage plainly
        print(f"config import      : FAILED ({exc})")
        return 1

    profile = Path(args.profile)
    if not profile.is_file():
        print(f"profile            : MISSING ({profile})")
        return 1
    try:
        config = load_v2_config(profile)
    except Exception as exc:  # noqa: BLE001
        print(f"profile load       : FAILED ({exc})")
        return 1

    # The live profile must still satisfy the runtime gates that main_robocup.py
    # applies, otherwise the servo edit would have broken hardware mission mode.
    try:
        from config.v2_runtime import RuntimeMode, runtime_constraints, validate_runtime_readiness

        for mode in (RuntimeMode.DRY_RUN, RuntimeMode.HARDWARE_PROBE, RuntimeMode.HARDWARE_MISSION):
            runtime_constraints(config, mode)
        pending = validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION)
        print(f"runtime gates      : OK (all modes build; HARDWARE_MISSION has "
              f"{len(pending)} outstanding requirement(s), unchanged by the servo)")
    except Exception as exc:  # noqa: BLE001
        print(f"runtime gates      : FAILED ({exc})")
        failures.append("profile no longer satisfies the runtime constraints")

    servo = getattr(config, "servo", None)
    if servo is None:
        print("servo config       : ABSENT (old model module is still installed)")
        return 1

    print(f"profile            : {profile}")
    print(f"servo enabled      : {servo.enabled}")
    print(f"chip selectors     : {servo.chip_device_match}")
    print(f"travel             : +/-{servo.travel_half_range_deg:g} deg over "
          f"{servo.pulse_min_us}..{servo.pulse_max_us} us at {servo.period_us} us frame")
    for angle in (-90.0, 0.0, 90.0):
        print(f"pulse @ {angle:+6.1f} deg    : {angle_to_pulse_us(MG90S_TRAVEL, angle)} us")
    print(f"pin                : {ROCK5A_PWM7_M0_PHYSICAL_PIN} ({ROCK5A_PWM7_M0_CHIP})")

    if servo.enabled:
        axis = build_servo(config, fake=True)
        if axis is None:
            failures.append("servo is enabled but the factory built no axis")
        elif axis.pulse_us_for(0.0) != MG90S_TRAVEL.pulse_mid_us:
            failures.append("factory-built axis does not map 0 deg to the mid pulse")

    chips = sorted(SYSFS_ROOT.glob("pwmchip*"))
    print(f"pwm chips present  : {len(chips)}")
    for chip in chips:
        try:
            resolved = chip.resolve()
        except OSError:
            resolved = "?"
        print(f"  {chip} -> {resolved}")

    try:
        selected = resolve_chip(SYSFS_ROOT, servo.chip_device_match)
    except PWMBackendError:
        print(f"servo pin channel  : NOT AVAILABLE - enable the {ROCK5A_PWM7_M0_OVERLAY} "
              "overlay and reboot (docs/SERVO_PWM7_M0.md)")
        if args.require_chip:
            failures.append("servo PWM channel is not available")
    else:
        print(f"servo pin channel  : {selected}")
        report_channel_state(selected / f"pwm{servo.channel}")

    if failures:
        print("\nFAILED:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("\nOK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
