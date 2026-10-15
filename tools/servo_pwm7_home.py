#!/usr/bin/env python3
"""Home the configured camera servo to 0 degrees and leave PWM holding it."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.servo_positioning import park_servo
from config.v2_factory import build_servo
from config.v2_loader import load_v2_config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).resolve().parents[1] / "configs" / "robocup_diffdrive.toml",
    )
    args = parser.parse_args(argv)

    servo = None
    try:
        config = load_v2_config(args.config)
        servo = build_servo(config)
        if servo is None:
            raise RuntimeError("camera servo is disabled in the robot config")
        park_servo(servo, 0.0)
        print(f"camera servo homed to 0 deg ({servo.pulse_us} us)", flush=True)
        return 0
    except Exception:
        logging.exception("camera servo startup homing failed")
        return 1
    finally:
        if servo is not None and servo.is_running:
            # LinuxSysfsPWMOutput.close() keeps the enabled kernel PWM state,
            # so the camera remains at 0 degrees after this one-shot exits.
            servo.close(hold=True)


if __name__ == "__main__":
    raise SystemExit(main())
