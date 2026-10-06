#!/usr/bin/env python3
"""Deploy the servo files to the board with a timestamped backup and rollback script.

Why this exists instead of ``tools/car_sync.py``: the board checkout is a
different schema-v2 lineage (see ``tools/board_servo_patch.py``), so a whole-source
snapshot would revert board-local loader leniencies and break its live profile.
This script touches only the files the servo needs and refuses to run if the
board currently has no PWM chip for Pin 28 (the run would be untestable).

Usage (from the repository root, after ``tools/board_servo_patch.py``):

    py -3 tools/board_servo_deploy.py [--dry-run]

It stages to ``/tmp/car-servo``, backs up every replaced file, installs with
``sudo``, writes a rollback script, and then verifies on the board that the
patched modules import and that the PWM chip is present.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

from car_ssh import BOARD_HOST, BOARD_USER, BOARD_PASS, connect  # noqa: E402

REPO = TOOLS.parent
BOARD_ROOT = "/home/radxa/car"
PATCH = REPO.parent / "_board_patch"

# Board path -> local source. Every entry is backed up before it is replaced.
FILES = {
    "code/components/servo_axis.py": REPO / "code/components/servo_axis.py",
    "code/components/__init__.py": REPO / "code/components/__init__.py",
    "code/config/v2_models.py": PATCH / "code/config/v2_models.py",
    "code/config/v2_loader.py": PATCH / "code/config/v2_loader.py",
    "code/config/v2_factory.py": PATCH / "code/config/v2_factory.py",
    "code/hal/pwm.py": PATCH / "code/hal/pwm.py",
    "tools/servo_pwm7_probe.py": REPO / "tools/servo_pwm7_probe.py",
    "tools/servo_pwm7_status.py": REPO / "tools/servo_pwm7_status.py",
    "tools/servo_backdrive_probe.py": REPO / "tools/servo_backdrive_probe.py",
    "tools/servo_pwm7_hold.py": REPO / "tools/servo_pwm7_hold.py",
    "tools/board_overlay.py": REPO / "tools/board_overlay.py",
    "configs/robocup_diffdrive.example.toml": REPO / "configs/robocup_diffdrive.example.toml",
}

# Only installed with --with-tests: the board needs the example profile to carry
# the [devices.servo] table and the two suites to cover the new behaviour.
TESTS = {
    "code/test/test_servo_axis.py": REPO / "code/test/test_servo_axis.py",
    "code/test/test_hal_pwm.py": REPO / "code/test/test_hal_pwm.py",
}


def run(client, command: str, *, use_sudo: bool = False, timeout: int = 120):
    if use_sudo:
        stdin, stdout, stderr = client.exec_command(
            f"sudo -S -p '' /bin/sh -c {command!r}", timeout=timeout
        )
        stdin.write(BOARD_PASS + "\n")
        stdin.flush()
        stdin.channel.shutdown_write()
    else:
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    status = stdout.channel.recv_exit_status()
    return status, out, err


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dry-run", action="store_true",
                        help="report what would change on the board without writing")
    parser.add_argument("--with-tests", action="store_true",
                        help="also install the servo test suites onto the board")
    parser.add_argument("--stamp", default=None, help="backup directory name suffix")
    args = parser.parse_args()

    files = dict(FILES)
    if args.with_tests:
        files.update(TESTS)

    missing = [str(path) for path in files.values() if not path.is_file()]
    if missing:
        print("missing local file(s):\n  " + "\n  ".join(missing), file=sys.stderr)
        print("run tools/board_servo_patch.py first", file=sys.stderr)
        return 2

    client = connect()
    try:
        status, out, err = run(client, f"cd {BOARD_ROOT} && git rev-parse HEAD && git status --short | wc -l")
        head = out.splitlines()[0] if out.strip() else "?"
        dirty = out.splitlines()[1] if len(out.splitlines()) > 1 else "?"
        print(f"board HEAD {head} (dirty files before deploy: {dirty})")

        stamp = args.stamp or __import__("datetime").datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = f"/home/radxa/car_servo_backups/{stamp}"
        staging = "/tmp/car-servo"

        status, out, err = run(client, f"rm -rf {staging} && mkdir -p {staging}")
        if status != 0:
            print(f"cannot prepare {staging}: {err}", file=sys.stderr)
            return 1

        with client.open_sftp() as sftp:
            for remote, local in files.items():
                target = f"{staging}/{remote.replace('/', '__')}"
                sftp.put(str(local), target, confirm=True)
                print(f"staged {local.name} -> {target}")

        if args.dry_run:
            print("\ndry run: nothing installed")
            return 0

        status, out, err = run(client, f"mkdir -p {backup}/files", use_sudo=True)
        if status != 0:
            print(f"cannot create backup {backup}: {err}", file=sys.stderr)
            return 1

        installed = []
        for remote in files:
            board_path = f"{BOARD_ROOT}/{remote}"
            backup_path = f"{backup}/files/{remote.replace('/', '__')}"
            # Back up, then install from staging; both in one sudo call so a
            # failure cannot leave a half-installed tree.
            command = (
                f"if [ -f {board_path} ]; then cp -p {board_path} {backup_path}; fi; "
                f"mkdir -p $(dirname {board_path}); "
                f"install -m 0644 {staging}/{remote.replace('/', '__')} {board_path} && "
                f"echo installed {remote}"
            )
            status, out, err = run(client, command, use_sudo=True)
            if status != 0:
                print(f"FAILED {remote}: {out}{err}", file=sys.stderr)
                print(f"restore with: sudo cp {backup_path} {board_path}", file=sys.stderr)
                return 1
            installed.append(remote)
            print(out.strip())

        rollback = backup + "/rollback.sh"
        restore_lines = "\n".join(
            f"cp -p {backup}/files/{remote.replace('/', '__')} {BOARD_ROOT}/{remote}"
            for remote in installed
        )
        run(
            client,
            f"printf '%s\\n' '#!/bin/sh' 'set -e' {restore_lines!r} > {rollback} && chmod +x {rollback}",
            use_sudo=True,
        )

        # Syntax-check with the board's own interpreter before declaring success:
        # the board runs Python 3.11, which rejects constructs the development
        # machine's 3.13 accepts (a multi-line f-string expression already slipped
        # through once), and a SyntaxError would only surface at run time.
        syntax = f"cd {BOARD_ROOT} && python3 -m compileall -q {' '.join(sorted(files))}"
        status, out, err = run(client, syntax)
        print(f"\nboard syntax check (python3): {'OK' if status == 0 else 'FAILED'}")
        if status != 0:
            print(out.strip() or err.strip(), file=sys.stderr)
            print("consider rollback:", file=sys.stderr)
            print(f"  sudo sh {rollback}", file=sys.stderr)
            return 1

        verify = f"cd {BOARD_ROOT} && python3 tools/servo_pwm7_status.py"
        status, out, err = run(client, verify)
        print("\nboard verification:")
        print(out.strip() or err.strip())
        if status != 0:
            # A missing PWM channel is expected until the overlay is enabled, and
            # servo_pwm7_status.py only fails for it under --require-chip.
            print("verification reported a problem (see above)", file=sys.stderr)
            print("consider rollback:", file=sys.stderr)
            print(f"  sudo sh {rollback}", file=sys.stderr)
            return 1

        print(f"\nbackup + rollback: sudo sh {rollback}")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
