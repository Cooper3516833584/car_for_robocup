#!/usr/bin/env python3
"""Run commands and copy files on the car's ROCK 5A over SSH.

Every hardware check in this repository is supposed to be re-runnable by someone
else, so the SSH entry point lives here instead of in a scratch session.

Credentials come from the environment first, then from the SSH config, and only
then is a password used:

``ROCK5A_HOST`` / ``ROCK5A_USER``
    Connection target; default ``ROCK-5A``, the alias in ``~/.ssh/config`` that
    pins the board's key. A bare host or IP works too.
``ROCK5A_KEY``
    Private key path for key authentication.
``ROCK5A_PASS``
    Password for boards that still need one, and for ``sudo -S`` on stdin.

No credential is stored in this file: an SSH key is the supported path on the
current rig, and the board's password must never be committed or passed as a
command argument (see the repository guidance).

Usage:
    py -3 tools/car_ssh.py run  "systemctl is-active battery-voltage-monitor.service"
    py -3 tools/car_ssh.py sudo "systemctl enable --now battery-voltage-monitor.service"
    py -3 tools/car_ssh.py put  code/test/battery-voltage-monitor.service /home/radxa/car/code/test/
    py -3 tools/car_ssh.py put  code/test/battery-voltage-monitor.service /etc/systemd/system/ --sudo
    py -3 tools/car_ssh.py get  /home/radxa/car/logs/task-board/result.json logs/task-board/result.json

``run`` and ``sudo`` print the exit status and both streams. ``--sudo`` uses
``sudo -S`` with the password on stdin, so no TTY is required.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

BOARD_HOST = os.environ.get("ROCK5A_HOST", "ROCK-5A")
BOARD_USER = os.environ.get("ROCK5A_USER", "radxa")
BOARD_PASS = os.environ.get("ROCK5A_PASS", "")
BOARD_KEY = os.environ.get("ROCK5A_KEY", "")

_PROJECT = Path(__file__).resolve().parent.parent  # car/


def _lookup_ssh_config() -> dict[str, str]:
    """Host/user/key for :data:`BOARD_HOST` from the user's SSH config, if any."""

    config = Path.home() / ".ssh" / "config"
    if not config.is_file():
        return {}
    try:
        import paramiko
    except ModuleNotFoundError:
        return {}
    try:
        parsed = paramiko.SSHConfig.from_path(str(config)).lookup(BOARD_HOST)
    except Exception:  # noqa: BLE001 - a broken config must not break the tool
        return {}
    result: dict[str, str] = {}
    if parsed.get("hostname"):
        result["hostname"] = str(parsed["hostname"])
    if parsed.get("user"):
        result["user"] = str(parsed["user"])
    identity = parsed.get("identityfile") or []
    if identity:
        first = identity[0] if isinstance(identity, (list, tuple)) else identity
        result["key"] = os.path.expanduser(str(first))
    return result


def connect():
    """Return a connected :class:`paramiko.SSHClient`, or raise the reason."""

    import paramiko

    from_config = _lookup_ssh_config()
    hostname = from_config.get("hostname", BOARD_HOST)
    user = BOARD_USER or from_config.get("user", "radxa")
    key_path = BOARD_KEY or from_config.get("key") or None

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    # Key first: that is the supported path on the current rig and it keeps the
    # secret out of the environment. Fall back to a password only if one is set.
    if key_path and Path(key_path).is_file():
        client.connect(
            hostname,
            username=user,
            key_filename=key_path,
            look_for_keys=False,
            allow_agent=True,
            timeout=15,
            banner_timeout=30,
            auth_timeout=30,
        )
        return client
    if BOARD_PASS:
        client.connect(
            hostname,
            username=user,
            password=BOARD_PASS,
            look_for_keys=True,
            allow_agent=True,
            timeout=15,
            banner_timeout=30,
            auth_timeout=30,
        )
        return client
    raise RuntimeError(
        f"no credentials for {user}@{hostname}: set ROCK5A_KEY (or add an "
        "IdentityFile for the host in ~/.ssh/config), or ROCK5A_PASS"
    )


def execute(client, command: str, *, use_sudo: bool) -> int:
    wrapped = f"sudo -S -p '' /bin/sh -c {command!r}" if use_sudo else command
    stdin, stdout, stderr = client.exec_command(wrapped, timeout=120)
    if use_sudo:
        if not BOARD_PASS:
            print("sudo requested but no password is available (set ROCK5A_PASS)", file=sys.stderr)
            return 2
        stdin.write(BOARD_PASS + "\n")
        stdin.flush()
        stdin.channel.shutdown_write()
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    status = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print(err.rstrip(), file=sys.stderr)
    print(f"[exit status: {status}]")
    return status


def copy(client, local: Path, remote: str, *, use_sudo: bool) -> int:
    if not local.is_file():
        print(f"missing local file: {local}", file=sys.stderr)
        return 1
    if use_sudo:
        staging = f"/tmp/{local.name}"
        client.open_sftp().put(str(local), staging, confirm=True)
        print(f"{local} -> {BOARD_USER}@{BOARD_HOST}:{staging} (staged)")
        return execute(
            client,
            f"install -m 0644 {staging} {remote} && rm -f {staging}",
            use_sudo=True,
        )
    sftp = client.open_sftp()
    target = remote.rstrip("/") + "/" + local.name if remote.endswith("/") else remote
    sftp.put(str(local), target, confirm=True)
    sftp.close()
    print(f"{local} -> {BOARD_USER}@{BOARD_HOST}:{target}")
    return 0


def main() -> int:
    # Board output may contain Chinese text and checkbox symbols on Windows.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("run", "sudo"):
        node = sub.add_parser(name, help=f"{name} a shell command on the board")
        node.add_argument("command")
    put = sub.add_parser("put", help="copy a local file to the board")
    put.add_argument("local")
    put.add_argument("remote")
    put.add_argument("--sudo", action="store_true", help="install to a root-owned path")
    get = sub.add_parser("get", help="download a board file")
    get.add_argument("remote")
    get.add_argument("local")
    args = parser.parse_args()

    try:
        client = connect()
    except Exception as exc:  # noqa: BLE001 - report the reason, never a traceback
        print(f"cannot connect to {BOARD_USER}@{BOARD_HOST}: {exc}", file=sys.stderr)
        return 2

    try:
        if args.action == "run":
            return execute(client, args.command, use_sudo=False)
        if args.action == "sudo":
            return execute(client, args.command, use_sudo=True)
        if args.action == "get":
            target = Path(args.local)
            if not target.is_absolute():
                target = _PROJECT / target
            target.parent.mkdir(parents=True, exist_ok=True)
            with client.open_sftp() as sftp:
                sftp.get(args.remote, str(target))
            print(f"{BOARD_USER}@{BOARD_HOST}:{args.remote} -> {target}")
            return 0
        return copy(
            client,
            (Path(args.local) if Path(args.local).is_absolute() else _PROJECT / args.local),
            args.remote,
            use_sudo=args.sudo,
        )
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
