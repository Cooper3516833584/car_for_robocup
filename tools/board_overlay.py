#!/usr/bin/env python3
"""Enable or disable a U-Boot device-tree overlay in ``/boot/extlinux/extlinux.conf``.

Radxa's own tool for this is the interactive ``rsetup``; this script exists so the
change is scriptable, reviewable and reversible from the repository instead of
being an undocumented manual edit. It never guesses the boot entry: it edits the
single ``fdtoverlays`` line, backs the file up first, and refuses to add an
overlay twice.

The board must be rebooted for the change to take effect.

    sudo python3 tools/board_overlay.py status
    sudo python3 tools/board_overlay.py enable  rk3588-pwm7-m0
    sudo python3 tools/board_overlay.py disable rk3588-pwm7-m0
    sudo python3 tools/board_overlay.py list

``disable`` only removes the overlay from the boot line; it does not rename or
delete the ``.dtbo`` file.
"""

from __future__ import annotations

import argparse
import datetime
from pathlib import Path
import shutil
import sys

CONF = Path("/boot/extlinux/extlinux.conf")
DTOBO_DIR = Path("/boot/dtbo")
BACKUP_DIR = Path("/boot/extlinux")


def overlay_path(name: str) -> str:
    """Resolve an overlay name to a bootable ``.dtbo`` path.

    Radxa disables an overlay by renaming the file to ``<name>.dtbo.disabled``
    rather than editing anything inside it, so the renamed file is still a valid
    compiled blob and can be listed on the ``fdtoverlays`` line as-is. This
    prefers an already-enabled ``.dtbo`` and otherwise accepts the ``.disabled``
    file, so nothing on the board has to be renamed or compiled.
    """

    if name.startswith("/"):
        return name
    if not name.endswith(".dtbo"):
        name += ".dtbo"
    direct = DTOBO_DIR / name
    if direct.exists():
        return str(direct)
    disabled = DTOBO_DIR / f"{name}.disabled"
    if disabled.exists():
        return str(disabled)
    return str(direct)


def available_overlays() -> list[str]:
    """Names of every overlay the board ships, including disabled ones."""

    names = {path.name for path in DTOBO_DIR.glob("*.dtbo")}
    names.update(
        path.name[: -len(".disabled")]
        for path in DTOBO_DIR.glob("*.dtbo.disabled")
    )
    return sorted(names)


def find_line(lines: list[str]) -> int:
    """Index of the single ``fdtoverlays`` line, or fail loudly."""

    found = [index for index, line in enumerate(lines) if line.strip().startswith("fdtoverlays")]
    if len(found) != 1:
        raise SystemExit(
            f"expected exactly one fdtoverlays line in {CONF}, found {len(found)}; "
            "refusing to guess which boot entry to edit"
        )
    return found[0]


def parse(value: str) -> tuple[str, list[str]]:
    """Split a boot line into its leading keyword and overlay paths."""

    parts = value.split()
    return parts[0], parts[1:]


def write(lines: list[str], *, backup: bool) -> Path | None:
    if backup:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        saved = BACKUP_DIR / f"{CONF.name}.bak-{stamp}"
        shutil.copy2(CONF, saved)
    else:
        saved = None
    CONF.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return saved


def load() -> list[str]:
    if not CONF.is_file():
        raise SystemExit(f"{CONF} does not exist; is this a U-Boot/extlinux board?")
    return CONF.read_text(encoding="utf-8").splitlines()


def command_status() -> int:
    lines = load()
    index = find_line(lines)
    keyword, active = parse(lines[index])
    print(f"config : {CONF}")
    print(f"line   : {index + 1}")
    for overlay in active:
        exists = "present" if Path(overlay).exists() else "MISSING"
        print(f"  active  {overlay} ({exists})")
    print(f"available overlays (including disabled): {len(available_overlays())}")
    return 0


def command_list(pattern: str) -> int:
    names = available_overlays()
    matches = [name for name in names if pattern in name]
    for name in matches:
        state = "enabled" if (DTOBO_DIR / name).exists() else "disabled"
        print(f"{name} ({state})")
    if not matches:
        print(f"no overlay name contains {pattern!r}", file=sys.stderr)
        return 1
    return 0


def command_toggle(name: str, *, enable: bool) -> int:
    target = overlay_path(name)
    if enable and not Path(target).exists():
        raise SystemExit(
            f"{target} does not exist; list candidates with: "
            f"python3 {Path(__file__).name} list <substring>"
        )
    lines = load()
    index = find_line(lines)
    keyword, active = parse(lines[index])
    if enable and target in active:
        print(f"already enabled: {target}")
        return 0
    if not enable and target not in active:
        print(f"not enabled: {target}")
        return 0
    updated = active + [target] if enable else [item for item in active if item != target]
    lines[index] = " ".join([keyword, *updated])
    saved = write(lines, backup=True)
    print(f"{'enabled' if enable else 'disabled'}: {target}")
    print(f"backup : {saved}")
    print(f"line   : {lines[index]}")
    print("\nreboot for the change to take effect: sudo reboot")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("status", help="show the active fdtoverlays line")
    listing = sub.add_parser("list", help="list .dtbo files whose name contains a substring")
    listing.add_argument("pattern", nargs="?", default="", help="substring to match")
    for action in ("enable", "disable"):
        node = sub.add_parser(action, help=f"{action} an overlay on the boot line")
        node.add_argument("name", help="overlay name, e.g. rk3588-pwm7-m0")
    args = parser.parse_args()

    if args.action == "status":
        return command_status()
    if args.action == "list":
        return command_list(args.pattern)
    return command_toggle(args.name, enable=args.action == "enable")


if __name__ == "__main__":
    raise SystemExit(main())
