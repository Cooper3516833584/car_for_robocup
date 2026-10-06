#!/usr/bin/env python3
"""Build board-side servo patches for a checkout whose config lineage differs.

The board checkout is a *different* schema-v2 lineage from the development
worktree (it has no ``CompetitionMapConfig``/``FootprintConfig`` and its loader
is deliberately lenient about retired tables). Overwriting those shared files
with this worktree's versions would break the board's own profile, so this
script applies only the additive servo pieces to the board's own copies:

1. ``code/config/v2_models.py``: insert ``ServoConfig`` + the ``servo`` field;
2. ``code/config/v2_loader.py``: accept the optional ``[devices.servo]`` table;
3. ``code/config/v2_factory.py``: insert ``build_servo``;
4. ``code/hal/pwm.py``: replace with the local file, because the servo pin needs
   the exact device-tree chip selector and that file has no board-local edits.

Each source file is fetched from the board (unless already staged), patched into
``../_board_patch``, and syntax-checked. Nothing touches the board: deployment is
``tools/board_servo_deploy.py``.

    py -3 tools/board_servo_patch.py            # fetch what is missing, then patch
    py -3 tools/board_servo_patch.py --refetch  # ignore the stage and re-fetch
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

STAGE = Path(__file__).resolve().parents[2] / "_board_stage"
PATCH = Path(__file__).resolve().parents[2] / "_board_patch"
LOCAL = Path(__file__).resolve().parents[1]
BOARD_ROOT = "/home/radxa/car"

# Board path -> staged copy. Everything here must come from the board, never
# from this worktree, or the patch would silently reintroduce lineage drift.
SOURCES = (
    "code/config/v2_models.py",
    "code/config/v2_loader.py",
    "code/config/v2_factory.py",
    "code/hal/pwm.py",
)

MODELS_ANCHOR = "@dataclass(frozen=True, slots=True)\nclass D500LocalizationConfig:"
MODELS_FIELD = "    relay: RelayConfig = RelayConfig()\n"
FACTORY_ANCHOR = "def build_pose_fusion(config: DifferentialRobotConfig):"
LOADER_IMPORT = "    RelayConfig,\n"
LOADER_UNKNOWN = '    unknown_devices = set(devices) - {"c10b", "d500", "t265", "relay"}\n'
LOADER_RELAY = """    relay = (
        RelayConfig()
        if devices.get("relay") is None
        else _build(RelayConfig, _table(devices, "relay", "devices.relay"), "devices.relay")
    )
"""
LOADER_SERVO = """
    # [devices.servo] is optional for the same reason: an absent table means "no
    # PWM servo axis fitted", so profiles written before it keep loading.
    servo = (
        ServoConfig()
        if devices.get("servo") is None
        else _build(ServoConfig, _table(devices, "servo", "devices.servo"), "devices.servo")
    )
"""
LOADER_RETURN = "        relay=relay,\n"


def _local_servo_class() -> str:
    """Extract the ServoConfig block verbatim from the local models module."""

    text = (LOCAL / "code/config/v2_models.py").read_text(encoding="utf-8")
    start = text.index("@dataclass(frozen=True, slots=True)\nclass ServoConfig:")
    end = text.index("@dataclass(frozen=True, slots=True)\nclass D500LocalizationConfig:")
    return text[start:end]


def _local_build_servo() -> str:
    """Extract the build_servo block verbatim from the local factory module."""

    text = (LOCAL / "code/config/v2_factory.py").read_text(encoding="utf-8")
    start = text.index("def build_servo(config: DifferentialRobotConfig, *, fake: bool = False):")
    end = text.index("def build_pose_fusion(config: DifferentialRobotConfig):")
    return text[start:end]


def _write(relative: str, text: str) -> None:
    target = PATCH / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8", newline="\n")
    compile(text, str(target), "exec")
    print(f"patched {relative} ({len(text.splitlines())} lines)")


def _strip(text: str, marker: str, end_marker: str) -> str:
    """Remove a block already present in a fetched board file.

    ``--refetch`` can land on a board where the servo patch has already been
    applied (that is exactly what happened when this script was written), so the
    servo pieces are stripped before being re-inserted. Without this the patch
    would either duplicate them or refuse to run at all.
    """

    if marker not in text:
        return text
    start = text.index(marker)
    end = text.index(end_marker, start)
    return text[:start] + text[end:]


def patch_models(refetch: bool) -> None:
    text = (STAGE / "code/config/v2_models.py").read_text(encoding="utf-8")
    if refetch:
        # Replace the whole class, not just remove it: an earlier patch revision
        # left a now-removed field (release_on_close) inside ServoConfig, and
        # leaving it behind would silently preserve the old default.
        text = _strip(text, "@dataclass(frozen=True, slots=True)\nclass ServoConfig:", MODELS_ANCHOR)
        text = text.replace("    servo: ServoConfig = ServoConfig()\n", "", 1)
    if "class ServoConfig" in text:
        raise SystemExit("board models already contain ServoConfig; re-run with --refetch")
    if MODELS_ANCHOR not in text or MODELS_FIELD not in text:
        raise SystemExit("board models layout changed; refusing to guess an insertion point")
    text = text.replace(MODELS_ANCHOR, _local_servo_class() + MODELS_ANCHOR, 1)
    text = text.replace(
        MODELS_FIELD, MODELS_FIELD + "    servo: ServoConfig = ServoConfig()\n", 1
    )
    if "release_on_close" in text.split("class ServoConfig:", 1)[1].split("@dataclass", 1)[0]:
        raise SystemExit("patched ServoConfig still carries the retired release_on_close field")
    _write("code/config/v2_models.py", text)


def patch_factory(refetch: bool) -> None:
    text = (STAGE / "code/config/v2_factory.py").read_text(encoding="utf-8")
    if refetch:
        text = _strip(text, "def build_servo(config: DifferentialRobotConfig, *, fake: bool = False):",
                      FACTORY_ANCHOR)
    if "def build_servo(" in text:
        raise SystemExit("board factory already contains build_servo; re-run with --refetch")
    if FACTORY_ANCHOR not in text:
        raise SystemExit("board factory layout changed; refusing to guess an insertion point")
    text = text.replace(FACTORY_ANCHOR, _local_build_servo() + FACTORY_ANCHOR, 1)
    _write("code/config/v2_factory.py", text)


def patch_loader(refetch: bool) -> None:
    """Teach the board's own loader to accept the optional [devices.servo] table."""

    text = (STAGE / "code/config/v2_loader.py").read_text(encoding="utf-8")
    if refetch:
        text = _strip(text, "\n    # [devices.servo] is optional for the same reason:",
                      "    return DifferentialRobotConfig(")
        text = text.replace("    ServoConfig,\n", "", 1)
        text = text.replace('"t265", "relay", "servo"}', '"t265", "relay"}', 1)
        text = text.replace("        servo=servo,\n", "", 1)
    if "ServoConfig" in text:
        raise SystemExit("board loader already handles ServoConfig; re-run with --refetch")
    for marker in (LOADER_IMPORT, LOADER_UNKNOWN, LOADER_RELAY, LOADER_RETURN):
        if marker not in text:
            raise SystemExit(f"board loader layout changed; missing marker:\n{marker!r}")
    text = text.replace(LOADER_IMPORT, LOADER_IMPORT + "    ServoConfig,\n", 1)
    text = text.replace(
        LOADER_UNKNOWN,
        '    unknown_devices = set(devices) - {"c10b", "d500", "t265", "relay", "servo"}\n',
        1,
    )
    text = text.replace(LOADER_RELAY, LOADER_RELAY + LOADER_SERVO, 1)
    text = text.replace(LOADER_RETURN, LOADER_RETURN + "        servo=servo,\n", 1)
    _write("code/config/v2_loader.py", text)


def patch_hal() -> None:
    _write("code/hal/pwm.py", (LOCAL / "code/hal/pwm.py").read_text(encoding="utf-8"))


def fetch_sources(refetch: bool) -> None:
    """Download the board copies this patch is built on top of.

    Only missing files are fetched unless ``refetch`` is given, so repeating the
    patch does not silently move the baseline to a half-updated board state.
    """

    from car_ssh import BOARD_HOST, BOARD_USER, connect

    wanted = [
        relative for relative in SOURCES
        if refetch or not (STAGE / relative).is_file()
    ]
    if not wanted:
        print(f"stage already complete ({len(SOURCES)} files); use --refetch to re-download")
        return

    print(f"fetching {len(wanted)} file(s) from {BOARD_USER}@{BOARD_HOST}")
    client = connect()
    try:
        with client.open_sftp() as sftp:
            for relative in wanted:
                target = STAGE / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                sftp.get(f"{BOARD_ROOT}/{relative}", str(target))
                print(f"  {relative} -> {target}")
    finally:
        client.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--refetch", action="store_true",
                        help="re-download every board source instead of reusing the stage")
    args = parser.parse_args()

    try:
        fetch_sources(args.refetch)
    except Exception as exc:  # noqa: BLE001 - report the connection reason plainly
        raise SystemExit(f"cannot fetch board sources: {exc}")

    patch_models(args.refetch)
    patch_factory(args.refetch)
    patch_loader(args.refetch)
    patch_hal()
    print(f"\nboard patches written to {PATCH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
