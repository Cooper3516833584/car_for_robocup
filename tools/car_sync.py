#!/usr/bin/env python3
"""Prepare or apply a source snapshot, backing up every changed board file first.

Prepare runs on the development computer. Apply runs on the board after the
archive and this script have been transferred by ``tools/car_ssh.py``. Neither
operation starts services or opens hardware.

This is the whole-tree channel. It is not the right tool when the board checkout
is on a different lineage from the worktree (then a snapshot can revert board-local
work); prefer pushing a branch and ``git pull --ff-only`` on the board, which is
what the repository guidance asks for.

    py -3 tools/car_sync.py prepare --output bundle.zip
    py -3 tools/car_sync.py fetch   --output board-edit/     # download board edits
    py -3 tools/car_sync.py apply   --bundle bundle.zip \
        --root /home/radxa/car --backup /home/radxa/car_sync_backups/<stamp>

``apply`` refuses to touch the board unless the target is the expected Git
checkout, every payload checksum matches, the protected config checksums are
preserved, and no mission or legacy launcher is running.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import sys
import tempfile
import zipfile


PROTECTED = {"configs/robocup_diffdrive.toml", "code/radar_center_config.json"}
PROTECTED_DIRECTORIES = {".git", ".venv", ".venv-task-board", "venv", "logs", "runtime",
                         ".agents", ".codex", ".claude", "__pycache__"}


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def safe_relative(name: str) -> PurePosixPath:
    path = PurePosixPath(name)
    if (not name or path.is_absolute() or ".." in path.parts or "\\" in name
            or ":" in name or path.as_posix() != name):
        raise ValueError(f"unsafe relative path: {name!r}")
    if name in PROTECTED or any(part in PROTECTED_DIRECTORIES for part in path.parts):
        raise ValueError(f"protected path: {name}")
    return path


def prepare(output: Path) -> None:
    root = Path(__file__).resolve().parents[1]

    def git(*arguments: str) -> bytes:
        return subprocess.check_output(["git", "-c", f"safe.directory={root.as_posix()}",
                                        "-C", str(root), *arguments])

    names = sorted(set(name.decode("utf-8") for name in
                       git("ls-files", "-z", "--cached", "--others", "--exclude-standard").split(b"\0") if name))
    deleted = sorted(name.decode("utf-8") for name in git("ls-files", "-z", "--deleted").split(b"\0") if name)
    documented_path = root / "docs/legacy_removed_files.txt"
    documented = set(documented_path.read_text(encoding="utf-8").splitlines()) if documented_path.is_file() else set()
    if not set(deleted).issubset(documented):
        raise ValueError("deletions include paths outside the documented legacy cleanup")
    output.parent.mkdir(parents=True, exist_ok=True)
    files = []
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            safe_relative(name)
            path = root / name
            if not path.exists():
                continue
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"source is not a regular file: {name}")
            data = path.read_bytes()
            archive.writestr("payload/" + name, data)
            files.append({"path": name, "sha256": digest(data), "size": len(data)})
        manifest = {"format": 1, "created_at": datetime.now(timezone.utc).isoformat(),
                    "source_head": git("rev-parse", "HEAD").decode().strip(),
                    "files": files, "delete": deleted,
                    "protected": sorted(PROTECTED),
                    "protected_directories": sorted(PROTECTED_DIRECTORIES)}
        archive.writestr("manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
    output.with_suffix(".manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"bundle": str(output.resolve()), "sha256": digest(output.read_bytes()),
                      "files": len(files), "documented_deletions": len(deleted),
                      "bytes": output.stat().st_size}, ensure_ascii=False))


def target_path(root: Path, name: str) -> Path:
    relative = safe_relative(name)
    target = root.joinpath(*relative.parts)
    for part in (target, *target.parents):
        if part == root:
            break
        if part.is_symlink():
            raise ValueError(f"board path contains a symlink: {name}")
    if target.exists() and not target.is_file():
        raise ValueError(f"board target is not a regular file: {name}")
    return target


def fetch_board(output: Path) -> None:
    """Save board worktree edits locally for comparison before source replacement."""

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from car_ssh import connect

    output.mkdir(parents=True, exist_ok=False)
    client = connect()
    try:
        def run(command: str) -> bytes:
            _, stdout, stderr = client.exec_command(command, timeout=30)
            data, error = stdout.read(), stderr.read()
            if stdout.channel.recv_exit_status() != 0:
                raise RuntimeError(error.decode(errors="replace"))
            return data

        head = run("git -C /home/radxa/car rev-parse HEAD").decode().strip()
        names = [name.decode("utf-8") for name in run(
            "git -C /home/radxa/car ls-files -z --modified --others --exclude-standard").split(b"\0") if name]
        (output / "board.diff").write_bytes(run("git -C /home/radxa/car diff --binary"))
        (output / "board.status.txt").write_bytes(run("git -C /home/radxa/car status --short"))
        files = []
        root = Path(__file__).resolve().parents[1]
        with client.open_sftp() as sftp:
            for name in names:
                safe_relative(name)
                target = output / "files" / name
                target.parent.mkdir(parents=True, exist_ok=True)
                sftp.get("/home/radxa/car/" + name, str(target))
                checksum = digest(target.read_bytes())
                local = root / name
                files.append({"path": name, "board_sha256": checksum,
                              "local_exists": local.is_file(),
                              "matches_local": local.is_file() and digest(local.read_bytes()) == checksum})
        report = {"head": head, "files": files}
        (output / "board.snapshot.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    finally:
        client.close()


def apply(bundle: Path, root: Path, backup: Path) -> None:
    if os.name != "posix":
        raise ValueError("apply is only supported on the Linux board")
    root, backup = root.resolve(strict=True), backup.resolve()
    if root != Path("/home/radxa/car") or not (root / ".git").is_dir():
        raise ValueError("expected the existing /home/radxa/car Git checkout")
    if not backup.is_relative_to(Path("/home/radxa/car_sync_backups")) or backup.exists():
        raise ValueError("backup must be a new directory inside /home/radxa/car_sync_backups")
    processes = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
    active = [line for line in processes.splitlines() if any(entry in line for entry in
              ("main_robocup.py", "main_task1.py", "main_task2.py",
               "mission_screen_launcher.py", "ackermann_base_node"))]
    active_services = subprocess.run(["systemctl", "is-active", "--quiet",
                                      "mission-screen-launcher.service", "car-nav2.service"],
                                     check=False)
    if active or active_services.returncode == 0:
        raise RuntimeError("autonomous motion or a legacy launcher is running; stop it before applying source")
    with zipfile.ZipFile(bundle) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        if manifest.get("format") != 1:
            raise ValueError("unsupported manifest format")
        entries = manifest["files"]
        names = [entry["path"] for entry in entries]
        if len(names) != len(set(names)) or set(names).intersection(manifest["delete"]):
            raise ValueError("duplicate or conflicting manifest paths")
        expected_members = {"manifest.json", *("payload/" + name for name in names)}
        if set(archive.namelist()) != expected_members or len(archive.namelist()) != len(expected_members):
            raise ValueError("unexpected archive members")
        for entry in entries:
            target_path(root, entry["path"])
            data = archive.read("payload/" + entry["path"])
            if len(data) != entry["size"] or digest(data) != entry["sha256"]:
                raise ValueError(f"payload checksum mismatch: {entry['path']}")
        documented = set(archive.read("payload/docs/legacy_removed_files.txt").decode("utf-8").splitlines())
        if not set(manifest["delete"]).issubset(documented):
            raise ValueError("undocumented deletion in bundle")
        changes = []
        for entry in entries:
            target = target_path(root, entry["path"])
            if not target.exists() or digest(target.read_bytes()) != entry["sha256"]:
                changes.append(entry)
        deletions = [name for name in manifest["delete"] if target_path(root, name).exists()]
        backup.mkdir(parents=True)
        preserved = {}
        for name in PROTECTED:
            path = root / name
            if path.is_file():
                preserved[name] = digest(path.read_bytes())
        report = {"manifest": manifest, "backup": str(backup),
                  "changed": [entry["path"] for entry in changes],
                  "deleted": deletions, "preserved_config_sha256": preserved, "completed": False}
        report_path = backup / "report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        # Finish all backups before changing the first source file.
        for name in report["changed"] + deletions:
            target = target_path(root, name)
            if target.exists():
                saved = backup / "files" / name
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(target, saved)
                if digest(target.read_bytes()) != digest(saved.read_bytes()):
                    raise OSError(f"backup checksum mismatch: {name}")
        for entry in changes:
            target = target_path(root, entry["path"])
            target.parent.mkdir(parents=True, exist_ok=True)
            mode = stat.S_IMODE(target.stat().st_mode) if target.exists() else 0o644
            descriptor, temporary = tempfile.mkstemp(prefix=".car-sync-", dir=target.parent)
            try:
                with os.fdopen(descriptor, "wb") as stream:
                    stream.write(archive.read("payload/" + entry["path"]))
                    stream.flush()
                    os.fsync(stream.fileno())
                os.chmod(temporary, mode)
                os.replace(temporary, target)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        for name in deletions:
            target_path(root, name).unlink()
        for entry in entries:
            if digest(target_path(root, entry["path"]).read_bytes()) != entry["sha256"]:
                raise OSError(f"board checksum mismatch: {entry['path']}")
        for name, checksum in preserved.items():
            if digest((root / name).read_bytes()) != checksum:
                raise OSError(f"protected config changed: {name}")
        if any(target_path(root, name).exists() for name in manifest["delete"]):
            raise OSError("legacy files remain after sync")
        report["completed"] = True
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"verified_files": len(entries), "changed": len(changes),
                          "deleted": len(deletions), "backup": str(backup),
                          "report": str(report_path), "completed": True}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    pack = sub.add_parser("prepare")
    pack.add_argument("--output", required=True, type=Path)
    fetch = sub.add_parser("fetch", help="download board edits without changing board source")
    fetch.add_argument("--output", required=True, type=Path)
    deploy = sub.add_parser("apply")
    deploy.add_argument("--bundle", required=True, type=Path)
    deploy.add_argument("--root", default=Path("/home/radxa/car"), type=Path)
    deploy.add_argument("--backup", required=True, type=Path)
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.output)
    elif args.action == "fetch":
        fetch_board(args.output)
    else:
        apply(args.bundle, args.root, args.backup)


if __name__ == "__main__":
    main()
