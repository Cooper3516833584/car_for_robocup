"""Workspace-local temporary directories for the offline tests.

``tempfile.TemporaryDirectory()`` creates a directory with a restrictive ACL and
then tries to reset permissions while removing it. Under the Windows sandbox the
host denies both the write into that fresh directory and the permission reset, so
every test that used one failed with ``PermissionError`` for environmental
reasons -- 97 of them at baseline, which made a real failure impossible to spot.

Tracking counters and names write into ``logs/tmp``; temporary *directories* are
created the same way, with the workspace's own permissions and with a best-effort
cleanup that never turns a passing test into an error.
"""

from __future__ import annotations

import atexit
from itertools import count
from pathlib import Path
import shutil

TMP_ROOT = Path(__file__).resolve().parents[1] / "logs" / "tmp"
_names = count()
_created: list[str] = []


def _sweep():
    """Remove everything this helper created, even if a test forgot to."""
    for path in list(_created):
        cleanup_temp_dir(path)
    _created.clear()


atexit.register(_sweep)


def temp_dir(prefix: str = "tmp") -> str:
    """Create and return a writable, randomly named directory in ``logs/tmp``."""
    TMP_ROOT.mkdir(parents=True, exist_ok=True)
    while True:
        candidate = TMP_ROOT / ("%s%04d" % (prefix, next(_names)))
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        _created.append(str(candidate))
        return str(candidate)


def cleanup_temp_dir(path) -> None:
    """Best-effort removal; a cleanup refusal must not fail a test."""
    try:
        shutil.rmtree(str(path), ignore_errors=True)
    except Exception:
        pass
    if str(path) in _created:
        _created.remove(str(path))


class WorkspaceTempDir:
    """Context manager shaped like ``tempfile.TemporaryDirectory``."""

    def __init__(self, prefix: str = "tmp"):
        self.name = temp_dir(prefix)

    def __enter__(self):
        return self.name

    def __exit__(self, *exc_info):
        cleanup_temp_dir(self.name)
        return False
