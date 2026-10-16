"""Temporary directories for the offline tests, outside the repository.

``tempfile.TemporaryDirectory()`` creates a directory with a restrictive ACL and
then tries to reset permissions while removing it. Under the Windows sandbox both
the write into that fresh directory and the permission reset were denied, so
every test that used one failed with ``PermissionError`` for environmental
reasons -- 97 of them at baseline, which made a real failure impossible to spot.

``tempfile.mkdtemp`` plus ``shutil.rmtree(ignore_errors=True)`` avoids the ACL
reset that failed, and it keeps the directory outside the repository, which some
tools require: ``tools/closed_loop_motion.py`` deliberately refuses an ``--output``
inside the repo so a live test cannot write artifacts into a tracked tree.
"""

from __future__ import annotations

import atexit
from itertools import count
from pathlib import Path
import shutil
import tempfile

_names = count()
_created: list[str] = []


def _sweep():
    """Remove everything this helper created, even if a test forgot to."""
    for path in list(_created):
        cleanup_temp_dir(path)
    _created.clear()


atexit.register(_sweep)


def temp_dir(prefix: str = "tmp") -> str:
    """Create and return a writable, randomly named directory.

    ``mkdtemp`` inherits the OS temp root's permissions, and cleanup never
    resets them, which is exactly what the failing ``TemporaryDirectory`` did.
    """
    path = tempfile.mkdtemp(prefix="dsh_%s_" % prefix)
    _created.append(path)
    return path


def cleanup_temp_dir(path) -> None:
    """Best-effort removal; a cleanup refusal must not fail a test."""
    try:
        shutil.rmtree(str(path), ignore_errors=True)
    except Exception:
        pass
    if str(path) in _created:
        _created.remove(str(path))


class WorkspaceTempDir:
    """Context manager shaped like ``tempfile.TemporaryDirectory``.

    ``TemporaryDirectory`` exposes a ``cleanup()`` method as well as the context
    protocol, and tests call it directly via ``addCleanup(temporary.cleanup)``.
    Omitting it turns a passing test into an ``AttributeError`` in teardown.
    """

    def __init__(self, prefix: str = "tmp"):
        self.name = temp_dir(prefix)
        self._cleaned = False

    def cleanup(self) -> None:
        if not self._cleaned:
            self._cleaned = True
            cleanup_temp_dir(self.name)

    def __enter__(self):
        return self.name

    def __exit__(self, *exc_info):
        self.cleanup()
        return False
