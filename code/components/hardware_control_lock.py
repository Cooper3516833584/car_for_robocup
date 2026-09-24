"""Shared non-blocking process lock for physical drive ownership."""

from __future__ import annotations

import os
from pathlib import Path
from typing import TextIO

try:
    import fcntl
except ModuleNotFoundError:  # Hardware lock is unused by Windows fake backends.
    fcntl = None


DEFAULT_HARDWARE_LOCK_PATH = "/run/lock/car-hardware.lock"


class HardwareLockError(RuntimeError):
    """Another process owns the configured hardware control lock."""


class HardwareControlLock:
    """Hold one advisory process lock until ``release`` or context exit."""

    def __init__(self, path: str | os.PathLike[str] | None) -> None:
        self.path = None if path is None else os.fspath(path)
        self._file: TextIO | None = None

    def acquire(self) -> None:
        if self.path is None or self._file is not None:
            return
        if fcntl is None:
            raise HardwareLockError("hardware locking is unavailable on this platform")
        try:
            lock_file = open(self.path, "w", encoding="utf-8")
        except OSError as exc:
            raise HardwareLockError(f"cannot open hardware lock {self.path}: {exc}") from exc
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            lock_file.close()
            raise HardwareLockError(f"hardware lock is held: {self.path}") from exc
        self._file = lock_file

    def release(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self) -> "HardwareControlLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.release()
