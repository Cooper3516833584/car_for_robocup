from __future__ import annotations

from components.c10b_diff_backend import FakeDriveBackend


class FakeC10BBackend(FakeDriveBackend):
    """Wheel-command recorder with injectable backend failure."""
