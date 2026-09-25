from __future__ import annotations

from typing import Callable


class FakeD500:
    def __init__(self, *, on_update: Callable | None = None, **_kwargs) -> None:
        self.on_update = on_update
        self.alignment = _kwargs.get("alignment")
        self.global_correction_mode = _kwargs.get("global_correction_mode")
        self.wall_localizer = None
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True
        self.stopped = False

    def emit(self, update) -> None:
        if self.on_update is None:
            raise RuntimeError("fake D500 has no update callback")
        self.on_update(update)

    def enable_wall_fusion(self, reference, **_kwargs):
        self.wall_localizer = reference
        return reference

    def stop(self) -> None:
        self.started = False
        self.stopped = True
