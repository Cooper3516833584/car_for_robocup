from __future__ import annotations

from collections import deque
from typing import Iterable

from components.t265_driver import T265RawPose


class FakeT265:
    def __init__(self, samples: Iterable[T265RawPose] = ()) -> None:
        self.samples = deque(samples)
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True
        self.stopped = False

    def read(self) -> T265RawPose | None:
        if not self.started:
            raise RuntimeError("fake T265 is not running")
        return self.samples.popleft() if self.samples else None

    def stop(self) -> None:
        self.started = False
        self.stopped = True
