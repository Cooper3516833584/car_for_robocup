from __future__ import annotations


class FakeClock:
    def __init__(self, now_s: float = 1.0) -> None:
        self.now_s = float(now_s)

    def __call__(self) -> float:
        return self.now_s

    def advance(self, delta_s: float) -> float:
        if delta_s < 0.0:
            raise ValueError("clock cannot move backwards")
        self.now_s += float(delta_s)
        return self.now_s
