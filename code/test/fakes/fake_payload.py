from __future__ import annotations


class FakePayload:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def actuate(self, action: str) -> None:
        self.actions.append(str(action))
