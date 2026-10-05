"""SI navigation goals shared by task actions and the path controller."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class NavigationGoal:
    x_m: float
    y_m: float
    yaw_rad: float | None = None

    def __post_init__(self) -> None:
        values = (self.x_m, self.y_m)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("goal position must be finite")
        if self.yaw_rad is not None and not math.isfinite(float(self.yaw_rad)):
            raise ValueError("goal yaw must be finite")
