"""A measured slam_map -> t265_odom transform in monotonic time."""

from __future__ import annotations

from dataclasses import dataclass
import math

from core.types import Pose2D


@dataclass(frozen=True, slots=True)
class SlamAnchor:
    pose: Pose2D
    timestamp_s: float
    valid: bool = True
    loop_closure: bool = False
    source: str = "slam_toolbox"

    def age_s(self, now_s: float) -> float:
        return max(0.0, now_s - self.timestamp_s)

    def is_finite(self) -> bool:
        return self.valid and all(math.isfinite(value) for value in (
            self.pose.x_m, self.pose.y_m, self.pose.yaw_rad,
            self.pose.timestamp_s, self.timestamp_s,
        ))
