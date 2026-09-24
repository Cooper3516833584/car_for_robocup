"""Pure two-wheel differential-drive kinematics in SI units."""

from __future__ import annotations

from dataclasses import dataclass
import math

from core.types import Twist2D, WheelSpeeds


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class DifferentialGeometry:
    """Physical distance between left and right drive-wheel contact centers."""

    track_width_m: float

    def __post_init__(self) -> None:
        width = _finite("track_width_m", self.track_width_m)
        if width <= 0.0:
            raise ValueError("track_width_m must be greater than zero")
        object.__setattr__(self, "track_width_m", width)


def scale_wheels_to_limit(wheels: WheelSpeeds, max_abs_m_s: float) -> WheelSpeeds:
    """Scale both wheels equally to preserve the requested curvature."""

    limit = _finite("max_abs_m_s", max_abs_m_s)
    if limit <= 0.0:
        raise ValueError("max_abs_m_s must be greater than zero")
    left = _finite("left_m_s", wheels.left_m_s)
    right = _finite("right_m_s", wheels.right_m_s)
    peak = max(abs(left), abs(right))
    if peak <= limit:
        return WheelSpeeds(left, right)
    scale = limit / peak
    return WheelSpeeds(left * scale, right * scale)


class DifferentialKinematics:
    """Convert body twist and signed wheel linear speeds using physical track."""

    def __init__(self, geometry: DifferentialGeometry) -> None:
        self.geometry = geometry

    def twist_to_wheels(self, cmd: Twist2D) -> WheelSpeeds:
        linear = _finite("linear_x_m_s", cmd.linear_x_m_s)
        angular = _finite("angular_z_rad_s", cmd.angular_z_rad_s)
        half_track_yaw_speed = angular * self.geometry.track_width_m / 2.0
        return WheelSpeeds(
            left_m_s=linear - half_track_yaw_speed,
            right_m_s=linear + half_track_yaw_speed,
        )

    def wheels_to_twist(self, wheels: WheelSpeeds) -> Twist2D:
        left = _finite("left_m_s", wheels.left_m_s)
        right = _finite("right_m_s", wheels.right_m_s)
        return Twist2D(
            linear_x_m_s=(right + left) / 2.0,
            angular_z_rad_s=(right - left) / self.geometry.track_width_m,
        )
