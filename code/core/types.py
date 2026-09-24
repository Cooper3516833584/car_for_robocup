"""Canonical SI motion and localization data types.

These types deliberately contain no device-specific protocol fields. Angles
are radians, lengths are metres, and timestamps are monotonic process seconds.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Pose2D:
    """Planar pose in metres and radians, with a monotonic timestamp."""

    x_m: float
    y_m: float
    yaw_rad: float
    timestamp_s: float


@dataclass(frozen=True, slots=True)
class Twist2D:
    """Planar body velocity: forward m/s and counter-clockwise rad/s."""

    linear_x_m_s: float
    angular_z_rad_s: float


@dataclass(frozen=True, slots=True)
class WheelSpeeds:
    """Signed left and right wheel linear velocities in metres per second."""

    left_m_s: float
    right_m_s: float


@dataclass(frozen=True, slots=True)
class PoseQuality:
    """Validity and optional confidence/freshness metadata for a pose."""

    source: str
    valid: bool
    degraded: bool
    position_confidence: float | None = None
    heading_confidence: float | None = None
    age_s: float | None = None
