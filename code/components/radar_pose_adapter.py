"""Thin adapter from validated legacy D500 poses to canonical SI poses."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from core.frames import compose_pose2d
from core.types import Pose2D

from .radar_driver import Pose2D as RadarPose2D


@dataclass(frozen=True, slots=True)
class LegacyRadarPoseSpec:
    distance_unit: Literal["cm", "m"] = "cm"
    yaw_unit: Literal["deg", "rad"] = "deg"
    yaw_positive: Literal["cw", "ccw"] = "cw"

    def __post_init__(self) -> None:
        if self.distance_unit not in {"cm", "m"}:
            raise ValueError("distance_unit must be 'cm' or 'm'")
        if self.yaw_unit not in {"deg", "rad"}:
            raise ValueError("yaw_unit must be 'deg' or 'rad'")
        if self.yaw_positive not in {"cw", "ccw"}:
            raise ValueError("yaw_positive must be 'cw' or 'ccw'")


class RadarPoseAdapter:
    """Convert a legacy radar pose and reference point to canonical ``Pose2D``.

    ``legacy_T_base_link`` is the fixed base-link pose in the legacy pose
    frame, expressed in metres/radians with CCW-positive yaw. D500's current
    ``RadarOdometry`` already applies ``RadarMount`` to scan points and reports
    the rear-axle/body reference, which coincides with this repository's
    differential ``base_link`` origin; use identity in that case.
    """

    def __init__(
        self,
        spec: LegacyRadarPoseSpec = LegacyRadarPoseSpec(),
        *,
        legacy_T_base_link: Pose2D = Pose2D(0.0, 0.0, 0.0, 0.0),
    ) -> None:
        values = (
            legacy_T_base_link.x_m,
            legacy_T_base_link.y_m,
            legacy_T_base_link.yaw_rad,
            legacy_T_base_link.timestamp_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("legacy_T_base_link must be finite")
        self.spec = spec
        self.legacy_T_base_link = legacy_T_base_link

    def to_map_base_pose(self, legacy_pose, *, timestamp_s: float) -> Pose2D:
        """Convert a legacy object with x/y/yaw fields to map-frame base pose."""

        scale = 0.01 if self.spec.distance_unit == "cm" else 1.0
        x = float(legacy_pose.x_cm if self.spec.distance_unit == "cm" else legacy_pose.x_m)
        y = float(legacy_pose.y_cm if self.spec.distance_unit == "cm" else legacy_pose.y_m)
        yaw_names = (
            ("yaw_cw_deg", "yaw_deg")
            if self.spec.yaw_unit == "deg"
            else ("yaw_cw_rad", "yaw_rad")
        )
        yaw_name = next((name for name in yaw_names if hasattr(legacy_pose, name)), None)
        if yaw_name is None:
            raise TypeError(f"legacy pose must provide one of {yaw_names}")
        yaw_value = float(getattr(legacy_pose, yaw_name))
        if self.spec.yaw_unit == "deg":
            yaw = math.radians(yaw_value)
        else:
            yaw = yaw_value
        if self.spec.yaw_positive == "cw":
            yaw = -yaw
        stamp = float(timestamp_s)
        values = (x, y, yaw, stamp)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("legacy radar pose and timestamp must be finite")
        map_T_legacy = Pose2D(x * scale, y * scale, yaw, stamp)
        map_T_base = compose_pose2d(map_T_legacy, self.legacy_T_base_link)
        return Pose2D(
            map_T_base.x_m,
            map_T_base.y_m,
            map_T_base.yaw_rad,
            stamp,
        )

    def from_map_base_pose(self, pose: Pose2D) -> RadarPose2D:
        """Convert a canonical map ``base_link`` pose into the radar frame.

        The D500 wall localizer expects ``radar_driver.Pose2D``: centimetres and
        clockwise-positive degrees.  Keeping this conversion in one place stops
        the two same-named types being mixed up, which otherwise fails with:
        ``AttributeError: 'Pose2D' object has no attribute 'x_cm'``.
        """

        values = (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pose must be finite")
        return RadarPose2D(
            x_cm=pose.x_m * 100.0,
            y_cm=pose.y_m * 100.0,
            yaw_cw_deg=-math.degrees(pose.yaw_rad),
        )


def compose_pose_with_delta_prior(
    prior: RadarPose2D,
    delta_x_m: float,
    delta_y_m: float,
    delta_yaw_rad: float,
) -> RadarPose2D:
    """Compose a car-frame delta onto a measured wall-frame start pose.

    The T265 adapter's local origin is wherever its pose stream started, which
    is not automatically the field origin or centre, so a wall predictor needs
    an explicit start prior.  ``prior`` is in the radar convention (cm,
    CW-positive degrees); the delta is in the car's own frame (metres,
    CCW-positive radians).
    """

    values = (prior.x_cm, prior.y_cm, prior.yaw_cw_deg,
              delta_x_m, delta_y_m, delta_yaw_rad)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("prior and delta must be finite")

    prior_yaw_ccw_deg = -prior.yaw_cw_deg
    total_yaw_ccw_deg = prior_yaw_ccw_deg + math.degrees(delta_yaw_rad)
    cos_a = math.cos(math.radians(prior_yaw_ccw_deg))
    sin_a = math.sin(math.radians(prior_yaw_ccw_deg))
    return RadarPose2D(
        prior.x_cm + (delta_x_m * cos_a - delta_y_m * sin_a) * 100.0,
        prior.y_cm + (delta_x_m * sin_a + delta_y_m * cos_a) * 100.0,
        -total_yaw_ccw_deg,
    )
