"""Thin adapter from validated legacy D500 poses to canonical SI poses."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from core.frames import compose_pose2d
from core.types import Pose2D


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
