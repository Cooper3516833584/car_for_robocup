"""Deterministic T265 odometry and D500 map-pose fusion state object."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from config.v2_models import FusionConfig
from core.frames import compose_pose2d, inverse_pose2d, normalize_angle_rad
from core.types import Pose2D, PoseQuality


class PoseFusionState(Enum):
    INITIALIZING = "initializing"
    OK = "ok"
    T265_DEGRADED = "t265_degraded"
    D500_DEGRADED = "d500_degraded"
    UNANCHORED = "unanchored"
    LOST = "lost"


@dataclass(frozen=True, slots=True)
class FusedPoseEstimate:
    pose: Pose2D | None
    state: PoseFusionState
    age_s: float | None
    source_flags: tuple[str, ...]
    t265_age_s: float | None
    d500_age_s: float | None
    last_d500_innovation_m: float | None
    last_d500_innovation_yaw_rad: float | None
    d500_accepted: bool | None
    rejection_reason: str | None = None


def _pose_valid(pose: Pose2D, quality: PoseQuality) -> bool:
    values = (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)
    return quality.valid and all(math.isfinite(float(value)) for value in values)


def _age(now_s: float, pose: Pose2D | None) -> float | None:
    if pose is None:
        return None
    age = now_s - pose.timestamp_s
    return age if age >= 0.0 else None


class PoseFusion:
    """Fuse relative ``odom_T_base`` with D500 ``map_T_base`` corrections.

    This object has no worker threads or device imports. Its callers supply
    canonical SI poses and the monotonic time used for each estimate.
    """

    def __init__(self, config: FusionConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._t265_pose: Pose2D | None = None
        self._t265_quality: PoseQuality | None = None
        self._d500_pose: Pose2D | None = None
        self._d500_quality: PoseQuality | None = None
        self._map_T_t265_odom: Pose2D | None = None
        self._global_anchor_established = False
        self._last_d500_innovation_m: float | None = None
        self._last_d500_innovation_yaw_rad: float | None = None
        self._last_d500_accepted: bool | None = None
        self._last_d500_rejection: str | None = None
        self._seen_t265 = False
        self._seen_d500 = False

    @property
    def map_T_t265_odom(self) -> Pose2D | None:
        return self._map_T_t265_odom

    @property
    def global_anchor_established(self) -> bool:
        return self._global_anchor_established

    def update_t265(self, pose: Pose2D, quality: PoseQuality) -> None:
        if not _pose_valid(pose, quality):
            return
        self._t265_pose = pose
        self._t265_quality = quality
        self._seen_t265 = True
        if self._map_T_t265_odom is None and self._d500_pose is not None:
            if self._pair_is_recent(self._d500_pose, pose):
                self._map_T_t265_odom = compose_pose2d(
                    self._d500_pose, inverse_pose2d(pose)
                )
                self._global_anchor_established = True
                self._last_d500_accepted = True
                self._last_d500_rejection = None

    def update_d500_absolute(self, pose: Pose2D, quality: PoseQuality) -> None:
        """Apply a D500 pose expressed in the absolute map frame."""
        self._last_d500_accepted = False
        self._last_d500_rejection = None
        if not _pose_valid(pose, quality):
            self._last_d500_rejection = "invalid_quality_or_pose"
            return
        self._seen_d500 = True

        if self._map_T_t265_odom is None:
            if self._t265_pose is not None and self._pair_is_recent(pose, self._t265_pose):
                self._map_T_t265_odom = compose_pose2d(
                    pose, inverse_pose2d(self._t265_pose)
                )
            self._accept_d500(pose, quality)
            return

        if self._t265_pose is None or not self._pair_is_recent(pose, self._t265_pose):
            # Keep a direct absolute fallback without shifting the relative map
            # transform from a stale/mismatched T265 sample.
            self._accept_d500(pose, quality)
            return

        predicted = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
        dx = pose.x_m - predicted.x_m
        dy = pose.y_m - predicted.y_m
        dyaw = normalize_angle_rad(pose.yaw_rad - predicted.yaw_rad)
        position_innovation = math.hypot(dx, dy)
        self._last_d500_innovation_m = position_innovation
        self._last_d500_innovation_yaw_rad = dyaw
        if position_innovation > self.config.max_position_innovation_m:
            self._last_d500_rejection = "position_innovation_gate"
            return
        if abs(dyaw) > self.config.max_yaw_innovation_rad:
            self._last_d500_rejection = "yaw_innovation_gate"
            return

        correction_distance = min(
            position_innovation * self.config.position_correction_gain,
            self.config.max_single_position_correction_m,
        )
        if position_innovation > 0.0:
            correction_x = dx * correction_distance / position_innovation
            correction_y = dy * correction_distance / position_innovation
        else:
            correction_x = correction_y = 0.0
        correction_yaw = max(
            -self.config.max_single_yaw_correction_rad,
            min(
                self.config.max_single_yaw_correction_rad,
                dyaw * self.config.yaw_correction_gain,
            ),
        )
        corrected_fused_pose = Pose2D(
            predicted.x_m + correction_x,
            predicted.y_m + correction_y,
            normalize_angle_rad(predicted.yaw_rad + correction_yaw),
            pose.timestamp_s,
        )
        # Re-solve map_T_odom from the corrected map_T_base and the current
        # odom_T_base. Directly adding yaw to map_T_odom rotates its translation
        # around the odom origin and creates a spurious position jump.
        self._map_T_t265_odom = compose_pose2d(
            corrected_fused_pose,
            inverse_pose2d(self._t265_pose),
        )
        self._accept_d500(pose, quality)

    def update_d500(self, pose: Pose2D, quality: PoseQuality) -> None:
        """Deprecated compatibility alias; ``pose`` must be absolute map_T_base."""
        self.update_d500_absolute(pose, quality)

    def estimate(self, now_s: float) -> FusedPoseEstimate:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite monotonic time")
        t265_age = _age(now, self._t265_pose)
        d500_age = _age(now, self._d500_pose)
        t265_fresh = (
            t265_age is not None
            and t265_age <= self.config.t265_max_age_s
            and self._t265_quality is not None
            and self._t265_quality.valid
        )
        d500_fresh = (
            d500_age is not None
            and d500_age <= self.config.d500_max_age_s
            and self._d500_quality is not None
            and self._d500_quality.valid
        )

        if t265_fresh and d500_fresh and self._map_T_t265_odom is not None:
            pose = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
            state = PoseFusionState.OK
            flags = ("t265", "d500", "fused")
            age = t265_age
        elif d500_fresh:
            pose = self._d500_pose
            state = PoseFusionState.T265_DEGRADED
            flags = ("d500",)
            age = d500_age
        elif t265_fresh and self._map_T_t265_odom is not None:
            pose = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
            state = PoseFusionState.D500_DEGRADED
            flags = ("t265", "dead_reckoning")
            age = t265_age
        elif t265_fresh:
            pose = None
            state = PoseFusionState.UNANCHORED
            flags = ("t265", "unanchored")
            age = t265_age
        elif not self._seen_t265 and not self._seen_d500:
            pose = None
            state = PoseFusionState.INITIALIZING
            flags = ()
            age = None
        else:
            pose = None
            state = PoseFusionState.LOST
            flags = ()
            known_ages = [age_value for age_value in (t265_age, d500_age) if age_value is not None]
            age = max(known_ages) if known_ages else None

        return FusedPoseEstimate(
            pose=pose,
            state=state,
            age_s=age,
            source_flags=flags,
            t265_age_s=t265_age,
            d500_age_s=d500_age,
            last_d500_innovation_m=self._last_d500_innovation_m,
            last_d500_innovation_yaw_rad=self._last_d500_innovation_yaw_rad,
            d500_accepted=self._last_d500_accepted,
            rejection_reason=self._last_d500_rejection,
        )

    def _accept_d500(self, pose: Pose2D, quality: PoseQuality) -> None:
        self._d500_pose = pose
        self._d500_quality = quality
        self._global_anchor_established = True
        self._last_d500_accepted = True
        self._last_d500_rejection = None

    def _pair_is_recent(self, d500: Pose2D, t265: Pose2D) -> bool:
        return abs(d500.timestamp_s - t265.timestamp_s) <= self.config.d500_max_age_s
