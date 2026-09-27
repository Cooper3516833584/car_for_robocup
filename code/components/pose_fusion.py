"""Time-aligned T265 odometry and D500 absolute-pose fusion."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math

from config.v2_models import FusionConfig
from core.frames import compose_pose2d, inverse_pose2d, normalize_angle_rad
from core.types import Pose2D, PoseQuality


T265_HISTORY_S = 3.0
MAX_INTERP_GAP_S = 0.080
T265_MIN_CONFIDENCE = 2.0 / 3.0
D500_MAX_INNOVATION_M = 0.30
D500_MAX_INNOVATION_YAW_RAD = math.radians(12.0)
ANCHOR_BLEND = 0.35
T265_JUMP_DT_S = 0.20
T265_JUMP_M = 0.50
T265_JUMP_YAW_RAD = math.radians(20.0)


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
    anchor_initialized: bool = False
    t265_confidence: float | None = None
    t265_time_alignment_ms: float | None = None
    t265_continuity_broken: bool = False


@dataclass(frozen=True, slots=True)
class _TimedPose:
    pose: Pose2D
    confidence: float
    alignment_error_s: float = 0.0


def _pose_valid(pose: Pose2D, quality: PoseQuality) -> bool:
    values = (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)
    return quality.valid and all(math.isfinite(float(value)) for value in values)


def _confidence(quality: PoseQuality) -> float:
    return 1.0 if quality.position_confidence is None else float(quality.position_confidence)


def _age(now_s: float, pose: Pose2D | None) -> float | None:
    if pose is None:
        return None
    age = now_s - pose.timestamp_s
    return age if age >= 0.0 else None


class PoseFusion:
    """Maintain ``map_T_t265_odom`` and project continuous T265 odometry."""

    def __init__(self, config: FusionConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._t265_pose: Pose2D | None = None
        self._t265_raw_pose: Pose2D | None = None
        self._t265_rebase = Pose2D(0.0, 0.0, 0.0, 0.0)
        self._t265_quality: PoseQuality | None = None
        self._t265_history: deque[_TimedPose] = deque()
        self._t265_continuity_broken = False
        self._d500_pose: Pose2D | None = None
        self._d500_quality: PoseQuality | None = None
        self._d500_global_fallback_pose: Pose2D | None = None
        self._d500_global_fallback_quality: PoseQuality | None = None
        self._d500_map_alignment_valid = False
        self._map_T_t265_odom: Pose2D | None = None
        self._last_d500_innovation_m: float | None = None
        self._last_d500_innovation_yaw_rad: float | None = None
        self._last_d500_accepted: bool | None = None
        self._last_d500_rejection: str | None = None
        self._last_t265_alignment_ms: float | None = None
        self._seen_t265 = False
        self._seen_d500 = False

    @property
    def map_T_t265_odom(self) -> Pose2D | None:
        return self._map_T_t265_odom

    @property
    def global_anchor_established(self) -> bool:
        return self._map_T_t265_odom is not None

    def update_t265(self, pose: Pose2D | None, quality: PoseQuality) -> None:
        """Add canonical adapter output, rebasing tracker frame jumps continuously."""
        if pose is None or not _pose_valid(pose, quality):
            self._t265_quality = quality
            self._seen_t265 = True
            self._break_t265_continuity()
            return
        confidence = _confidence(quality)
        if confidence < T265_MIN_CONFIDENCE:
            self._t265_quality = quality
            self._seen_t265 = True
            self._break_t265_continuity()
            return
        if self._t265_history and pose.timestamp_s <= self._t265_history[-1].pose.timestamp_s:
            return

        if self._t265_raw_pose is not None and self._t265_pose is not None:
            dt = pose.timestamp_s - self._t265_raw_pose.timestamp_s
            if not self._t265_continuity_broken and dt >= T265_JUMP_DT_S:
                self._break_t265_continuity()
            elif not self._t265_continuity_broken and 0.0 < dt < T265_JUMP_DT_S:
                delta = compose_pose2d(inverse_pose2d(self._t265_raw_pose), pose)
                if (math.hypot(delta.x_m, delta.y_m) > T265_JUMP_M
                        or abs(delta.yaw_rad) > T265_JUMP_YAW_RAD):
                    self._t265_rebase = compose_pose2d(
                        self._t265_pose, inverse_pose2d(pose)
                    )
                    self._t265_history.clear()

        if self._t265_continuity_broken:
            if self._fresh_d500_global_fallback(pose.timestamp_s):
                assert self._d500_global_fallback_pose is not None
                self._map_T_t265_odom = compose_pose2d(
                    self._d500_global_fallback_pose, inverse_pose2d(pose)
                )
                self._t265_rebase = Pose2D(0.0, 0.0, 0.0, pose.timestamp_s)
                self._t265_continuity_broken = False
                self._t265_history.clear()
                continuous = pose
            else:
                self._t265_raw_pose = pose
                self._t265_quality = quality
                self._seen_t265 = True
                return
        else:
            continuous = compose_pose2d(self._t265_rebase, pose)
            continuous = Pose2D(
                continuous.x_m, continuous.y_m, continuous.yaw_rad, pose.timestamp_s
            )

        self._t265_raw_pose = pose
        self._t265_pose = continuous
        self._t265_quality = quality
        self._seen_t265 = True
        self._t265_history.append(_TimedPose(continuous, confidence))
        cutoff = pose.timestamp_s - T265_HISTORY_S
        while len(self._t265_history) > 2 and self._t265_history[1].pose.timestamp_s < cutoff:
            self._t265_history.popleft()

    def update_d500_global_fallback(
        self,
        pose: Pose2D,
        quality: PoseQuality,
        *,
        map_alignment_valid: bool,
    ) -> None:
        """Cache propagated D500 map pose; this channel never changes the anchor."""
        if not map_alignment_valid or not _pose_valid(pose, quality) or quality.degraded:
            return
        self._d500_map_alignment_valid = True
        self._d500_global_fallback_pose = pose
        self._d500_global_fallback_quality = quality
        self._seen_d500 = True

    def _fresh_d500_global_fallback(self, now_s: float) -> bool:
        pose = self._d500_global_fallback_pose
        quality = self._d500_global_fallback_quality
        if not self._d500_map_alignment_valid or pose is None or quality is None or not quality.valid:
            return False
        age = float(now_s) - pose.timestamp_s
        return 0.0 <= age <= self.config.d500_max_age_s

    def _break_t265_continuity(self) -> None:
        self._t265_continuity_broken = True
        self._t265_history.clear()

    def _sample_t265(self, timestamp_s: float) -> _TimedPose | None:
        if not self._t265_history:
            return None
        if timestamp_s < self._t265_history[-1].pose.timestamp_s - T265_HISTORY_S:
            return None
        previous: _TimedPose | None = None
        for current in self._t265_history:
            delta = current.pose.timestamp_s - timestamp_s
            if abs(delta) <= 1e-9:
                return current
            if delta > 0.0:
                if previous is None:
                    return None
                left_gap = timestamp_s - previous.pose.timestamp_s
                right_gap = current.pose.timestamp_s - timestamp_s
                if left_gap < 0.0 or right_gap < 0.0 or max(left_gap, right_gap) > MAX_INTERP_GAP_S:
                    return None
                span = current.pose.timestamp_s - previous.pose.timestamp_s
                if span <= 0.0:
                    return None
                u = left_gap / span
                yaw_delta = normalize_angle_rad(current.pose.yaw_rad - previous.pose.yaw_rad)
                return _TimedPose(
                    Pose2D(
                        previous.pose.x_m + u * (current.pose.x_m - previous.pose.x_m),
                        previous.pose.y_m + u * (current.pose.y_m - previous.pose.y_m),
                        normalize_angle_rad(previous.pose.yaw_rad + u * yaw_delta),
                        timestamp_s,
                    ),
                    min(previous.confidence, current.confidence),
                    max(left_gap, right_gap),
                )
            previous = current
        return None

    def update_d500_absolute(self, pose: Pose2D, quality: PoseQuality) -> None:
        """Correct the map anchor using a GLOBAL pose at its measurement time."""
        self._last_d500_accepted = False
        self._last_d500_rejection = None
        self._last_d500_innovation_m = None
        self._last_d500_innovation_yaw_rad = None
        self._last_t265_alignment_ms = None
        if not _pose_valid(pose, quality) or quality.degraded:
            self._last_d500_rejection = "invalid_or_degraded_global_pose"
            return
        self._seen_d500 = True

        aligned = self._sample_t265(pose.timestamp_s)
        if aligned is None:
            self._last_d500_rejection = "t265_time_alignment_unavailable"
            return
        self._last_t265_alignment_ms = aligned.alignment_error_s * 1000.0
        if aligned.confidence < T265_MIN_CONFIDENCE:
            self._last_d500_rejection = "t265_low_confidence_at_d500_time"
            return

        anchor_observation = compose_pose2d(pose, inverse_pose2d(aligned.pose))
        if self._map_T_t265_odom is None:
            self._map_T_t265_odom = Pose2D(
                anchor_observation.x_m, anchor_observation.y_m,
                anchor_observation.yaw_rad, pose.timestamp_s,
            )
            self._last_d500_innovation_m = 0.0
            self._last_d500_innovation_yaw_rad = 0.0
            self._accept_d500(pose, quality)
            return

        predicted = compose_pose2d(self._map_T_t265_odom, aligned.pose)
        dx = pose.x_m - predicted.x_m
        dy = pose.y_m - predicted.y_m
        dyaw = normalize_angle_rad(pose.yaw_rad - predicted.yaw_rad)
        innovation_m = math.hypot(dx, dy)
        self._last_d500_innovation_m = innovation_m
        self._last_d500_innovation_yaw_rad = dyaw
        if innovation_m > D500_MAX_INNOVATION_M:
            self._last_d500_rejection = "position_innovation_gate"
            return
        if abs(dyaw) > D500_MAX_INNOVATION_YAW_RAD:
            self._last_d500_rejection = "yaw_innovation_gate"
            return

        current = self._map_T_t265_odom
        self._map_T_t265_odom = Pose2D(
            current.x_m + ANCHOR_BLEND * (anchor_observation.x_m - current.x_m),
            current.y_m + ANCHOR_BLEND * (anchor_observation.y_m - current.y_m),
            normalize_angle_rad(current.yaw_rad + ANCHOR_BLEND * normalize_angle_rad(anchor_observation.yaw_rad - current.yaw_rad)),
            pose.timestamp_s,
        )
        self._accept_d500(pose, quality)

    def update_d500(self, pose: Pose2D, quality: PoseQuality) -> None:
        """Compatibility alias; input must be a reliable GLOBAL map pose."""
        self.update_d500_absolute(pose, quality)

    def estimate(self, now_s: float) -> FusedPoseEstimate:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite monotonic time")
        t265_age = _age(now, self._t265_pose)
        d500_age = _age(now, self._d500_pose)
        t265_fresh = (
            t265_age is not None and t265_age <= self.config.t265_max_age_s
            and self._t265_quality is not None and self._t265_quality.valid
            and _confidence(self._t265_quality) >= T265_MIN_CONFIDENCE
            and not self._t265_continuity_broken
        )
        if self._t265_pose is not None and not t265_fresh:
            self._break_t265_continuity()
        d500_fresh = (
            d500_age is not None and d500_age <= self.config.d500_max_age_s
            and self._d500_quality is not None and self._d500_quality.valid
        )
        fallback_fresh = self._fresh_d500_global_fallback(now)
        if t265_fresh and d500_fresh and self._map_T_t265_odom is not None:
            pose = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
            state, flags, age = PoseFusionState.OK, ("t265", "d500", "fused"), t265_age
        elif t265_fresh and self._map_T_t265_odom is not None:
            pose = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
            state, flags, age = PoseFusionState.D500_DEGRADED, ("t265", "dead_reckoning"), t265_age
        elif fallback_fresh:
            pose = self._d500_global_fallback_pose
            fallback_age = now - pose.timestamp_s
            state, flags, age = PoseFusionState.T265_DEGRADED, ("d500_global_fallback",), fallback_age
        elif t265_fresh:
            pose = None
            state, flags, age = PoseFusionState.UNANCHORED, ("t265", "unanchored"), t265_age
        elif not self._seen_t265 and not self._seen_d500:
            pose = None
            state, flags, age = PoseFusionState.INITIALIZING, (), None
        else:
            pose = None
            state, flags = PoseFusionState.LOST, ()
            ages = [value for value in (t265_age, d500_age) if value is not None]
            age = max(ages) if ages else None

        return FusedPoseEstimate(
            pose, state, age, flags, t265_age, d500_age,
            self._last_d500_innovation_m, self._last_d500_innovation_yaw_rad,
            self._last_d500_accepted, self._last_d500_rejection,
            self.global_anchor_established,
            None if self._t265_quality is None else _confidence(self._t265_quality),
            self._last_t265_alignment_ms,
            self._t265_continuity_broken,
        )

    def _accept_d500(self, pose: Pose2D, quality: PoseQuality) -> None:
        self._d500_pose = pose
        self._d500_quality = quality
        self._last_d500_accepted = True
        self._last_d500_rejection = None
