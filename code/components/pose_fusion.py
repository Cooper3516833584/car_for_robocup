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
D500_MAX_INNOVATION_M = 0.30
D500_MAX_INNOVATION_YAW_RAD = math.radians(12.0)
# The slam_toolbox anchor corrects a transform that is rigidly fixed for a
# consistent T265/SLAM pair, so a disagreement of tens of centimetres is a
# scan-matching outlier rather than a real change.  Measured on the car: a
# 0.297 m anchor innovation (just inside D500_MAX_INNOVATION_M) yanked the fused
# pose 0.106 m in a single 50 ms sample and made every reverse drive_distance
# action diverge.  Reject those instead of blending them; the separate
# loop-closure consensus path above still admits deliberate map migrations.
SLAM_ANCHOR_MAX_INNOVATION_M = 0.10
# How many successive, mutually coherent SLAM anchors are needed before a
# persistent disagreement is treated as a real map/odometry re-solve rather
# than a scan-matching outlier.  Anchors arrive with the published scan rate
# (~4.5 Hz on the car), so three samples corroborate in well under a second
# while still rejecting a single bad transform.
SLAM_ANCHOR_CONSENSUS_SAMPLES = 3
ANCHOR_BLEND = 0.35
SLAM_LOOP_MAX_M = 1.0
SLAM_LOOP_MAX_YAW_RAD = math.radians(35.0)
SLAM_LOOP_CONSISTENCY_M = 0.10
SLAM_LOOP_CONSISTENCY_YAW_RAD = math.radians(5.0)
SLAM_LOOP_MIGRATION_S = 0.5
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

    def __init__(self, config: FusionConfig, *, backend: str = "legacy", require_field_anchor: bool = False) -> None:
        if backend not in {"legacy", "slam_toolbox"}:
            raise ValueError("unknown localization backend")
        self.config = config
        self.backend = backend
        self.require_field_anchor = require_field_anchor
        self._min_t265_confidence = config.t265_min_tracker_confidence / 3.0
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
        self._field_T_slam_map: Pose2D | None = None
        self._slam_received_s: float | None = None
        self._slam_candidates: deque[Pose2D] = deque(maxlen=3)
        self._slam_migration_start: Pose2D | None = None
        self._slam_migration_target: Pose2D | None = None
        self._slam_migration_started_s: float | None = None
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
    def continuous_t265_pose(self) -> Pose2D | None:
        return None if self._t265_continuity_broken else self._t265_pose

    @property
    def slam_feed_t265_pose(self) -> Pose2D | None:
        """Continuous T265 odometry for the SLAM bridge, ungated by the nav gate.

        ``continuous_t265_pose`` deliberately returns ``None`` while
        ``_t265_continuity_broken`` is set so navigation never consumes
        odometry that may have jumped.  The SLAM bridge must not be starved by
        that same gate: it is the only producer of ``map_T_t265_odom``, so
        cutting its input also removes the only mechanism that can clear the
        gate.  That is the latched ``fusion_source: LOST`` failure seen on
        2026-10-04 (empty scan window forever, ``scan_publish_count`` 0).

        This accessor always hands over rebased, continuous odometry -- the
        latest continuous pose, or the raw sample mapped through the active
        rebase when a newer raw sample has arrived but has not been adopted as
        a continuous pose yet.  It never changes the navigation gate.
        """
        raw = self._t265_raw_pose
        if raw is not None and (self._t265_pose is None or raw.timestamp_s > self._t265_pose.timestamp_s):
            return compose_pose2d(self._t265_rebase, raw)
        return self._t265_pose

    @property
    def field_T_slam_map(self) -> Pose2D | None:
        return self._field_T_slam_map

    def slam_anchor_age_s(self, now_s: float) -> float | None:
        if self._slam_received_s is None:
            return None
        return max(0.0, now_s - self._slam_received_s)

    @property
    def global_anchor_established(self) -> bool:
        return self._map_T_t265_odom is not None and (
            self.backend != "slam_toolbox" or not self.require_field_anchor
            or self._field_T_slam_map is not None
        )

    def update_slam_anchor(self, anchor: Pose2D, *, valid: bool, timestamp_s: float,
                           loop_closure: bool = False) -> None:
        """Accept a measured slam_map_T_t265_odom without changing T265 odometry."""
        self._last_d500_accepted = False
        self._last_d500_rejection = None
        if self.backend != "slam_toolbox":
            self._last_d500_rejection = "wrong_backend"
            return
        if (not valid or not all(math.isfinite(value) for value in
                                 (anchor.x_m, anchor.y_m, anchor.yaw_rad, timestamp_s))):
            self._last_d500_rejection = "invalid_slam_anchor"
            return
        if self._slam_received_s is not None and timestamp_s <= self._slam_received_s:
            self._last_d500_rejection = "out_of_order_slam_anchor"
            return
        if (self.continuous_t265_pose is None or self._t265_quality is None
                or not self._t265_quality.valid
                or abs(timestamp_s - self._t265_pose.timestamp_s) > self.config.t265_max_age_s):
            self._last_d500_rejection = "t265_unavailable_for_slam"
            return
        self._seen_d500 = True
        current = self._map_T_t265_odom
        if current is None:
            self._map_T_t265_odom = Pose2D(anchor.x_m, anchor.y_m, anchor.yaw_rad, timestamp_s)
            self._slam_received_s = timestamp_s
            self._last_d500_innovation_m = 0.0
            self._last_d500_innovation_yaw_rad = 0.0
            self._last_d500_accepted = True
            return

        delta_m = math.hypot(anchor.x_m - current.x_m, anchor.y_m - current.y_m)
        delta_yaw = normalize_angle_rad(anchor.yaw_rad - current.yaw_rad)
        self._last_d500_innovation_m = delta_m
        self._last_d500_innovation_yaw_rad = delta_yaw
        if self._slam_migration_target is not None:
            target = self._slam_migration_target
            if (math.hypot(anchor.x_m - target.x_m, anchor.y_m - target.y_m)
                    <= SLAM_LOOP_CONSISTENCY_M
                    and abs(normalize_angle_rad(anchor.yaw_rad - target.yaw_rad))
                    <= SLAM_LOOP_CONSISTENCY_YAW_RAD):
                self._slam_received_s = timestamp_s
                self._last_d500_accepted = True
            else:
                self._last_d500_rejection = "loop_migration_target_changed"
            return
        if delta_m <= SLAM_ANCHOR_MAX_INNOVATION_M and abs(delta_yaw) <= D500_MAX_INNOVATION_YAW_RAD:
            self._slam_candidates.clear()
            self._map_T_t265_odom = Pose2D(
                current.x_m + ANCHOR_BLEND * (anchor.x_m - current.x_m),
                current.y_m + ANCHOR_BLEND * (anchor.y_m - current.y_m),
                normalize_angle_rad(current.yaw_rad + ANCHOR_BLEND * delta_yaw),
                timestamp_s,
            )
            self._slam_received_s = timestamp_s
            self._last_d500_accepted = True
            return
        if delta_m > SLAM_LOOP_MAX_M or abs(delta_yaw) > SLAM_LOOP_MAX_YAW_RAD:
            # Far too large to be a map re-solve: treat as a scan-matching
            # outlier and drop it outright.
            self._slam_candidates.clear()
            self._last_d500_rejection = "slam_innovation_gate"
            return
        # A *single* large innovation is still rejected (2026-10-04: a one-shot
        # 0.297 m anchor step moved the fused pose 0.106 m in one sample and made
        # every reverse drive_distance action diverge).  A *repeatable*,
        # coherent disagreement is a different thing: slam_toolbox re-solved the
        # pose graph and its map->odom transform really moved.  Requiring
        # `loop_closure` for that path latched fusion into d500_degraded forever
        # when slam_toolbox announced no closure event -- measured on the car
        # 2026-10-05: a persistent 0.331 m step at t=46 s never recovered and
        # needed a process restart.  Corroborate with SLAM_ANCHOR_CONSENSUS_SAMPLES
        # coherent transforms instead, then ramp to the new anchor over
        # SLAM_LOOP_MIGRATION_S so the fused pose still never jumps in one
        # sample.  An isolated outlier is cleared by the next in-gate anchor
        # above; the loop-closure event is no longer a precondition.
        if self._slam_candidates:
            previous = self._slam_candidates[-1]
            if (math.hypot(anchor.x_m - previous.x_m, anchor.y_m - previous.y_m)
                    > SLAM_LOOP_CONSISTENCY_M
                    or abs(normalize_angle_rad(anchor.yaw_rad - previous.yaw_rad))
                    > SLAM_LOOP_CONSISTENCY_YAW_RAD):
                self._slam_candidates.clear()
        self._slam_candidates.append(anchor)
        if len(self._slam_candidates) < SLAM_ANCHOR_CONSENSUS_SAMPLES:
            # Keep the historical reason for an announced loop closure; report a
            # plain persistent disagreement with the innovation gate so the two
            # cases stay distinguishable in the field log.
            self._last_d500_rejection = (
                "loop_anchor_waiting_for_consensus" if loop_closure else "slam_innovation_gate")
            return
        self._slam_migration_start = current
        self._slam_migration_target = anchor
        self._slam_migration_started_s = timestamp_s
        self._slam_candidates.clear()
        self._slam_received_s = timestamp_s
        self._last_d500_accepted = True

    def update_field_wall(self, pose: Pose2D, quality: PoseQuality) -> bool:
        """Align field to slam_map using an accepted fixed-wall observation."""
        if self.backend != "slam_toolbox" or not _pose_valid(pose, quality) or quality.degraded:
            return False
        aligned = self._sample_t265(pose.timestamp_s)
        if (aligned is None or aligned.confidence < self._min_t265_confidence
                or self._map_T_t265_odom is None or self._slam_received_s is None
                or abs(pose.timestamp_s - self._slam_received_s) > 0.5
                or self._slam_migration_target is not None):
            return False
        slam_base = compose_pose2d(self._map_T_t265_odom, aligned.pose)
        candidate = compose_pose2d(pose, inverse_pose2d(slam_base))
        old = self._field_T_slam_map
        if old is None:
            self._field_T_slam_map = candidate
            return True
        dx, dy = candidate.x_m - old.x_m, candidate.y_m - old.y_m
        dyaw = normalize_angle_rad(candidate.yaw_rad - old.yaw_rad)
        if math.hypot(dx, dy) > D500_MAX_INNOVATION_M or abs(dyaw) > D500_MAX_INNOVATION_YAW_RAD:
            return False
        self._field_T_slam_map = Pose2D(
            old.x_m + ANCHOR_BLEND * dx,
            old.y_m + ANCHOR_BLEND * dy,
            normalize_angle_rad(old.yaw_rad + ANCHOR_BLEND * dyaw),
            pose.timestamp_s,
        )
        return True

    def _advance_slam_migration(self, now_s: float) -> None:
        if (self._slam_migration_target is None or self._slam_migration_start is None
                or self._slam_migration_started_s is None):
            return
        start, target = self._slam_migration_start, self._slam_migration_target
        fraction = min(1.0, max(0.0, (now_s - self._slam_migration_started_s) / SLAM_LOOP_MIGRATION_S))
        self._map_T_t265_odom = Pose2D(
            start.x_m + fraction * (target.x_m - start.x_m),
            start.y_m + fraction * (target.y_m - start.y_m),
            normalize_angle_rad(start.yaw_rad + fraction * normalize_angle_rad(target.yaw_rad - start.yaw_rad)),
            now_s,
        )
        if fraction >= 1.0:
            self._slam_migration_start = self._slam_migration_target = None
            self._slam_migration_started_s = None

    def update_t265(self, pose: Pose2D | None, quality: PoseQuality) -> None:
        """Add canonical adapter output, rebasing tracker frame jumps continuously."""
        if pose is None or not _pose_valid(pose, quality):
            self._t265_quality = quality
            self._seen_t265 = True
            self._break_t265_continuity()
            return
        confidence = _confidence(quality)
        if confidence < self._min_t265_confidence:
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
            if self._t265_pose is None:
                # The gate latched before any continuous odometry existed (the
                # usual cause is a low-confidence or missing first frame at
                # start-up).  There is nothing to be discontinuous with, so
                # adopt this sample as the new odometry origin.  Without this
                # branch the very first break latched forever: every later
                # valid sample took the early return below, ``_t265_pose`` was
                # never set again, ``continuous_t265_pose`` stayed ``None``,
                # slam_toolbox was never fed and the fused pose stayed LOST
                # until the process was restarted.
                self._t265_rebase = Pose2D(0.0, 0.0, 0.0, pose.timestamp_s)
                self._t265_continuity_broken = False
                self._t265_history.clear()
                continuous = pose
            elif self.backend == "slam_toolbox":
                # Keep odom continuous after tracker recovery.  Navigation
                # remains gated until slam_toolbox supplies a fresh correction.
                self._t265_rebase = compose_pose2d(
                    self._t265_pose, inverse_pose2d(pose)
                )
                self._t265_continuity_broken = False
                self._t265_history.clear()
                continuous = compose_pose2d(self._t265_rebase, pose)
            elif self._fresh_d500_global_fallback(pose.timestamp_s):
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
        if aligned.confidence < self._min_t265_confidence:
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
        if self.backend == "slam_toolbox":
            self._advance_slam_migration(now)
        t265_age = _age(now, self._t265_pose)
        d500_age = (None if self._slam_received_s is None else max(0.0, now - self._slam_received_s)) if self.backend == "slam_toolbox" else _age(now, self._d500_pose)
        t265_fresh = (
            t265_age is not None and t265_age <= self.config.t265_max_age_s
            and self._t265_quality is not None and self._t265_quality.valid
            and _confidence(self._t265_quality) >= self._min_t265_confidence
            and not self._t265_continuity_broken
        )
        if self._t265_pose is not None and not t265_fresh:
            self._break_t265_continuity()
        d500_fresh = (self.backend == "slam_toolbox" and d500_age is not None and d500_age <= 0.5) or (
            self.backend != "slam_toolbox" and
            d500_age is not None and d500_age <= self.config.d500_max_age_s
            and self._d500_quality is not None and self._d500_quality.valid
        )
        fallback_fresh = self.backend != "slam_toolbox" and self._fresh_d500_global_fallback(now)
        if t265_fresh and d500_fresh and self._map_T_t265_odom is not None:
            pose = compose_pose2d(self._map_T_t265_odom, self._t265_pose)
            state, flags, age = PoseFusionState.OK, (("t265", "slam", "fused") if self.backend == "slam_toolbox" else ("t265", "d500", "fused")), t265_age
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

        if (self.backend == "slam_toolbox" and pose is not None and self.require_field_anchor):
            if self._field_T_slam_map is None:
                pose = None
                state, flags = PoseFusionState.UNANCHORED, ("t265", "slam", "field_unanchored")
            else:
                pose = compose_pose2d(self._field_T_slam_map, pose)

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
