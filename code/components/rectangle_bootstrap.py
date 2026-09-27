#!/usr/bin/env python3
"""Rectangle-boundary bootstrap that reports orientation ambiguity explicitly.

Why this cannot return a single pose
------------------------------------
Observing only the four safety-net edges of a **square** field, these four
global orientations are geometrically indistinguishable:

    yaw, yaw + 90 deg, yaw + 180 deg, yaw + 270 deg

Every one of them explains the same point cloud equally well.  Numerical noise
will always make one candidate score marginally better, so picking the best
scoring candidate would silently build a map that is rotated by a multiple of
90 degrees.  A confidently wrong coordinate frame is far more dangerous than no
frame at all: navigation would drive in the wrong direction with full
confidence.

T265 cannot resolve this either.  Its verified ``yaw = pi`` mount establishes
that T265 ``+X`` agrees with ``base_link +X``; that is only a *relative* motion
convention.  It says nothing about whether the car's nose currently points
along the competition field's ``+X``, ``+Y``, ``-X`` or ``-Y``.

Therefore this module returns a *candidate set* plus an explicit
``orientation_resolved`` flag.  Only ``resolve_bootstrap_candidates()`` -- fed
by something that genuinely carries global meaning, such as a rule-mandated
fixed start heading, a unique field landmark, or an operator's one-off choice --
may collapse the set to one.  Until then the answer is AMBIGUOUS and no map
anchor may be established.

Partial visibility
------------------
The D500 reaches about 6.5 m and the competition field is 10 x 10 m, so both
opposite edges of one axis are simultaneously visible only when the car sits
roughly 3.5-6.5 m from each.  Requiring all four edges would fail over most of
the field, so two perpendicular edges are enough to generate candidates.
A single edge cannot: it leaves the position along the wall unobservable, so no
full 3-DoF bootstrap is possible from it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .radar_diagnostics import extract_lines
from .radar_driver import Pose2D, rotate_cw

__all__ = [
    "OrientationAmbiguity",
    "RectangleBootstrapCandidate",
    "RectangleBootstrapResult",
    "bootstrap_rectangle",
    "resolve_bootstrap_candidates",
]


class OrientationAmbiguity:
    """How many global orientations remain consistent with the observation."""

    RESOLVED = 1
    MIRRORED = 2        # a non-square rectangle: 180 deg ambiguity remains
    SQUARE = 4          # square field: 90 deg ambiguity remains


@dataclass(frozen=True, slots=True)
class RectangleBootstrapCandidate:
    """One globally consistent interpretation of the observed rectangle."""

    pose_in_world: Pose2D
    rotation_index: int
    """0..3, the multiple of 90 degrees applied to the local frame."""
    rms_cm: float


@dataclass(frozen=True, slots=True)
class RectangleBootstrapResult:
    candidates: tuple[RectangleBootstrapCandidate, ...] = ()
    geometry_valid: bool = False
    orientation_resolved: bool = False
    ambiguity_order: int = 0
    observed_width_cm: float = 0.0
    observed_height_cm: float = 0.0
    reason: str | None = None
    diagnostics: dict = field(default_factory=dict)

    @property
    def candidate_count(self) -> int:
        return len(self.candidates)


def _axis_clusters(values, *, tolerance_cm: float, min_points: int):
    """Group 1-D coordinates into tight clusters, densest and tightest first."""

    ordered = sorted(float(v) for v in values)
    clusters = []
    start = 0
    while start < len(ordered):
        end = start
        while (
            end + 1 < len(ordered)
            and ordered[end + 1] - ordered[start] <= tolerance_cm
        ):
            end += 1
        members = ordered[start : end + 1]
        if len(members) >= min_points:
            centre = members[len(members) // 2]
            rms = math.sqrt(
                sum((m - centre) ** 2 for m in members) / len(members)
            )
            clusters.append((centre, len(members), rms))
        start += 1
    return clusters


def _pick_walls(clusters, *, field_cm: float, min_span_cm: float,
                tolerance_cm: float):
    """Choose the two clusters that best form a known-extent wall pair.

    Returns ``(near, far)`` centres or ``None``.  The pair must reproduce the
    configured field extent, which is what rejects interior clutter: a box edge
    or a short interior wall will not sit a full field width away from another
    edge, so it cannot form a valid pair.
    """

    best = None
    for i in range(len(clusters)):
        for j in range(i + 1, len(clusters)):
            separation = abs(clusters[j][0] - clusters[i][0])
            if separation < min_span_cm:
                continue
            error = abs(separation - field_cm)
            if error > tolerance_cm:
                continue
            score = (error, -(clusters[i][1] + clusters[j][1]))
            if best is None or score < best[0]:
                best = (score, i, j)
    if best is None:
        return None
    _, i, j = best
    near, far = clusters[i], clusters[j]
    if near[0] > far[0]:
        near, far = far, near
    return near, far


def _rotate_pose(pose: Pose2D, yaw_cw_deg: float) -> Pose2D:
    """Rotate a local pose into a frame rotated by ``yaw_cw_deg``."""

    x, y = rotate_cw(pose.x_cm, pose.y_cm, yaw_cw_deg)
    return Pose2D(x, y, pose.yaw_cw_deg + yaw_cw_deg)


def _translate_pose(pose: Pose2D, dx_cm: float, dy_cm: float) -> Pose2D:
    return Pose2D(pose.x_cm + dx_cm, pose.y_cm + dy_cm, pose.yaw_cw_deg)


def bootstrap_rectangle(
    points,
    *,
    field_width_cm: float,
    field_height_cm: float,
    min_line_span_cm: float = 50.0,
    extent_tolerance_cm: float = 60.0,
    cluster_tolerance_cm: float = 8.0,
    min_points_per_wall: int = 12,
    min_axis_error_deg: float = 15.0,
) -> RectangleBootstrapResult:
    """Fit the safety-net rectangle and return every consistent global pose.

    ``points`` are the accumulated body-frame points of several complete scans
    while the car is stationary, so no deskew or map transform is involved.

    A square field yields four candidates and ``orientation_resolved = False``.
    A non-square rectangle yields two (the 180 deg ambiguity remains).  One
    edge only is not enough for a full bootstrap.
    """

    if field_width_cm <= 0 or field_height_cm <= 0:
        raise ValueError("field dimensions must be positive")

    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) < 4 * max(2, min_points_per_wall):
        return RectangleBootstrapResult(
            reason="not_enough_points", diagnostics={"points": len(pts)}
        )

    # Dominant edge direction, then work in a frame aligned with it.
    fits = extract_lines(
        pts,
        inlier_gate_cm=max(4.0, cluster_tolerance_cm / 2.0),
        min_points=min_points_per_wall,
        min_span_cm=min_line_span_cm,
        max_lines=8,
    )
    if not fits:
        return RectangleBootstrapResult(
            reason="no_edge_lines", diagnostics={"points": len(pts)}
        )

    def undirected_delta(a_deg: float, b_deg: float) -> float:
        return abs(((a_deg - b_deg) + 90.0) % 180.0 - 90.0)

    longest = max(fits, key=lambda item: item[0].span_cm)[0]
    base_angle = longest.angle_deg

    x_like = []   # edges perpendicular to base_angle (constant along base)
    y_like = []   # edges parallel to base_angle
    for fit, _ in fits:
        if undirected_delta(fit.angle_deg, base_angle) <= min_axis_error_deg:
            y_like.append(fit)
        elif abs(undirected_delta(fit.angle_deg, base_angle) - 90.0) <= min_axis_error_deg:
            x_like.append(fit)
    if not x_like or not y_like:
        return RectangleBootstrapResult(
            reason="needs_two_perpendicular_edges",
            diagnostics={"x_like": len(x_like), "y_like": len(y_like)},
        )

    # Rotate every point into the frame where the base direction is +X.
    cos_a = math.cos(math.radians(base_angle))
    sin_a = math.sin(math.radians(base_angle))
    rotated = []
    for x, y in pts:
        rx = cos_a * x + sin_a * y
        ry = -sin_a * x + cos_a * y
        rotated.append((rx, ry))

    xs = [p[0] for p in rotated]
    ys = [p[1] for p in rotated]

    clusters_y = _axis_clusters(ys, tolerance_cm=cluster_tolerance_cm,
                                min_points=min_points_per_wall)
    clusters_x = _axis_clusters(xs, tolerance_cm=cluster_tolerance_cm,
                                min_points=min_points_per_wall)

    # In this rotated frame the base-parallel edges vary along y.
    pair_h = _pick_walls(clusters_y, field_cm=field_height_cm,
                         min_span_cm=min_line_span_cm,
                         tolerance_cm=extent_tolerance_cm)
    pair_w = _pick_walls(clusters_x, field_cm=field_width_cm,
                         min_span_cm=min_line_span_cm,
                         tolerance_cm=extent_tolerance_cm)

    if pair_h is None and pair_w is None:
        # Two perpendicular edges but neither opposite pair is visible: this is
        # the common corner case and still determines yaw plus the distances to
        # those two edges.  Candidates are bounded by the corner hypothesis.
        return _corner_bootstrap(
            rotated, clusters_x, clusters_y, base_angle,
            field_width_cm=field_width_cm, field_height_cm=field_height_cm,
            diagnostics={"x_like": len(x_like), "y_like": len(y_like)},
        )
    if pair_h is None or pair_w is None:
        return RectangleBootstrapResult(
            reason="incomplete_rectangle",
            diagnostics={
                "x_clusters": len(clusters_x),
                "y_clusters": len(clusters_y),
                "pair_w": pair_w is not None,
                "pair_h": pair_h is not None,
            },
        )

    x_near, x_far = pair_w
    y_near, y_far = pair_h
    observed_w = x_far[0] - x_near[0]
    observed_h = y_far[0] - y_near[0]
    rms = math.sqrt(
        (x_near[2] ** 2 + x_far[2] ** 2 + y_near[2] ** 2 + y_far[2] ** 2) / 4.0
    )

    # Robot pose in the rectangle's own corner frame: shift the rotated-frame
    # pose so the near walls become x=0 / y=0.
    robot_local = Pose2D(0.0, 0.0, base_angle)
    robot_corner = _translate_pose(robot_local, -x_near[0], -y_near[0])

    square = abs(observed_w - observed_h) <= extent_tolerance_cm
    return _enumerate_candidates(
        robot_corner,
        field_width_cm=field_width_cm,
        field_height_cm=field_height_cm,
        rms_cm=rms,
        square=square,
        observed_width_cm=observed_w,
        observed_height_cm=observed_h,
        diagnostics={
            "x_like": len(x_like),
            "y_like": len(y_like),
            "observed_width_cm": observed_w,
            "observed_height_cm": observed_h,
        },
    )


def _corner_bootstrap(rotated, clusters_x, clusters_y, base_angle, *,
                      field_width_cm, field_height_cm, diagnostics):
    """Two perpendicular edges only: yaw plus two distances, no full position."""

    if not clusters_x or not clusters_y:
        return RectangleBootstrapResult(
            reason="needs_two_perpendicular_edges", diagnostics=diagnostics
        )
    # Which of the two visible edges is nearer decides the corner, but the
    # along-wall positions stay unobservable, so a full 3-DoF bootstrap is not
    # possible.  Report it explicitly rather than inventing a position.
    return RectangleBootstrapResult(
        reason="single_axis_position_unobservable",
        diagnostics=diagnostics,
    )


def _enumerate_candidates(
    robot_corner: Pose2D, *, field_width_cm, field_height_cm, rms_cm,
    square: bool, observed_width_cm, observed_height_cm, diagnostics,
):
    """Build every global pose consistent with a rectangle observation.

    A square field admits all four corners as equally valid world origins.
    For a non-square field the 90 deg and 270 deg hypotheses additionally swap
    the width and height axes, which contradicts the measured extents, so only
    the 0 deg and 180 deg hypotheses survive: the 180 deg ambiguity is intrinsic
    to observing edges and cannot be removed by dimensions alone.
    """

    corners = [
        (0.0, 0.0, 0.0),
        (field_width_cm, 0.0, 90.0),
        (field_width_cm, field_height_cm, 180.0),
        (0.0, field_height_cm, 270.0),
    ]
    if not square:
        corners = [corners[0], corners[2]]

    candidates = []
    for index, (corner_x, corner_y, yaw_cw_deg) in enumerate(corners):
        rotated = _rotate_pose(robot_corner, yaw_cw_deg)
        pose = _translate_pose(rotated, corner_x, corner_y)
        candidates.append(
            RectangleBootstrapCandidate(
                pose_in_world=pose, rotation_index=index, rms_cm=rms_cm
            )
        )

    order = OrientationAmbiguity.SQUARE if square else OrientationAmbiguity.MIRRORED
    return RectangleBootstrapResult(
        candidates=tuple(candidates),
        geometry_valid=True,
        # Never resolved here: no global information has been consumed.
        orientation_resolved=False,
        ambiguity_order=order,
        observed_width_cm=observed_width_cm,
        observed_height_cm=observed_height_cm,
        reason="ambiguous_orientation",
        diagnostics=diagnostics,
    )


def resolve_bootstrap_candidates(candidates, prior=None):
    """Collapse an ambiguous candidate set using genuinely global information.

    ``prior`` must carry real global meaning, for example a rule-mandated fixed
    start heading, a surveyed start region, a unique field landmark, or an
    operator's one-off discrete choice.  T265 relative motion is NOT such a
    prior and must never be passed here.

    Returns ``(result, reason)``; until a prior exists the result stays
    ambiguous on purpose.
    """

    if not candidates:
        return candidates, "no_candidates"
    if prior is None:
        return candidates, "no_global_prior"
    if candidates.orientation_resolved:
        return candidates, "already_resolved"

    selector = getattr(prior, "select_candidate", None)
    if selector is None:
        return candidates, "prior_has_no_selector"
    chosen = selector(candidates.candidates)
    if chosen is None:
        return candidates, "prior_selected_nothing"
    if chosen not in candidates.candidates:
        return candidates, "prior_selected_unknown_candidate"

    return (
        RectangleBootstrapResult(
            candidates=(chosen,),
            geometry_valid=candidates.geometry_valid,
            orientation_resolved=True,
            ambiguity_order=candidates.ambiguity_order,
            observed_width_cm=candidates.observed_width_cm,
            observed_height_cm=candidates.observed_height_cm,
            reason="resolved_by_prior",
            diagnostics=dict(candidates.diagnostics),
        ),
        "resolved_by_prior",
    )
