"""SE(2) operations for canonical metre/radian poses.

``compose_pose2d(a, b)`` composes ``a_T_b`` with ``b_T_c`` and returns
``a_T_c``. In other words, ``a`` is the parent/world transform and ``b`` is
the child-relative pose. Points are represented as ``(x_m, y_m)`` tuples.
"""

from __future__ import annotations

import math

from .types import Pose2D


def normalize_angle_rad(angle: float) -> float:
    """Normalize an angle to ``[-pi, pi)``."""

    value = float(angle)
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    normalized = (value + math.pi) % (2.0 * math.pi) - math.pi
    return 0.0 if abs(normalized) < 1e-15 else normalized


def compose_pose2d(a: Pose2D, b: Pose2D) -> Pose2D:
    """Compose ``a_T_b`` and ``b_T_c`` to produce ``a_T_c``.

    The result timestamp is inherited from ``b`` (the latest/local pose
    sample). Callers combining asynchronous observations should set an
    application-appropriate timestamp explicitly when constructing inputs.
    """

    cosine = math.cos(a.yaw_rad)
    sine = math.sin(a.yaw_rad)
    return Pose2D(
        x_m=a.x_m + cosine * b.x_m - sine * b.y_m,
        y_m=a.y_m + sine * b.x_m + cosine * b.y_m,
        yaw_rad=normalize_angle_rad(a.yaw_rad + b.yaw_rad),
        timestamp_s=b.timestamp_s,
    )


def inverse_pose2d(pose: Pose2D) -> Pose2D:
    """Return the inverse rigid transform, retaining the input timestamp."""

    inverse_yaw = normalize_angle_rad(-pose.yaw_rad)
    cosine = math.cos(inverse_yaw)
    sine = math.sin(inverse_yaw)
    return Pose2D(
        x_m=-(cosine * pose.x_m - sine * pose.y_m),
        y_m=-(sine * pose.x_m + cosine * pose.y_m),
        yaw_rad=inverse_yaw,
        timestamp_s=pose.timestamp_s,
    )


def transform_pose2d(parent_T_child: Pose2D, child_pose: Pose2D) -> Pose2D:
    """Express a child-frame pose in the parent frame using ``parent_T_child``."""

    return compose_pose2d(parent_T_child, child_pose)


def transform_point2d(
    parent_T_child: Pose2D, point_child_m: tuple[float, float]
) -> tuple[float, float]:
    """Transform one ``(x, y)`` point from child to parent coordinates."""

    x_child, y_child = map(float, point_child_m)
    cosine = math.cos(parent_T_child.yaw_rad)
    sine = math.sin(parent_T_child.yaw_rad)
    return (
        parent_T_child.x_m + cosine * x_child - sine * y_child,
        parent_T_child.y_m + sine * x_child + cosine * y_child,
    )
