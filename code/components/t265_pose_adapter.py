"""Convert native T265 poses to canonical base_link ``Pose2D`` updates."""

from __future__ import annotations

from dataclasses import dataclass
import math

from config.v2_models import SensorMount3DConfig
from core.types import Pose2D, PoseQuality

from .t265_driver import T265RawPose

_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
# T265 native (right, up, back) vector -> robotics (forward, left, up).
_NATIVE_TO_ROBOT = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


@dataclass(frozen=True, slots=True)
class Transform3D:
    translation_xyz_m: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class T265PoseUpdate:
    pose: Pose2D | None
    quality: PoseQuality
    reason: str | None = None


def _mat_vec(matrix, vector):
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))


def _matrix_to_quaternion(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    return (x, y, z, w)


def _quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _rpy_matrix(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def compose_transform3d(parent_T_child: Transform3D, child_T_object: Transform3D) -> Transform3D:
    rotated = _mat_vec(parent_T_child.rotation_matrix, child_T_object.translation_xyz_m)
    return Transform3D(
        tuple(parent_T_child.translation_xyz_m[i] + rotated[i] for i in range(3)),
        _mat_mul(parent_T_child.rotation_matrix, child_T_object.rotation_matrix),
    )


def inverse_transform3d(transform: Transform3D) -> Transform3D:
    inverse_rotation = _transpose(transform.rotation_matrix)
    translated = _mat_vec(inverse_rotation, transform.translation_xyz_m)
    return Transform3D(tuple(-value for value in translated), inverse_rotation)


def mount_transform(config: SensorMount3DConfig) -> Transform3D:
    return Transform3D(
        (config.x_m, config.y_m, config.z_m),
        _rpy_matrix(config.roll_rad, config.pitch_rad, config.yaw_rad),
    )


def _native_pose_to_robot(raw: T265RawPose) -> Transform3D:
    native_rotation = _quaternion_to_matrix(raw.quaternion_xyzw)
    basis_inverse = _transpose(_NATIVE_TO_ROBOT)
    robot_rotation = _mat_mul(_mat_mul(_NATIVE_TO_ROBOT, native_rotation), basis_inverse)
    robot_translation = _mat_vec(_NATIVE_TO_ROBOT, raw.translation_xyz)
    return Transform3D(robot_translation, robot_rotation)


def _wrap_yaw(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class T265PoseAdapter:
    """Apply T265 axis conversion, mount compensation, rebasing and quality gates."""

    def __init__(
        self,
        mount: SensorMount3DConfig,
        *,
        min_tracker_confidence: int = 2,
        max_age_s: float = 0.15,
        max_translation_jump_m: float = 1.0,
        max_yaw_jump_rad: float = 1.0,
    ) -> None:
        self.mount = mount_transform(mount)
        self.mount_inverse = inverse_transform3d(self.mount)
        self.min_tracker_confidence = int(min_tracker_confidence)
        self.max_age_s = self._positive("max_age_s", max_age_s)
        self.max_translation_jump_m = self._positive("max_translation_jump_m", max_translation_jump_m)
        self.max_yaw_jump_rad = self._positive("max_yaw_jump_rad", max_yaw_jump_rad)
        if not 0 <= self.min_tracker_confidence <= 3:
            raise ValueError("min_tracker_confidence must be in [0, 3]")
        self._origin_inverse: Transform3D | None = None
        self._last_pose: Pose2D | None = None

    @staticmethod
    def _positive(name: str, value: float) -> float:
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return number

    def reset(self) -> None:
        self._origin_inverse = None
        self._last_pose = None

    def adapt(self, raw: T265RawPose, *, now_s: float) -> T265PoseUpdate:
        now = float(now_s)
        age = now - float(raw.host_monotonic_s)
        if not math.isfinite(now) or not math.isfinite(age) or age < 0.0 or age > self.max_age_s:
            return self._invalid(raw, max(0.0, age) if math.isfinite(age) else None, "stale_or_future_sample")
        if raw.tracker_confidence < self.min_tracker_confidence:
            return self._invalid(raw, age, "tracker_confidence_too_low")
        if len(raw.translation_xyz) != 3 or len(raw.quaternion_xyzw) != 4:
            return self._invalid(raw, age, "invalid_sample_shape")
        values = (*raw.translation_xyz, *raw.quaternion_xyzw)
        if not all(math.isfinite(float(value)) for value in values):
            return self._invalid(raw, age, "non_finite_pose")
        norm = math.sqrt(sum(float(value) ** 2 for value in raw.quaternion_xyzw))
        if not 0.8 <= norm <= 1.2:
            return self._invalid(raw, age, "invalid_quaternion_norm")

        odom_T_t265 = _native_pose_to_robot(raw)
        odom_T_base = compose_transform3d(odom_T_t265, self.mount_inverse)
        if self._origin_inverse is None:
            self._origin_inverse = inverse_transform3d(odom_T_base)
        local_T_base = compose_transform3d(self._origin_inverse, odom_T_base)
        matrix = local_T_base.rotation_matrix
        yaw = math.atan2(matrix[1][0], matrix[0][0])
        pose = Pose2D(
            local_T_base.translation_xyz_m[0],
            local_T_base.translation_xyz_m[1],
            _wrap_yaw(yaw),
            float(raw.host_monotonic_s),
        )

        if self._last_pose is not None:
            distance = math.hypot(pose.x_m - self._last_pose.x_m, pose.y_m - self._last_pose.y_m)
            yaw_delta = abs(_wrap_yaw(pose.yaw_rad - self._last_pose.yaw_rad))
            if distance > self.max_translation_jump_m or yaw_delta > self.max_yaw_jump_rad:
                return self._invalid(raw, age, "pose_jump")
        self._last_pose = pose
        confidence = min(1.0, max(0.0, raw.tracker_confidence / 3.0))
        return T265PoseUpdate(
            pose,
            PoseQuality("t265", True, False, confidence, confidence, age),
        )

    @staticmethod
    def _invalid(raw: T265RawPose, age_s: float | None, reason: str) -> T265PoseUpdate:
        return T265PoseUpdate(
            None,
            PoseQuality("t265", False, True, age_s=age_s),
            reason,
        )
