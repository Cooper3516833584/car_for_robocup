from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.t265_driver import FakeT265PoseSource, T265RawPose
from components.t265_pose_adapter import (
    T265PoseAdapter,
    _matrix_to_quaternion,
    _mat_mul,
    _transpose,
)
from config.v2_models import SensorMount3DConfig
from config.v2_factory import build_t265_source
from config.v2_loader import load_v2_config

_ROBOT_TO_NATIVE = _transpose(((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))


def robot_pose_raw(
    x: float,
    y: float,
    yaw: float,
    timestamp: float,
    *,
    confidence: int = 3,
    quaternion: tuple[float, float, float, float] | None = None,
) -> T265RawPose:
    cy, sy = math.cos(yaw), math.sin(yaw)
    robot_rotation = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    native_rotation = _mat_mul(_mat_mul(_ROBOT_TO_NATIVE, robot_rotation), _transpose(_ROBOT_TO_NATIVE))
    native_translation = tuple(
        sum(_ROBOT_TO_NATIVE[row][col] * (x, y, 0.0)[col] for col in range(3))
        for row in range(3)
    )
    return T265RawPose(
        translation_xyz=native_translation,
        quaternion_xyzw=_matrix_to_quaternion(native_rotation) if quaternion is None else quaternion,
        velocity_xyz=None,
        angular_velocity_xyz=None,
        tracker_confidence=confidence,
        mapper_confidence=None,
        device_timestamp_ms=timestamp * 1000.0,
        host_monotonic_s=timestamp,
    )


def mount(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return SensorMount3DConfig(x, y, z, roll, pitch, yaw)


class T265PoseAdapterTests(unittest.TestCase):
    def test_identity_mount_and_first_sample_rebase(self) -> None:
        adapter = T265PoseAdapter(mount())
        update = adapter.adapt(robot_pose_raw(2.0, -1.0, 0.6, 1.0), now_s=1.01)

        self.assertTrue(update.quality.valid)
        self.assertIsNotNone(update.pose)
        self.assertAlmostEqual(update.pose.x_m, 0.0)
        self.assertAlmostEqual(update.pose.y_m, 0.0)
        self.assertAlmostEqual(update.pose.yaw_rad, 0.0)

    def test_ninety_degree_mount_yaw_is_compensated(self) -> None:
        adapter = T265PoseAdapter(mount(yaw=math.pi / 2.0), max_yaw_jump_rad=2.0)
        first = adapter.adapt(robot_pose_raw(0.0, 0.0, math.pi / 2.0, 1.0), now_s=1.0)
        second = adapter.adapt(robot_pose_raw(0.0, 0.0, math.pi, 1.1), now_s=1.1)

        self.assertAlmostEqual(first.pose.yaw_rad, 0.0, places=12)
        self.assertAlmostEqual(second.pose.yaw_rad, math.pi / 2.0, places=12)

    def test_sensor_forward_translation_maps_to_base_forward(self) -> None:
        adapter = T265PoseAdapter(mount())
        adapter.adapt(robot_pose_raw(0.0, 0.0, 0.0, 1.0), now_s=1.0)
        update = adapter.adapt(robot_pose_raw(0.4, 0.0, 0.0, 1.1), now_s=1.1)

        self.assertAlmostEqual(update.pose.x_m, 0.4, places=12)
        self.assertAlmostEqual(update.pose.y_m, 0.0, places=12)

    def test_mount_offset_removes_false_translation_during_in_place_rotation(self) -> None:
        adapter = T265PoseAdapter(mount(x=0.2), max_yaw_jump_rad=2.0)
        first = adapter.adapt(robot_pose_raw(0.2, 0.0, 0.0, 1.0), now_s=1.0)
        second = adapter.adapt(robot_pose_raw(0.0, 0.2, math.pi / 2.0, 1.1), now_s=1.1)

        self.assertTrue(first.quality.valid)
        self.assertTrue(second.quality.valid)
        self.assertAlmostEqual(second.pose.x_m, 0.0, places=12)
        self.assertAlmostEqual(second.pose.y_m, 0.0, places=12)
        self.assertAlmostEqual(second.pose.yaw_rad, math.pi / 2.0, places=12)

    def test_low_tracker_confidence_is_invalid(self) -> None:
        update = T265PoseAdapter(mount()).adapt(
            robot_pose_raw(0.0, 0.0, 0.0, 1.0, confidence=1), now_s=1.0
        )
        self.assertFalse(update.quality.valid)
        self.assertEqual(update.reason, "tracker_confidence_too_low")
        self.assertIsNone(update.pose)

    def test_stale_sample_is_invalid(self) -> None:
        update = T265PoseAdapter(mount(), max_age_s=0.15).adapt(
            robot_pose_raw(0.0, 0.0, 0.0, 1.0), now_s=1.2
        )
        self.assertFalse(update.quality.valid)
        self.assertEqual(update.reason, "stale_or_future_sample")

    def test_nan_quaternion_is_invalid(self) -> None:
        update = T265PoseAdapter(mount()).adapt(
            robot_pose_raw(0.0, 0.0, 0.0, 1.0, quaternion=(float("nan"), 0.0, 0.0, 1.0)),
            now_s=1.0,
        )
        self.assertFalse(update.quality.valid)
        self.assertEqual(update.reason, "non_finite_pose")

    def test_large_jump_is_rejected_without_rebasing(self) -> None:
        adapter = T265PoseAdapter(mount(), max_translation_jump_m=0.5)
        adapter.adapt(robot_pose_raw(0.0, 0.0, 0.0, 1.0), now_s=1.0)
        rejected = adapter.adapt(robot_pose_raw(2.0, 0.0, 0.0, 1.1), now_s=1.1)
        accepted = adapter.adapt(robot_pose_raw(0.1, 0.0, 0.0, 1.2), now_s=1.2)

        self.assertEqual(rejected.reason, "pose_jump")
        self.assertTrue(accepted.quality.valid)
        self.assertAlmostEqual(accepted.pose.x_m, 0.1, places=12)

    def test_fake_source_replays_samples_without_hardware(self) -> None:
        first = robot_pose_raw(0.0, 0.0, 0.0, 1.0)
        source = FakeT265PoseSource([first], repeat_last=True)
        source.start()

        self.assertIs(source.read(), first)
        self.assertIs(source.read(), first)
        self.assertIs(source.read(), first)
        source.stop()
        self.assertTrue(source.stopped)

    def test_factory_builds_fake_without_starting_a_device(self) -> None:
        source = build_t265_source(load_v2_config(), fake=True)
        self.assertIsInstance(source, FakeT265PoseSource)
        self.assertFalse(source.started)


if __name__ == "__main__":
    unittest.main()
