from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.differential_drive import DifferentialDrive
from components.differential_kinematics import DifferentialGeometry
from components.radar_driver import GlobalCorrectionMode, Pose2D as RadarPose2D
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality
from localization.field_reference import build_field_wall_reference
import robocup_runtime


def ready_config():
    source = load_v2_config()
    return replace(
        source,
        calibration=replace(source.calibration, geometry_measured=True, sensor_extrinsics_measured=True),
        competition_map=replace(source.competition_map, measured=True),
        footprint=replace(source.footprint, measured=True),
        d500_localization=replace(source.d500_localization, reference_measured=True),
        t265=replace(source.t265, enabled=False),
    )


def fake_drive(config, *, clock):
    backend = FakeDriveBackend(max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s)
    return DifferentialDrive(
        backend,
        geometry=DifferentialGeometry(config.geometry.drive_track_width_m),
        max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s,
        max_linear_speed_m_s=config.drive.max_linear_speed_m_s,
        max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s,
        max_linear_accel_m_s2=config.drive.max_linear_accel_m_s2,
        max_angular_accel_rad_s2=config.drive.max_angular_accel_rad_s2,
        command_timeout_s=config.drive.command_timeout_s,
        clock=clock,
    )


class D500AbsoluteLocalizationTests(unittest.TestCase):
    def test_reference_builder_does_not_assume_unconfigured_far_walls(self) -> None:
        config = replace(
            ready_config().d500_localization,
            field_width_m=8.0,
            field_height_m=6.0,
            use_front_wall=False,
            use_left_wall=False,
        )
        reference = build_field_wall_reference(config)
        self.assertEqual(reference.back_wall_x_cm, 0.0)
        self.assertEqual(reference.right_wall_y_cm, 0.0)
        self.assertIsNone(reference.front_wall_x_cm)
        self.assertIsNone(reference.left_wall_y_cm)

    def test_hardware_runtime_instantiates_absolute_wall_localizer(self) -> None:
        config = ready_config()
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: fake_drive(cfg, clock=kwargs["clock"]),
        ):
            runtime = robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=lambda: 1.0)
        radar = runtime.d500_source
        self.assertIsNotNone(radar.alignment)
        self.assertIsNotNone(radar.wall_localizer)
        self.assertIs(radar.global_correction_mode, GlobalCorrectionMode.UPDATE_ALIGNMENT)
        runtime.close()

    def _runtime(self):
        config = ready_config()
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: fake_drive(cfg, clock=kwargs["clock"]),
        ):
            return robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=lambda: 1.0)

    def test_runtime_adapts_absolute_pose_and_local_only_fallback(self) -> None:
        runtime = self._runtime()
        update = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 200.0, 0.0), icp=None),
            global_pose=RadarPose2D(150.0, 250.0, 10.0),
            global_is_absolute=True,
            global_confidence=0.9,
            wall_fusion=None,
        )
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        estimate = runtime.fusion.estimate(1.0)
        self.assertAlmostEqual(estimate.pose.x_m, 1.5)
        self.assertAlmostEqual(estimate.pose.y_m, 2.5)
        self.assertAlmostEqual(estimate.pose.yaw_rad, -0.1745329252)

        local_runtime = self._runtime()
        update.global_is_absolute = False
        update.global_confidence = None
        local_runtime.on_d500_update(update)
        local_runtime._consume_d500(1.0)
        local_pose = local_runtime.fusion.estimate(1.0).pose
        self.assertAlmostEqual(local_pose.x_m, 1.0)
        self.assertAlmostEqual(local_pose.y_m, 2.0)
        local_runtime.close()
        runtime.close()

    def test_low_confidence_and_large_position_jump_are_rejected_as_absolute(self) -> None:
        runtime = self._runtime()
        runtime.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("d500", True, False))
        sample = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 100.0, 0.0), icp=None),
            global_pose=RadarPose2D(120.0, 100.0, 0.0),
            global_is_absolute=True,
            global_confidence=0.2,
            wall_fusion=None,
        )
        runtime.on_d500_update(sample)
        runtime._consume_d500(1.0)
        self.assertEqual(runtime.d500_abs_reject_low_confidence, 1)
        self.assertAlmostEqual(runtime.fusion.estimate(1.0).pose.x_m, 1.0)

        sample.global_confidence = 0.9
        sample.global_pose = RadarPose2D(300.0, 100.0, 0.0)
        runtime.on_d500_update(sample)
        runtime._consume_d500(1.0)
        self.assertEqual(runtime.d500_abs_reject_position_gate, 1)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
