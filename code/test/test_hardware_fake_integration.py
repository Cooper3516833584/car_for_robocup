from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from components.c10b_diff_backend import FakeDriveBackend
from components.differential_drive import DifferentialDrive
from components.differential_kinematics import DifferentialGeometry
from components.navigation_common import NavigationGoal, NavigationGrid
from components.radar_driver import Pose2D as RadarPose2D
from components.t265_driver import T265RawPose
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import PoseQuality
from fakes.fake_c10b_backend import FakeC10BBackend
from fakes.fake_clock import FakeClock
from fakes.fake_d500 import FakeD500
from fakes.fake_payload import FakePayload
from fakes.fake_t265 import FakeT265
import robocup_runtime


def ready_config():
    source = load_v2_config()
    return replace(
        source,
        calibration=replace(source.calibration, geometry_measured=True, sensor_extrinsics_measured=True),
        competition_map=replace(source.competition_map, measured=True),
        footprint=replace(source.footprint, measured=True),
        d500_localization=replace(source.d500_localization, reference_measured=True),
    )


def make_drive(config, clock, backend=None):
    return DifferentialDrive(
        backend or FakeC10BBackend(max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s),
        geometry=DifferentialGeometry(config.geometry.drive_track_width_m),
        max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s,
        max_linear_speed_m_s=config.drive.max_linear_speed_m_s,
        max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s,
        max_linear_accel_m_s2=config.drive.max_linear_accel_m_s2,
        max_angular_accel_rad_s2=config.drive.max_angular_accel_rad_s2,
        command_timeout_s=config.drive.command_timeout_s,
        clock=clock,
    )


def t265_sample(now_s: float, forward_m: float = 0.0) -> T265RawPose:
    return T265RawPose(
        translation_xyz=(0.0, 0.0, -forward_m),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        velocity_xyz=(0.0, 0.0, 0.0),
        angular_velocity_xyz=(0.0, 0.0, 0.0),
        tracker_confidence=3,
        mapper_confidence=3,
        device_timestamp_ms=now_s * 1000.0,
        host_monotonic_s=now_s,
    )


def radar_update(*, local_x_cm: float, global_x_cm: float, confidence: float = 0.95):
    return types.SimpleNamespace(
        odometry=types.SimpleNamespace(
            accepted=True,
            pose=RadarPose2D(local_x_cm, 0.0, 0.0),
            icp=None,
            rejection_reason=None,
        ),
        global_pose=RadarPose2D(global_x_cm, 0.0, 0.0),
        map_alignment_established=True,
        map_pose_valid=True,
        absolute_observation_available=True,
        absolute_observation_accepted=True,
        global_confidence=confidence,
        wall_fusion=None,
    )


class HardwareFakeIntegrationTests(unittest.TestCase):
    def build_runtime(self, clock, t265, d500):
        config = ready_config()
        with (
            patch.object(robocup_runtime, "build_differential_drive", side_effect=lambda cfg, **_: make_drive(cfg, clock)),
            patch.object(robocup_runtime, "RealSenseT265PoseSource", return_value=t265),
            patch("components.radar_driver.D500RadarComponent", return_value=d500),
        ):
            runtime = robocup_runtime.build_runtime(
                config,
                RuntimeMode.HARDWARE_MISSION,
                clock=clock,
                world=NavigationGrid(120, 120, 0.1, origin_x_m=-6.0, origin_y_m=-6.0),
            )
        return runtime

    def test_t265_navigation_d500_correction_and_sensor_loss_safe_stop(self) -> None:
        clock = FakeClock(20.0)
        t265 = FakeT265([t265_sample(20.0)])
        d500 = FakeD500()
        runtime = self.build_runtime(clock, t265, d500)
        payload = FakePayload()
        self.assertEqual(payload.actions, [])
        runtime.start()
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        d500.emit(radar_update(local_x_cm=0.0, global_x_cm=0.0))
        first = runtime.step()
        self.assertIsNone(first.error)
        self.assertIsNotNone(first.navigation)
        self.assertTrue(runtime.drive.backend.commands)
        first_x = runtime.fusion.estimate(clock()).pose.x_m

        clock.advance(0.05)
        t265.samples.append(t265_sample(clock(), forward_m=0.02))
        second = runtime.step()
        self.assertIsNone(second.error)
        second_x = runtime.fusion.estimate(clock()).pose.x_m
        self.assertGreater(second_x, first_x)
        self.assertGreater(abs(runtime.drive.backend.commands[-1].left_m_s) + abs(runtime.drive.backend.commands[-1].right_m_s), 0.0)

        clock.advance(0.05)
        t265.samples.append(t265_sample(clock(), forward_m=0.04))
        d500.emit(radar_update(local_x_cm=4.0, global_x_cm=6.0))
        corrected = runtime.step()
        self.assertIsNone(corrected.error)
        corrected_x = runtime.fusion.estimate(clock()).pose.x_m
        self.assertGreater(corrected_x, second_x)
        self.assertLess(corrected_x - second_x, 0.10)

        clock.advance(0.85)
        lost = runtime.step()
        self.assertEqual(lost.mission_state, robocup_runtime.RobocupMissionState.SAFE_STOP)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()
        self.assertTrue(t265.stopped)
        self.assertTrue(d500.stopped)

    def test_hardware_backend_exception_enters_error_and_stops(self) -> None:
        clock = FakeClock(30.0)
        t265 = FakeT265([t265_sample(30.0)])
        d500 = FakeD500()
        runtime = self.build_runtime(clock, t265, d500)
        runtime.start()
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        d500.emit(radar_update(local_x_cm=0.0, global_x_cm=0.0))
        runtime.step()
        runtime.drive.backend.fail_with = OSError("fake C10B failure")
        clock.advance(0.05)
        t265.samples.append(t265_sample(clock()))
        result = runtime.step()
        self.assertEqual(result.mission_state, robocup_runtime.RobocupMissionState.ERROR)
        self.assertIn("fake C10B failure", result.error)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()

    def test_required_global_pose_blocks_local_only_motion_until_wall_anchor(self) -> None:
        clock = FakeClock(25.0)
        t265 = FakeT265([t265_sample(25.0)])
        d500 = FakeD500()
        runtime = self.build_runtime(clock, t265, d500)
        runtime.start()
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        local_only = radar_update(local_x_cm=0.0, global_x_cm=0.0)
        local_only.global_pose = None
        local_only.map_alignment_established = False
        local_only.map_pose_valid = False
        local_only.absolute_observation_available = False
        local_only.absolute_observation_accepted = False
        local_only.global_confidence = None
        d500.emit(local_only)
        waiting = runtime.step()
        self.assertIsNone(waiting.error)
        self.assertIsNone(waiting.navigation)
        self.assertFalse(runtime.drive.is_running)

        clock.advance(0.05)
        t265.samples.append(t265_sample(clock()))
        d500.emit(radar_update(local_x_cm=0.0, global_x_cm=0.0))
        anchored = runtime.step()
        self.assertIsNone(anchored.error)
        self.assertIsNotNone(anchored.navigation)
        self.assertTrue(runtime.drive.backend.commands)
        runtime.close()

    def test_competition_map_builder_failure_refuses_runtime(self) -> None:
        config = ready_config()
        clock = FakeClock(40.0)
        with (
            patch.object(robocup_runtime, "build_differential_drive", side_effect=lambda cfg, **_: make_drive(cfg, clock)),
            patch.object(robocup_runtime, "RealSenseT265PoseSource", return_value=FakeT265()),
            patch("components.radar_driver.D500RadarComponent", return_value=FakeD500()),
            patch.object(robocup_runtime, "build_competition_world", side_effect=ValueError("invalid field map")),
        ):
            with self.assertRaisesRegex(ValueError, "invalid field map"):
                robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=clock)


if __name__ == "__main__":
    unittest.main()
