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
from components.radar_driver import (
    D500_TIMESTAMP_MODULUS_MS,
    GlobalCorrectionMode,
    Pose2D as RadarPose2D,
)
from components.sensor_clock import DeviceClockMapper
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality
from core.frames import compose_pose2d
from localization.field_reference import build_field_wall_reference
import robocup_runtime


def ready_config():
    source = load_v2_config()
    return replace(
        source,
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
    def test_device_scan_timestamp_is_unwrapped_and_independent_of_callback_delay(self) -> None:
        runtime = self._runtime()
        # The D500 counter wraps at 30000 ms, measured on the car over 240 s:
        # raw values span 0..29999 and all eight observed wraps started at
        # 29997..29999 and landed on 0..2.
        self.assertEqual(D500_TIMESTAMP_MODULUS_MS, 30000)
        self.assertAlmostEqual(runtime._map_d500_timestamp(29990, 10.0), 10.0)
        # The wrap between scans is absorbed; the 200 ms callback delay is not
        # applied to the measurement timestamp.
        self.assertAlmostEqual(runtime._map_d500_timestamp(10, 10.2), 10.02)
        runtime.close()

    def test_measured_modulus_survives_repeated_wraps_monotonically(self) -> None:
        """Every real wrap must be absorbed, not mistaken for a clock reset."""

        runtime = self._runtime()
        values = [29990, 29995, 5, 10, 29999, 3, 8, 29997, 1]
        stamps = []
        for index, raw in enumerate(values):
            stamps.append(
                runtime._map_d500_timestamp(raw, 100.0 + 0.15 * index)
            )
        for earlier, later in zip(stamps, stamps[1:]):
            self.assertGreater(
                later, earlier,
                "mapped D500 timestamps must stay strictly increasing across a wrap",
            )
        # Three wraps of a 30000 ms counter advance device time by well under a
        # second in total; a wrong modulus would instead let the output drift
        # along the callback timeline.
        self.assertLess(stamps[-1] - stamps[0], 1.2)
        runtime.close()

    def test_wrong_modulus_pins_the_wrapped_sample_to_callback_time(self) -> None:
        """Pin why the modulus must be the measured value.

        With 0x10000 the real wrap delta (-29998) is not below -modulus/2
        (-32768), so it never registers as a wrap and the accumulator keeps the
        raw negative delta.  The mapping then collapses onto the callback
        timeline rather than the device timeline: with a 1.5 s callback delay the
        wrapped sample lands at 11.5 s instead of 10.002 s.
        """

        wrong = DeviceClockMapper(modulus_ms=0x10000)
        right = DeviceClockMapper(modulus_ms=D500_TIMESTAMP_MODULUS_MS)
        for mapper in (wrong, right):
            mapper.map_milliseconds(29998, 10.0)
        wrong_s = wrong.map_milliseconds(0, 11.5)
        right_s = right.map_milliseconds(0, 11.5)
        self.assertAlmostEqual(right_s, 10.002, places=6)
        self.assertGreater(
            abs(wrong_s - right_s), 1.0,
            "the wrong modulus should visibly misplace the wrapped timestamp",
        )

    def test_callback_delay_is_used_only_on_a_genuine_clock_reset(self) -> None:
        """A genuine reset legitimately restarts from the received time."""

        mapper = DeviceClockMapper(modulus_ms=D500_TIMESTAMP_MODULUS_MS)
        mapper.map_milliseconds(1000, 5.0)
        # Jumping backwards by far more than half the modulus is a real reset;
        # only then does the received time take over.
        self.assertAlmostEqual(mapper.map_milliseconds(900, 42.0), 42.0)
        # ...and the following sample continues from the device clock again.
        self.assertAlmostEqual(mapper.map_milliseconds(910, 42.1), 42.01)

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

    def test_runtime_adapts_absolute_pose_and_keeps_local_only_out_of_fusion(self) -> None:
        runtime = self._runtime()
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False))
        update = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 200.0, 0.0), icp=None),
            global_pose=RadarPose2D(150.0, 250.0, 10.0),
            map_alignment_established=True,
            map_pose_valid=True,
            absolute_observation_available=True,
            absolute_observation_accepted=True,
            global_confidence=0.9,
            wall_fusion=None,
        )
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        estimate = runtime.fusion.estimate(1.0)
        self.assertAlmostEqual(estimate.pose.x_m, 1.5)
        self.assertAlmostEqual(estimate.pose.y_m, 2.5)
        self.assertAlmostEqual(estimate.pose.yaw_rad, -0.1745329252)
        anchor = runtime.fusion.map_T_t265_odom
        update.global_pose = RadarPose2D(160.0, 250.0, 10.0)
        update.absolute_observation_accepted = False
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        self.assertEqual(runtime.fusion.map_T_t265_odom, anchor)
        self.assertEqual(runtime.fusion._d500_global_fallback_pose.x_m, 1.6)

        local_runtime = self._runtime()
        update.map_alignment_established = False
        update.map_pose_valid = False
        update.absolute_observation_available = False
        update.absolute_observation_accepted = False
        update.global_pose = None
        update.global_confidence = None
        local_runtime.on_d500_update(update)
        local_runtime._consume_d500(1.0)
        self.assertIsNone(local_runtime.fusion.estimate(1.0).pose)
        self.assertIsNone(local_runtime.fusion.map_T_t265_odom)
        self.assertIsNone(local_runtime.fusion.estimate(1.0).d500_accepted)
        local_runtime.close()
        runtime.close()

    def test_runtime_does_not_trust_identity_until_absolute_then_allows_d500_only(self) -> None:
        runtime = self._runtime()
        # An absolute observation can only be accepted once T265 provides a pose
        # to align it against; without one fusion rejects with
        # "t265_time_alignment_unavailable" and the anchor must stay untrusted.
        runtime.fusion.update_t265(
            Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False)
        )
        update = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 0.0, 0.0), icp=None),
            global_pose=RadarPose2D(100.0, 0.0, 0.0),
            map_alignment_established=True,
            map_pose_valid=True,
            absolute_observation_available=False,
            absolute_observation_accepted=False,
            global_confidence=None,
            wall_fusion=None,
        )
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        self.assertFalse(runtime._d500_global_alignment_trusted)
        self.assertIsNone(runtime.fusion.estimate(1.0).pose)

        update.absolute_observation_available = True
        update.absolute_observation_accepted = True
        update.global_confidence = 0.9
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        estimate = runtime.fusion.estimate(1.0)
        # Only a fusion-accepted absolute observation may set the trusted flag.
        self.assertTrue(estimate.d500_accepted)
        self.assertTrue(runtime._d500_global_alignment_trusted)
        # Accepting the observation now establishes the map anchor, which is the
        # D500 map pose composed with the T265 pose it was aligned against.
        anchor = runtime.fusion.map_T_t265_odom
        self.assertIsNotNone(anchor)
        self.assertAlmostEqual(anchor.x_m, 1.0)
        self.assertAlmostEqual(anchor.y_m, 0.0)
        # With the anchor established and T265 fresh, the fused state upgrades
        # from the D500-only fallback to the normal fused channel.
        self.assertIn("fused", estimate.source_flags)
        self.assertAlmostEqual(estimate.pose.x_m, 1.0)
        self.assertTrue(runtime._hardware_global_localization_ready(1.0))
        runtime.close()

    def test_candidate_is_not_trusted_without_a_usable_t265_alignment(self) -> None:
        """A candidate fusion cannot align must not make the map look trusted.

        Regression for the premature-trust defect: the trusted flag used to be
        set before PoseFusion decided, so an absolute candidate that could not
        be aligned still advertised a trustworthy global map.
        """

        runtime = self._runtime()
        update = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 0.0, 0.0), icp=None),
            global_pose=RadarPose2D(100.0, 0.0, 0.0),
            map_alignment_established=True,
            map_pose_valid=True,
            absolute_observation_available=True,
            absolute_observation_accepted=True,
            global_confidence=0.9,
            wall_fusion=None,
        )
        runtime.on_d500_update(update)
        runtime._consume_d500(1.0)
        estimate = runtime.fusion.estimate(1.0)
        self.assertFalse(estimate.d500_accepted)
        self.assertFalse(runtime._d500_global_alignment_trusted)
        self.assertIsNone(runtime._last_d500_absolute_update_s)
        self.assertFalse(runtime._hardware_global_localization_ready(1.0))
        runtime.close()

    def test_low_confidence_and_large_position_jump_are_rejected_as_absolute(self) -> None:
        runtime = self._runtime()
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False))
        runtime.fusion.update_d500_absolute(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("d500", True, False))
        sample = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(100.0, 100.0, 0.0), icp=None),
            global_pose=RadarPose2D(120.0, 100.0, 0.0),
            map_alignment_established=True,
            map_pose_valid=True,
            absolute_observation_available=True,
            absolute_observation_accepted=True,
            global_confidence=0.2,
            wall_fusion=None,
        )
        runtime.on_d500_update(sample)
        runtime._consume_d500(1.0)
        self.assertEqual(runtime.d500_abs_reject_low_confidence, 1)
        self.assertAlmostEqual(runtime.fusion.estimate(1.0).pose.x_m, 0.0)

        sample.global_confidence = 0.9
        sample.global_pose = RadarPose2D(300.0, 100.0, 0.0)
        runtime.on_d500_update(sample)
        runtime._consume_d500(1.0)
        self.assertFalse(runtime.fusion.estimate(1.0).d500_accepted)
        self.assertEqual(runtime.fusion.estimate(1.0).rejection_reason, "position_innovation_gate")
        runtime.close()

    def test_first_reliable_global_pose_anchors_across_local_odom_origin(self) -> None:
        runtime = self._runtime()
        odom_pose = Pose2D(0.2, 0.1, 0.0, 1.0)
        runtime.fusion.update_t265(odom_pose, PoseQuality("t265", True, False))
        low = types.SimpleNamespace(
            odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(20.0, 10.0, 0.0), icp=None),
            global_pose=RadarPose2D(420.0, 310.0, -11.4591559),
            map_alignment_established=True,
            map_pose_valid=True,
            absolute_observation_available=True,
            absolute_observation_accepted=True,
            global_confidence=0.2,
            wall_fusion=None,
        )
        runtime.on_d500_update(low)
        runtime._consume_d500(1.0)
        self.assertFalse(runtime.fusion.global_anchor_established)

        low.global_confidence = 0.9
        runtime.on_d500_update(low)
        runtime._consume_d500(1.0)
        self.assertTrue(runtime.fusion.global_anchor_established)
        self.assertEqual(runtime.d500_abs_accept_count, 1)
        anchored = compose_pose2d(runtime.fusion.map_T_t265_odom, odom_pose)
        self.assertAlmostEqual(anchored.x_m, 4.2, places=6)
        self.assertAlmostEqual(anchored.y_m, 3.1, places=6)
        self.assertAlmostEqual(anchored.yaw_rad, 0.2, places=6)
        runtime.close()

    def test_propagated_map_pose_does_not_refresh_absolute_freshness(self) -> None:
        now = [1.0]
        config = ready_config()
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: fake_drive(cfg, clock=kwargs["clock"]),
        ):
            runtime = robocup_runtime.build_runtime(
                config,
                RuntimeMode.HARDWARE_MISSION,
                clock=lambda: now[0],
            )

        def update(*, absolute: bool, x_cm: float):
            return types.SimpleNamespace(
                odometry=types.SimpleNamespace(accepted=True, pose=RadarPose2D(x_cm, 0.0, 0.0), icp=None),
                global_pose=RadarPose2D(x_cm, 0.0, 0.0),
                map_alignment_established=True,
                map_pose_valid=True,
                absolute_observation_available=absolute,
                absolute_observation_accepted=absolute,
                global_confidence=0.9 if absolute else None,
                wall_fusion=None,
            )

        runtime.on_d500_update(update(absolute=True, x_cm=100.0))
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False))
        runtime._consume_d500(now[0])
        self.assertEqual(runtime._last_d500_absolute_update_s, 1.0)
        self.assertEqual(runtime._last_d500_map_pose_update_s, 1.0)
        self.assertTrue(runtime._hardware_global_localization_ready(now[0]))

        now[0] = 1.0 + robocup_runtime.D500_ANCHOR_STALE_AFTER_S + 0.1
        stamp = 1.1
        while stamp <= now[0] + 1e-9:
            runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, stamp), PoseQuality("t265", True, False))
            stamp += 0.1
        runtime.on_d500_update(update(absolute=False, x_cm=110.0))
        runtime._consume_d500(now[0])
        self.assertEqual(runtime._last_d500_absolute_update_s, 1.0)
        self.assertEqual(runtime._last_d500_map_pose_update_s, now[0])
        # The anchor is a different concept from the raw measurement timeout:
        # it survives a D500 gap and only stops counting once it is older than
        # the anchor window.
        self.assertFalse(runtime._hardware_global_localization_ready(now[0]))

        # A fresh accepted absolute observation after a long gap re-establishes
        # the anchor.  The T265 samples must keep advancing monotonically, or
        # fusion correctly treats the stream as discontinuous.
        now[0] = 20.0
        stamp = 6.2
        while stamp <= now[0] + 1e-9:
            runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, stamp), PoseQuality("t265", True, False))
            stamp += 0.1
        runtime.on_d500_update(update(absolute=True, x_cm=100.0))
        runtime._consume_d500(now[0])
        self.assertEqual(runtime._last_d500_absolute_update_s, now[0])
        self.assertTrue(runtime._hardware_global_localization_ready(now[0]))
        runtime.close()


if __name__ == "__main__":
    unittest.main()
