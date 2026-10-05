"""Hardware-free tests for the Phase 3 runtime wiring.

Covers three separate concerns:

1. the fused/T265 map pose reaches the D500 as a wall-association hint, with the
   core (m, CCW rad) -> radar (cm, CW deg) conversion applied exactly once;
2. the trusted-anchor flag and the last-absolute-accepted time are refreshed
   only after ``PoseFusion`` actually accepts the candidate -- producing a
   candidate is not acceptance;
3. a short gap in raw D500 measurements must not destroy an already established
   map anchor, because those are two different freshness concepts.
"""

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
    GlobalCorrectionMode,
    Pose2D as RadarPose2D,
)
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality
import robocup_runtime


def ready_config():
    source = load_v2_config()
    return replace(
        source,
        calibration=replace(source.calibration, geometry_measured=True,
                            sensor_extrinsics_measured=True),
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


def d500_update(*, accepted: bool, absolute: bool, x_cm: float,
                local_pose_valid=None, confidence=0.9):
    return types.SimpleNamespace(
        odometry=types.SimpleNamespace(
            accepted=accepted, pose=RadarPose2D(x_cm, 0.0, 0.0), icp=None,
            rejection_reason=None if accepted else "translation gate",
        ),
        global_pose=RadarPose2D(x_cm, 0.0, 0.0),
        map_alignment_established=True,
        map_pose_valid=True,
        absolute_observation_available=absolute,
        absolute_observation_accepted=absolute,
        global_confidence=confidence if absolute else None,
        wall_fusion=None,
        local_pose_valid=accepted if local_pose_valid is None else local_pose_valid,
    )


class _RecordingD500:
    """Stands in for the real component, recording pushed pose hints."""

    def __init__(self):
        self.hints = []

    def set_global_pose_hint(self, pose):
        self.hints.append(pose)


class PoseHintWiringTests(unittest.TestCase):
    def _runtime(self, clock):
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: fake_drive(cfg, clock=kwargs["clock"]),
        ):
            return robocup_runtime.build_runtime(
                ready_config(), RuntimeMode.HARDWARE_MISSION, clock=clock
            )

    def test_hint_converts_core_pose_into_radar_convention_once(self) -> None:
        """The hint must be a cm / CW-deg radar pose, converted exactly once.

        The fused estimate is stubbed so this test isolates the wiring and the
        unit conversion.  Composing a T265 delta into a map pose is PoseFusion's
        responsibility and is covered by its own tests.
        """

        now = [5.0]
        runtime = self._runtime(lambda: now[0])
        recorder = _RecordingD500()
        runtime.d500_source = recorder
        runtime.d500_fake = False

        stub = types.SimpleNamespace(
            pose=Pose2D(1.0, 2.0, 0.5, now[0]), anchor_initialized=True
        )
        with patch.object(runtime.fusion, "estimate", return_value=stub):
            runtime._publish_d500_pose_hint(now[0])

        self.assertTrue(recorder.hints, "a hint must be published once anchored")
        hint = recorder.hints[-1]
        self.assertIsInstance(hint, RadarPose2D)
        # core is metres / CCW radians; radar is centimetres / CW degrees.
        self.assertAlmostEqual(hint.x_cm, 100.0, places=6)
        self.assertAlmostEqual(hint.y_cm, 200.0, places=6)
        self.assertAlmostEqual(hint.yaw_cw_deg, -28.64788975654116, places=6)
        runtime.close()

    def test_hint_is_none_while_the_estimate_is_unanchored(self) -> None:
        now = [5.0]
        runtime = self._runtime(lambda: now[0])
        recorder = _RecordingD500()
        runtime.d500_source = recorder
        runtime.d500_fake = False
        stub = types.SimpleNamespace(
            pose=Pose2D(1.0, 2.0, 0.5, now[0]), anchor_initialized=False
        )
        with patch.object(runtime.fusion, "estimate", return_value=stub):
            runtime._publish_d500_pose_hint(now[0])
        self.assertEqual(recorder.hints, [None])
        runtime.close()

    def test_hint_is_cleared_before_any_anchor_exists(self) -> None:
        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        recorder = _RecordingD500()
        runtime.d500_source = recorder
        runtime.d500_fake = False

        runtime._publish_d500_pose_hint(now[0])
        self.assertEqual(recorder.hints, [None])
        runtime.close()

    def test_missing_hint_api_is_tolerated(self) -> None:
        """A component without the hint API must not break the loop."""

        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        runtime.d500_source = types.SimpleNamespace()
        runtime.d500_fake = False
        runtime._publish_d500_pose_hint(now[0])   # must not raise
        runtime.close()


class FreshnessRefreshTests(unittest.TestCase):
    def _runtime(self, clock):
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: fake_drive(cfg, clock=kwargs["clock"]),
        ):
            return robocup_runtime.build_runtime(
                ready_config(), RuntimeMode.HARDWARE_MISSION, clock=clock
            )

    def test_candidate_produced_is_not_trusted_until_fusion_accepts(self) -> None:
        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        # map_pose_valid and absolute availability alone must not set the flag.
        runtime.on_d500_update(d500_update(accepted=True, absolute=False, x_cm=100.0))
        runtime._consume_d500(now[0])
        self.assertFalse(runtime._d500_global_alignment_trusted)
        self.assertIsNone(runtime._last_d500_absolute_update_s)
        runtime.close()

    def test_fusion_accept_establishes_the_anchor(self) -> None:
        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        runtime.fusion.update_t265(
            Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False)
        )
        runtime.on_d500_update(d500_update(accepted=True, absolute=True, x_cm=100.0))
        runtime._consume_d500(now[0])
        estimate = runtime.fusion.estimate(now[0])
        self.assertTrue(estimate.d500_accepted)
        self.assertTrue(runtime._d500_global_alignment_trusted)
        self.assertEqual(runtime._last_d500_absolute_update_s, now[0])
        self.assertTrue(runtime._hardware_global_localization_ready(now[0]))
        runtime.close()

    def test_established_anchor_survives_a_raw_measurement_gap(self) -> None:
        """A short D500 gap must not drop an accepted anchor.

        This replaces the previous expectation that readiness follows
        ``d500_max_age_s`` (0.5 s).  Those are different concepts: the anchor
        stays valid while T265 keeps propagating, and only an anchor older than
        the anchor window stops counting as global readiness.
        """

        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        runtime.fusion.update_t265(
            Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False)
        )
        runtime.on_d500_update(d500_update(accepted=True, absolute=True, x_cm=100.0))
        runtime._consume_d500(now[0])
        self.assertTrue(runtime._hardware_global_localization_ready(now[0]))

        # Past the raw measurement timeout but well inside the anchor window.
        now[0] = 1.0 + ready_config().fusion.d500_max_age_s + 0.1
        stamp = 1.1
        while stamp <= now[0] + 1e-9:
            runtime.fusion.update_t265(
                Pose2D(0.0, 0.0, 0.0, stamp), PoseQuality("t265", True, False)
            )
            stamp += 0.1
        self.assertTrue(
            runtime._hardware_global_localization_ready(now[0]),
            "an established anchor must not flap on a single missing D500 scan",
        )

        # Past the anchor window: stop claiming a trustworthy global map.
        now[0] = 1.0 + robocup_runtime.D500_ANCHOR_STALE_AFTER_S + 1.0
        stamp = 1.8
        while stamp <= now[0] + 1e-9:
            runtime.fusion.update_t265(
                Pose2D(0.0, 0.0, 0.0, stamp), PoseQuality("t265", True, False)
            )
            stamp += 0.1
        self.assertFalse(runtime._hardware_global_localization_ready(now[0]))
        runtime.close()

    def test_rejected_candidate_does_not_refresh_the_anchor_time(self) -> None:
        now = [1.0]
        runtime = self._runtime(lambda: now[0])
        runtime.fusion.update_t265(
            Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False)
        )
        runtime.on_d500_update(d500_update(accepted=True, absolute=True, x_cm=100.0))
        runtime._consume_d500(now[0])
        first_time = runtime._last_d500_absolute_update_s

        # A wildly inconsistent candidate must be rejected by the innovation
        # gate and leave the recorded anchor time untouched.
        now[0] = 2.0
        runtime.fusion.update_t265(
            Pose2D(0.0, 0.0, 0.0, 2.0), PoseQuality("t265", True, False)
        )
        runtime.on_d500_update(d500_update(accepted=True, absolute=True, x_cm=9000.0))
        runtime._consume_d500(now[0])
        self.assertEqual(runtime._last_d500_absolute_update_s, first_time)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
