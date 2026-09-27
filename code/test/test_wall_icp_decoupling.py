"""Hardware-free tests for decoupling the wall observer from D500 ICP acceptance.

Regression guard for a real structural defect: ``D500RadarComponent`` used to
call ``wall_localizer.observe()`` only when ``odometry_update.accepted`` was
true.  In-place rotation makes scan-to-scan ICP reject often (measured 8-30%
acceptance on the car, mostly ``translation gate``), so the absolute boundary
path was switched off exactly when it was needed most.

The requirement is that a complete scan always gets a wall-observation attempt;
ICP acceptance may only affect local odometry validity and diagnostics, never
whether the absolute observer runs.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import (  # noqa: E402
    D500RadarComponent,
    DroneGlobalAlignment,
    Pose2D,
    RadarOdometryUpdate,
    RadarPacket,
    RadarPoint,
    RadarScan,
    RectangularWallReference,
    WallFusionConfig,
    WallLineConfig,
    WallLineLocalizer,
    rotate_cw,
)


def rectangular_wall_scan(pose_in_wall: Pose2D) -> RadarScan:
    """A synthetic scan of the back and right edges of the wall rectangle."""

    wall_points: list[tuple[float, float]] = []
    wall_points.extend((0.0, float(y_cm)) for y_cm in range(0, 205, 5))
    wall_points.extend((float(x_cm), 0.0) for x_cm in range(0, 245, 5))
    radar_points = []
    for wall_x, wall_y in wall_points:
        relative_x = wall_x - pose_in_wall.x_cm
        relative_y = wall_y - pose_in_wall.y_cm
        body_x, body_y = rotate_cw(relative_x, relative_y, -pose_in_wall.yaw_cw_deg)
        distance_cm = math.hypot(body_x, body_y)
        angle_cw = math.degrees(math.atan2(-body_y, body_x)) % 360.0
        radar_points.append(RadarPoint(angle_cw, round(distance_cm * 10), 200))
    return RadarScan(tuple(radar_points), 0, 3600)


class OneScanAssembler:
    def __init__(self, scan: RadarScan) -> None:
        self.scan = scan

    def feed(self, packet):
        return [self.scan]


class RejectingOdometry:
    """ICP that always rejects, as happens during in-place rotation."""

    def __init__(self, pose: Pose2D, reason: str = "translation gate") -> None:
        self.pose = pose
        self.reason = reason
        self.calls = 0

    def update(self, scan):
        self.calls += 1
        return RadarOdometryUpdate(
            self.pose, False, True, None, self.reason
        )


class WallPathIndependentOfIcpTests(unittest.TestCase):
    def setUp(self) -> None:
        self.wall_to_global = DroneGlobalAlignment(500, 200, 90)
        self.reference = RectangularWallReference(self.wall_to_global)
        self.localizer = WallLineLocalizer(
            self.reference,
            config=WallLineConfig(min_points_per_wall=10, min_line_span_cm=40),
        )
        self.true_wall_pose = Pose2D(100, 80, 15)
        self.scan = rectangular_wall_scan(self.true_wall_pose)
        self.fusion_config = WallFusionConfig(
            update_every_scans=1,
            position_gain=1.0,
            yaw_gain=1.0,
            max_position_correction_cm=20.0,
            max_yaw_correction_deg=10.0,
            consistency_samples=1,
        )

    def _component(self, odometry):
        return D500RadarComponent(
            alignment=self.wall_to_global,
            assembler=OneScanAssembler(self.scan),
            odometry=odometry,
            wall_localizer=self.localizer,
            wall_fusion_config=self.fusion_config,
        )

    def test_wall_observation_attempted_although_icp_rejected(self) -> None:
        """ICP rejection must not switch the absolute observer off."""

        odometry = RejectingOdometry(Pose2D(108, 73, 20))
        component = self._component(odometry)
        updates = component.process_packet(RadarPacket(3600, 0, 1, 0, ()))

        self.assertEqual(odometry.calls, 1, "ICP must still run for diagnostics")
        self.assertEqual(len(updates), 1)
        update = updates[0]
        self.assertFalse(update.odometry.accepted)
        self.assertIsNotNone(
            update.wall_fusion,
            "wall fusion must be attempted even though ICP rejected",
        )
        self.assertTrue(update.wall_fusion.attempted)
        self.assertIsNotNone(
            update.wall_fusion.observation,
            "the wall observation must reach the update despite ICP rejection",
        )
        self.assertTrue(update.absolute_observation_available)

    def test_absolute_observation_reaches_a_valid_global_pose(self) -> None:
        """With a usable wall observation the fused pose must be admissible."""

        odometry = RejectingOdometry(Pose2D(108, 73, 20))
        component = self._component(odometry)
        update = component.process_packet(RadarPacket(3600, 0, 1, 0, ()))[0]

        self.assertTrue(update.absolute_observation_accepted)
        self.assertIsNotNone(update.global_pose)
        fused_wall = self.wall_to_global.pose_to_local(update.global_pose)
        self.assertAlmostEqual(fused_wall.x_cm, self.true_wall_pose.x_cm, delta=3.0)
        self.assertAlmostEqual(fused_wall.y_cm, self.true_wall_pose.y_cm, delta=3.0)

    def test_icp_rejection_is_still_reported_for_diagnostics(self) -> None:
        """Decoupling must not hide the ICP verdict."""

        odometry = RejectingOdometry(Pose2D(108, 73, 20), reason="error gate")
        update = self._component(odometry).process_packet(
            RadarPacket(3600, 0, 1, 0, ())
        )[0]
        self.assertFalse(update.odometry.accepted)
        self.assertEqual(update.odometry.rejection_reason, "error gate")
        self.assertTrue(update.odometry.initialized)

    def test_pose_hint_is_preferred_over_icp_pose_for_association(self) -> None:
        """A fused/T265 hint must drive wall association when available."""

        # Hint sits close to the truth; the rejected ICP pose is far off, so a
        # hint-driven association still finds the walls while an ICP-driven one
        # would not.
        hint = self.wall_to_global.pose_to_global(Pose2D(102, 78, 16))
        odometry = RejectingOdometry(Pose2D(400, 380, 120))
        component = self._component(odometry)
        component.set_global_pose_hint(hint)
        update = component.process_packet(RadarPacket(3600, 0, 1, 0, ()))[0]

        self.assertIsNotNone(update.wall_fusion)
        self.assertIsNotNone(
            update.wall_fusion.observation,
            "hint-driven association should still observe the walls",
        )

    def test_no_observation_without_localizer(self) -> None:
        """Sanity: with no wall localizer nothing absolute is available."""

        component = D500RadarComponent(
            alignment=self.wall_to_global,
            assembler=OneScanAssembler(self.scan),
            odometry=RejectingOdometry(Pose2D(108, 73, 20)),
            wall_localizer=None,
        )
        update = component.process_packet(RadarPacket(3600, 0, 1, 0, ()))[0]
        self.assertFalse(update.absolute_observation_available)
        self.assertIsNone(update.wall_fusion)


if __name__ == "__main__":
    unittest.main()
