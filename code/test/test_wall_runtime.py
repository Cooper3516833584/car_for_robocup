"""Full wall runtime chain: raw scan -> observer -> navigation -> drive -> world.

The simulated world knows the true chassis pose, but only to *generate* D500
format scans and to assert the outcome. The controller never reads it: it sees
fitted wall lines from the real ``WallObserver`` and ``wall_navigation`` code,
and the real ``DifferentialDrive`` limiting with the ``FakeDriveBackend``.
"""

from dataclasses import dataclass, replace
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.differential_drive import DifferentialDrive
from components.differential_kinematics import DifferentialGeometry
from components.wall_navigation import (
    CORNER_TOLERANCE_M,
    FollowRightWall,
    REAR_BEARING_RAD,
    RIGHT_BEARING_RAD,
    WallActionLimits,
    WallTracker,
    drop_turn,
    is_front_wall,
    is_rear_wall,
    is_right_wall,
)
from components.wall_observer import WallObserver
from core.types import Twist2D
from robocup_runtime import WallRuntime, WallMission, WallStep


class RectangularField:
    """A rectangular enclosure with four real walls, in world coordinates."""

    def __init__(self, *, half_width=2.5, half_length=3.5):
        self.half_width = half_width
        self.half_length = half_length

    def lines_in_body(self, x, y, yaw, *, span=3.0):
        """Return (normal_body, rho_body) for each world wall."""
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        def to_body(px, py):
            dx, dy = px - x, py - y
            return (cos_y * dx + sin_y * dy, -sin_y * dx + cos_y * dy)

        def rotate(nx, ny):
            return (cos_y * nx + sin_y * ny, -sin_y * nx + cos_y * ny)

        walls = (
            ((0.0, -1.0), (0.0, -self.half_width)),   # world right
            ((0.0, 1.0), (0.0, self.half_width)),     # world left
            ((1.0, 0.0), (self.half_length, 0.0)),    # world front (+x)
            ((-1.0, 0.0), (-self.half_length, 0.0)),  # world rear (-x)
        )
        result = []
        for normal_world, point_world in walls:
            normal_body = rotate(*normal_world)
            offset = to_body(*point_world)
            rho = normal_body[0] * offset[0] + normal_body[1] * offset[1]
            result.append((normal_body, rho))
        return result

    def observe(self, x, y, yaw, *, points_per_wall=45, span=3.0, jitter_m=0.004, rng=None):
        """Generate base-frame points as the real D500 would report them."""
        rng = random.Random(0) if rng is None else rng
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        points = []
        for normal, rho in self.lines_in_body(x, y, yaw):
            tangent = (-normal[1], normal[0])
            centre = (normal[0] * rho, normal[1] * rho)
            for index in range(points_per_wall):
                along = -span / 2 + span * index / (points_per_wall - 1)
                px = centre[0] + along * tangent[0] + rng.gauss(0.0, jitter_m) * normal[0]
                py = centre[1] + along * tangent[1] + rng.gauss(0.0, jitter_m) * normal[1]
                points.append((px, py))
        return points


class SimulatedBase:
    """Truth model used only to generate scans and to score the result."""

    def __init__(self, *, x=0.0, y=0.0, yaw=0.0, field=None):
        self.x, self.y, self.yaw = x, y, yaw
        self.field = field or RectangularField()

    def integrate(self, command, dt):
        self.x += command.linear_x_m_s * math.cos(self.yaw) * dt
        self.y += command.linear_x_m_s * math.sin(self.yaw) * dt
        self.yaw += command.angular_z_rad_s * dt

    def body_wall(self, predicate):
        """Return ``(normal_body, rho_body)`` for the nearest matching wall."""
        found = [entry for entry in self.field.lines_in_body(self.x, self.y, self.yaw)
                 if predicate(entry[0], entry[1])]
        if not found:
            return None
        return min(found, key=lambda item: abs(item[1]))

    def rho_to(self, world_normal):
        """True perpendicular distance from the chassis to one world wall."""
        if world_normal == (0.0, -1.0):
            return abs(self.y - (-self.field.half_width))
        if world_normal == (0.0, 1.0):
            return abs(self.field.half_width - self.y)
        if world_normal == (1.0, 0.0):
            return abs(self.field.half_length - self.x)
        return abs(self.x - (-self.field.half_length))


def make_limits(**overrides):
    base = dict(
        follow_speed_m_s=0.18, vision_speed_m_s=0.08, minimum_speed_m_s=0.045,
        lookahead_m=0.20, track_width_m=0.198, wheel_speed_m_s=0.30,
        max_linear_speed_m_s=0.20, max_angular_speed_rad_s=0.80,
        max_linear_accel_m_s2=0.30, max_angular_accel_rad_s2=1.00)
    base.update(overrides)
    return WallActionLimits(**base)


class FakeObjectGuardTests(unittest.TestCase):
    """I02/I13: the wall chain must import and build without the fused stack."""

    def test_i02_the_wall_modules_never_import_the_fused_stack(self):
        import ast
        import components.wall_observer as observer_module
        import components.wall_navigation as navigation_module
        forbidden = {"PoseFusion", "T265PoseAdapter", "RealSenseT265PoseSource",
                     "D500RadarComponent", "ICPScanMatcher", "RadarOdometry",
                     "SlamBridge", "rclpy", "pyrealsense2", "DifferentialNavigator",
                     "BasicMotionController", "WallLineLocalizer"}
        for module in (observer_module, navigation_module):
            tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
            imported = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imported.update(alias.name.split(".")[-1] for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    imported.update(alias.name for alias in node.names)
                    imported.add((node.module or "").split(".")[-1])
            self.assertEqual(imported & forbidden, set(),
                             f"{module.__name__} imports fused-stack names")

    def test_i02_a_wall_runtime_builds_with_injected_fakes(self):
        from config.v2_loader import load_v2_config
        config = load_v2_config()
        backend = FakeDriveBackend()
        drive = DifferentialDrive(
            backend,
            geometry=DifferentialGeometry(track_width_m=config.geometry.drive_track_width_m),
            max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s,
            max_linear_speed_m_s=config.drive.max_linear_speed_m_s,
            max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s,
            max_linear_accel_m_s2=config.drive.max_linear_accel_m_s2,
            max_angular_accel_rad_s2=config.drive.max_angular_accel_rad_s2,
            command_timeout_s=config.drive.command_timeout_s,
            hardware_lock_path=None)
        observer = WallObserver(serial_driver=None)
        runtime = WallRuntime(config, __import__("config.v2_runtime", fromlist=["RuntimeMode"]).RuntimeMode.DRY_RUN,
                              drive=drive, observer=observer)
        self.assertIsNone(runtime.action)
        self.assertFalse(runtime.stop_latched)
        self.assertIsInstance(runtime.mission, WallMission)


class StopLatchTests(unittest.TestCase):
    def make_runtime(self):
        from config.v2_loader import load_v2_config
        from config.v2_runtime import RuntimeMode
        config = load_v2_config()
        backend = FakeDriveBackend()
        # The fake clock must already be past DifferentialDrive.start()'s own
        # stamp, or the first command looks like a backwards time step.
        clock = [1.0]
        drive = DifferentialDrive(
            backend,
            geometry=DifferentialGeometry(track_width_m=config.geometry.drive_track_width_m),
            max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s,
            max_linear_speed_m_s=config.drive.max_linear_speed_m_s,
            max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s,
            max_linear_accel_m_s2=config.drive.max_linear_accel_m_s2,
            max_angular_accel_rad_s2=config.drive.max_angular_accel_rad_s2,
            command_timeout_s=config.drive.command_timeout_s,
            hardware_lock_path=None,
            clock=lambda: clock[0])
        observer = WallObserver(clock=lambda: clock[0], serial_driver=None)
        runtime = WallRuntime(config, RuntimeMode.DRY_RUN, drive=drive, observer=observer,
                              clock=lambda: clock[0])
        return runtime, backend, clock

    def test_i07_a_stop_between_compute_and_send_blocks_the_non_zero_command(self):
        runtime, backend, clock = self.make_runtime()
        runtime.start()
        clock[0] += 0.05
        # The STOP lands after the command was computed but before it is sent.
        runtime.request_stop("test stop")
        self.assertIsNone(runtime._send(Twist2D(0.15, 0.0), clock[0]))
        self.assertTrue(runtime.stop_latched)
        self.assertEqual(len(backend.commands), 0)

    def test_i07_a_normally_sent_command_reaches_the_backend_once(self):
        runtime, backend, clock = self.make_runtime()
        runtime.start()
        clock[0] += 0.05
        self.assertIsNotNone(runtime._send(Twist2D(0.15, 0.0), clock[0]))
        self.assertEqual(len(backend.commands), 1)

    def test_i11_a_new_run_clears_the_previous_stop_latch_and_action(self):
        runtime, backend, clock = self.make_runtime()
        runtime.start()
        runtime.request_stop("first run stop")
        self.assertTrue(runtime.stop_latched)
        runtime.close()
        fresh, backend2, clock2 = self.make_runtime()
        fresh.start()
        self.assertFalse(fresh.stop_latched)
        self.assertIsNone(fresh.action)
        self.assertEqual(fresh.mission.state.value, "init")

    def test_i09_a_stopped_step_reports_zero_and_the_reason(self):
        runtime, backend, clock = self.make_runtime()
        runtime.start()
        clock[0] += 0.05
        runtime.request_stop("operator")
        step = runtime.step()
        self.assertEqual(step.command.linear_x_m_s, 0.0)
        self.assertEqual(step.phase, "stopped")
        self.assertEqual(step.error, "operator")

    def test_close_is_idempotent_and_forbids_further_output(self):
        runtime, backend, clock = self.make_runtime()
        runtime.start()
        clock[0] += 0.05
        runtime.close()
        self.assertIsNone(runtime._send(Twist2D(0.2, 0.0), clock[0]))
        runtime.close()


class ClosedLoopLegTests(unittest.TestCase):
    """One full leg through the real observer, controller and drive."""

    def build(self, *, x=0.0, y=0.0, yaw=0.0, field=None):
        from config.v2_loader import load_v2_config
        from config.v2_runtime import RuntimeMode
        config = load_v2_config()
        backend = FakeDriveBackend()
        clock = [1.0]
        drive = DifferentialDrive(
            backend,
            geometry=DifferentialGeometry(track_width_m=config.geometry.drive_track_width_m),
            max_wheel_speed_m_s=config.drive.max_wheel_speed_m_s,
            max_linear_speed_m_s=config.drive.max_linear_speed_m_s,
            max_angular_speed_rad_s=config.drive.max_angular_speed_rad_s,
            max_linear_accel_m_s2=config.drive.max_linear_accel_m_s2,
            max_angular_accel_rad_s2=config.drive.max_angular_accel_rad_s2,
            command_timeout_s=config.drive.command_timeout_s,
            hardware_lock_path=None,
            clock=lambda: clock[0])
        observer = WallObserver(
            clock=lambda: clock[0], serial_driver=None,
            # One reproducible extraction per simulated scan, mirroring how a
            # real revolution differs from the previous one. Seeding the observer
            # once instead would pin every scan to the same RANSAC stream, which
            # is both unrealistic and needlessly fragile.
            seed_source=lambda: int(clock[0] * 1000))
        runtime = WallRuntime(config, RuntimeMode.DRY_RUN, drive=drive, observer=observer,
                              clock=lambda: clock[0])
        runtime.start()
        world = SimulatedBase(x=x, y=y, yaw=yaw, field=field or RectangularField())
        return runtime, world, clock, backend

    def run_leg(self, runtime, world, clock, *, target_right, direction, steps,
                terminal_role=None, terminal_target=None, dt=0.05):
        """Drive the loop; return the finished action and the world afterwards."""
        limits = runtime.limits
        tracker = WallTracker()
        action = FollowRightWall(target_distance_m=target_right, direction=direction,
                                 limits=limits, tracker=tracker,
                                 terminal_role=terminal_role,
                                 terminal_target_m=terminal_target,
                                 clock=lambda: clock[0])
        rng = random.Random(7)
        for _ in range(steps):
            clock[0] += dt
            rng.seed(int(clock[0] * 1000))
            runtime.observer.publish_points(
                world.field.observe(world.x, world.y, world.yaw, rng=rng),
                observed_at_s=clock[0])
            runtime.action = action
            step = runtime.step()
            world.integrate(step.limited_command or step.command, dt)
            if action.outcome is not None and action.outcome.done:
                break
        return action, world

    def test_a_forward_leg_converges_on_the_measured_wall_distance(self):
        runtime, world, clock, backend = self.build(yaw=0.0)
        # World right wall at y = -2.5; start 0.4 m too far from the 1.6 m target.
        world.y = -0.5
        action, _ = self.run_leg(runtime, world, clock, target_right=1.6, direction=+1,
                                 steps=500)
        self.assertLess(abs(world.rho_to((0.0, -1.0)) - 1.6), 0.04)
        runtime.close()

    def test_a_forward_leg_converges_when_starting_too_close(self):
        runtime, world, clock, backend = self.build(yaw=0.0)
        world.y = -1.3  # 1.2 m from the world y=-2.5 wall
        action, _ = self.run_leg(runtime, world, clock, target_right=1.6, direction=+1,
                                 steps=600)
        self.assertLess(abs(world.rho_to((0.0, -1.0)) - 1.6), 0.04)
        runtime.close()

    def test_a_reversing_leg_also_converges(self):
        runtime, world, clock, backend = self.build(yaw=0.0)
        world.y = -1.0
        action, _ = self.run_leg(runtime, world, clock, target_right=1.6, direction=-1,
                                 steps=600)
        self.assertLess(abs(world.rho_to((0.0, -1.0)) - 1.6), 0.05)
        runtime.close()

    def test_a_leg_stops_at_the_front_wall_terminal(self):
        runtime, world, clock, backend = self.build(yaw=0.0)
        world.y = -2.5 + 1.6
        # Front wall at world x = +3.5; stop at 1.9 m base distance.
        action, _ = self.run_leg(runtime, world, clock, target_right=1.6,
                                 direction=+1, steps=900,
                                 terminal_role=is_front_wall, terminal_target=1.9)
        self.assertIsNotNone(action.outcome)
        self.assertTrue(action.outcome.done)
        self.assertEqual(action.outcome.reason, "terminal_reached")
        self.assertAlmostEqual(world.rho_to((1.0, 0.0)), 1.9, delta=CORNER_TOLERANCE_M + 0.03)
        runtime.close()

    def test_a_reversing_leg_stops_on_the_rear_wall(self):
        # The leg reverses with the nose unchanged, so the wall alongside stays
        # the right wall and the wall it backs into is astern. Facing +x the
        # world y=-2.5 wall is on the right and the world x=-3.5 wall is behind.
        runtime, world, clock, backend = self.build(yaw=0.0)
        world.y = -2.5 + 1.6
        world.x = -3.5 + 2.2   # 2.2 m in front of the wall it will reverse into
        action, _ = self.run_leg(runtime, world, clock, target_right=1.6,
                                 direction=-1, steps=1200,
                                 terminal_role=is_rear_wall, terminal_target=1.49)
        self.assertTrue(action.outcome.done, f"reverse leg failed: {action.outcome.reason}")
        self.assertEqual(action.outcome.reason, "terminal_reached")
        # Backed up until the rear wall reached the stop band, one control
        # interval of deceleration either side of the 1.49 m target.
        self.assertAlmostEqual(world.rho_to((-1.0, 0.0)), 1.49,
                               delta=CORNER_TOLERANCE_M + 0.03)
        # Reversing must not have spun the chassis round.
        self.assertLess(abs(world.yaw), math.radians(10))
        runtime.close()

    def test_a_corner_turn_makes_the_front_wall_the_new_right_wall(self):
        runtime, world, clock, backend = self.build(yaw=0.0)
        world.y = -2.5 + 1.6
        world.x = 0.0
        limits = runtime.limits
        tracker = WallTracker()
        rng = random.Random(11)
        # The wall in front of the chassis is the terminal of the leg that just
        # ended; the corner turn must keep tracking *that* wall.
        front = world.body_wall(lambda n, r: n[0] > 0.5)
        self.assertIsNotNone(front)
        front_bearing = math.atan2(front[0][1], front[0][0])
        turn = drop_turn(front_bearing, RIGHT_BEARING_RAD, limits=limits, tracker=tracker,
                         turn_direction=+1, clock=lambda: clock[0])
        for _ in range(1200):
            clock[0] += 0.05
            rng.seed(int(clock[0] * 1000))
            runtime.observer.publish_points(
                world.field.observe(world.x, world.y, world.yaw, rng=rng),
                observed_at_s=clock[0])
            runtime.action = turn
            step = runtime.step()
            world.integrate(step.limited_command or step.command, 0.05)
            if turn.outcome is not None and turn.outcome.done:
                break
        self.assertIsNotNone(turn.outcome, "the corner turn never finished")
        self.assertTrue(turn.outcome.done, f"turn failed: {turn.outcome.reason}")
        # After a left turn of about 90 degrees the front wall is on the right.
        self.assertGreater(abs(world.yaw), math.radians(70))
        self.assertLess(abs(abs(world.yaw) - math.pi / 2), math.radians(8))
        followed = tracker.wall
        self.assertIsNotNone(followed)
        self.assertLess(followed.bearing_rad, -math.radians(80))
        runtime.close()


def role_named(rho_entry, normal):
    del rho_entry, normal
    return True


if __name__ == '__main__':
    unittest.main()
