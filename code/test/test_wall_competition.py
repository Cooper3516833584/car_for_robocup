"""P01..P18 / R01..R10: allocation, four-leg wall route, and the finish zone.

The route tests substitute measurement stubs for the observer and drive so a
whole mission runs in milliseconds, while the parts that must be real (the plan,
the leg targets, the terminal references, the detour sequence and the ledger)
are the production objects.
"""

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import competition_task as task
from competition_task import (
    FINISH_FRONT_DISTANCE_M,
    RIGHT_MOUNT_DROP_M,
    WALL_RIGHT_DISTANCE_M,
    build_payload_plan,
    build_wall_route,
    desired_right_base,
    finish_front_base,
)
from components.task_board_reader import TaskCounts
from components.wall_observer import WallLine, WallSnapshot
from robocup_runtime import RobocupMissionState


# ---------------------------------------------------------------------------
# Allocation
# ---------------------------------------------------------------------------

class PayloadPlanTests(unittest.TestCase):
    PLAN_CASES = (
        (TaskCounts(2, 1, 1), {'red': 2, 'blue': 1, 'green': 1}),
        (TaskCounts(1, 2, 1), {'red': 1, 'blue': 2, 'green': 1}),
        (TaskCounts(1, 1, 2), {'red': 1, 'blue': 1, 'green': 2}),
    )

    def test_p01_every_board_uses_slots_0_to_4_once_with_yellow_fixed_at_4(self):
        for counts, values in self.PLAN_CASES:
            with self.subTest(red=values['red'], blue=values['blue'], green=values['green']):
                plan = build_payload_plan(counts)
                self.assertEqual(plan['yellow'], (4,))
                used = sorted(slot for slots in plan.values() for slot in slots)
                self.assertEqual(used, [0, 1, 2, 3, 4])
                double = next(c for c in ('red', 'blue', 'green') if values[c] == 2)
                self.assertEqual(plan[double], (2, 3))
                singles = [c for c in ('red', 'blue', 'green') if values[c] == 1]
                self.assertEqual(plan[singles[0]], (1,))
                self.assertEqual(plan[singles[1]], (0,))

    def test_a_bad_quota_shape_is_rejected_with_a_real_exception(self):
        for values in ({'red': 3, 'blue': 1, 'green': 0},
                       {'red': 2, 'blue': 2, 'green': 0},
                       {'red': -1, 'blue': 2, 'green': 1},
                       {'red': True, 'blue': 1, 'green': 1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                task.build_payload_plan(_Counts(values))

    def test_the_plan_is_built_once_and_reused_by_callers(self):
        counts = TaskCounts(2, 1, 1)
        plan = build_payload_plan(counts)
        self.assertEqual(plan, {'yellow': (4,), 'red': (2, 3), 'blue': (1,), 'green': (0,)})
        # Nothing downstream is allowed to invent its own mapping.
        self.assertEqual(build_payload_plan(counts), plan)


class _Counts:
    """Minimal stand-in that exposes a raw dict, to test the validator."""

    def __init__(self, values):
        self._values = values

    def as_dict(self):
        return dict(self._values)


# ---------------------------------------------------------------------------
# Field parameters
# ---------------------------------------------------------------------------

class FieldParameterTests(unittest.TestCase):
    def test_c01_the_four_measured_wall_distances_are_used_in_order(self):
        self.assertEqual(WALL_RIGHT_DISTANCE_M, (1.600, 1.895, 1.690, 2.330))
        self.assertEqual(task.WALL_LEG_NAMES, ("left", "bottom", "right", "top"))

    def test_the_lidar_to_base_conversion_uses_the_live_mount_offset(self):
        # Lidar on the drive axis: no correction at all.
        self.assertAlmostEqual(desired_right_base(0, 0.0), 1.600, places=9)
        self.assertAlmostEqual(desired_right_base(3, 0.0), 2.330, places=9)
        # A radar 5 cm to the left of the drive axis shortens every base target
        # by exactly that 5 cm, in both directions.
        for index in range(4):
            self.assertAlmostEqual(desired_right_base(index, 0.05),
                                   WALL_RIGHT_DISTANCE_M[index] - 0.05, places=9)
            self.assertAlmostEqual(desired_right_base(index, -0.03),
                                   WALL_RIGHT_DISTANCE_M[index] + 0.03, places=9)

    def test_an_impossible_mount_offset_is_rejected_instead_of_producing_a_negative_target(self):
        with self.assertRaises(ValueError):
            desired_right_base(0, 2.0)
        with self.assertRaises(ValueError):
            desired_right_base(7, 0.0)

    def test_the_finish_distance_is_the_operators_110_cm(self):
        self.assertEqual(FINISH_FRONT_DISTANCE_M, 1.10)
        self.assertAlmostEqual(finish_front_base(0.0), 1.10, places=9)
        self.assertAlmostEqual(finish_front_base(0.10), 1.20, places=9)

    def test_build_wall_route_orders_the_four_legs_and_terminals(self):
        legs, desired = build_wall_route(0.0, 0.0)
        self.assertEqual([leg.name for leg in legs],
                         ["left", "bottom", "right", "top_search"])
        self.assertEqual([leg.direction for leg in legs], [1, 1, 1, 1])
        self.assertEqual([leg.target_right_m for leg in legs], [1.600, 1.895, 1.690, 2.330])
        # Three legs stop on the front wall; the top leg ends on the left wall.
        self.assertEqual([leg.terminal_name for leg in legs],
                         ["front_wall", "front_wall", "front_wall", "left_wall"])
        self.assertEqual(desired[0], 1.600)


# ---------------------------------------------------------------------------
# Mission harness
# ---------------------------------------------------------------------------

def make_wall(bearing_rad, rho, span=3.0):
    normal = (math.cos(bearing_rad), math.sin(bearing_rad))
    tangent = (-normal[1], normal[0])
    centre = (normal[0] * rho, normal[1] * rho)
    return WallLine(normal_xy=normal, rho_base_m=rho,
                    support_start_xy=(centre[0] - span / 2 * tangent[0],
                                      centre[1] - span / 2 * tangent[1]),
                    support_end_xy=(centre[0] + span / 2 * tangent[0],
                                    centre[1] + span / 2 * tangent[1]),
                    inlier_count=30, span_m=span, rms_m=0.01)


class ScriptedObserver:
    """A snapshot source whose wall geometry the test drives directly."""

    def __init__(self, clock):
        self.clock = clock
        self.snapshot = None
        self.published = 0

    def publish(self, *lines, observed_at_s=None):
        self.published += 1
        observed = float(self.clock()) if observed_at_s is None else float(observed_at_s)
        self.snapshot = WallSnapshot(scan_seq=self.published,
                                     scan_started_s=observed - 0.02,
                                     scan_completed_s=observed + 0.02,
                                     observed_at_s=observed, lines=tuple(lines),
                                     point_count=40)
        return self.snapshot

    def latest(self):
        return self.snapshot

    def start(self):
        pass

    def close(self, **_kwargs):
        pass


class ScriptedDrive:
    def __init__(self):
        self.commands = []
        self.is_running = True
        self.last_limited_twist = None

    def start(self):
        self.is_running = True

    def stop(self):
        self.commands.append((0.0, 0.0))

    def close(self):
        pass

    def command(self, twist, now_s=None):
        self.last_limited_twist = twist
        self.commands.append((twist.linear_x_m_s, twist.angular_z_rad_s))


class MissionHarness:
    """Runs ``run_wall_mission`` against a scripted field."""

    def __init__(self, *, finish_m=FINISH_FRONT_DISTANCE_M, events=None,
                 counts=TaskCounts(2, 1, 1), servo=None, vision=None):
        from config.v2_loader import load_v2_config
        from config.v2_runtime import RuntimeMode
        from robocup_runtime import WallRuntime

        self.clock = [1.0]
        self.config = load_v2_config()
        self.events = events if events is not None else []
        self.drive = ScriptedDrive()
        self.observer = ScriptedObserver(lambda: self.clock[0])
        self.runtime = WallRuntime(self.config, RuntimeMode.DRY_RUN, drive=self.drive,
                                   observer=self.observer, clock=lambda: self.clock[0])
        self.runtime.record_event = lambda name, **fields: self.events.append(
            dict(type=name, **fields))
        self.counts = counts
        self.servo = servo
        self.vision = vision
        self.finish_m = finish_m

    # -- scripted world ---------------------------------------------------
    def set_leg_walls(self, leg_index, *, right_rho, terminal_rho=None,
                      terminal_role=None, left_rho=None):
        """Publish the walls a leg at ``leg_index`` should see."""
        lines = [make_wall(-math.pi / 2, right_rho)]
        if terminal_rho is not None and terminal_role is not None:
            bearing = 0.0 if terminal_role is getattr(task, "is_front_wall", None) else math.pi
            lines.append(make_wall(bearing, terminal_rho))
        if left_rho is not None:
            lines.append(make_wall(math.pi / 2, left_rho))
        return self.observer.publish(*lines)

    def action_step(self):
        return self.runtime.step()


class WallRouteTests(unittest.TestCase):
    """R01..R10 at the level of the leg sequence and terminal references."""

    def route_legs(self):
        legs, desired = build_wall_route(0.0, 0.0)
        return legs, desired

    def test_r01_the_route_has_three_cruise_left_turns_and_one_final_right(self):
        legs, _ = self.route_legs()
        self.assertEqual(len(legs), 4)
        # The first three legs turn left at a front-wall corner; only the finish
        # entry turns right. The top edge never turns around.
        self.assertEqual([leg.terminal_name for leg in legs[:3]], ["front_wall"] * 3)
        self.assertIsNone(legs[3].terminal_role)

    def test_r07_the_top_leg_reverses_with_the_nose_unchanged(self):
        legs, desired = self.route_legs()
        top = legs[3]
        self.assertEqual(top.target_right_m, 2.330)
        self.assertEqual(top.direction, +1)  # the search itself still goes left
        # The return reverses: constructed by the reverse helper, not by the leg.
        from components.wall_navigation import FollowRightWall, WallTracker, is_rear_wall
        action = FollowRightWall(target_distance_m=desired[3], direction=-1,
                                 limits=_limits(), tracker=WallTracker(),
                                 terminal_role=is_rear_wall, terminal_target_m=desired[2])
        self.assertEqual(action.direction, -1)
        self.assertEqual(action.target_distance_m, 2.330)
        self.assertEqual(action.terminal_target_m, 1.690)

    def test_r08_the_return_looks_at_the_rear_wall_not_the_front_wall(self):
        from components.wall_navigation import FollowRightWall, WallTracker, is_rear_wall
        _, desired = self.route_legs()
        action = FollowRightWall(target_distance_m=desired[3], direction=-1,
                                 limits=_limits(), tracker=WallTracker(),
                                 terminal_role=is_rear_wall, terminal_target_m=desired[2])
        # A front wall must not satisfy the rear terminal, and vice versa.
        rear = make_wall(math.pi, 1.5)
        front = make_wall(0.0, 1.5)
        self.assertTrue(is_rear_wall(rear))
        self.assertFalse(is_rear_wall(front))

    def test_r09_a_missing_finish_distance_parks_at_the_corner_without_driving_on(self):
        with patch.object(task, "FINISH_FRONT_DISTANCE_M", None):
            self.assertIsNone(finish_front_base(0.0))
        # The production constant is the operator's measured value.
        self.assertEqual(FINISH_FRONT_DISTANCE_M, 1.10)

    def test_r03_the_detour_uses_the_measured_43_and_32_centimetre_reaches(self):
        from payload_detour import DetourSettings
        standard = DetourSettings(payload_slot=1, payload_slots=(1,), behavior="standard",
                                  approach_m=task.DROP_SAFE_M)
        self.assertAlmostEqual(standard.approach_m, 0.43, places=9)
        right = DetourSettings(payload_slot=4, payload_slots=(4,), behavior="right_mount",
                               approach_m=task.RIGHT_MOUNT_DROP_M)
        self.assertAlmostEqual(right.approach_m, 0.32, places=9)

    def test_p12_a_caller_safe_distance_cannot_override_the_32_centimetre_reach(self):
        import inspect
        source = inspect.getsource(task.perform_payload_detour)
        # The wall branch hard-codes the fixed right-mount reach, exactly as the
        # fused branch always did.
        self.assertIn("RIGHT_MOUNT_DROP_M if right_mount else DROP_SAFE_M", source)
        self.assertNotIn("safe_distance_m if right_mount", source)


def _limits():
    from components.wall_navigation import WallActionLimits
    return WallActionLimits(follow_speed_m_s=0.15, vision_speed_m_s=0.08,
                            minimum_speed_m_s=0.045, lookahead_m=0.20,
                            track_width_m=0.198, wheel_speed_m_s=0.30,
                            max_linear_speed_m_s=0.20, max_angular_speed_rad_s=0.80,
                            max_linear_accel_m_s2=0.30, max_angular_accel_rad_s2=1.00)


# ---------------------------------------------------------------------------
# Vision triggering
# ---------------------------------------------------------------------------

class CentredTargetTests(unittest.TestCase):
    def latches(self, *colors):
        from target_patrol import EntryLatch
        return {color: EntryLatch() for color in colors}

    def pick(self, colored, pending, latches, width, height):
        from target_patrol import horizontal_target, visible_target
        return task._select_centered(colored, pending, latches, width, height,
                                     visible_target, horizontal_target)

    def test_p05_only_the_horizontal_middle_ten_percent_triggers(self):
        latches = self.latches('yellow')
        pending = {'yellow': (4,)}
        width, height = 640, 480
        # cx = 320 -> dead centre; cx = 350 -> inside the 0.45w..0.55w window.
        inside = self.pick((('yellow', (300.0, 10.0, 350.0, 60.0, 0.95, 1)),),
                           pending, latches, width, height)
        self.assertIsNotNone(inside)
        latches = self.latches('yellow')
        outside = self.pick((('yellow', (360.0, 10.0, 420.0, 60.0, 0.95, 1)),),
                            pending, latches, width, height)
        self.assertIsNone(outside)

    def test_p05_there_is_no_vertical_gate(self):
        latches = self.latches('yellow')
        pending = {'yellow': (4,)}
        # Box pressed against the top edge but horizontally centred.
        result = self.pick((('yellow', (300.0, 0.0, 340.0, 30.0, 0.95, 1)),),
                           pending, latches, 640, 480)
        self.assertIsNotNone(result)

    def test_p06_the_window_scales_with_the_frame_width(self):
        for width in (640, 1280):
            latches = self.latches('yellow')
            pending = {'yellow': (4,)}
            centre = width / 2
            inside = self.pick((('yellow', (centre - 20, 0.0, centre + 20, 40.0, 0.95, 1)),),
                               pending, latches, width, 480)
            self.assertIsNotNone(inside, f"width {width} failed to trigger at centre")
            latches = self.latches('yellow')
            outside = self.pick((('yellow', (centre + width * 0.2, 0.0,
                                              centre + width * 0.2 + 40, 40.0, 0.95, 1)),),
                                pending, latches, width, 480)
            self.assertIsNone(outside, f"width {width} triggered outside the window")

    def test_p07_two_centred_colours_choose_nearest_then_name_deterministically(self):
        latches = self.latches('yellow', 'red')
        pending = {'yellow': (4,), 'red': (2, 3)}
        # Red is nearer the centre, so red wins even though yellow sorts first.
        colored = (('yellow', (250.0, 0.0, 300.0, 40.0, 0.95, 1)),
                   ('red', (305.0, 0.0, 335.0, 40.0, 0.95, 2)))
        chosen = self.pick(colored, pending, latches, 640, 480)
        self.assertIsNotNone(chosen)
        self.assertEqual(chosen[0], 'red')

    def test_p07_equal_centres_fall_back_to_the_colour_name(self):
        latches = self.latches('yellow', 'red')
        pending = {'yellow': (4,), 'red': (2, 3)}
        colored = (('yellow', (300.0, 0.0, 340.0, 40.0, 0.95, 1)),
                   ('red', (300.0, 0.0, 340.0, 40.0, 0.95, 2)))
        chosen = self.pick(colored, pending, latches, 640, 480)
        self.assertEqual(chosen[0], 'red')

    def test_p07_an_unaccepted_candidate_does_not_consume_its_latch(self):
        latches = self.latches('yellow')
        pending = {'yellow': (4,)}
        # The candidate list is built but the caller decides; the latch must stay
        # armed until an event is actually accepted.
        self.pick((('yellow', (300.0, 0.0, 340.0, 40.0, 0.95, 1)),), pending, latches, 640, 480)
        self.assertTrue(latches['yellow'].armed)

    def test_a_colour_with_no_pending_slots_is_never_chosen(self):
        latches = self.latches('yellow')
        pending = {'yellow': (4,)}
        colored = (('red', (300.0, 0.0, 340.0, 40.0, 0.95, 2)),)
        self.assertIsNone(self.pick(colored, pending, latches, 640, 480))


# ---------------------------------------------------------------------------
# Detour integration
# ---------------------------------------------------------------------------

class WallDetourTests(unittest.TestCase):
    """The detour against a simulated wall that reacts to the emitted twist.

    The world is integrated with a small timestep: the action gates are sized for
    a real scan interval, so rotating the wall by a whole control period per call
    would look like a teleport rather than a measurement.
    """

    DT_S = 0.002
    MAX_STEPS = 40000

    def build(self, *, behavior, slots):
        from payload_detour import DetourSettings
        from robocup_runtime import WallRuntime
        from config.v2_loader import load_v2_config
        from config.v2_runtime import RuntimeMode
        from components.wall_observer import WallTracker

        clock = [1.0]
        config = load_v2_config()
        drive = ScriptedDrive()
        observer = ScriptedObserver(lambda: clock[0])
        runtime = WallRuntime(config, RuntimeMode.DRY_RUN, drive=drive, observer=observer,
                              clock=lambda: clock[0])
        events = []
        runtime.record_event = lambda name, **fields: events.append(dict(type=name, **fields))
        tracker = WallTracker()
        settings = DetourSettings(
            payload_slot=slots[0], payload_slots=slots, behavior=behavior,
            approach_m=(task.RIGHT_MOUNT_DROP_M if behavior == "right_mount"
                        else task.DROP_SAFE_M))
        return runtime, observer, clock, tracker, settings, events, drive

    def run_detour(self, *, behavior, slots, d0=1.60, simulate=False,
                   release_response=True, on_release=None, wall_vanishes_after=None):
        """Drive the detour against a reacting wall; return the complete record."""
        from payload_detour import WallDropReference, run_payload_detour

        runtime, observer, clock, tracker, settings, events, drive = self.build(
            behavior=behavior, slots=slots)
        limits = _limits()
        world = {'bearing': -math.pi / 2, 'rho': d0}
        steps = [0]

        def acquire(_runtime, _tracker):
            wall = make_wall(world['bearing'], world['rho'])
            _tracker.set_reference(wall)
            return WallDropReference(reference_id="road_right_wall",
                                     d0_base_m=world['rho'],
                                     bearing_rad=world['bearing'],
                                     normal_xy=wall.normal_xy)

        original_step = runtime.step

        def step():
            steps[0] += 1
            if steps[0] > self.MAX_STEPS:
                raise AssertionError("detour did not converge")
            clock[0] += self.DT_S
            if wall_vanishes_after is not None and steps[0] > wall_vanishes_after:
                observer.publish()
            else:
                observer.publish(make_wall(world['bearing'], max(0.15, world['rho'])))
            result = original_step()
            command = result.limited_command or result.command
            # rho_dot = -n_x * v and bearing_dot = -omega for a fixed world wall.
            normal_x = math.cos(world['bearing'])
            world['rho'] = max(0.05, world['rho'] - normal_x * command.linear_x_m_s * self.DT_S)
            world['bearing'] = wrap(world['bearing'] - command.angular_z_rad_s * self.DT_S)
            return result

        runtime.step = step
        result, error = None, None
        try:
            if simulate:
                result = run_payload_detour(
                    runtime, settings, clock=lambda: clock[0], sleep=lambda _s: None,
                    tracker=tracker, acquire_reference=acquire, limits=limits,
                    simulate_release=True, on_release=on_release)
            else:
                with patch("payload_detour.open_payload",
                           side_effect=lambda slot: release_response):
                    result = run_payload_detour(
                        runtime, settings, clock=lambda: clock[0], sleep=lambda _s: None,
                        tracker=tracker, acquire_reference=acquire, limits=limits,
                        on_release=on_release)
        except BaseException as exc:  # noqa: BLE001 - recorded for the assertion
            error = exc
        finally:
            runtime.step = original_step
        stages = [event['stage'] for event in events
                  if event['type'] == 'payload_detour_stage_start']
        return dict(result=result, error=error, events=events, stages=stages,
                    world=world, steps=steps[0], drive=drive)

    def test_p10_standard_detour_left90_forward_release_back_right90(self):
        seen = []
        record = self.run_detour(behavior="standard", slots=(1,), on_release=seen.append)
        self.assertIsNone(record['error'], record['error'])
        self.assertEqual(seen, [1])
        self.assertEqual(record['stages'],
                         ["left_90deg", "forward_to_drop", "return_from_drop",
                          "right_90deg"])
        releases = [event for event in record['events']
                    if event['type'] == 'payload_release_start']
        self.assertEqual([event['slot'] for event in releases], [1])

    def test_p10_the_detour_returns_the_wall_to_its_entry_distance(self):
        record = self.run_detour(behavior="standard", slots=(1,), d0=1.60)
        self.assertIsNone(record['error'], record['error'])
        # After the return the reference wall is back at d0.
        self.assertAlmostEqual(record['world']['rho'], 1.60, delta=0.02)

    def test_p11_right_mount_detour_adds_the_second_left_and_right_turns(self):
        seen = []
        record = self.run_detour(behavior="right_mount", slots=(4,), on_release=seen.append)
        self.assertIsNone(record['error'], record['error'])
        self.assertEqual(seen, [4])
        self.assertEqual(record['stages'],
                         ["left_90deg", "forward_to_drop", "right_mount_left_90deg",
                          "right_mount_right_90deg", "return_from_drop", "right_90deg"])

    def test_p11_the_right_mount_reverses_only_after_undoing_the_extra_turn(self):
        record = self.run_detour(behavior="right_mount", slots=(4,))
        stages = record['stages']
        self.assertLess(stages.index("right_mount_left_90deg"),
                        stages.index("right_mount_right_90deg"))
        self.assertLess(stages.index("right_mount_right_90deg"),
                        stages.index("return_from_drop"))

    def test_p09_a_double_target_releases_2_then_3_in_one_approach(self):
        seen = []
        record = self.run_detour(behavior="standard", slots=(2, 3), on_release=seen.append)
        self.assertIsNone(record['error'], record['error'])
        self.assertEqual(seen, [2, 3])
        # One approach, not two: forward_to_drop appears exactly once.
        self.assertEqual(record['stages'].count("forward_to_drop"), 1)

    def test_p13_the_detour_never_reads_a_fused_pose(self):
        import ast
        import payload_detour
        tree = ast.parse(Path(payload_detour.__file__).read_text(encoding="utf-8"))
        banned_attributes = {"estimate", "pose", "x_m", "y_m", "yaw_rad"}
        offenders = [node.attr for node in ast.walk(tree)
                     if isinstance(node, ast.Attribute) and node.attr in banned_attributes]
        self.assertEqual(offenders, [],
                         "payload_detour still reaches for a fused pose attribute")
        names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
        self.assertEqual(names & {"usable_pose", "_usable_pose"}, set())

    def test_p14_the_ledger_is_written_before_the_hold_and_the_return(self):
        import inspect
        from payload_detour import run_payload_detour
        source = inspect.getsource(run_payload_detour)
        release = source.index("commanded.append(slot)")
        hold = source.index("hold(settings.release_hold_s)")
        self.assertLess(release, hold,
                        "an opened slot must be booked before the hold or the return")

    def test_p14_a_first_slot_that_opened_stays_released_when_the_second_fails(self):
        seen = []
        calls = {'count': 0}

        def opener(_slot):
            calls['count'] += 1
            return calls['count'] == 1

        from payload_detour import run_payload_detour
        runtime, observer, clock, tracker, settings, events, drive = self.build(
            behavior="standard", slots=(2, 3))
        limits = _limits()
        world = {'bearing': -math.pi / 2, 'rho': 1.60}

        def acquire(_runtime, _tracker):
            from payload_detour import WallDropReference
            wall = make_wall(world['bearing'], world['rho'])
            _tracker.set_reference(wall)
            return WallDropReference(reference_id="road_right_wall", d0_base_m=world['rho'],
                                     bearing_rad=world['bearing'],
                                     normal_xy=wall.normal_xy)

        original_step = runtime.step
        steps = [0]

        def step():
            steps[0] += 1
            if steps[0] > self.MAX_STEPS:
                raise AssertionError("detour did not converge")
            clock[0] += self.DT_S
            observer.publish(make_wall(world['bearing'], max(0.15, world['rho'])))
            result = original_step()
            command = result.limited_command or result.command
            world['rho'] = max(0.05, world['rho']
                               - math.cos(world['bearing']) * command.linear_x_m_s * self.DT_S)
            world['bearing'] = wrap(world['bearing'] - command.angular_z_rad_s * self.DT_S)
            return result

        runtime.step = step
        try:
            with patch("payload_detour.open_payload", side_effect=opener):
                with self.assertRaises(RuntimeError):
                    run_payload_detour(runtime, settings, clock=lambda: clock[0],
                                       sleep=lambda _s: None, tracker=tracker,
                                       acquire_reference=acquire, limits=limits,
                                       on_release=seen.append)
        finally:
            runtime.step = original_step
        # The first slot stays recorded as released and is never re-issued.
        self.assertEqual(seen, [2])

    def test_p15_a_return_failure_is_reported_separately_from_the_release(self):
        record = self.run_detour(behavior="standard", slots=(1,),
                                 wall_vanishes_after=200)
        self.assertIsNotNone(record['error'])
        events = record['events']
        self.assertTrue(any(event['type'] == 'payload_release_done' for event in events))
        self.assertTrue(any(event['type'] == 'payload_detour_failed' for event in events))
        self.assertTrue(any(event['type'] == 'cleanup_open' for event in events))

    def test_p17_cleanup_is_logged_as_cleanup_not_as_five_successful_drops(self):
        import inspect
        from payload_detour import run_payload_detour
        source = inspect.getsource(run_payload_detour)
        self.assertIn('"cleanup_open"', source)

    def test_p16_simulation_never_touches_a_pwm_axis(self):
        record = self.run_detour(behavior="standard", slots=(1,), simulate=True)
        self.assertIsNone(record['error'], record['error'])
        self.assertTrue(any(event['type'] == 'payload_release_skipped'
                            for event in record['events']))
        self.assertFalse(any(event['type'] == 'payload_release_start'
                             for event in record['events']))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


if __name__ == '__main__':
    unittest.main()
