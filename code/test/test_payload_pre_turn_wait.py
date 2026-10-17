"""Real detour controllers with fake wall scans and actuator outputs."""

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mission_control import task_scope
from components.wall_observer import WallTracker
from payload_detour import DetourSettings, WallDropReference, run_payload_detour
from test_wall_competition import MissionHarness, _limits, make_wall


class PreTurnWaitTests(unittest.TestCase):
    def run_detour(self, *, missing=None, wait_ticks=0, rear_only=False,
                   behavior="standard", stop_at=None):
        h = MissionHarness()
        h.runtime.start()
        self.addCleanup(h.runtime.close)
        world = dict(x=4.0, y=1.6, yaw=0.0)
        tracker = WallTracker()
        slots = (4,) if behavior == "right_mount" else (2, 3)
        settings = DetourSettings(payload_slot=slots[0], payload_slots=slots,
                                  behavior=behavior,
                                  approach_m=.32 if behavior == "right_mount" else .43)
        original_step = h.runtime.step
        ticks, commands, released = [], [], []

        def step():
            self.assertLess(len(ticks), 4000, "detour failed to converge")
            h.clock[0] += .05
            x, y, yaw = world["x"], world["y"], world["yaw"]
            walls = [make_wall(angle - yaw, distance, span=8.0)
                     for angle, distance in ((math.pi, x), (-math.pi / 2, y),
                                             (0, 8 - x), (math.pi / 2, 8 - y))]
            waiting = len(ticks) < wait_ticks
            if waiting and missing == "none":
                h.observer.snapshot = None
            elif waiting and missing == "stale":
                h.observer.publish(*walls, observed_at_s=h.clock[0] - 1)
            elif waiting:
                h.observer.publish(walls[1], walls[3])  # Side walls cannot measure 5 cm.
            else:
                h.observer.publish(*(walls[:2] + walls[3:] if rear_only else walls))
            result = original_step()
            command = result.limited_command or result.command
            ticks.append(command)
            if waiting:
                self.assertIsNone(h.runtime.action)
                self.assertEqual((command.linear_x_m_s, command.angular_z_rad_s), (0, 0))
                self.assertFalse(released)
            else:
                commands.append((command.linear_x_m_s, command.angular_z_rad_s))
            world["x"] += command.linear_x_m_s * math.cos(yaw) * .05
            world["y"] += command.linear_x_m_s * math.sin(yaw) * .05
            world["yaw"] += command.angular_z_rad_s * .05
            return result

        def acquire(_runtime, target):
            road = make_wall(-math.pi / 2, 1.6)
            target.set_reference(road)
            return WallDropReference(reference_id="road_right_wall", d0_base_m=1.6,
                                     bearing_rad=road.bearing_rad, normal_xy=road.normal_xy)

        def guard():
            if stop_at is not None and len(ticks) >= stop_at:
                raise RuntimeError("diagnostic STOP")

        def release(slot=None):
            released.append(slot)
            return True

        h.runtime.step = step
        with patch("payload_detour.open_payload", side_effect=release), task_scope(guard):
            if stop_at is not None:
                with self.assertRaisesRegex(RuntimeError, "diagnostic STOP"):
                    run_payload_detour(h.runtime, settings, tracker=tracker,
                                       acquire_reference=acquire, limits=_limits(),
                                       clock=lambda: h.clock[0], sleep=lambda _: None)
                # STOP retains the existing all-slot cleanup, never a target release.
                self.assertEqual(released, [None])
                self.assertEqual(len(ticks), stop_at)
                self.assertIsNone(h.runtime.action)
                self.assertEqual(h.drive.commands[-1], (0, 0))
                return
            result = run_payload_detour(h.runtime, settings, tracker=tracker,
                                        acquire_reference=acquire, limits=_limits(),
                                        clock=lambda: h.clock[0], sleep=lambda _: None)
        self.assertEqual(result, slots)
        self.assertEqual(released, list(slots))
        advances = [e for e in h.events if e["type"] == "payload_pre_turn_advance"]
        self.assertEqual(len(advances), 1)
        self.assertEqual(advances[0]["distance_m"], .05)
        self.assertAlmostEqual(advances[0]["target_base_m"], 4.05 if rear_only else 3.95)
        self.assertEqual(h.drive.commands[-1], (0, 0))
        return commands, world

    def test_missing_scan_stale_scan_and_side_walls_resume_normal_sequence(self):
        for behavior in ("standard", "right_mount"):
            baseline, end = self.run_detour(behavior=behavior)
            for missing in ("none", "stale", "sides"):
                with self.subTest(behavior=behavior, missing=missing):
                    commands, recovered_end = self.run_detour(
                        behavior=behavior, missing=missing, wait_ticks=20)
                    # Extra stopped ticks cannot change any subsequent motion command.
                    self.assertEqual(commands, baseline)
                    self.assertEqual(recovered_end, end)

    def test_rear_wall_can_supply_the_same_measured_advance_after_wait(self):
        self.run_detour(missing="sides", wait_ticks=20, rear_only=True)

    def test_stop_interrupts_a_persistent_missing_wall_wait(self):
        self.run_detour(missing="sides", wait_ticks=1000, stop_at=20)


if __name__ == "__main__":
    unittest.main()
