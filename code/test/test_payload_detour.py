"""Payload detour contract that is shared by both feedback sources.

The **wall** detour itself is exercised end to end in
``test_wall_competition.py`` (turn order, 43/32 cm reaches, release ledger,
cleanup logging, a lost reference wall). What lives here is the part common to
every detour: the settings contract that decides which slots a behaviour may
release and which reach it may use.
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from payload_detour import DetourSettings


class DetourSettingsTests(unittest.TestCase):
    def test_behavior_validation_rejects_unknown_or_mixed_mechanisms(self):
        for behavior, slots in (('unknown', (0,)), ('right_mount', (0,)),
                                ('right_mount', (2, 3)), ('right_mount', (4, 0)),
                                ('standard', (4,)), ('standard', (1, 4))):
            with self.subTest(behavior=behavior, slots=slots), self.assertRaises(ValueError):
                DetourSettings(payload_slots=slots, behavior=behavior)

    def test_right_mount_is_reserved_for_slot_4_alone(self):
        settings = DetourSettings(payload_slot=4, payload_slots=(4,),
                                  behavior='right_mount', approach_m=0.32)
        self.assertEqual(settings.release_slots, (4,))
        self.assertAlmostEqual(settings.approach_m, 0.32, places=9)

    def test_standard_never_carries_slot_4(self):
        settings = DetourSettings(payload_slot=1, payload_slots=(1,), behavior='standard')
        self.assertEqual(settings.release_slots, (1,))
        self.assertNotIn(4, settings.release_slots)

    def test_grouped_slots_reject_reuse_and_out_of_range(self):
        for slots in ((2, 2), (2, 4), (True,), ('2',)):
            with self.subTest(slots=slots), self.assertRaises(ValueError):
                DetourSettings(payload_slots=slots)

    def test_out_of_range_slot_is_rejected(self):
        for slot in (5, -1, True, '4'):
            with self.subTest(slot=slot), self.assertRaises(ValueError):
                DetourSettings(payload_slot=slot)

    def test_non_positive_geometry_is_rejected(self):
        for field, value in (('approach_m', 0.0), ('approach_m', -0.1),
                             ('release_hold_s', 0.0), ('position_tolerance_m', 0.0),
                             ('patrol_slow_speed_m_s', 0.0)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                DetourSettings(**{field: value})

    def test_the_default_release_slot_is_the_single_declared_slot(self):
        settings = DetourSettings(payload_slot=2)
        self.assertEqual(settings.release_slots, (2,))


if __name__ == "__main__":
    unittest.main()
