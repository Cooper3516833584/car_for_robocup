"""Run-button settings reach the actual wall mission without opening hardware."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import car_full_flow_test as test_entry
import main_robocup as main_entry
import competition_task as task
from components.competition_options import CompetitionOptions
from components.task_board_reader import TaskCounts
from test_wall_competition import MissionHarness, make_wall


class OperatorSettingsTests(unittest.TestCase):
    def test_main_run_button_starts_formal_mission_without_arguments(self):
        with patch('components.local_mission_control.run_local', return_value=0) as run, \
                patch.object(main_entry, 'WALL_RIGHT_DISTANCE_M', (1.2, 1.3, 1.4, 1.5)), \
                patch.object(main_entry, 'STANDARD_DROP_M', .48), \
                patch.object(main_entry, 'RIGHT_MOUNT_DROP_M', .35):
            self.assertEqual(main_entry.launch([]), 0)
        settings = CompetitionOptions(**run.call_args.args[0])
        self.assertEqual(settings.wall_right_distance_m, (1.2, 1.3, 1.4, 1.5))
        self.assertEqual(settings.standard_drop_m, .48)
        self.assertEqual(settings.right_mount_drop_m, .35)
        self.assertIsNone(settings.task_counts)

    def test_test_run_button_supplies_fixed_121_and_its_own_distances(self):
        normal = main_entry.operator_options()
        with patch.object(test_entry, '_run_local', return_value=0) as run, \
                patch.object(test_entry, 'WALL_RIGHT_DISTANCE_M', (2.1, 2.2, 2.3, 2.4)), \
                patch.object(test_entry, 'STANDARD_DROP_M', .52), \
                patch.object(test_entry, 'RIGHT_MOUNT_DROP_M', .38):
            self.assertEqual(test_entry.main([]), 0)
        settings = CompetitionOptions(**run.call_args.args[0])
        self.assertEqual(settings.task_counts, (1, 2, 1))
        self.assertEqual(settings.wall_right_distance_m, (2.1, 2.2, 2.3, 2.4))
        self.assertEqual(settings.standard_drop_m, .52)
        self.assertEqual(settings.right_mount_drop_m, .38)
        self.assertEqual(main_entry.operator_options(), normal)

    def test_fixed_rule_skips_ocr_and_custom_distances_reach_all_four_legs(self):
        for counts in (None, (1, 2, 1)):
            with self.subTest(counts=counts):
                h = MissionHarness()
                h.runtime.start()
                options = CompetitionOptions(wall_right_distance_m=(1.2, 1.3, 1.4, 1.5),
                                             finish_front_m=.9, task_counts=counts)
                h.runtime.competition_options = options
                wall = make_wall(0, 1.3)
                reverse = Mock(terminal_wall=wall)
                with patch('config.v2_factory.build_servo', return_value=Mock(is_running=False)), \
                        patch('components.servo_positioning.park_servo'), \
                        patch.object(task, 'read_task_board', return_value=TaskCounts(2, 1, 1)) as read, \
                        patch.object(task, 'patrol_wall_leg', return_value=(True, False, wall)) as patrol, \
                        patch.object(task, 'turn_left_at_corner'), \
                        patch.object(task, 'reverse_top_to_finish_corner', return_value=reverse), \
                        patch.object(task, 'enter_finish_zone', return_value=True):
                    result = task.run_wall_mission(h.runtime)
                h.runtime.close()
                self.assertTrue(result.parking_done)
                self.assertEqual([c.args[1].target_right_m for c in patrol.call_args_list],
                                 [1.2, 1.3, 1.4, 1.5])
                if counts is None:
                    read.assert_called_once()
                    self.assertEqual(h.runtime.mission.task_counts.key, (2, 1, 1))
                else:
                    read.assert_not_called()
                    self.assertEqual(h.runtime.mission.task_counts.key, (1, 2, 1))
                self.assertAlmostEqual(task.finish_front_base(.05, options=options), .95)


if __name__ == '__main__':
    unittest.main()
