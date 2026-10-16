"""Hardware-free checks for the human-readable payload action log.

Target numbering follows ``payload_drop_log.TARGET_ORDER`` (yellow, red, blue,
green), and the log renders the shared payload plan verbatim rather than
rebuilding slot numbers of its own.
"""

from datetime import datetime, timezone
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir

from components.payload_drop_log import PayloadDropLog
from components.task_board_reader import TaskCounts
from competition_task import DroneAllocation, build_payload_plan


class PayloadDropLogTests(unittest.TestCase):
    def test_allocation_and_individual_servo_actions(self):
        counts = TaskCounts(2, 1, 1)
        plan = build_payload_plan(counts)
        with WorkspaceTempDir() as directory:
            log = PayloadDropLog(directory, counts,
                                 {'red': 2, 'blue': 1, 'green': 1},
                                 DroneAllocation(0, 0, 0), plan)
            initial = log.path.read_text(encoding='utf-8')
            self.assertIn('任务识别结果：红色 2 份，蓝色 1 份，绿色 1 份', initial)
            # The car carries all five payloads; no drone task is sent.
            self.assertIn('无人机：通信已禁用', initial)
            self.assertEqual(initial.count('舵机动作时刻：未投放'), 5)
            when = datetime(2026, 10, 16, 12, 34, 56, 789000, tzinfo=timezone.utc)
            log.record_release(2, when)
            updated = log.path.read_text(encoding='utf-8')
            self.assertIn('目标点2（红色）｜第 1 份（槽位 2）｜舵机动作时刻：', updated)
            self.assertEqual(updated.count('舵机动作时刻：未投放'), 4)
            # Slot 4 is the yellow right-mount drop, target point 1.
            log.record_release(4, when)
            updated = log.path.read_text(encoding='utf-8')
            self.assertIn('目标点1（黄色）｜第 1 份（槽位 4）｜舵机动作时刻：', updated)
            self.assertEqual(updated.count('舵机动作时刻：未投放'), 3)
            self.assertEqual(len(list(Path(directory).glob('payload_*.txt'))), 1)

    def test_log_consumes_the_plan_instead_of_rebuilding_it(self):
        counts = TaskCounts(1, 2, 1)  # blue is the double item
        plan = build_payload_plan(counts)
        self.assertEqual(plan, {'yellow': (4,), 'blue': (2, 3),
                                'red': (1,), 'green': (0,)})
        with WorkspaceTempDir() as directory:
            log = PayloadDropLog(directory, counts, counts.as_dict(),
                                 DroneAllocation(0, 0, 0), plan)
            self.assertEqual(dict(log.targets), plan)
            self.assertEqual(log.target_for_slot(4), 'yellow')
            self.assertEqual(log.target_for_slot(0), 'green')
            text = log.path.read_text(encoding='utf-8')
            # Target numbering follows TARGET_ORDER: yellow, red, blue, green.
            self.assertIn('目标点1（黄色）1 份，槽位 4', text)
            self.assertIn('目标点2（红色）1 份，槽位 1', text)
            self.assertIn('目标点3（蓝色）2 份，槽位 2、3', text)
            self.assertIn('目标点4（绿色）1 份，槽位 0', text)
            # The double item is released in one approach in slot order 2 then 3.
            self.assertIn('目标点3（蓝色）｜第 1 份（槽位 2）', text)
            self.assertIn('目标点3（蓝色）｜第 2 份（槽位 3）', text)

    def test_plan_rejects_a_bad_shape_or_a_slot_collision(self):
        with self.assertRaises(ValueError):
            # Green and blue both claim slot 1: the plan must use 0..4 once.
            PayloadDropLog._ordered_targets({'yellow': (4,), 'red': (2, 3),
                                             'blue': (1,), 'green': (1,)})
        with self.assertRaises(ValueError):
            PayloadDropLog._ordered_targets({'yellow': (4,), 'red': (2, 3),
                                             'blue': (1,)})

    def test_a_log_whose_plan_is_missing_a_colour_is_refused(self):
        counts = TaskCounts(1, 1, 2)
        plan = build_payload_plan(counts)
        broken = {key: value for key, value in plan.items() if key != 'yellow'}
        with self.assertRaises(ValueError):
            PayloadDropLog._ordered_targets(broken)


if __name__ == '__main__':
    unittest.main()
