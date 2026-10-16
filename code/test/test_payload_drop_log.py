"""Hardware-free checks for the human-readable payload action log."""

from datetime import datetime, timezone
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.payload_drop_log import PayloadDropLog
from components.task_board_reader import TaskCounts
from competition_task import DroneAllocation


class PayloadDropLogTests(unittest.TestCase):
    def test_allocation_and_individual_servo_actions(self):
        with tempfile.TemporaryDirectory() as directory:
            log = PayloadDropLog(directory, TaskCounts(2, 1, 1),
                                 {'red': 2, 'blue': 1}, DroneAllocation(0, 0, 1))
            initial = log.path.read_text(encoding='utf-8')
            self.assertIn('任务识别结果：红色 2 份，蓝色 1 份，绿色 1 份', initial)
            self.assertIn('无人机：黄色目标 1 份；绿色目标 1 份', initial)
            self.assertEqual(initial.count('舵机动作时刻：未投放'), 4)
            when = datetime(2026, 10, 16, 12, 34, 56, 789000,
                            tzinfo=timezone.utc)
            log.record_release(2, when)
            updated = log.path.read_text(encoding='utf-8')
            self.assertIn('目标点2（红色）｜第 1 份（槽位 2）｜舵机动作时刻：', updated)
            self.assertEqual(updated.count('舵机动作时刻：未投放'), 3)
            self.assertNotIn('舵机动作时刻：', updated.split('无人机：', 1)[1].split('小车投放记录', 1)[0])
            self.assertEqual(len(list(Path(directory).glob('payload_*.txt'))), 1)


if __name__ == '__main__':
    unittest.main()
