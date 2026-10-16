"""Human-readable allocation and payload servo action log for one full run."""

from datetime import datetime
import os
from pathlib import Path


COLOR_NAMES = {'red': '红色', 'blue': '蓝色', 'green': '绿色', 'yellow': '黄色'}


class PayloadDropLog:
    def __init__(self, directory, counts, car_counts, drone_counts):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.started = datetime.now().astimezone()
        self.task_id = self.started.strftime('%Y%m%d-%H%M%S-%f')
        self.path = self.directory / ('payload_' + self.task_id + '.txt')
        double = next(color for color, count in car_counts.items() if count == 2)
        single = next(color for color, count in car_counts.items() if count == 1)
        self.targets = (('yellow', (0,)), (double, (2, 3)), (single, (1,)))
        self.counts = counts.as_dict()
        self.drone_counts = drone_counts.as_dict()
        self.actions = {}
        with self.path.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(self._render())

    def record_release(self, slot, when):
        if slot not in (0, 1, 2, 3):
            raise ValueError('unknown payload slot')
        self.actions.setdefault(slot, when.astimezone().isoformat(sep=' ', timespec='milliseconds'))
        temporary = self.path.with_suffix('.tmp')
        with temporary.open('w', encoding='utf-8', newline='\n') as stream:
            stream.write(self._render())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path)

    def _render(self):
        amounts = '，'.join('%s %d 份' % (COLOR_NAMES[color], self.counts[color])
                           for color in ('red', 'blue', 'green'))
        lines = ['任务编号：' + self.task_id, '任务识别结果：' + amounts, '任务分配']
        lines.append('小车：' + '；'.join('目标点%d（%s）%d 份，槽位 %s' % (
            index, COLOR_NAMES[color], len(slots), '、'.join(map(str, slots)))
            for index, (color, slots) in enumerate(self.targets, 1)))
        lines.append('无人机：黄色目标 1 份；' + '；'.join('%s目标 %d 份' % (
            COLOR_NAMES[color], amount) for color, amount in self.drone_counts.items() if amount))
        lines.append('小车投放记录')
        for index, (color, slots) in enumerate(self.targets, 1):
            for item, slot in enumerate(slots, 1):
                lines.append('目标点%d（%s）｜第 %d 份（槽位 %d）｜舵机动作时刻：%s' % (
                    index, COLOR_NAMES[color], item, slot, self.actions.get(slot, '未投放')))
        return '\n'.join(lines) + '\n'
