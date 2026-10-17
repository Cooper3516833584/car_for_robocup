"""Human-readable allocation and payload servo action log for one full run."""

from datetime import datetime
import os
from pathlib import Path


COLOR_NAMES = {'red': '红色', 'blue': '蓝色', 'green': '绿色', 'yellow': '黄色'}
# Field-diagram order, then the double/single ordering the plan guarantees.
TARGET_ORDER = ('yellow', 'red', 'blue', 'green')


class PayloadDropLog:
    """Renders the one shared payload plan; it never re-derives slot numbers."""

    def __init__(self, directory, counts, car_counts, drone_counts, plan):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.started = datetime.now().astimezone()
        self.task_id = self.started.strftime('%Y%m%d-%H%M%S-%f')
        self.path = self.directory / ('payload_' + self.task_id + '.txt')
        self.targets = self._ordered_targets(plan)
        self.counts = counts.as_dict()
        self.drone_counts = drone_counts.as_dict()
        self.actions = {}
        with self.path.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(self._render())

    @staticmethod
    def _ordered_targets(plan):
        missing = [color for color in TARGET_ORDER if color not in plan]
        if missing:
            raise ValueError('payload plan is missing colours: %s' % ', '.join(missing))
        targets = tuple((color, tuple(plan[color])) for color in TARGET_ORDER)
        used = sorted(slot for _, slots in targets for slot in slots)
        if used != list(range(5)):
            raise ValueError('payload plan must use each slot 0..4 once, got %s' % used)
        return targets

    def target_for_slot(self, slot):
        for color, slots in self.targets:
            if slot in slots:
                return color
        return None

    def record_release(self, slot, when):
        if isinstance(slot, bool) or not isinstance(slot, int) or slot not in range(5):
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
        # The car alone carries all five payloads; no drone task is sent.
        lines.append('小车投放记录')
        for index, (color, slots) in enumerate(self.targets, 1):
            for item, slot in enumerate(slots, 1):
                lines.append('目标点%d（%s）｜第 %d 份（槽位 %d）｜舵机动作时刻：%s' % (
                    index, COLOR_NAMES[color], item, slot, self.actions.get(slot, '未投放')))
        return '\n'.join(lines) + '\n'
