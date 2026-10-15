"""Wire contract for a task-board allocation; identical on car and drone."""
from dataclasses import dataclass
import struct

# Version 2 assigns the drone exactly one optional item, plus implicit yellow.
TASK_VERSION = 2
_TASK = struct.Struct("<BI3B")
_START = struct.Struct("<BI")


@dataclass(frozen=True)
class DropTask:
    task_id: int
    red: int
    blue: int
    green: int

    def __post_init__(self):
        if isinstance(self.task_id, bool) or not isinstance(self.task_id, int) or not 1 <= self.task_id <= 0xffffffff:
            raise ValueError("task_id must be 1..0xffffffff")
        counts = (self.red, self.blue, self.green)
        if any(isinstance(n, bool) or not isinstance(n, int) or not 0 <= n <= 1 for n in counts):
            raise ValueError("counts must be integers in 0..1")
        if sum(counts) != 1:
            raise ValueError("red/blue/green counts must total 1")

    def allocation(self):
        return dict(red=self.red, blue=self.blue, green=self.green)


def encode_drop_task(task):
    DropTask(task.task_id, task.red, task.blue, task.green)
    return _TASK.pack(TASK_VERSION, task.task_id, task.red, task.blue, task.green)


def decode_drop_task(data):
    if len(data) != _TASK.size:
        raise ValueError("invalid drop-task length")
    version, task_id, red, blue, green = _TASK.unpack(data)
    if version != TASK_VERSION:
        raise ValueError("unsupported drop-task version")
    return DropTask(task_id, red, blue, green)


def encode_drop_start(task_id):
    # Reuse task-id validation without accepting bools or overflowing integers.
    DropTask(task_id, 1, 0, 0)
    return _START.pack(TASK_VERSION, task_id)


def decode_drop_start(data):
    if len(data) != _START.size:
        raise ValueError("invalid drop-start length")
    version, task_id = _START.unpack(data)
    if version != TASK_VERSION:
        raise ValueError("unsupported drop-start version")
    DropTask(task_id, 1, 0, 0)
    return task_id
