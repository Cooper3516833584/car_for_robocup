"""Wire contract for a task-board allocation; identical on car and drone."""
from dataclasses import dataclass
import struct

# Version 3 assigns no optional RGB items; one mandatory yellow is implicit.
TASK_VERSION = 3
_TASK = struct.Struct("<BI3B")
_START = struct.Struct("<BI")


def _validate_task_id(task_id):
    if isinstance(task_id, bool) or not isinstance(task_id, int) or not 1 <= task_id <= 0xffffffff:
        raise ValueError("task_id must be 1..0xffffffff")
    return task_id


@dataclass(frozen=True)
class DropTask:
    task_id: int
    red: int
    blue: int
    green: int

    def __post_init__(self):
        _validate_task_id(self.task_id)
        counts = (self.red, self.blue, self.green)
        if any(isinstance(n, bool) or not isinstance(n, int) or n != 0 for n in counts):
            raise ValueError("drone optional red/blue/green quota must be all zero")

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
    _validate_task_id(task_id)
    return _START.pack(TASK_VERSION, task_id)


def decode_drop_start(data):
    if len(data) != _START.size:
        raise ValueError("invalid drop-start length")
    version, task_id = _START.unpack(data)
    if version != TASK_VERSION:
        raise ValueError("unsupported drop-start version")
    _validate_task_id(task_id)
    return task_id
