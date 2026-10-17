"""Per-run operator settings shared by the radio owner and VS Code launchers."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CompetitionOptions:
    wall_right_distance_m: tuple = (1.530, 1.825, 1.620, 2.260)
    standard_drop_m: float = .43
    right_mount_drop_m: float = .32
    finish_front_m: float = 1.10
    task_counts: tuple | None = None

    def __post_init__(self):
        object.__setattr__(self, 'wall_right_distance_m', tuple(self.wall_right_distance_m))
        if len(self.wall_right_distance_m) != 4:
            raise ValueError('four right-wall distances are required: left, bottom, right, top')
        distances = (*self.wall_right_distance_m, self.standard_drop_m,
                     self.right_mount_drop_m, self.finish_front_m)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or v <= 0 for v in distances):
            raise ValueError('mission distances must be positive finite metres')
        if self.task_counts is not None:
            counts = tuple(self.task_counts)
            if len(counts) != 3 or any(type(v) is not int for v in counts) or sorted(counts) != [1, 1, 2]:
                raise ValueError('task counts must be red/blue/green with one double and two singles')
            object.__setattr__(self, 'task_counts', counts)

    def as_dict(self):
        from dataclasses import asdict
        return asdict(self)
