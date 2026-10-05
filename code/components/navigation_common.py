"""Small, SI-only map and path utilities for differential navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


class PathPlanningError(RuntimeError):
    """No safe path exists in the provided occupancy grid."""


@dataclass(frozen=True, slots=True)
class NavigationGoal:
    x_m: float
    y_m: float
    yaw_rad: float | None = None

    def __post_init__(self) -> None:
        values = (self.x_m, self.y_m)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("goal position must be finite")
        if self.yaw_rad is not None and not math.isfinite(float(self.yaw_rad)):
            raise ValueError("goal yaw must be finite")


class NavigationGrid:
    """2-D occupancy grid where unknown/out-of-bounds cells are occupied."""

    def __init__(
        self,
        width: int,
        height: int,
        resolution_m: float,
        *,
        origin_x_m: float = 0.0,
        origin_y_m: float = 0.0,
        occupied_cells: Iterable[tuple[int, int]] = (),
        unknown_cells: Iterable[tuple[int, int]] = (),
    ) -> None:
        if width <= 0 or height <= 0:
            raise ValueError("grid width and height must be positive")
        values = (resolution_m, origin_x_m, origin_y_m)
        if not all(math.isfinite(float(value)) for value in values) or resolution_m <= 0.0:
            raise ValueError("grid resolution/origin must be finite and resolution positive")
        self.width = int(width)
        self.height = int(height)
        self.resolution_m = float(resolution_m)
        self.origin_x_m = float(origin_x_m)
        self.origin_y_m = float(origin_y_m)
        self.blocked_cells = set(occupied_cells) | set(unknown_cells)
        self.revision = 0

    @classmethod
    def from_rows(
        cls,
        rows: Sequence[Sequence[int | None]],
        resolution_m: float,
        *,
        origin_x_m: float = 0.0,
        origin_y_m: float = 0.0,
        occupied_threshold: int = 50,
    ) -> "NavigationGrid":
        if not rows or not rows[0]:
            raise ValueError("grid rows must not be empty")
        width = len(rows[0])
        if any(len(row) != width for row in rows):
            raise ValueError("grid rows must all have the same width")
        occupied: set[tuple[int, int]] = set()
        unknown: set[tuple[int, int]] = set()
        for y, row in enumerate(rows):
            for x, value in enumerate(row):
                if value is None or value < 0:
                    unknown.add((x, y))
                elif value >= occupied_threshold:
                    occupied.add((x, y))
        return cls(
            width,
            len(rows),
            resolution_m,
            origin_x_m=origin_x_m,
            origin_y_m=origin_y_m,
            occupied_cells=occupied,
            unknown_cells=unknown,
        )

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x_m,
            self.origin_y_m,
            self.origin_x_m + self.width * self.resolution_m,
            self.origin_y_m + self.height * self.resolution_m,
        )

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        x = math.floor((x_m - self.origin_x_m) / self.resolution_m)
        y = math.floor((y_m - self.origin_y_m) / self.resolution_m)
        if 0 <= x < self.width and 0 <= y < self.height:
            return (x, y)
        return None

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        x, y = cell
        return (
            self.origin_x_m + (x + 0.5) * self.resolution_m,
            self.origin_y_m + (y + 0.5) * self.resolution_m,
        )

    def set_blocked(self, cell: tuple[int, int], blocked: bool = True) -> None:
        if not (0 <= cell[0] < self.width and 0 <= cell[1] < self.height):
            raise ValueError("cell is outside the grid")
        if blocked:
            self.blocked_cells.add(cell)
        else:
            self.blocked_cells.discard(cell)
        self.revision += 1
