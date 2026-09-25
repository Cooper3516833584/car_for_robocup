"""Rasterize configured competition geometry into a safe navigation grid."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .navigation_common import NavigationGrid


@dataclass(frozen=True, slots=True)
class CompetitionMapSpec:
    width_m: float
    height_m: float
    resolution_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    static_obstacles: tuple[tuple[float, float, float, float], ...] = ()
    allowed_regions: tuple[tuple[float, float, float, float], ...] = ()


def build_competition_navigation_grid(
    spec: CompetitionMapSpec,
    robot_radius_m: float,
    safety_margin_m: float,
) -> NavigationGrid:
    """Create a grid with obstacle/allowed-region occupancy and conservative inflation.

    Rectangles are ``(x_min, y_min, x_max, y_max)`` in map metres. The outer
    cell ring is blocked so planned robot centers cannot travel along the map edge.
    """

    positive = (spec.width_m, spec.height_m, spec.resolution_m)
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("map dimensions and resolution must be finite and positive")
    if not all(math.isfinite(value) for value in (spec.origin_x_m, spec.origin_y_m)):
        raise ValueError("map origin must be finite")
    if not math.isfinite(robot_radius_m) or robot_radius_m < 0.0:
        raise ValueError("robot radius must be finite and non-negative")
    if not math.isfinite(safety_margin_m) or safety_margin_m < 0.0:
        raise ValueError("safety margin must be finite and non-negative")
    width_f = spec.width_m / spec.resolution_m
    height_f = spec.height_m / spec.resolution_m
    width, height = round(width_f), round(height_f)
    if not math.isclose(width_f, width, abs_tol=1e-9) or not math.isclose(height_f, height, abs_tol=1e-9):
        raise ValueError("map dimensions must be exact multiples of resolution")
    obstacles = tuple(_validate_rect(rect) for rect in spec.static_obstacles)
    allowed = tuple(_validate_rect(rect) for rect in spec.allowed_regions)
    raw = NavigationGrid(
        width,
        height,
        spec.resolution_m,
        origin_x_m=spec.origin_x_m,
        origin_y_m=spec.origin_y_m,
    )
    min_x, min_y, max_x, max_y = raw.bounds
    raw_blocked: set[tuple[int, int]] = set()
    for y in range(height):
        for x in range(width):
            cell = (x, y)
            wx, wy = raw.cell_to_world(cell)
            on_boundary = x == 0 or y == 0 or x == width - 1 or y == height - 1
            outside_allowed = bool(allowed) and not any(_contains(rect, wx, wy) for rect in allowed)
            inside_obstacle = any(_contains(rect, wx, wy) for rect in obstacles)
            if on_boundary or outside_allowed or inside_obstacle:
                raw_blocked.add(cell)

    inflate = robot_radius_m + safety_margin_m
    blocked = set(raw_blocked)
    if inflate:
        cells = math.ceil(inflate / spec.resolution_m)
        for ox, oy in raw_blocked:
            cx, cy = raw.cell_to_world((ox, oy))
            for y in range(max(0, oy - cells), min(height, oy + cells + 1)):
                for x in range(max(0, ox - cells), min(width, ox + cells + 1)):
                    wx, wy = raw.cell_to_world((x, y))
                    if math.hypot(wx - cx, wy - cy) <= inflate + spec.resolution_m * math.sqrt(2.0) / 2.0:
                        blocked.add((x, y))
    return NavigationGrid(
        width,
        height,
        spec.resolution_m,
        origin_x_m=spec.origin_x_m,
        origin_y_m=spec.origin_y_m,
        occupied_cells=blocked,
    )


def _validate_rect(rect: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    if len(rect) != 4:
        raise ValueError("map rectangles must contain x_min, y_min, x_max, y_max")
    values = tuple(float(value) for value in rect)
    if not all(math.isfinite(value) for value in values) or values[2] <= values[0] or values[3] <= values[1]:
        raise ValueError("map rectangle must be finite with positive area")
    return values


def _contains(rect: tuple[float, float, float, float], x: float, y: float) -> bool:
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]
