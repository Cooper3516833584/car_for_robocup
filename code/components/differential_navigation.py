"""Grid planning and pose-based navigation for the differential platform."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import heapq
import itertools
import math

from config.v2_models import DifferentialGeometryConfig, DifferentialDriveConfig, NavigationConfig
from core.frames import normalize_angle_rad
from core.types import Pose2D, Twist2D

from .navigation_common import NavigationGoal, NavigationGrid, PathPlanningError


class NavigationState(Enum):
    IDLE = "idle"
    ROTATING_TO_PATH = "rotating_to_path"
    TRACKING = "tracking"
    FINAL_ALIGN = "final_align"
    GOAL_REACHED = "goal_reached"
    BLOCKED = "blocked"
    POSE_LOST = "pose_lost"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class NavigationOutput:
    command: Twist2D
    state: NavigationState
    path: tuple[tuple[float, float], ...]
    diagnostics: dict[str, float | str | bool]


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    command: Twist2D
    state: NavigationState
    diagnostics: dict[str, float | str | bool]


class DifferentialPathPlanner:
    """8-connected A* with circular footprint inflation and LOS smoothing."""

    def __init__(
        self,
        geometry: DifferentialGeometryConfig,
        *,
        safety_margin_m: float,
    ) -> None:
        self.geometry = geometry
        self.safety_margin_m = float(safety_margin_m)
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0.0:
            raise ValueError("safety_margin_m must be finite and non-negative")
        front = geometry.drive_axle_to_body_center_x_m + geometry.body_length_m / 2.0
        rear = geometry.body_length_m / 2.0 - geometry.drive_axle_to_body_center_x_m
        half_width = geometry.body_width_m / 2.0
        if min(front, rear, half_width) <= 0.0:
            raise ValueError("body extents around base_link must all be positive")
        self.front_extent_m = front
        self.rear_extent_m = rear
        self.left_extent_m = half_width
        self.right_extent_m = half_width
        self.clearance_radius_m = math.hypot(max(front, rear), half_width) + self.safety_margin_m

    def plan(
        self,
        start_xy_m: tuple[float, float],
        goal_xy_m: tuple[float, float],
        grid: NavigationGrid,
    ) -> tuple[tuple[float, float], ...]:
        start_cell = grid.world_to_cell(*start_xy_m)
        goal_cell = grid.world_to_cell(*goal_xy_m)
        if start_cell is None:
            raise PathPlanningError("start pose is outside the known map")
        if goal_cell is None:
            raise PathPlanningError("goal is outside the known map")
        blocked = self._inflate_obstacles(grid)
        if start_cell in blocked:
            raise PathPlanningError("start footprint intersects an obstacle or map boundary")
        if goal_cell in blocked:
            raise PathPlanningError("goal footprint intersects an obstacle or map boundary")
        if start_cell == goal_cell:
            if not self._line_is_free(start_xy_m, goal_xy_m, grid, blocked):
                raise PathPlanningError("no collision-free path inside the start cell")
            return (start_xy_m, goal_xy_m)

        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        sequence = itertools.count()
        heapq.heappush(open_heap, (self._heuristic(start_cell, goal_cell), next(sequence), start_cell))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        cost_so_far = {start_cell: 0.0}
        neighbors = (
            (-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
            (-1, 0, 1.0), (1, 0, 1.0),
            (-1, 1, math.sqrt(2.0)), (0, 1, 1.0), (1, 1, math.sqrt(2.0)),
        )
        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current == goal_cell:
                break
            for dx, dy, move_cost in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not (0 <= nxt[0] < grid.width and 0 <= nxt[1] < grid.height):
                    continue
                if nxt in blocked:
                    continue
                if dx and dy and (
                    (current[0] + dx, current[1]) in blocked
                    or (current[0], current[1] + dy) in blocked
                ):
                    continue
                new_cost = cost_so_far[current] + move_cost
                if new_cost >= cost_so_far.get(nxt, math.inf):
                    continue
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current
                priority = new_cost + self._heuristic(nxt, goal_cell)
                heapq.heappush(open_heap, (priority, next(sequence), nxt))
        else:
            raise PathPlanningError("no collision-free path")

        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(came_from[cells[-1]])
        cells.reverse()
        raw = [start_xy_m]
        raw.extend(grid.cell_to_world(cell) for cell in cells[1:-1])
        raw.append(goal_xy_m)
        return self._shortcut(raw, grid, blocked)

    def _inflate_obstacles(self, grid: NavigationGrid) -> set[tuple[int, int]]:
        blocked = set()
        radius = self.clearance_radius_m + grid.resolution_m * math.sqrt(2.0) / 2.0
        cells = math.ceil(radius / grid.resolution_m)
        for obstacle_x, obstacle_y in grid.blocked_cells:
            if not (0 <= obstacle_x < grid.width and 0 <= obstacle_y < grid.height):
                continue
            ox, oy = grid.cell_to_world((obstacle_x, obstacle_y))
            for y in range(max(0, obstacle_y - cells), min(grid.height, obstacle_y + cells + 1)):
                for x in range(max(0, obstacle_x - cells), min(grid.width, obstacle_x + cells + 1)):
                    px, py = grid.cell_to_world((x, y))
                    if math.hypot(px - ox, py - oy) <= radius:
                        blocked.add((x, y))

        min_x, min_y, max_x, max_y = grid.bounds
        for y in range(grid.height):
            for x in range(grid.width):
                px, py = grid.cell_to_world((x, y))
                if (
                    px - min_x < self.clearance_radius_m
                    or max_x - px < self.clearance_radius_m
                    or py - min_y < self.clearance_radius_m
                    or max_y - py < self.clearance_radius_m
                ):
                    blocked.add((x, y))
        return blocked

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    @staticmethod
    def _line_is_free(
        a: tuple[float, float],
        b: tuple[float, float],
        grid: NavigationGrid,
        blocked: set[tuple[int, int]],
    ) -> bool:
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        samples = max(1, math.ceil(distance / (grid.resolution_m * 0.35)))
        for index in range(samples + 1):
            fraction = index / samples
            point = (a[0] + (b[0] - a[0]) * fraction, a[1] + (b[1] - a[1]) * fraction)
            cell = grid.world_to_cell(*point)
            if cell is None or cell in blocked:
                return False
        return True

    def _shortcut(
        self,
        path: list[tuple[float, float]],
        grid: NavigationGrid,
        blocked: set[tuple[int, int]],
    ) -> tuple[tuple[float, float], ...]:
        if len(path) <= 2:
            return tuple(path)
        output = [path[0]]
        index = 0
        while index < len(path) - 1:
            next_index = len(path) - 1
            while next_index > index + 1 and not self._line_is_free(
                path[index], path[next_index], grid, blocked
            ):
                next_index -= 1
            output.append(path[next_index])
            index = next_index
        return tuple(output)


class DifferentialPathController:
    """Pure pursuit/rotate-first controller returning only body ``Twist2D``."""

    def __init__(
        self,
        navigation: NavigationConfig,
        drive: DifferentialDriveConfig,
    ) -> None:
        self.navigation = navigation
        self.drive = drive
        self._path_index = 0

    def reset_path(self) -> None:
        self._path_index = 0

    def compute(
        self,
        pose: Pose2D,
        path: tuple[tuple[float, float], ...],
        goal: NavigationGoal,
        *,
        protocol_compat: bool = False,
    ) -> ControllerOutput:
        dx_goal, dy_goal = goal.x_m - pose.x_m, goal.y_m - pose.y_m
        distance_goal = math.hypot(dx_goal, dy_goal)
        if distance_goal <= self.navigation.position_tolerance_m:
            if goal.yaw_rad is None:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.GOAL_REACHED, {"goal_distance_m": distance_goal})
            yaw_error = normalize_angle_rad(goal.yaw_rad - pose.yaw_rad)
            if abs(yaw_error) <= self.navigation.yaw_tolerance_rad:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.GOAL_REACHED, {"goal_distance_m": distance_goal, "yaw_error_rad": yaw_error})
            if not self.drive.allow_in_place_rotation:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "final_yaw_requires_in_place_rotation", "yaw_error_rad": yaw_error})
            omega = self._clamp(
                self.navigation.final_yaw_gain * yaw_error,
                self.drive.max_angular_speed_rad_s,
            )
            return ControllerOutput(Twist2D(0.0, omega), NavigationState.FINAL_ALIGN, {"goal_distance_m": distance_goal, "yaw_error_rad": yaw_error})

        target = self._lookahead_point(pose, path)
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle_rad(heading - pose.yaw_rad)
        if protocol_compat:
            if abs(heading_error) > self.navigation.yaw_tolerance_rad:
                if not self.drive.allow_in_place_rotation:
                    return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "compatibility_path_requires_in_place_rotation", "heading_error_rad": heading_error})
                omega = self._clamp(self.navigation.path_yaw_gain * heading_error, self.drive.max_angular_speed_rad_s)
                return ControllerOutput(Twist2D(0.0, omega), NavigationState.ROTATING_TO_PATH, {"heading_error_rad": heading_error, "compatibility_rotate_first": True})
            speed = self._forward_speed(distance_goal, heading_error)
            return ControllerOutput(Twist2D(speed, 0.0), NavigationState.TRACKING, {"heading_error_rad": heading_error, "compatibility_rotate_first": True})

        if abs(heading_error) > self.navigation.rotate_in_place_threshold_rad:
            if not self.drive.allow_in_place_rotation:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "path_heading_requires_in_place_rotation", "heading_error_rad": heading_error})
            omega = self._clamp(self.navigation.path_yaw_gain * heading_error, self.drive.max_angular_speed_rad_s)
            return ControllerOutput(Twist2D(0.0, omega), NavigationState.ROTATING_TO_PATH, {"heading_error_rad": heading_error})

        local_x = math.cos(pose.yaw_rad) * dx + math.sin(pose.yaw_rad) * dy
        local_y = -math.sin(pose.yaw_rad) * dx + math.cos(pose.yaw_rad) * dy
        lookahead_sq = max(dx * dx + dy * dy, 1e-9)
        curvature = 2.0 * local_y / lookahead_sq
        speed = self._forward_speed(distance_goal, heading_error)
        omega = self._clamp(speed * curvature, self.drive.max_angular_speed_rad_s)
        return ControllerOutput(
            Twist2D(speed, omega),
            NavigationState.TRACKING,
            {"heading_error_rad": heading_error, "curvature_inv_m": curvature, "goal_distance_m": distance_goal},
        )

    def _forward_speed(self, distance_goal: float, heading_error: float) -> float:
        speed_scale = max(0.25, math.cos(min(abs(heading_error), math.pi / 2.0)))
        if self.navigation.slowdown_distance_m > 0.0:
            speed_scale *= min(1.0, distance_goal / self.navigation.slowdown_distance_m)
        return min(self.drive.max_linear_speed_m_s, self.drive.max_linear_speed_m_s * speed_scale)

    def _lookahead_point(
        self, pose: Pose2D, path: tuple[tuple[float, float], ...]
    ) -> tuple[float, float]:
        if not path:
            raise PathPlanningError("path is empty")
        nearest = min(
            range(self._path_index, len(path)),
            key=lambda index: (path[index][0] - pose.x_m) ** 2 + (path[index][1] - pose.y_m) ** 2,
        )
        self._path_index = nearest
        remaining = self.navigation.lookahead_m
        current = (pose.x_m, pose.y_m)
        for point in path[nearest + 1 :]:
            segment = math.hypot(point[0] - current[0], point[1] - current[1])
            if segment >= remaining and segment > 1e-12:
                fraction = remaining / segment
                return (current[0] + (point[0] - current[0]) * fraction, current[1] + (point[1] - current[1]) * fraction)
            remaining -= segment
            current = point
        return path[-1]

    @staticmethod
    def _clamp(value: float, maximum: float) -> float:
        return max(-maximum, min(maximum, value))


class DifferentialNavigator:
    """Planner/controller state machine; motor I/O stays in runtime."""

    def __init__(
        self,
        geometry: DifferentialGeometryConfig,
        drive: DifferentialDriveConfig,
        navigation: NavigationConfig,
    ) -> None:
        self.geometry = geometry
        self.drive = drive
        self.navigation = navigation
        self.planner = DifferentialPathPlanner(geometry, safety_margin_m=navigation.safety_margin_m)
        self.controller = DifferentialPathController(navigation, drive)
        self.goal: NavigationGoal | None = None
        self._path: tuple[tuple[float, float], ...] = ()
        self._grid_identity: int | None = None
        self._grid_revision: int | None = None
        self._state = NavigationState.IDLE

    def set_goal(self, goal: NavigationGoal) -> None:
        self.goal = goal
        self._path = ()
        self._grid_identity = None
        self._grid_revision = None
        self.controller.reset_path()
        self._state = NavigationState.IDLE

    def clear_goal(self) -> None:
        self.goal = None
        self._path = ()
        self._grid_identity = None
        self._grid_revision = None
        self.controller.reset_path()
        self._state = NavigationState.IDLE

    def step(
        self,
        pose: Pose2D | None,
        grid: NavigationGrid | None,
        *,
        now_s: float,
        pose_state: object = "ok",
    ) -> NavigationOutput:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        state_name = getattr(pose_state, "value", pose_state)
        if pose is None or state_name not in {"ok", "t265_degraded", "d500_degraded"}:
            self._state = NavigationState.POSE_LOST
            return self._output(Twist2D(0.0, 0.0), {"pose_state": str(state_name)})
        if not all(math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)):
            self._state = NavigationState.POSE_LOST
            return self._output(Twist2D(0.0, 0.0), {"reason": "non_finite_pose"})
        if self.goal is None:
            self._state = NavigationState.IDLE
            return self._output(Twist2D(0.0, 0.0), {})
        if grid is None:
            self._state = NavigationState.BLOCKED
            return self._output(Twist2D(0.0, 0.0), {"reason": "map_unavailable"})

        if (
            not self._path
            or self._grid_identity != id(grid)
            or self._grid_revision != grid.revision
        ):
            try:
                self._path = self.planner.plan(
                    (pose.x_m, pose.y_m), (self.goal.x_m, self.goal.y_m), grid
                )
            except PathPlanningError as exc:
                self._path = ()
                self._state = NavigationState.BLOCKED
                return self._output(Twist2D(0.0, 0.0), {"reason": str(exc)})
            self._grid_identity = id(grid)
            self._grid_revision = grid.revision
            self.controller.reset_path()

        result = self.controller.compute(
            pose,
            self._path,
            self.goal,
            protocol_compat=self.drive.protocol_mode == "ackermann_firmware_compat",
        )
        command = result.command
        if state_name in {"d500_degraded", "t265_degraded"}:
            command = Twist2D(
                command.linear_x_m_s * self.navigation.degraded_speed_scale,
                command.angular_z_rad_s * self.navigation.degraded_speed_scale,
            )
        self._state = result.state
        return self._output(command, result.diagnostics)

    def _output(self, command: Twist2D, diagnostics: dict[str, float | str | bool]) -> NavigationOutput:
        return NavigationOutput(command, self._state, self._path, diagnostics)
