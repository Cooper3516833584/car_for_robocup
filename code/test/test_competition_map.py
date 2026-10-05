from __future__ import annotations

from pathlib import Path
import sys
import unittest
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.competition_map import CompetitionMapSpec, build_competition_navigation_grid
from components.navigation_common import PathPlanningError
from config.v2_factory import build_competition_world, build_differential_navigator
from config.v2_loader import load_v2_config


class CompetitionMapTests(unittest.TestCase):
    def test_dimensions_origin_resolution_and_physical_obstacle_raster(self) -> None:
        grid = build_competition_navigation_grid(
            CompetitionMapSpec(
                width_m=8.0,
                height_m=8.0,
                resolution_m=0.1,
                origin_x_m=-4.0,
                origin_y_m=-4.0,
                static_obstacles=((-0.2, -0.2, 0.2, 0.2),),
            )
        )
        self.assertEqual((grid.width, grid.height), (80, 80))
        self.assertEqual(grid.bounds, (-4.0, -4.0, 4.0, 4.0))
        self.assertEqual(grid.resolution_m, 0.1)
        obstacle = grid.world_to_cell(0.0, 0.0)
        near_obstacle = grid.world_to_cell(0.45, 0.0)
        free = grid.world_to_cell(2.0, 2.0)
        self.assertIn(obstacle, grid.blocked_cells)
        self.assertNotIn(near_obstacle, grid.blocked_cells)
        self.assertNotIn(free, grid.blocked_cells)

    def test_boundaries_are_blocked_and_legal_interior_endpoints_remain_free(self) -> None:
        grid = build_competition_navigation_grid(
            CompetitionMapSpec(6.0, 6.0, 0.1, -3.0, -3.0)
        )
        boundary = grid.world_to_cell(-2.95, 0.0)
        start = grid.world_to_cell(-2.0, 0.0)
        goal = grid.world_to_cell(2.0, 0.0)
        self.assertIn(boundary, grid.blocked_cells)
        self.assertIsNotNone(start)
        self.assertIsNotNone(goal)
        self.assertNotIn(start, grid.blocked_cells)
        self.assertNotIn(goal, grid.blocked_cells)

    def test_allowed_region_blocks_cells_outside_it(self) -> None:
        grid = build_competition_navigation_grid(
            CompetitionMapSpec(
                4.0,
                4.0,
                0.1,
                -2.0,
                -2.0,
                allowed_regions=((-1.0, -1.0, 1.0, 1.0),),
            )
        )
        self.assertIn(grid.world_to_cell(1.5, 0.0), grid.blocked_cells)
        self.assertNotIn(grid.world_to_cell(0.0, 0.0), grid.blocked_cells)

    def test_configured_world_and_planner_allow_only_physically_wide_corridor(self) -> None:
        source = load_v2_config()
        geometry = replace(
            source.geometry,
            body_length_m=0.4,
            body_width_m=0.3,
            drive_axle_to_body_center_x_m=0.0,
        )
        legal_config = replace(
            source,
            geometry=geometry,
            competition_map=replace(
                source.competition_map,
                width_m=10.0,
                height_m=6.0,
                resolution_m=0.05,
                static_obstacles=((0.0, 0.0, 10.0, 2.0), (0.0, 3.0, 10.0, 6.0)),
            ),
        )
        legal_grid = build_competition_world(legal_config)
        navigator = build_differential_navigator(legal_config)
        path = navigator.planner.plan((0.8, 2.5), (9.0, 2.5), legal_grid)
        self.assertGreaterEqual(len(path), 2)

        narrow_config = replace(
            legal_config,
            competition_map=replace(
                legal_config.competition_map,
                static_obstacles=((0.0, 0.0, 10.0, 2.0), (0.0, 2.5, 10.0, 6.0)),
            ),
        )
        narrow_grid = build_competition_world(narrow_config)
        narrow_navigator = build_differential_navigator(narrow_config)
        with self.assertRaises(PathPlanningError):
            narrow_navigator.planner.plan((0.8, 2.25), (9.0, 2.25), narrow_grid)


if __name__ == "__main__":
    unittest.main()
