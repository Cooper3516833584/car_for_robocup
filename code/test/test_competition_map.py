from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.competition_map import CompetitionMapSpec, build_competition_navigation_grid


class CompetitionMapTests(unittest.TestCase):
    def test_dimensions_origin_resolution_raster_and_inflation(self) -> None:
        grid = build_competition_navigation_grid(
            CompetitionMapSpec(
                width_m=8.0,
                height_m=8.0,
                resolution_m=0.1,
                origin_x_m=-4.0,
                origin_y_m=-4.0,
                static_obstacles=((-0.2, -0.2, 0.2, 0.2),),
            ),
            robot_radius_m=0.25,
            safety_margin_m=0.10,
        )
        self.assertEqual((grid.width, grid.height), (80, 80))
        self.assertEqual(grid.bounds, (-4.0, -4.0, 4.0, 4.0))
        self.assertEqual(grid.resolution_m, 0.1)
        obstacle = grid.world_to_cell(0.0, 0.0)
        near_obstacle = grid.world_to_cell(0.3, 0.0)
        free = grid.world_to_cell(2.0, 2.0)
        self.assertIn(obstacle, grid.blocked_cells)
        self.assertIn(near_obstacle, grid.blocked_cells)
        self.assertNotIn(free, grid.blocked_cells)

    def test_boundaries_are_blocked_and_legal_interior_endpoints_remain_free(self) -> None:
        grid = build_competition_navigation_grid(
            CompetitionMapSpec(6.0, 6.0, 0.1, -3.0, -3.0),
            robot_radius_m=0.20,
            safety_margin_m=0.05,
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
            ),
            robot_radius_m=0.0,
            safety_margin_m=0.0,
        )
        self.assertIn(grid.world_to_cell(1.5, 0.0), grid.blocked_cells)
        self.assertNotIn(grid.world_to_cell(0.0, 0.0), grid.blocked_cells)


if __name__ == "__main__":
    unittest.main()
