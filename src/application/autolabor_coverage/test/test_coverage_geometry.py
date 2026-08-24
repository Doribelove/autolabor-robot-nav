#!/usr/bin/env python3

from pathlib import Path
import math
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# Keep a sourced catkin devel package ahead of the source helper so generated
# ``autolabor_coverage.msg`` modules remain importable when tests share a
# process.  In a standalone geometry run the appended source path is still
# sufficient.
sys.path.append(str(PACKAGE_ROOT / "src"))

from autolabor_coverage.coverage_geometry import (  # noqa: E402
    CoveragePlanner,
    GridMap,
    Point,
    Swath,
    order_swaths,
    rasterize_swept_cells,
    validate_polygon,
)


class CoverageGeometryTest(unittest.TestCase):
    @staticmethod
    def grid_with_obstacle(obstacle=None):
        width = 120
        height = 100
        data = [0] * (width * height)
        if obstacle:
            x0, y0, x1, y1, value = obstacle
            for y in range(y0, y1):
                for x in range(x0, x1):
                    data[y * width + x] = value
        return GridMap(width, height, 0.1, 0.0, 0.0, data)

    @staticmethod
    def rectangle():
        return [Point(2.0, 2.0), Point(9.0, 2.0),
                Point(9.0, 7.0), Point(2.0, 7.0)]

    def test_free_rectangle_generates_footprint_safe_swaths(self):
        planner = CoveragePlanner(self.grid_with_obstacle())
        plan = planner.plan(self.rectangle(), 0.70, 0.15)
        self.assertGreater(len(plan.swaths), 2)
        self.assertAlmostEqual(0.595, plan.spacing, places=3)
        self.assertGreater(plan.reachable_area, 0.0)
        self.assertLessEqual(plan.reachable_area, plan.requested_area)
        for swath in plan.swaths:
            self.assertGreaterEqual(swath.length, 1.2)
            self.assertTrue(math.isfinite(swath.start.x))
            self.assertTrue(math.isfinite(swath.end.y))

    def test_one_metre_cleaning_width_uses_point_eight_five_lane_spacing(self):
        plan = CoveragePlanner(self.grid_with_obstacle()).plan(
            self.rectangle(), 1.00, 0.15
        )
        self.assertAlmostEqual(0.85, plan.spacing, places=6)

    def test_occupied_band_splits_or_clips_the_cleaning_lines(self):
        free = CoveragePlanner(self.grid_with_obstacle()).plan(
            self.rectangle(), 0.70, 0.15)
        blocked_grid = self.grid_with_obstacle((53, 10, 67, 85, 100))
        blocked = CoveragePlanner(blocked_grid).plan(
            self.rectangle(), 0.70, 0.15)
        self.assertGreater(blocked.unreachable_area, free.unreachable_area)
        for swath in blocked.swaths:
            for point in (swath.start, swath.end):
                self.assertFalse(5.3 <= point.x < 6.7 and 1.0 <= point.y < 8.5)

    def test_unknown_cells_are_not_treated_as_cleanable(self):
        grid = self.grid_with_obstacle((45, 35, 75, 55, -1))
        plan = CoveragePlanner(grid).plan(self.rectangle(), 0.70, 0.15)
        self.assertGreater(plan.unreachable_area, 0.0)

    def test_disconnected_free_map_is_clipped_from_vehicle_component(self):
        grid = self.grid_with_obstacle((58, 0, 62, 100, 100))
        plan = CoveragePlanner(grid).plan(
            self.rectangle(), 0.70, 0.15, reachable_seed=Point(3.0, 3.0))
        self.assertGreater(plan.unreachable_area, 0.0)
        self.assertTrue(plan.swaths)
        for swath in plan.swaths:
            self.assertLess(swath.start.x, 5.8)
            self.assertLess(swath.end.x, 5.8)

    def test_self_intersection_and_duplicate_edges_are_rejected(self):
        valid, reason = validate_polygon(
            [Point(1, 1), Point(4, 4), Point(1, 4), Point(4, 1)])
        self.assertFalse(valid)
        self.assertIn("self-intersecting", reason)
        valid, reason = validate_polygon(
            [Point(1, 1), Point(1, 1), Point(4, 4)])
        self.assertFalse(valid)
        self.assertIn("duplicate", reason)

    def test_route_chooses_the_nearest_orientation_for_each_selected_swath(self):
        plan = CoveragePlanner(self.grid_with_obstacle()).plan(
            self.rectangle(), 0.70, 0.15)
        current = Point(plan.swaths[0].end.x, plan.swaths[0].end.y)
        route = order_swaths(plan.swaths, current, plan.spacing, 1.20)
        first = route[0]
        self.assertLessEqual(
            math.hypot(current.x - first.start.x, current.y - first.start.y),
            math.hypot(current.x - first.end.x, current.y - first.end.y),
        )

    def test_route_entry_uses_vehicle_heading_when_distances_are_equal(self):
        swath = Swath(Point(1.0, 0.0), Point(-1.0, 0.0), 0.0, 2.0)
        route = order_swaths(
            [swath], Point(0.0, 0.0), 0.85, 1.35, current_yaw=0.0
        )
        self.assertEqual(Point(-1.0, 0.0), route[0].start)
        self.assertEqual(Point(1.0, 0.0), route[0].end)

    def test_swept_area_cells_are_clipped_and_do_not_double_count(self):
        grid = GridMap(50, 50, 0.1, 0.0, 0.0, [0] * 2500)
        polygon = [Point(1.0, 1.0), Point(4.0, 1.0),
                   Point(4.0, 4.0), Point(1.0, 4.0)]
        first = rasterize_swept_cells(
            grid, polygon, Point(1.5, 2.0), Point(3.5, 2.0), 1.0
        )
        repeated = set(first)
        repeated.update(rasterize_swept_cells(
            grid, polygon, Point(1.5, 2.0), Point(3.5, 2.0), 1.0
        ))
        self.assertEqual(first, repeated)
        self.assertGreater(len(first) * grid.resolution ** 2, 2.0)
        for cell_x, cell_y in first:
            self.assertGreaterEqual((cell_x + 0.5) * grid.resolution, 1.0)
            self.assertLessEqual((cell_x + 0.5) * grid.resolution, 4.0)


if __name__ == "__main__":
    unittest.main()
