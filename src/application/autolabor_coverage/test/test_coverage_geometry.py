#!/usr/bin/env python3

from pathlib import Path
import itertools
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
    CoverageTimeParameters,
    GridMap,
    Point,
    Swath,
    _dubins_path_components,
    _optimize_orientations_for_order,
    estimate_route_time,
    occupancy_grid_digest,
    order_swaths,
    rasterize_swept_cells,
    validate_polygon,
)


def _oracle_heading_delta(first, second):
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


def _oracle_connector_cost(current, current_yaw, entry, sweep_yaw,
                           minimum_turning_radius):
    distance = math.hypot(current.x - entry.x, current.y - entry.y)
    if current_yaw is None or not math.isfinite(current_yaw):
        return distance
    turn_arc = minimum_turning_radius * _oracle_heading_delta(
        current_yaw, sweep_yaw)
    approach_deficit = max(
        0.0,
        min(turn_arc, minimum_turning_radius) - distance,
    )
    return distance + turn_arc + 10.0 * approach_deficit


def _oracle_route_cost(swaths, index_order, directions, current, current_yaw,
                       minimum_turning_radius):
    total = 0.0
    cursor = current
    cursor_yaw = current_yaw
    for index, direction in zip(index_order, directions):
        swath = swaths[index]
        if direction == 0:
            entry, exit_point = swath.start, swath.end
        else:
            entry, exit_point = swath.end, swath.start
        sweep_yaw = math.atan2(
            exit_point.y - entry.y,
            exit_point.x - entry.x,
        )
        total += _oracle_connector_cost(
            cursor,
            cursor_yaw,
            entry,
            sweep_yaw,
            minimum_turning_radius,
        ) + swath.length
        cursor = exit_point
        cursor_yaw = sweep_yaw
    return total


def _cyclic_orders(order):
    order = tuple(order)
    for offset in range(len(order)):
        yield order[offset:] + order[:offset]


def _allowed_index_orders(base_order):
    seen = set()
    for cycle in (tuple(base_order), tuple(reversed(base_order))):
        for candidate in _cyclic_orders(cycle):
            if candidate in seen:
                continue
            seen.add(candidate)
            yield candidate


def _oracle_best(swaths, index_orders, current, current_yaw,
                 minimum_turning_radius):
    best = None
    for index_order in index_orders:
        for directions in itertools.product((0, 1), repeat=len(index_order)):
            candidate = (
                _oracle_route_cost(
                    swaths,
                    index_order,
                    directions,
                    current,
                    current_yaw,
                    minimum_turning_radius,
                ),
                tuple(index_order),
                directions,
            )
            if best is None or candidate < best:
                best = candidate
    return best


def _oracle_greedy_for_order(swaths, index_order, current, current_yaw,
                             minimum_turning_radius):
    total = 0.0
    cursor = current
    cursor_yaw = current_yaw
    directions = []
    for index in index_order:
        swath = swaths[index]
        candidates = []
        for direction in (0, 1):
            if direction == 0:
                entry, exit_point = swath.start, swath.end
            else:
                entry, exit_point = swath.end, swath.start
            sweep_yaw = math.atan2(
                exit_point.y - entry.y,
                exit_point.x - entry.x,
            )
            candidates.append((
                _oracle_connector_cost(
                    cursor,
                    cursor_yaw,
                    entry,
                    sweep_yaw,
                    minimum_turning_radius,
                ),
                direction,
                exit_point,
                sweep_yaw,
            ))
        connector_cost, direction, cursor, cursor_yaw = min(candidates)
        total += connector_cost + swath.length
        directions.append(direction)
    return total, tuple(directions)


def _turn_friendly_base_order(swath_count, spacing, minimum_turning_radius):
    stride = max(1, int(math.ceil(
        2.0 * minimum_turning_radius / spacing)))
    return tuple(
        index
        for residue in range(stride)
        for index in range(residue, swath_count, stride)
    )


def _route_signature(route, original_swaths):
    signature = []
    used = set()
    for routed in route:
        matches = []
        for index, original in enumerate(original_swaths):
            if index in used or routed.scan_v != original.scan_v:
                continue
            if routed.start == original.start and routed.end == original.end:
                matches.append((index, 0))
            elif routed.start == original.end and routed.end == original.start:
                matches.append((index, 1))
        if len(matches) != 1:
            raise AssertionError("route does not map uniquely to the input swaths")
        used.add(matches[0][0])
        signature.append(matches[0])
    return tuple(index for index, _ in signature), tuple(
        direction for _, direction in signature)


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

    def test_occupancy_grid_digest_covers_complete_spatial_identity(self):
        arguments = dict(
            frame_id="map",
            width=2,
            height=2,
            resolution=0.05,
            origin_position=(1.0, 2.0, 3.0),
            origin_orientation=(0.0, 0.0, 0.0, 1.0),
            data=(0, -1, 100, 0),
        )
        baseline = occupancy_grid_digest(**arguments)
        self.assertEqual(64, len(baseline))
        self.assertEqual(baseline, occupancy_grid_digest(**arguments))
        mutations = (
            ("frame_id", "other_map"),
            ("width", 1),
            ("height", 4),
            ("resolution", 0.10),
            ("origin_position", (1.0, 2.0, 3.1)),
            ("origin_orientation", (0.0, 0.0, 0.1, 0.995)),
            ("data", (0, -1, 99, 0)),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                changed = dict(arguments)
                changed[field] = value
                # Keep dimensions/payload length valid in dimension subtests.
                if field in ("width", "height"):
                    changed["data"] = tuple(0 for _ in range(
                        changed["width"] * changed["height"]
                    ))
                self.assertNotEqual(baseline, occupancy_grid_digest(**changed))

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

    def test_route_avoids_an_orientation_only_ackermann_entry(self):
        swath = Swath(Point(0.0, 0.0), Point(3.0, 0.0), 0.0, 3.0)
        route = order_swaths(
            [swath], Point(0.0, 0.0), 0.85, 1.35, current_yaw=math.pi
        )
        self.assertEqual(Point(3.0, 0.0), route[0].start)
        self.assertEqual(Point(0.0, 0.0), route[0].end)

    def test_route_minimizes_complete_cost_across_allowed_cyclic_orders(self):
        spacing = 0.85
        radius = 1.35
        current = Point(10.0, 4.0)
        current_yaw = -0.5 * math.pi
        swaths = [
            Swath(Point(-3.0, 0.00), Point(7.0, 0.00), 0.00, 10.0),
            Swath(Point(3.0, 0.85), Point(13.0, 0.85), 0.85, 10.0),
            Swath(Point(-8.0, 1.70), Point(2.0, 1.70), 1.70, 10.0),
            Swath(Point(8.0, 2.55), Point(16.0, 2.55), 2.55, 8.0),
            Swath(Point(4.0, 3.40), Point(12.0, 3.40), 3.40, 8.0),
        ]
        base_order = _turn_friendly_base_order(len(swaths), spacing, radius)
        oracle = _oracle_best(
            swaths,
            _allowed_index_orders(base_order),
            current,
            current_yaw,
            radius,
        )
        route = order_swaths(
            swaths, current, spacing, radius, current_yaw=current_yaw)
        index_order, directions = _route_signature(route, swaths)
        actual_cost = _oracle_route_cost(
            swaths, index_order, directions, current, current_yaw, radius)

        self.assertEqual((0, 4, 1, 2, 3), base_order)
        self.assertEqual((3, 0, 4, 1, 2), oracle[1])
        self.assertEqual((1, 1, 0, 1, 1), oracle[2])
        self.assertAlmostEqual(77.34820688593743, oracle[0], places=9)
        self.assertEqual(oracle[1:], (index_order, directions))
        self.assertAlmostEqual(oracle[0], actual_cost, places=9)

        old_greedy_cost = _oracle_route_cost(
            swaths,
            (4, 1, 2, 3, 0),
            (1, 0, 1, 0, 1),
            current,
            current_yaw,
            radius,
        )
        self.assertAlmostEqual(106.32193720200752, old_greedy_cost, places=9)
        self.assertLess(actual_cost, old_greedy_cost)

    def test_route_enumerates_reverse_cyclic_orders(self):
        spacing = 0.85
        radius = 1.35
        current = Point(10.0, 2.0)
        current_yaw = -0.5 * math.pi
        swaths = [
            Swath(Point(-9.0, 0.00), Point(-4.0, 0.00), 0.00, 5.0),
            Swath(Point(-7.0, 0.85), Point(2.0, 0.85), 0.85, 9.0),
            Swath(Point(-5.0, 1.70), Point(-1.0, 1.70), 1.70, 4.0),
            Swath(Point(4.0, 2.55), Point(13.0, 2.55), 2.55, 9.0),
            Swath(Point(0.0, 3.40), Point(10.0, 3.40), 3.40, 10.0),
        ]
        base_order = _turn_friendly_base_order(len(swaths), spacing, radius)
        forward_best = _oracle_best(
            swaths,
            _cyclic_orders(base_order),
            current,
            current_yaw,
            radius,
        )
        oracle = _oracle_best(
            swaths,
            _allowed_index_orders(base_order),
            current,
            current_yaw,
            radius,
        )
        route = order_swaths(
            swaths, current, spacing, radius, current_yaw=current_yaw)
        index_order, directions = _route_signature(route, swaths)
        actual_cost = _oracle_route_cost(
            swaths, index_order, directions, current, current_yaw, radius)

        self.assertEqual((3, 2, 1, 4, 0), oracle[1])
        self.assertEqual((1, 1, 0, 1, 1), oracle[2])
        self.assertAlmostEqual(71.54408057301828, oracle[0], places=9)
        self.assertEqual((4, 1, 2, 3, 0), forward_best[1])
        self.assertEqual((1, 1, 1, 1, 1), forward_best[2])
        self.assertAlmostEqual(76.23787531792067, forward_best[0], places=9)
        self.assertLess(oracle[0], forward_best[0])
        self.assertEqual(oracle[1:], (index_order, directions))
        self.assertAlmostEqual(oracle[0], actual_cost, places=9)

    def test_fixed_order_direction_dp_beats_greedy_endpoint_selection(self):
        radius = 1.35
        current = Point(7.0, -2.0)
        current_yaw = -0.5 * math.pi
        index_order = (1, 2, 0)
        swaths = [
            Swath(Point(-10.0, 0.00), Point(-3.0, 0.00), 0.00, 7.0),
            Swath(Point(1.0, 0.85), Point(7.0, 0.85), 0.85, 6.0),
            Swath(Point(2.0, 1.70), Point(9.0, 1.70), 1.70, 7.0),
        ]
        oracle = _oracle_best(
            swaths, (index_order,), current, current_yaw, radius)
        greedy_cost, greedy_directions = _oracle_greedy_for_order(
            swaths, index_order, current, current_yaw, radius)
        route, optimized_cost = _optimize_orientations_for_order(
            swaths, index_order, current, current_yaw, radius)
        routed_order, directions = _route_signature(route, swaths)

        self.assertEqual((1, 1, 1), oracle[2])
        self.assertAlmostEqual(38.29670293316519, oracle[0], places=9)
        self.assertEqual((1, 0, 1), greedy_directions)
        self.assertAlmostEqual(47.26072941278843, greedy_cost, places=9)
        self.assertEqual(index_order, routed_order)
        self.assertEqual(oracle[2], directions)
        self.assertAlmostEqual(oracle[0], optimized_cost, places=9)
        self.assertLess(optimized_cost, greedy_cost)

    def test_time_estimate_uses_rest_to_rest_motion_limits(self):
        route = [Swath(Point(0.0, 0.0), Point(2.0, 0.0), 0.0, 2.0)]
        parameters = CoverageTimeParameters(
            max_forward_speed_mps=1.0,
            max_reverse_speed_mps=0.3,
            max_angular_speed_rps=0.6,
            linear_accel_mps2=1.0,
            angular_accel_rps2=0.5,
            allow_reverse=False,
            direction_change_penalty_sec=0.0,
            segment_handoff_penalty_sec=0.0,
        )
        estimate = estimate_route_time(
            route, Point(0.0, 0.0), 0.0, 1.35, parameters)
        self.assertAlmostEqual(3.0, estimate.total_time_sec, places=9)
        self.assertAlmostEqual(3.0, estimate.sweep_time_sec, places=9)
        self.assertAlmostEqual(0.0, estimate.transit_time_sec, places=9)

    def test_dubins_proxy_respects_m2_turning_radius(self):
        radius = 1.35
        straight = _dubins_path_components(
            Point(0.0, 0.0), 0.0,
            Point(4.0, 0.0), 0.0,
            radius,
        )
        quarter_turn = _dubins_path_components(
            Point(0.0, 0.0), 0.0,
            Point(radius, radius), 0.5 * math.pi,
            radius,
        )
        self.assertIsNotNone(straight)
        self.assertAlmostEqual(4.0, straight[0], places=9)
        self.assertAlmostEqual(0.0, straight[1], places=9)
        self.assertIsNotNone(quarter_turn)
        self.assertAlmostEqual(0.5 * math.pi * radius,
                               quarter_turn[0], places=8)
        self.assertAlmostEqual(quarter_turn[0], quarter_turn[1], places=8)

    def test_dubins_proxy_penalizes_close_opposite_heading_entry(self):
        radius = 1.35
        components = _dubins_path_components(
            Point(0.0, 0.0), 0.0,
            Point(0.85, 0.0), math.pi,
            radius,
        )
        self.assertIsNotNone(components)
        self.assertGreater(components[0], 2.0 * radius)

    def test_time_estimate_selects_reverse_only_when_allowed(self):
        route = [Swath(Point(-4.0, 0.0), Point(-1.0, 0.0), 0.0, 3.0)]
        common = dict(
            max_forward_speed_mps=0.8,
            max_reverse_speed_mps=0.5,
            max_angular_speed_rps=0.6,
            linear_accel_mps2=1.0,
            angular_accel_rps2=0.5,
            direction_change_penalty_sec=0.5,
            segment_handoff_penalty_sec=0.0,
        )
        reversing = estimate_route_time(
            route,
            Point(0.0, 0.0),
            0.0,
            1.35,
            CoverageTimeParameters(allow_reverse=True, **common),
        )
        forward_only = estimate_route_time(
            route,
            Point(0.0, 0.0),
            0.0,
            1.35,
            CoverageTimeParameters(allow_reverse=False, **common),
        )
        self.assertEqual(1, reversing.reverse_transitions)
        self.assertEqual(0, forward_only.reverse_transitions)
        self.assertLess(reversing.total_time_sec, forward_only.total_time_sec)

    def test_time_route_search_matches_complete_small_route_oracle(self):
        radius = 1.35
        current = Point(6.0, -2.0)
        current_yaw = 0.75 * math.pi
        parameters = CoverageTimeParameters(
            max_forward_speed_mps=0.9,
            max_reverse_speed_mps=0.25,
            max_angular_speed_rps=0.55,
            linear_accel_mps2=1.2,
            angular_accel_rps2=0.45,
            allow_reverse=True,
            direction_change_penalty_sec=1.3,
            segment_handoff_penalty_sec=0.4,
        )
        swaths = [
            Swath(Point(-7.0, 0.0), Point(-1.0, 0.0), 0.0, 6.0),
            Swath(Point(2.0, 0.9), Point(8.0, 0.9), 0.9, 6.0),
            Swath(Point(-3.0, 1.8), Point(4.0, 1.8), 1.8, 7.0),
            Swath(Point(7.0, 2.7), Point(12.0, 2.7), 2.7, 5.0),
        ]
        oracle = None
        for index_order in itertools.permutations(range(len(swaths))):
            for directions in itertools.product((0, 1), repeat=len(swaths)):
                route = []
                for index, direction in zip(index_order, directions):
                    swath = swaths[index]
                    route.append(
                        swath if direction == 0 else
                        Swath(swath.end, swath.start, swath.scan_v,
                              swath.length)
                    )
                estimate = estimate_route_time(
                    route, current, current_yaw, radius, parameters)
                signature = (index_order, directions)
                candidate = (estimate.total_time_sec, signature)
                if oracle is None or candidate < oracle:
                    oracle = candidate

        route, estimate = order_swaths(
            swaths,
            current,
            0.9,
            radius,
            current_yaw=current_yaw,
            time_parameters=parameters,
            return_estimate=True,
            time_search_beam_width=512,
        )
        signature = _route_signature(route, swaths)
        self.assertEqual(oracle[1], signature)
        self.assertAlmostEqual(oracle[0], estimate.total_time_sec, places=9)

    def test_static_connector_distance_accounts_for_obstacle_detour(self):
        grid = self.grid_with_obstacle((50, 0, 60, 70, 100))
        start = Point(4.0, 2.0)
        end = Point(7.0, 2.0)
        distance = grid.shortest_known_free_distance(start, end)
        self.assertIsNotNone(distance)
        self.assertGreater(distance, math.hypot(end.x - start.x,
                                                end.y - start.y))

    def test_planner_selects_and_reports_seconds_based_route(self):
        parameters = CoverageTimeParameters()
        plan = CoveragePlanner(self.grid_with_obstacle()).plan(
            self.rectangle(),
            1.0,
            0.15,
            route_origin=Point(1.5, 1.5),
            route_yaw=0.0,
            time_parameters=parameters,
            time_search_beam_width=64,
        )
        self.assertGreater(plan.estimated_total_time_sec, 0.0)
        self.assertGreater(plan.estimated_sweep_time_sec, 0.0)
        self.assertGreater(plan.estimated_transit_time_sec, 0.0)
        self.assertAlmostEqual(plan.score, plan.estimated_total_time_sec)
        self.assertEqual(
            plan.estimated_total_time_sec,
            plan.estimated_sweep_time_sec + plan.estimated_transit_time_sec,
        )

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
