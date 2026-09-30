"""Arena occupancy-map adapter for the independent PCT comparison study.

The historical PCT benchmark models static geometry as corridors, circles and
boxes.  Rich Arena worlds contain meshes that cannot be represented faithfully
by that manifest.  This module keeps the frozen candidate generator and policy
semantics, but replaces only the static-surface query with a conservative
distance field built from the Arena map.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

from .benchmark_runtime import DirectionalBenchmarkRuntimeAdapter


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class ArenaOccupancyDistanceField:
    """Conservative map-to-route distance queries for one frozen Arena map."""

    def __init__(
        self,
        map_yaml_path,
        *,
        route_origin_world_x_m,
        route_origin_world_y_m,
        route_yaw_world_rad,
        free_pixel_minimum=250,
    ):
        self.map_yaml_path = Path(map_yaml_path).resolve()
        with self.map_yaml_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        self.image_path = (self.map_yaml_path.parent / metadata["image"]).resolve()
        self.resolution_m = float(metadata["resolution"])
        origin = metadata["origin"]
        self.map_origin_world_x_m = float(origin[0])
        self.map_origin_world_y_m = float(origin[1])
        self.map_origin_yaw_rad = float(origin[2])
        if abs(self.map_origin_yaw_rad) > 1.0e-12:
            raise ValueError("ROTATED_ARENA_OCCUPANCY_MAP_UNSUPPORTED")
        if self.resolution_m <= 0.0:
            raise ValueError("NONPOSITIVE_ARENA_MAP_RESOLUTION")

        image = np.asarray(Image.open(self.image_path).convert("L"), dtype=np.uint8)
        if image.ndim != 2 or min(image.shape) < 2:
            raise ValueError("ARENA_MAP_IMAGE_INVALID")
        self.image = image
        self.height, self.width = image.shape
        self.free = image >= int(free_pixel_minimum)
        # Unknown cells are deliberately treated as occupied.  EDT measures
        # pixel-centre to pixel-centre distance.  For an arbitrary query inside
        # the nearest free pixel, subtract one half-cell diagonal for the query
        # offset and another for the occupied pixel extent.  The resulting
        # nearest-centre lookup is a lower bound on point-to-nonfree-cell-AABB
        # distance (at the cost of at most sqrt(2) cells of conservatism).
        raw_distance = distance_transform_edt(self.free) * self.resolution_m
        self.grid_discretization_subtraction_m = (
            math.sqrt(2.0) * self.resolution_m
        )
        self.distance_m = np.maximum(
            0.0, raw_distance - self.grid_discretization_subtraction_m
        )

        self.route_origin_world_x_m = float(route_origin_world_x_m)
        self.route_origin_world_y_m = float(route_origin_world_y_m)
        self.route_yaw_world_rad = float(route_yaw_world_rad)
        if not all(
            math.isfinite(value)
            for value in (
                self.route_origin_world_x_m,
                self.route_origin_world_y_m,
                self.route_yaw_world_rad,
            )
        ):
            raise ValueError("NONFINITE_ARENA_ROUTE_TRANSFORM")
        self.cos_yaw = math.cos(self.route_yaw_world_rad)
        self.sin_yaw = math.sin(self.route_yaw_world_rad)

    def route_to_world(self, points):
        points = np.asarray(points, dtype=np.float64)
        original_shape = points.shape
        points = np.atleast_2d(points)
        if points.shape[1] != 2:
            raise ValueError("ARENA_ROUTE_POINT_SHAPE_INVALID")
        result = np.empty_like(points)
        result[:, 0] = (
            self.route_origin_world_x_m
            + self.cos_yaw * points[:, 0]
            - self.sin_yaw * points[:, 1]
        )
        result[:, 1] = (
            self.route_origin_world_y_m
            + self.sin_yaw * points[:, 0]
            + self.cos_yaw * points[:, 1]
        )
        return result[0] if len(original_shape) == 1 else result

    def world_to_route(self, points):
        points = np.asarray(points, dtype=np.float64)
        original_shape = points.shape
        points = np.atleast_2d(points)
        if points.shape[1] != 2:
            raise ValueError("ARENA_WORLD_POINT_SHAPE_INVALID")
        dx = points[:, 0] - self.route_origin_world_x_m
        dy = points[:, 1] - self.route_origin_world_y_m
        result = np.empty_like(points)
        result[:, 0] = self.cos_yaw * dx + self.sin_yaw * dy
        result[:, 1] = -self.sin_yaw * dx + self.cos_yaw * dy
        return result[0] if len(original_shape) == 1 else result

    def _grid_coordinates(self, route_points):
        world = np.atleast_2d(self.route_to_world(route_points))
        # ROS map origin is the lower-left *cell boundary*.  Pixel centres are
        # therefore offset by half a cell from integer world-grid coordinates.
        columns = (
            (world[:, 0] - self.map_origin_world_x_m) / self.resolution_m
            - 0.5
        )
        rows = (
            self.height
            - 0.5
            - (world[:, 1] - self.map_origin_world_y_m) / self.resolution_m
        )
        return rows, columns, world

    def clearance_m(self, route_points):
        """Return a conservative centre-to-nonfree distance for route points."""
        route_points = np.asarray(route_points, dtype=np.float64)
        scalar = route_points.ndim == 1
        rows, columns, world = self._grid_coordinates(route_points)
        inside = (
            (rows >= -0.5)
            & (rows <= self.height - 0.5)
            & (columns >= -0.5)
            & (columns <= self.width - 0.5)
        )
        result = np.zeros(len(rows), dtype=np.float64)
        if np.any(inside):
            row_index = np.rint(rows[inside]).astype(np.int64)
            column_index = np.rint(columns[inside]).astype(np.int64)
            row_index = np.clip(row_index, 0, self.height - 1)
            column_index = np.clip(column_index, 0, self.width - 1)
            result[inside] = self.distance_m[row_index, column_index]
            # Outside-map space is unknown/occupied.  Include exact distance
            # to the four map boundaries in the lower bound.
            active_world = world[inside]
            boundary_distance = np.minimum.reduce(
                (
                    active_world[:, 0] - self.map_origin_world_x_m,
                    self.map_origin_world_x_m
                    + self.width * self.resolution_m
                    - active_world[:, 0],
                    active_world[:, 1] - self.map_origin_world_y_m,
                    self.map_origin_world_y_m
                    + self.height * self.resolution_m
                    - active_world[:, 1],
                )
            )
            result[inside] = np.minimum(
                result[inside], np.maximum(0.0, boundary_distance)
            )
        return float(result[0]) if scalar else result

    def static_surface_mask(self, route_points, tolerance_m):
        route_points = np.asarray(route_points, dtype=np.float64)
        if route_points.size == 0:
            return np.zeros(0, dtype=bool)
        route_points = np.atleast_2d(route_points)
        rows, columns, _world = self._grid_coordinates(route_points)
        outside = (
            (rows < -0.5)
            | (rows > self.height - 0.5)
            | (columns < -0.5)
            | (columns > self.width - 0.5)
        )
        # Outside-map returns are static/unknown rather than dynamic.  This is
        # fail-closed and prevents a boundary scan from becoming a fake actor.
        return outside | (self.clearance_m(route_points) <= float(tolerance_m))

    def audit(self):
        return {
            "map_yaml_path": str(self.map_yaml_path),
            "map_yaml_sha256": _sha256(self.map_yaml_path),
            "map_image_path": str(self.image_path),
            "map_image_sha256": _sha256(self.image_path),
            "map_shape": [int(self.height), int(self.width)],
            "resolution_m": self.resolution_m,
            "unknown_treated_as_occupied": True,
            "strict_grid_discretization_subtraction_m": (
                self.grid_discretization_subtraction_m
            ),
            "route_origin_world_m": [
                self.route_origin_world_x_m,
                self.route_origin_world_y_m,
            ],
            "route_yaw_world_rad": self.route_yaw_world_rad,
        }


class ArenaMapDirectionalBenchmarkRuntimeAdapter(
    DirectionalBenchmarkRuntimeAdapter
):
    """Frozen R2 runtime with only the static geometry backend replaced."""

    def __init__(
        self,
        *args,
        map_yaml_path,
        route_origin_world_x_m,
        route_origin_world_y_m,
        route_yaw_world_rad,
        map_static_surface_tolerance_m=0.35,
        **kwargs,
    ):
        self.arena_map = ArenaOccupancyDistanceField(
            map_yaml_path,
            route_origin_world_x_m=route_origin_world_x_m,
            route_origin_world_y_m=route_origin_world_y_m,
            route_yaw_world_rad=route_yaw_world_rad,
        )
        self.map_static_surface_tolerance_m = float(
            map_static_surface_tolerance_m
        )
        kwargs["wall_half_width_m"] = 1000.0
        kwargs["static_circles"] = ()
        kwargs["static_boxes"] = ()
        kwargs["static_surface_association_tolerance_m"] = (
            self.map_static_surface_tolerance_m
        )
        super().__init__(*args, **kwargs)

    def _known_static_point_mask(self, points):
        return self.arena_map.static_surface_mask(
            points, self.map_static_surface_tolerance_m
        )

    def _known_static_geometry_clearance(self, state):
        centre_distance = self.arena_map.clearance_m(
            np.asarray([float(state[0]), float(state[1])], dtype=np.float64)
        )
        return (
            centre_distance - self.robot_radius_m - self.shield_inflation_m
        )

    def _central_static_route_release_x(self):
        # This independent qualification route contains no registered central
        # obstacle.  The occupancy distance field still masks every unsafe
        # preview; no synthetic release coordinate is inferred from the map.
        return float("-inf")

    def candidate_safety_diagnostics(self, pose, speed, yaw_rate):
        diagnostics = super().candidate_safety_diagnostics(pose, speed, yaw_rate)
        diagnostics["arena_static_map"] = self.arena_map.audit()
        diagnostics["ego_static_map_centre_clearance_m"] = (
            self.arena_map.clearance_m(np.asarray(pose[:2], dtype=np.float64))
        )
        return diagnostics
