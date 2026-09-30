from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from v5_e2e_scope_rl.arena_benchmark_runtime import ArenaOccupancyDistanceField


ROOT = Path(__file__).resolve().parents[4]
MAP_YAML = ROOT / (
    "src/navigation_arena/arena-rosnav-3D/simulator_setup/maps/hospital/map.yaml"
)


def test_hospital_route_transform_and_clearance_contract():
    field = ArenaOccupancyDistanceField(
        MAP_YAML,
        route_origin_world_x_m=0.0,
        route_origin_world_y_m=11.2,
        route_yaw_world_rad=0.0,
    )
    world = field.route_to_world(np.asarray([-4.0, 0.0]))
    assert np.allclose(world, [-4.0, 11.2])
    assert np.allclose(field.world_to_route(world), [-4.0, 0.0])
    assert field.clearance_m(np.asarray([-4.0, 0.0])) > 1.0


def test_hospital_policy_band_is_clear_for_frozen_candidate_offsets():
    field = ArenaOccupancyDistanceField(
        MAP_YAML,
        route_origin_world_x_m=0.0,
        route_origin_world_y_m=11.2,
        route_yaw_world_rad=0.0,
    )
    warmup_points = [[x_m, 0.0] for x_m in np.linspace(-5.0, -4.0, 21)]
    candidate_band_points = []
    for x_m in np.linspace(-4.0, 4.2, 83):
        for y_m in np.linspace(-1.35, 1.35, 55):
            candidate_band_points.append([x_m, y_m])
    # Frozen shield radius is 0.46 + 0.33 = 0.79 m.  The offline route
    # audit retains at least 0.20 m additional map clearance.
    assert float(np.min(field.clearance_m(np.asarray(warmup_points)))) >= 1.05
    assert (
        float(np.min(field.clearance_m(np.asarray(candidate_band_points))))
        >= 1.05
    )


def test_distance_field_is_lower_bound_around_occupied_cell(tmp_path):
    image = np.full((5, 5), 255, dtype=np.uint8)
    image[2, 2] = 0
    image_path = tmp_path / "map.pgm"
    Image.fromarray(image).save(image_path)
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": "map.pgm",
                "resolution": 1.0,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        encoding="utf-8",
    )
    field = ArenaOccupancyDistanceField(
        yaml_path,
        route_origin_world_x_m=0.0,
        route_origin_world_y_m=0.0,
        route_yaw_world_rad=0.0,
    )
    samples = np.asarray(
        [
            [0.01, 0.01],
            [1.49, 1.49],
            [1.51, 1.51],
            [2.00, 1.49],
            [2.51, 2.51],
            [3.49, 3.49],
            [3.99, 2.00],
        ],
        dtype=np.float64,
    )
    # ROS map origin is a cell boundary. Occupied pixel (row=2, col=2) is
    # centred at world (2.5, 2.5) and spans [2.0, 3.0] on both axes.
    dx = np.maximum(np.abs(samples[:, 0] - 2.5) - 0.5, 0.0)
    dy = np.maximum(np.abs(samples[:, 1] - 2.5) - 0.5, 0.0)
    exact_aabb_distance = np.hypot(dx, dy)
    observed = field.clearance_m(samples)
    assert np.all(observed <= exact_aabb_distance + 1.0e-12)
    assert np.all(observed >= 0.0)
