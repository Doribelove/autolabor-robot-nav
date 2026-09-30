import importlib.util
from pathlib import Path
import sys

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_03_c5_post_conflict_reachable.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_03_c5_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
CONTRACT = BUILDER.load_yaml(BUILDER.CONTRACT)
PLANNER = BUILDER.effective_contract(CONTRACT)
MODEL = BUILDER.load_yaml(Path(CONTRACT["upstream"]["predictor_path"]))


def snapshot(certified=True, angle=90.0, coordinate=0.7, speed=1.0):
    return {
        "scene_snapshot_id": "c5_fixture",
        "world_model_seq": 1,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {"x_m": -2.2, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.0},
        "actor": {
            "coordinate_m": coordinate,
            "speed_mps": speed,
            "path_angle_deg": angle,
            "monotone_coordinate_certified": certified,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


def test_post_conflict_certificate_admits_verified_motion():
    value = snapshot()
    result = core.plan(value, MODEL, PLANNER)
    selected = result["decision"]["selected_candidate_id"]
    assert selected != "certified_stop_v000"
    candidate = next(
        item for item in result["candidate_set"]["candidates"]
        if item["candidate_id"] == selected
    )
    tube = core.predict_occupancy_tube(value, candidate, MODEL, PLANNER)
    assert tube["safety_certified_post_conflict_reachable"]
    assert min(
        item["lower_coordinate_m"] for item in tube["samples"]
    ) >= value["actor"]["coordinate_m"]


def test_missing_or_out_of_support_certificate_fails_closed():
    for value in (
        snapshot(certified=False),
        snapshot(angle=69.0),
        snapshot(angle=111.0),
        snapshot(speed=0.11),
    ):
        result = core.plan(value, MODEL, PLANNER)
        assert result["decision"]["selected_candidate_id"] == (
            "certified_stop_v000"
        )
        assert not core.actor_post_conflict_reachable(value, PLANNER)


def test_builder_is_deterministic_and_passes():
    first = BUILDER.build()
    second = BUILDER.build()
    assert first == second
    report = yaml.safe_load(first[BUILDER.REPORT])
    assert report["overall_pass"]
    assert report["metrics"]["reachable_lower_bound_conservative_rate"] == 1.0
