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
    "build_v3_03_c0_closed_loop.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_03_c0_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
CONTRACT = BUILDER.load_yaml(BUILDER.CONTRACT)
PLANNER = BUILDER.effective_contract(CONTRACT)
MODEL = BUILDER.load_yaml(Path(CONTRACT["upstream"]["predictor_path"]))


def test_cleared_certificate_restores_motion():
    value = BUILDER.snapshot("cleared", 1.35, 0.9, 90.0, True)
    result = core.plan(value, MODEL, PLANNER)
    assert result["decision"]["selected_candidate_id"] == "feasible_v120"
    assert core.actor_cleared(value, PLANNER)


def test_missing_certificate_fails_closed():
    value = BUILDER.snapshot("uncertified", 1.35, 0.9, 90.0, False)
    result = core.plan(value, MODEL, PLANNER)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert not core.actor_cleared(value, PLANNER)


def test_stop_candidate_is_valid_from_maximum_speed():
    value = BUILDER.snapshot(
        "fast", -2.6, 0.2, 90.0, True, -2.6, 1.2
    )
    candidate_set = core.generate_candidate_set(value, PLANNER)
    stop = next(
        item for item in candidate_set["candidates"] if item["certified_stop"]
    )
    assert not core.validate_candidate(stop, value, PLANNER)
    assert stop["points"][-1]["longitudinal_velocity_mps"] == 0.0


def test_builder_is_deterministic_and_passes():
    first = BUILDER.build()
    second = BUILDER.build()
    assert first == second
    report = yaml.safe_load(first[BUILDER.REPORT])
    assert report["overall_pass"]
    assert report["metrics"]["minimum_cleared_geometric_margin_m"] >= 0.10
