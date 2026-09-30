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
    "build_v3_03_c3_commitment_invariance.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_03_c3_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
CONTRACT = BUILDER.load_yaml(BUILDER.CONTRACT)
PLANNER = BUILDER.effective_contract(CONTRACT)
MODEL = BUILDER.load_yaml(Path(CONTRACT["upstream"]["predictor_path"]))


def test_c2_pre_failure_state_is_stopped_before_domain_exit():
    value = BUILDER.snapshot(
        "c2_failure", -1.70, 0.515, -2.1609, 0.50, 72.0
    )
    result = core.plan(value, MODEL, PLANNER)
    selected = result["decision"]["selected_candidate_id"]
    evaluation = next(
        item for item in result["evaluations"]
        if item["candidate_id"] == selected
    )
    assert selected == "certified_stop_v000"
    assert evaluation["accepted"]
    motion = next(
        item for item in result["evaluations"]
        if item["candidate_id"] == "feasible_v060"
    )
    assert "COMMITMENT_BRAKING_DOMAIN_INVALID" in motion["rejection_codes"]


def test_cleared_certificate_exempts_conflict_braking_domain():
    value = BUILDER.snapshot("cleared", -0.5, 0.8, 1.35, 0.9, 90.0)
    result = core.plan(value, MODEL, PLANNER)
    assert result["decision"]["selected_candidate_id"] == "feasible_v120"


def test_builder_is_deterministic_and_passes():
    first = BUILDER.build()
    second = BUILDER.build()
    assert first == second
    report = yaml.safe_load(first[BUILDER.REPORT])
    assert report["overall_pass"]
    assert report["metrics"]["selected_commitment_invariant_rate"] == 1.0
