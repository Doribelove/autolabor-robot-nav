import copy
import importlib.util
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
from v3_scope_planner import core

MODULE_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"
SPEC = importlib.util.spec_from_file_location("v3_06_c1_test", MODULE_PATH)
V = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V)


def load(path):
    return yaml.safe_load((W / path).read_bytes())


def test_schedule_is_fresh_and_exact():
    schedule = V.exact_schedule(V.effective_contract())
    assert schedule["row_count"] == 36
    assert schedule["seeds"] == [6511, 6512, 6513, 6514]
    assert len({row["unit_id"] for row in schedule["rows"]}) == 36


def test_density_methods_remain_inside_accepted_set():
    episode = load(
        "artifacts/v3/development/v3_06_b0/execution/episodes/"
        "v3_06_b0_development_6501_cautious_v3_greedy_control/"
        "episode_receipt.yaml"
    )
    snapshot = episode["decisions"][0]["snapshot_runtime_input_without_policy"]
    planner = load(
        "artifacts/v3/development/v3_06_c0/"
        "v3_06_c0_effective_planner_contract.yaml"
    )
    model = load(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    )
    original = core.plan(snapshot, model, planner)
    for method in V.METHOD_LEVELS:
        result = V.apply_selection_method(original, {"method": method}, planner)
        selected = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"]
            == result["decision"]["selected_candidate_id"]
        )
        assert selected["accepted"]
