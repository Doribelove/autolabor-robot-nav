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


MODULE_PATH = (
    W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_b0.py"
)
SPEC = importlib.util.spec_from_file_location("v3_06_b0_test_module", MODULE_PATH)
V = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V)


def load(path):
    return yaml.safe_load((W / path).read_bytes())


def fixture():
    episode = load(
        "artifacts/v3/formal/v3_05_b0/execution/episodes/"
        "v3_05_b0_formal_6451_assertive_v3_analytic/episode_receipt.yaml"
    )
    planner = load(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_effective_planner_contract.yaml"
    )
    model = load(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    )
    return (
        episode["decisions"][0]["snapshot_runtime_input_without_policy"],
        planner,
        model,
    )


def test_exact_schedule_is_fresh_and_bounded():
    contract = V.effective_contract()
    schedule = V.exact_schedule(contract)
    assert schedule["row_count"] == 48
    assert schedule["seeds"] == [6501, 6502, 6503, 6504]
    assert len({row["unit_id"] for row in schedule["rows"]}) == 48


def test_all_ranker_variants_select_only_accepted_candidates():
    snapshot, planner, model = fixture()
    original = core.plan(snapshot, model, planner)
    for method in V.METHOD_WEIGHTS:
        result = V.apply_selection_method(
            original, {"method": method}, planner
        )
        selected = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"]
            == result["decision"]["selected_candidate_id"]
        )
        assert selected["accepted"]


def test_faulted_snapshot_falls_back_to_certified_stop():
    snapshot, planner, model = fixture()
    faulted = copy.deepcopy(snapshot)
    faulted["health"]["localization_ok"] = False
    original = core.plan(faulted, model, planner)
    result = V.apply_selection_method(
        original, {"method": "anticipatory_w100"}, planner
    )
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert result["decision"]["safety_fallback_used"]
