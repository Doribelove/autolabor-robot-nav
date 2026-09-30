import copy
from pathlib import Path

import yaml

from v3_scope_planner import thesis_benchmark_methods as methods


W = Path("/home/robot/robot_ws_base_rl")
MODEL = yaml.safe_load((W / "artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml").read_bytes())
PLANNER = yaml.safe_load((W / "artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml").read_bytes())
BASELINE_MODEL = yaml.safe_load((W / "artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml").read_bytes())
BASELINE_PLANNER = yaml.safe_load((W / "artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml").read_bytes())
REPORT = next(
    yaml.safe_load(path.read_bytes())
    for path in (W / "artifacts/v3/actual_teb/v3_11_f1_execution/episodes").glob("*/episode_report.yaml")
    if yaml.safe_load(path.read_bytes())["method"] == "v3_spatial_safe"
)


def snapshot():
    value = copy.deepcopy(REPORT["v3_plan"]["snapshot"])
    value["scene_snapshot_id"] = "v3_19_unit_test"
    return value


def test_single_factor_ablation_replaces_only_supported_spatial_heads():
    derived = methods.derive_no_candidate_conditioning_model(MODEL)
    for mode in ("cautious", "hesitant"):
        assert derived["supported_action_heads"][mode]["spatial_left_v120"] == derived["supported_action_heads"][mode]["certified_stop_v000"]
    assert MODEL["supported_action_heads"]["cautious"]["spatial_left_v120"] != MODEL["supported_action_heads"]["cautious"]["certified_stop_v000"]
    assert derived["ablation"]["candidate_generator_changed"] is False
    assert derived["ablation"]["hard_verifier_changed"] is False


def test_all_offline_planner_adapters_return_accepted_executable_candidate():
    main = methods.plan_spatial(snapshot(), MODEL, PLANNER)
    ablation = methods.plan_spatial(snapshot(), methods.derive_no_candidate_conditioning_model(MODEL), PLANNER, "v3_no_candidate_conditioning")
    fixed = methods.plan_fixed_v120(snapshot(), BASELINE_MODEL, BASELINE_PLANNER)
    for value in (main, ablation, fixed):
        assert value["selected_evaluation"]["accepted"] is True
        assert value["selected_candidate"]["points"]
        assert value["selected_candidate_id"] == value["selected_candidate"]["candidate_id"]
