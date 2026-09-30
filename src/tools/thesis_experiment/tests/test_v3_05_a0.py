import importlib.util
from pathlib import Path
import sys

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_a0.py"
)
RUNNER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


V3 = load(MODULE_PATH, "v3_05_a0_test_module")
RUNNER = load(RUNNER_PATH, "v3_05_a0_test_runner")


def test_formal_schedule_is_paired_and_counterbalanced():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 270
    for block_id in {row["block_id"] for row in schedule["rows"]}:
        block = [row for row in schedule["rows"] if row["block_id"] == block_id]
        assert len(block) == 9
        for policy in ("cautious", "assertive", "hesitant"):
            methods = {
                row["method"]
                for row in block
                if row["actor_policy_evaluator_only"] == policy
            }
            assert methods == {
                "v3_analytic",
                "fixed_v120_shielded",
                "fixed_v080_shielded",
            }


def test_shielded_fixed_methods_never_select_rejected_candidate():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    planner = V3.load_yaml_path(
        WORKSPACE / contract["upstream"]["planner_contract_path"]
    )
    model = V3.load_yaml_path(
        WORKSPACE / contract["frozen_model"]["path"]
    )
    snapshot = {
        "scene_snapshot_id": "method_fixture",
        "world_model_seq": 1,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {"x_m": -2.2, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.0},
        "actor": {
            "coordinate_m": 0.7,
            "speed_mps": 1.0,
            "path_angle_deg": 90.0,
            "monotone_coordinate_certified": True,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }
    base = core.plan(snapshot, model, planner)
    for method in ("fixed_v120_shielded", "fixed_v080_shielded"):
        result = RUNNER.apply_selection_method(
            base, {"method": method}, planner
        )
        selected = next(
            item for item in result["evaluations"]
            if item["candidate_id"]
            == result["decision"]["selected_candidate_id"]
        )
        assert selected["accepted"]
        assert result["decision"]["policy_kind"] == method


def test_design_does_not_claim_actual_teb():
    raw = yaml.safe_load(
        (WORKSPACE / V3.CONTRACT_RELATIVE).read_text(encoding="utf-8")
    )
    assert not raw["methods"]["secondary_reference"]["actual_teb_claim"]
