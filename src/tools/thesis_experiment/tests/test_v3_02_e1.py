import importlib.util
from pathlib import Path
import sys


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_e1.py"
)
SPEC = importlib.util.spec_from_file_location("v3_02_e1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_fresh_schedule_and_frozen_mode_gate():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 45
    assert {row["seed"] for row in schedule["rows"]} == {6321, 6322, 6323}
    assert sum(row["certified_stop"] for row in schedule["rows"]) == 9
    assert contract["frozen_model"]["refit_or_recalibration_allowed"] is False
    assert contract["authority_boundary"]["gate_write_allowed_after_test"] is False


def test_mode_gate_uses_only_observed_speed_and_has_ambiguity_stop():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    planner = V3.load_yaml_path(
        WORKSPACE
        / contract["observed_mode_gate"]["effective_planner_contract_path"]
    )
    gate = planner["predictor_runtime"]["mode_inference"]
    assert core.infer_observed_mode(0.2, gate) == "cautious"
    assert core.infer_observed_mode(0.325, gate) == "hesitant"
    assert core.infer_observed_mode(1.2, gate) == "assertive"
    assert gate["hidden_policy_runtime_input"] is False
