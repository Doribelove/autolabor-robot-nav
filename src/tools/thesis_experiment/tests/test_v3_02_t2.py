import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t2.py"
SPEC = importlib.util.spec_from_file_location("v3_02_t2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_fresh_test_schedule_and_corrected_model():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert {row["seed"] for row in schedule["rows"]} == {6231, 6232, 6233}
    assert len(schedule["rows"]) == 36
    assert contract["frozen_model"]["conformal_score"] == (
        "actual_mode_absolute_residual"
    )
