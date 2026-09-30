import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d2.py"
SPEC = importlib.util.spec_from_file_location("v3_02_d2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_fresh_schedule_and_frozen_feasible_model():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 45
    assert {row["seed"] for row in schedule["rows"]} == {6281, 6282, 6283}
    assert sum(row["certified_stop"] for row in schedule["rows"]) == 9
    assert contract["frozen_model"]["refit_or_recalibration_allowed"] is False
