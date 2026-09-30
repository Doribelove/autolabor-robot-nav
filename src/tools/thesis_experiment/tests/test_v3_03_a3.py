import importlib.util
from pathlib import Path

WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a3.py"
SPEC = importlib.util.spec_from_file_location("v3_03_a3", PATH)
V3 = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(V3)


def test_snapshot_repair_and_fresh_schedule():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert {row["seed"] for row in schedule["rows"]} == {6311, 6312, 6313}
    factor = contract["single_factor_repair"]
    assert factor["factor_count"] == 1
    assert factor["factor_id"] == "decision_time_actor_snapshot_extractor"
    assert not factor["planner_core_changed"]
