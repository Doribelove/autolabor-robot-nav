import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"
SPEC = importlib.util.spec_from_file_location("v3_03_a1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_fresh_paired_schedule_and_zero_high_risk_authority():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert schedule["decision_count"] == 9
    assert schedule["simulation_unit_count"] == 18
    assert {row["seed"] for row in schedule["rows"]} == {6291, 6292, 6293}
    assert [row["phase"] for row in schedule["rows"]] == [
        "common_prefix_probe",
        "selected_candidate_execution",
    ] * 9
    assert contract["authority"]["cmd_vel_publish_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
