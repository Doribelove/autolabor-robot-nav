import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a2.py"
SPEC = importlib.util.spec_from_file_location("v3_03_a2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_single_factor_and_fresh_schedule():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert schedule["simulation_unit_count"] == 18
    assert {row["seed"] for row in schedule["rows"]} == {6301, 6302, 6303}
    assert contract["single_factor_repair"]["factor_count"] == 1
    assert not contract["single_factor_repair"]["planner_core_changed"]


def test_short_probe_parser_preserves_schema_and_decision_coverage(tmp_path):
    path = tmp_path / "trace.csv"
    header = (
        "sim_time_s,ego_x_m,ego_y_m,ego_yaw_rad,"
        "actor_x_m,actor_y_m,actor_yaw_rad\n"
    )
    rows = [
        "{},-2.6,0,0,0,-2,1.57".format(0.05 * (index + 1))
        for index in range(11)
    ]
    path.write_text(header + "\n".join(rows) + "\n", encoding="utf-8")
    parsed = V3.load_trace(path)
    assert len(parsed) == 11
    assert parsed[-1]["sim_time_s"] >= 0.5
