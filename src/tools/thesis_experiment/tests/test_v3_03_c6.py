import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c6.py"
)
SPEC = importlib.util.spec_from_file_location("v3_03_c6", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_single_factor_c5_binding_and_fresh_schedule():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 9
    assert {row["seed"] for row in schedule["rows"]} == {6361, 6362, 6363}
    assert contract["upstream"]["planner_contract_sha256"] == (
        "6f2ec1530b67785dfc8f5a397ab416847f35f0c6ffc9e6f45bbf633a50057761"
    )
    factor = contract["single_factor_repair"]
    assert factor["factor_count"] == 1
    assert factor["integration_gates_changed"] is False


def test_duration_aware_parser_retained(tmp_path):
    path = tmp_path / "trace.csv"
    rows = [
        "sim_time_s,ego_x_m,ego_y_m,ego_yaw_rad,actor_x_m,actor_y_m,actor_yaw_rad"
    ]
    rows.extend("{},0,0,0,0,0,0".format(0.05 * index) for index in range(6))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    assert len(V3.load_trace(path)) == 6
