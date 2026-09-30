import importlib.util
from pathlib import Path

import pytest


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c2.py"
)
SPEC = importlib.util.spec_from_file_location("v3_03_c2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_single_factor_and_fresh_schedule():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 9
    assert {row["seed"] for row in schedule["rows"]} == {6341, 6342, 6343}
    factor = contract["single_factor_repair"]
    assert factor["factor_count"] == 1
    assert factor["integration_gates_changed"] is False


def test_segment_parser_accepts_quarter_second_fixture(tmp_path):
    path = tmp_path / "trace.csv"
    rows = [
        "sim_time_s,ego_x_m,ego_y_m,ego_yaw_rad,actor_x_m,actor_y_m,actor_yaw_rad"
    ]
    for index in range(6):
        time_s = 0.05 * index
        rows.append(
            "{},0,0,0,0,0,0".format(time_s)
        )
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    assert len(V3.load_trace(path)) == 6


def test_segment_parser_rejects_short_or_nonfinite_fixture(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text(
        "sim_time_s,ego_x_m,ego_y_m,ego_yaw_rad,actor_x_m,actor_y_m,actor_yaw_rad\n"
        "0,0,0,0,0,0,0\n"
        "0.05,nan,0,0,0,0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(V3.V301Error):
        V3.load_trace(path)
