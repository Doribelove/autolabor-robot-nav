import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"
SPEC = importlib.util.spec_from_file_location("v3_02_d1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_schedule_and_feasible_candidate_support():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 135
    assert schedule["training_rows"] == 90
    assert schedule["calibration_rows"] == 45
    assert {row["ego_speed_mps"] for row in schedule["rows"]} == {
        0.0, 0.6, 0.8, 1.0, 1.2
    }


def test_stop_and_motion_waypoints_are_bounded_and_monotonic():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    schedule = V3.exact_schedule(contract)
    representative = {}
    for row in schedule["rows"]:
        representative.setdefault(row["ego_speed_mps"], row)
    for speed, row in representative.items():
        points = V3.ego_waypoints(contract, row)
        assert all(right[0] > left[0] for left, right in zip(points, points[1:]))
        result = V3.planned_kinematics(contract, row)
        assert result["maximum_speed_mps"] <= 1.2 + 1e-9
        assert result["minimum_speed_mps"] >= -1e-9
        assert result["maximum_acceleration_mps2"] <= 0.61
        assert result["maximum_deceleration_mps2"] <= 0.81
        if speed == 0.0:
            assert abs(result["terminal_x_m"] - (-2.2)) <= 1e-9
