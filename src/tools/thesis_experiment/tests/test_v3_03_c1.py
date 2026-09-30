import importlib.util
from pathlib import Path
import sys


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

V3_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
)
RUNNER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


V3 = load(V3_PATH, "v3_03_c1_test_v3")
RUNNER = load(RUNNER_PATH, "v3_03_c1_test_runner")


def test_fresh_schedule_and_zero_hardware_authority():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 9
    assert {row["seed"] for row in schedule["rows"]} == {6331, 6332, 6333}
    assert contract["authority_boundary"]["hardware_device_access_authorized"] is False
    assert contract["closed_loop"]["evaluator_policy_passed_to_planner"] is False


def test_segment_world_carries_actor_speed_and_time(tmp_path):
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    row = V3.exact_schedule(contract)["rows"][0]
    state = {
        "global_time_s": 1.25,
        "ego_x_m": -2.2,
        "ego_y_m": 0.0,
        "ego_yaw_rad": 0.0,
        "ego_speed_mps": 0.0,
        "actor_coordinate_m": -1.5,
        "actor_speed_mps": 0.32,
    }
    world = RUNNER.render_segment(
        contract,
        row,
        state,
        [[0.0, [-2.2, 0.0, 0.0]], [0.25, [-2.2, 0.0, 0.0]]],
        0.25,
        tmp_path / "raw.csv",
        tmp_path / "done",
        "fixture",
    )
    assert "<initial_speed>0.320000000</initial_speed>" in world
    assert "<time_offset>1.250000000</time_offset>" in world


def test_frozen_planner_has_certificate_bound_clearance_bypass():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    planner = V3.load_yaml_path(
        WORKSPACE / contract["upstream"]["planner_contract_path"]
    )
    assert (
        planner["predictor_runtime"]["cleared_actor_bypass"][
            "required_snapshot_actor_flag"
        ]
        == "monotone_coordinate_certified"
    )
