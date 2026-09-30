import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_b0.py"
)
RUNNER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


V3 = load(MODULE_PATH, "v3_05_b0_test_module")
RUNNER = load(RUNNER_PATH, "v3_05_b0_test_runner")


def test_fresh_complete_paired_schedule_and_unchanged_statistics():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 270
    assert set(schedule["seeds"]) == set(range(6451, 6481))
    assert contract["formal_statistics"]["minimum_point_reduction"] == 0.05
    assert contract["formal_statistics"][
        "minimum_one_sided_lower_reduction"
    ] == 0.02


def test_all_allocated_ports_are_unique_and_below_ephemeral_range():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    schedule = V3.exact_schedule(contract)
    ports = [
        RUNNER.gazebo_master_port(contract, row["sequence"], decision)
        for row in schedule["rows"]
        for decision in range(39)
    ]
    assert len(ports) == len(set(ports)) == 10530
    assert min(ports) == 12050
    assert max(ports) == 25538
    assert max(ports) < 32768
