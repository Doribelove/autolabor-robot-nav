import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c2.py"
)
SPEC = importlib.util.spec_from_file_location("v3_02_c2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_common_prefix_test_schedule_and_frozen_model():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert {row["seed"] for row in schedule["rows"]} == {6261, 6262, 6263}
    assert len(schedule["rows"]) == 36
    assert contract["scene"]["ego"]["common_prefix"] == {
        "duration_s": 0.5,
        "speed_mps": 0.8,
        "end_x_m": -2.6,
    }
    assert len(V3.model_features({
        "actor_conflict_coordinate_at_observation_m": -2.5,
        "actor_observed_speed_mps": 1.0,
        "candidate_ego_speed_mps": 0.8,
    })) == 10
