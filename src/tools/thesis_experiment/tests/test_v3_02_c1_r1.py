import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1_r1.py"
)
SPEC = importlib.util.spec_from_file_location("v3_02_c1_r1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_complete_quadratic_basis():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    features = V3.model_features(
        {
            "actor_conflict_coordinate_at_observation_m": -2.0,
            "actor_observed_speed_mps": 1.0,
            "candidate_ego_speed_mps": 0.8,
        }
    )
    assert len(features) == 10
    assert features[7] == -2.0
