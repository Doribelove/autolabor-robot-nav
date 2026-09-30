import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_a1_r1.py"
)
SPEC = importlib.util.spec_from_file_location("v3_02_a1_r1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_single_physical_clip_factor_and_test_firewall():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    effective = V3.effective_contract(contract)
    clip = effective["predictor_design"]["physical_coordinate_clip"]
    assert clip["lower_bound"] == "actor_coordinate_at_observation"
    assert clip["upper_bound_m"] == 3.5
    assert contract["authority"]["held_out_test_access_authorized"] is False
