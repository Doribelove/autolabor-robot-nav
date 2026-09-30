import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_b1.py"
SPEC = importlib.util.spec_from_file_location("v3_02_b1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_actual_mode_score_and_fresh_test_firewall():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    assert V3.CONFORMAL_SCORE == "actual_mode_absolute_residual"
    assert contract["authority"]["fresh_future_test_seeds_reserved"] == [
        6231,
        6232,
        6233,
    ]
