import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"


def test_v3_08_a1_fresh_closed_schedule():
    spec = importlib.util.spec_from_file_location("v3_08_a1_test", M)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    contract = module.load_yaml_relative(W, module.CONTRACT_RELATIVE)
    preregistration = module.load_yaml_relative(W, module.PREREGISTRATION_RELATIVE)
    module.validate_contract(contract, preregistration)
    schedule = module.exact_schedule(contract)
    assert len(schedule["rows"]) == 18
    assert sorted({row["seed"] for row in schedule["rows"]}) == list(range(6671, 6677))
    assert all(row["attempt"] == 1 for row in schedule["rows"])
