import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_b1.py"


def test_v3_07_b1_releases_only_reserved_heldout_support():
    spec = importlib.util.spec_from_file_location("v3_07_b1_test", M)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    contract = module.load_yaml_relative(W, module.CONTRACT_RELATIVE)
    preregistration = module.load_yaml_relative(W, module.PREREGISTRATION_RELATIVE)
    module.validate_contract(contract, preregistration)
    schedule = module.exact_schedule(contract)
    assert len(schedule["rows"]) == 20
    assert schedule["held_out_test_rows"] == 20
    assert sorted({row["seed"] for row in schedule["rows"]}) == [6661, 6662, 6663, 6664]
    assert not any(
        row["actor_policy_evaluator_only"] == "assertive"
        and row["candidate_id_evaluator_only"] == "spatial_left_v120"
        for row in schedule["rows"]
    )
