import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"
)
SPEC = importlib.util.spec_from_file_location("v3_01_r2", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_r2_single_factor_and_fresh_schedule():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert {row["seed"] for row in schedule["rows"]} == {6121, 6122, 6123}
    assert len(schedule["rows"]) == 36
    assert contract["single_factor_repair"]["repaired_value"] == 0.0


def test_actor_source_has_zero_threshold_jitter():
    source = (
        WORKSPACE
        / "src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"
    ).read_text(encoding="utf-8")
    assert "const double jitter = 0.0 * v3SignedSeedJitter" in source
