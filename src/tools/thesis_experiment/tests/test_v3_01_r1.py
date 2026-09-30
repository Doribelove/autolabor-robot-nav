import importlib.util
from pathlib import Path


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"
)
SPEC = importlib.util.spec_from_file_location("v3_01_r1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_repair_is_single_factor_and_schedule_is_fresh():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert len(schedule["rows"]) == 36
    assert {row["seed"] for row in schedule["rows"]} == {6111, 6112, 6113}
    assert all("_r1_" in row["unit_id"] for row in schedule["rows"])
    assert contract["single_factor_repair"]["factor_count"] == 1


def test_repaired_world_matches_parent_scene_semantics():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    row = V3.exact_schedule(contract)["rows"][0]
    world = V3.render_world(
        contract,
        row,
        Path("/tmp/v3_01_r1/raw.csv"),
        Path("/tmp/v3_01_r1/done"),
    )
    assert "libv3_reactive_actor_plugin.so" in world
    assert "libv3_episode_recorder_plugin.so" in world
    assert "<duration>8.000000000</duration>" in world
