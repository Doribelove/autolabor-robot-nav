import importlib.util
from pathlib import Path
import tempfile

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)
SPEC = importlib.util.spec_from_file_location("v3_01_qualification", MODULE_PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def load_contracts():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    return contract, prereg


def test_contract_and_schedule_are_exact():
    contract, prereg = load_contracts()
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert schedule["row_count"] == 36
    assert len({row["unit_id"] for row in schedule["rows"]}) == 36
    assert {row["seed"] for row in schedule["rows"]} == {6101, 6102, 6103}


def test_world_never_exposes_candidate_identity_to_actor_plugin():
    contract, _ = load_contracts()
    row = V3.exact_schedule(contract)["rows"][0]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        world = V3.render_world(
            contract, row, root / "raw.csv", root / "done.marker"
        )
    actor_plugin = world.split(
        '<plugin name="candidate_blind_reactive_actor"', 1
    )[1].split("</plugin>", 1)[0]
    assert "candidate_id" not in actor_plugin
    assert "ego_speed_mps" not in actor_plugin
    assert "<ego_model>ego_probe</ego_model>" in actor_plugin
    assert "<policy>cautious</policy>" in actor_plugin


def test_world_is_deterministic():
    contract, _ = load_contracts()
    row = V3.exact_schedule(contract)["rows"][17]
    root = Path("/tmp/v3_01_determinism")
    first = V3.render_world(contract, row, root / "raw.csv", root / "done")
    second = V3.render_world(contract, row, root / "raw.csv", root / "done")
    assert first == second


def test_duplicate_yaml_keys_fail_closed(tmp_path):
    path = tmp_path / "duplicate.yaml"
    path.write_text("a: 1\na: 2\n", encoding="utf-8")
    try:
        V3.load_yaml_path(path)
    except V3.V301Error:
        pass
    else:
        raise AssertionError("duplicate YAML was accepted")
