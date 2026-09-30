import importlib.util
from pathlib import Path
import tempfile


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
)
SPEC = importlib.util.spec_from_file_location("v3_02_predictor", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def contracts():
    return (
        V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE),
        V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE),
    )


def test_schedule_split_and_held_out_firewall():
    contract, prereg = contracts()
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    assert schedule["row_count"] == 108
    assert sum(row["role_evaluator_only"] == "training" for row in schedule["rows"]) == 72
    assert sum(row["role_evaluator_only"] == "calibration" for row in schedule["rows"]) == 36
    assert not ({6221, 6222, 6223} & {row["seed"] for row in schedule["rows"]})


def test_world_actor_plugin_has_no_candidate_or_role_label():
    contract, _ = contracts()
    row = V3.exact_schedule(contract)["rows"][0]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        world = V3.render_world(contract, row, root / "raw.csv", root / "done")
    actor = world.split(
        '<plugin name="candidate_blind_reactive_actor"', 1
    )[1].split("</plugin>", 1)[0]
    assert "candidate_id" not in actor
    assert "role" not in actor
    assert "<ego_model>ego_probe</ego_model>" in actor


def test_candidate_feature_is_explicit():
    low = V3.feature_vector(-2.5, 1.0, 0.6)
    high = V3.feature_vector(-2.5, 1.0, 1.2)
    assert low != high
    assert len(low) == 7
