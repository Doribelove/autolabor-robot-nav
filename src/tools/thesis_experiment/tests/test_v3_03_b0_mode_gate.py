import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core

BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/build_v3_03_b0_mode_gate.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_03_b0_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
CONTRACT = BUILDER.load_yaml(BUILDER.CONTRACT)
EFFECTIVE = BUILDER.effective_contract(CONTRACT)
MODEL = BUILDER.load_yaml(Path(CONTRACT["upstream"]["predictor_path"]))


def snapshot(speed):
    return {
        "scene_snapshot_id": "b0_test",
        "world_model_seq": 1,
        "snapshot_age_s": 0.0,
        "predictor_latency_ms": 0.0,
        "verifier_latency_ms": 0.0,
        "ego": {
            "x_m": -2.6,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "speed_mps": 0.8,
        },
        "actor": {
            "coordinate_m": -2.6,
            "speed_mps": speed,
            "path_angle_deg": 90.0,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


@pytest.mark.parametrize(
    "speed,expected",
    [(0.20, "cautious"), (0.325, "hesitant"), (1.20, "assertive")],
)
def test_observable_speed_selects_one_mode(speed, expected):
    candidate = core.generate_candidate_set(snapshot(speed), EFFECTIVE)[
        "candidates"
    ][1]
    tube = core.predict_occupancy_tube(
        snapshot(speed), candidate, MODEL, EFFECTIVE
    )
    assert tube["inferred_mode"] == expected
    assert tube["active_mode_heads"] == [expected]


@pytest.mark.parametrize(
    "speed",
    [
        CONTRACT["mode_inference"]["cautious_hesitant_threshold_mps"],
        CONTRACT["mode_inference"]["hesitant_assertive_threshold_mps"],
    ],
)
def test_ambiguous_speed_fails_closed(speed):
    result = core.plan(snapshot(float(speed)), MODEL, EFFECTIVE)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert all(
        not item["accepted"]
        for item in result["evaluations"]
        if item["candidate_id"] != "certified_stop_v000"
    )


def test_nonfinite_speed_fails_closed():
    result = core.plan(snapshot(float("nan")), MODEL, EFFECTIVE)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"


def test_effective_contract_preserves_safety_and_removes_hidden_label():
    parent = BUILDER.load_yaml(
        Path(CONTRACT["upstream"]["planner_contract_path"])
    )
    assert EFFECTIVE["hard_verifier"] == parent["hard_verifier"]
    assert EFFECTIVE["candidate_generator"] == parent["candidate_generator"]
    assert (
        EFFECTIVE["predictor_runtime"]["mode_inference"][
            "hidden_policy_runtime_input"
        ]
        is False
    )


def test_builder_replays_deterministically():
    first = BUILDER.build()
    second = BUILDER.build()
    assert first == second
    report = yaml.safe_load(first[BUILDER.REPORT])
    assert report["overall_pass"]
    assert (
        report["retrospective_a3_descriptive_replay"][
            "used_for_gate_or_threshold"
        ]
        is False
    )
