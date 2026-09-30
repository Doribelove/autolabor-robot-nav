import copy
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

CONTRACT = yaml.safe_load(
    (
        WORKSPACE
        / "config/thesis_experiments/v3/v3_03_a0_safe_planner_contract.yaml"
    ).read_text(encoding="utf-8")
)
MODEL = yaml.safe_load(
    (
        WORKSPACE
        / "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ).read_text(encoding="utf-8")
)


def snapshot():
    return {
        "scene_snapshot_id": "unit",
        "world_model_seq": 1,
        "snapshot_age_s": 0.05,
        "predictor_latency_ms": 1.0,
        "verifier_latency_ms": 1.0,
        "ego": {"x_m": -2.6, "y_m": 0.0, "yaw_rad": 0.0, "speed_mps": 0.8},
        "actor": {
            "coordinate_m": -2.8,
            "speed_mps": 0.9,
            "path_angle_deg": 90.0,
        },
        "health": {
            "localization_ok": True,
            "transform_ok": True,
            "actor_track_ok": True,
            "command_chain_ok": True,
        },
    }


def test_generated_set_is_valid_deterministic_and_has_backups():
    first = core.generate_candidate_set(snapshot(), CONTRACT)
    second = core.generate_candidate_set(snapshot(), CONTRACT)
    assert first == second
    assert len(first["candidates"]) == 6
    assert sum(item["certified_stop"] for item in first["candidates"]) == 1
    assert sum(item["teb_fallback"] for item in first["candidates"]) == 1
    assert all(
        not core.validate_candidate(item, snapshot(), CONTRACT)
        for item in first["candidates"]
    )


def test_nominal_planner_selects_only_an_accepted_candidate():
    result = core.plan(snapshot(), MODEL, CONTRACT)
    selected = result["decision"]["selected_candidate_id"]
    evaluation = next(
        item for item in result["evaluations"] if item["candidate_id"] == selected
    )
    assert evaluation["accepted"]
    assert selected == "feasible_v060"


@pytest.mark.parametrize(
    "field",
    ["localization_ok", "transform_ok", "actor_track_ok", "command_chain_ok"],
)
def test_health_faults_fail_closed_to_certified_stop(field):
    value = snapshot()
    value["health"][field] = False
    result = core.plan(value, MODEL, CONTRACT)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert result["decision"]["safety_fallback_used"]


def test_digest_tamper_uses_unverified_emergency_stop():
    value = snapshot()
    candidate_set = core.generate_candidate_set(value, CONTRACT)
    candidate_set["candidates"][1]["points"][1]["x_m"] += 0.01
    result = core.evaluate_candidate_set(value, candidate_set, MODEL, CONTRACT)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert result["decision"]["fallback_reason"] == "EMERGENCY_STOP_UNVERIFIED"
    assert not any(item["accepted"] for item in result["evaluations"])


def test_missing_certified_stop_is_terminal():
    value = snapshot()
    candidate_set = core.generate_candidate_set(value, CONTRACT)
    candidate_set["candidates"] = [
        item for item in candidate_set["candidates"] if not item["certified_stop"]
    ]
    body = dict(candidate_set)
    body.pop("candidate_set_digest")
    candidate_set["candidate_set_digest"] = core.digest(body)
    with pytest.raises(core.PlannerError, match="CERTIFIED_STOP_MISSING"):
        core.evaluate_candidate_set(value, candidate_set, MODEL, CONTRACT)


def test_nonfinite_predictor_input_fails_closed():
    value = snapshot()
    value["actor"]["speed_mps"] = float("nan")
    result = core.plan(value, MODEL, CONTRACT)
    assert result["decision"]["selected_candidate_id"] == "certified_stop_v000"
    assert all(
        not item["accepted"]
        for item in result["evaluations"]
        if item["candidate_id"] != "certified_stop_v000"
    )
