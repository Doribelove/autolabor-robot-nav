import copy
import csv
import math
import tempfile
from pathlib import Path

import pytest
import yaml

from thesis_experiment import (
    SCENE_FAMILIES,
    V2ContractError,
    V2EvaluationError,
    V2SceneError,
    compile_v2_manifest,
    evaluate_v2_episode,
    load_v2_scene_manifest,
    load_v2_trace,
    load_v2_yaml,
    render_v2_scene_sdf,
    validate_evaluation_contract,
    validate_simulation_contract,
    validate_v2_scene_manifest,
)
from thesis_experiment.v2_evaluator import TRACE_COLUMNS


WORKSPACE = Path(__file__).resolve().parents[4]
SIMULATION_CONTRACT = WORKSPACE / "config/thesis_experiments/v2/simulation_contract.yaml"
EVALUATION_CONTRACT = WORKSPACE / "config/thesis_experiments/v2/evaluation_contract.yaml"
SCENE_MANIFEST = WORKSPACE / "experiments/manifests/v2/scenes/v2_02_foundation_scenes.yaml"


def test_v2_02_machine_contracts_are_valid_but_not_runtime_ready():
    simulation = load_v2_yaml(SIMULATION_CONTRACT)
    evaluation = load_v2_yaml(EVALUATION_CONTRACT)
    assert validate_simulation_contract(simulation) is simulation
    assert validate_evaluation_contract(evaluation) is evaluation
    assert simulation["runtime_ready"] is False
    assert evaluation["runtime_ready"] is False
    assert simulation["claims"]["training_allowed"] is False
    assert simulation["claims"]["real_vehicle_safety_claim_allowed"] is False


def test_simulation_contract_rejects_zero_distance_and_bad_authority_order():
    simulation = load_v2_yaml(SIMULATION_CONTRACT)
    bad_distance = copy.deepcopy(simulation)
    bad_distance["regression_gates"]["braking"]["minimum_stopping_distance_m"] = 0.0
    with pytest.raises(V2ContractError):
        validate_simulation_contract(bad_distance)
    bad_brake = copy.deepcopy(simulation)
    bad_brake["actuator"]["max_brake_deceleration_mps2"] = 0.5
    with pytest.raises(V2ContractError):
        validate_simulation_contract(bad_brake)


def test_evaluation_contract_rejects_policy_label_leakage():
    evaluation = load_v2_yaml(EVALUATION_CONTRACT)
    leaked = copy.deepcopy(evaluation)
    leaked["policy_boundary"]["runtime_policy_reads_manifest_labels"] = True
    with pytest.raises(V2ContractError):
        validate_evaluation_contract(leaked)


def test_scene_manifest_covers_five_families_and_compiles_deterministically():
    manifest = load_v2_scene_manifest(SCENE_MANIFEST, WORKSPACE)
    assert tuple(scene["family"] for scene in manifest["scenes"]) == SCENE_FAMILIES
    first = compile_v2_manifest(manifest, WORKSPACE)
    second = compile_v2_manifest(manifest, WORKSPACE)
    assert first == second
    assert len({instance["instance_sha256"] for instance in first}) == 5
    dynamic = next(instance for instance in first if instance["scene"]["family"] == "DYNAMIC")
    world = render_v2_scene_sdf(dynamic)
    assert "libv2_trajectory_actor_plugin.so" in world
    assert "crossing-agent-1" in world


def test_scene_manifest_rejects_short_cruise_and_runtime_label_topic():
    manifest = yaml.safe_load(SCENE_MANIFEST.read_text(encoding="utf-8"))
    short = copy.deepcopy(manifest)
    short["scenes"][0]["goal"]["x_m"] = 5.0
    with pytest.raises(V2SceneError):
        validate_v2_scene_manifest(short, WORKSPACE)
    leaked = copy.deepcopy(manifest)
    leaked["policy_boundary"]["runtime_label_topics"] = ["/scene/family"]
    with pytest.raises(V2SceneError):
        validate_v2_scene_manifest(leaked, WORKSPACE)


def _row(stamp, x, y, velocity, goal_distance, gear="FORWARD", **changes):
    row = {
        "stamp_s": float(stamp),
        "x_m": float(x),
        "y_m": float(y),
        "yaw_rad": 0.0,
        "linear_velocity_mps": float(velocity),
        "angular_velocity_radps": 0.0,
        "commanded_speed_mps": abs(float(velocity)),
        "clearance_m": 1.0,
        "goal_distance_m": float(goal_distance),
        "collision": False,
        "goal_reached": False,
        "contact_count": 0,
        "topology_id": "topology-a",
        "global_replan_count": 0,
        "recovery_count": 0,
        "gear": gear,
        "predicted_ttc_s": None,
    }
    row.update(changes)
    return row


def _successful_rows(scene):
    start, goal = scene["start"], scene["goal"]
    middle_x = 0.5 * (start["x_m"] + goal["x_m"])
    middle_y = 0.5 * (start["y_m"] + goal["y_m"])
    gear = "REVERSE" if scene["family"] == "MANEUVER" else "FORWARD"
    velocity = -0.3 if gear == "REVERSE" else 0.5
    rows = [
        _row(0.0, start["x_m"], start["y_m"], 0.0,
             math.hypot(goal["x_m"] - start["x_m"], goal["y_m"] - start["y_m"]),
             gear="NEUTRAL"),
        _row(1.0, middle_x, middle_y, velocity,
             math.hypot(goal["x_m"] - middle_x, goal["y_m"] - middle_y),
             gear=gear),
        _row(2.0, goal["x_m"], goal["y_m"], 0.0, 0.0, gear=gear,
             goal_reached=True),
    ]
    if scene["family"] == "DYNAMIC":
        rows[1]["predicted_ttc_s"] = 2.5
        rows[1]["clearance_m"] = 0.8
    return rows


def test_unified_evaluator_emits_family_metrics_for_all_five_families():
    instances = compile_v2_manifest(
        load_v2_scene_manifest(SCENE_MANIFEST, WORKSPACE), WORKSPACE
    )
    reports = []
    for instance in instances:
        report = evaluate_v2_episode(
            instance, _successful_rows(instance["scene"]), "a" * 64
        )
        assert report["formal_result"] is False
        assert report["runtime_ready"] is False
        assert report["termination"] == "SUCCESS"
        assert report["metrics"]["common"]["success"] is True
        assert report["metrics"]["common"]["path_length_m"] > 0.0
        reports.append(report)
    assert {report["family"] for report in reports} == set(SCENE_FAMILIES)
    assert "unnecessary_deceleration_count" in reports[0]["metrics"]["family"]
    assert "minimum_predicted_ttc_s" in reports[1]["metrics"]["family"]
    assert "topology_switch_count" in reports[2]["metrics"]["family"]
    assert "lateral_rms_m" in reports[3]["metrics"]["family"]
    assert "gear_switch_count" in reports[4]["metrics"]["family"]


def test_evaluator_collision_precedes_goal_and_detects_instance_corruption():
    instance = compile_v2_manifest(
        load_v2_scene_manifest(SCENE_MANIFEST, WORKSPACE), WORKSPACE
    )[0]
    rows = _successful_rows(instance["scene"])
    rows[-1]["collision"] = True
    report = evaluate_v2_episode(instance, rows, "b" * 64)
    assert report["termination"] == "COLLISION"
    assert report["metrics"]["common"]["success"] is False
    corrupted = copy.deepcopy(instance)
    corrupted["scene"]["seed"] += 1
    with pytest.raises(V2EvaluationError):
        evaluate_v2_episode(corrupted, rows, "b" * 64)


def test_trace_loader_is_strict_about_columns_time_and_gears():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "trace.csv"
        rows = [
            _row(0.0, 0.0, 0.0, 0.0, 1.0, gear="NEUTRAL"),
            _row(1.0, 1.0, 0.0, 0.0, 0.0, goal_reached=True),
        ]
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        assert len(load_v2_trace(path)) == 2
        rows[1]["stamp_s"] = 0.0
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        with pytest.raises(V2EvaluationError):
            load_v2_trace(path)
