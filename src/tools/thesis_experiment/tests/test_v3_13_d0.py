from pathlib import Path
import importlib.util

import pytest
import yaml


W = Path("/home/robot/robot_ws_base_rl")
CORE_PATH = (
    W
    / "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core.py"
)
CONTRACT_PATH = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_13_d0_foundation.py"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CORE = load(CORE_PATH, "v3_13_d0_core_test")
BUILDER = load(BUILDER_PATH, "v3_13_d0_builder_test")
CONTRACT = yaml.safe_load(CONTRACT_PATH.read_text())


def test_four_candidates_are_unique_valid_and_include_one_stop():
    value = CORE.generate_candidate_set(BUILDER.base_snapshot(), CONTRACT)
    identifiers = [item["candidate_id"] for item in value["candidates"]]
    assert len(identifiers) == len(set(identifiers)) == 4
    assert sum(item["certified_stop"] for item in value["candidates"]) == 1
    assert all(
        not CORE.validate_candidate(item, CONTRACT)
        for item in value["candidates"]
    )


def test_spatial_candidates_are_exact_mirrors():
    value = CORE.generate_candidate_set(BUILDER.base_snapshot(), CONTRACT)
    by_id = {item["candidate_id"]: item for item in value["candidates"]}
    left = by_id["spatial_left_v120"]["points"]
    right = by_id["spatial_right_v120"]["points"]
    assert len(left) == len(right)
    for a, b in zip(left, right):
        assert a["x_m"] == pytest.approx(b["x_m"], abs=1e-12)
        assert a["y_m"] == pytest.approx(-b["y_m"], abs=1e-12)
        assert a["yaw_rad"] == pytest.approx(-b["yaw_rad"], abs=1e-12)
        assert a["steering_angle_rad"] == pytest.approx(
            -b["steering_angle_rad"], abs=1e-12
        )


def test_actor_order_does_not_change_evaluation_or_decision():
    snapshot = BUILDER.base_snapshot("order_test")
    forward = CORE.plan(snapshot, CONTRACT["offline_fixture_model"], CONTRACT)
    reverse = dict(snapshot)
    reverse["actors"] = list(reversed(snapshot["actors"]))
    backward = CORE.plan(reverse, CONTRACT["offline_fixture_model"], CONTRACT)
    assert forward["decision"]["selected_candidate_id"] == backward[
        "decision"
    ]["selected_candidate_id"]
    assert [
        (x["candidate_id"], x["accepted"], x["rejection_codes"])
        for x in forward["evaluations"]
    ] == [
        (x["candidate_id"], x["accepted"], x["rejection_codes"])
        for x in backward["evaluations"]
    ]


def test_hidden_policy_label_is_rejected_from_runtime_actor():
    snapshot = BUILDER.base_snapshot("hidden_label")
    snapshot["actors"][0]["actor_policy"] = "cautious"
    with pytest.raises(
        CORE.MultiActorPlannerError, match="HIDDEN_POLICY_RUNTIME_LEAK"
    ):
        CORE.plan(snapshot, CONTRACT["offline_fixture_model"], CONTRACT)


def test_stale_track_fails_closed():
    snapshot = BUILDER.base_snapshot("stale")
    snapshot["actors"][1]["age_s"] = 0.25
    with pytest.raises(
        CORE.MultiActorPlannerError, match="NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP"
    ):
        CORE.plan(snapshot, CONTRACT["offline_fixture_model"], CONTRACT)


def test_differential_fixtures_exercise_distinct_actions():
    results = {}
    for snapshot in BUILDER.fixture_snapshots():
        if snapshot["scene_snapshot_id"].endswith("stale_track_fixture"):
            continue
        result = CORE.plan(
            snapshot, CONTRACT["offline_fixture_model"], CONTRACT
        )
        results[snapshot["scene_snapshot_id"].replace("v3_13_d0_", "")] = (
            result["decision"]["selected_candidate_id"]
        )
    assert results["left_blocked_fixture"] == "spatial_right_v120"
    assert results["right_blocked_fixture"] == "spatial_left_v120"
    assert results["narrow_corridor_fixture"] == "straight_v120"
    assert (
        results["joint_actor_aggregation_fixture"]
        == "certified_stop_v000"
    )


def test_joint_actor_aggregation_attributes_opposite_spatial_blocks():
    snapshot = next(
        item
        for item in BUILDER.fixture_snapshots()
        if item["scene_snapshot_id"].endswith(
            "joint_actor_aggregation_fixture"
        )
    )
    candidates = {
        item["candidate_id"]: item
        for item in CORE.generate_candidate_set(snapshot, CONTRACT)[
            "candidates"
        ]
    }
    expected = {
        1: ("spatial_left_v120", "spatial_right_v120"),
        2: ("spatial_right_v120", "spatial_left_v120"),
    }
    for actor in snapshot["actors"]:
        one_actor = dict(snapshot)
        one_actor["actors"] = [actor]
        blocked, clear = expected[actor["track_id"]]
        assert not CORE.evaluate_candidate(
            one_actor,
            candidates[blocked],
            CONTRACT["offline_fixture_model"],
            CONTRACT,
        )["accepted"]
        assert CORE.evaluate_candidate(
            one_actor,
            candidates[clear],
            CONTRACT["offline_fixture_model"],
            CONTRACT,
        )["accepted"]


def test_clone_schema_requires_positive_full_support():
    candidate_ids = [
        item["candidate_id"]
        for item in CONTRACT["candidate_generator"]["candidates"]
    ]
    rows = [
        {
            "candidate_id": candidate_id,
            "initial_state_digest": "same",
            "behavior_propensity": 0.25,
            "evaluator_hidden_type": "cc",
            "hidden_policy_runtime_input": False,
        }
        for candidate_id in candidate_ids
    ]
    receipt = CORE.validate_cloned_intervention_block(rows, candidate_ids)
    assert receipt["full_action_support"] is True
    assert receipt["minimum_propensity"] == pytest.approx(0.25)
    rows[0]["behavior_propensity"] = 0.0
    with pytest.raises(CORE.MultiActorPlannerError, match="ZERO_PROPENSITY"):
        CORE.validate_cloned_intervention_block(rows, candidate_ids)


def test_contract_has_zero_execution_and_training_authority():
    authority = CONTRACT["authority"]
    assert authority["ros_or_gazebo_execution_authorized"] is False
    assert authority["evidence_seed_consumption_authorized"] is False
    assert authority["training_authorized"] is False
    assert authority["hardware_device_access_authorized"] is False
    assert authority["real_vehicle_authorized"] is False
