#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    ROOT
    / "src/tools/thesis_experiment/scripts/v3_13_multiactor_design.py"
)
CONTRACT_PATH = (
    ROOT
    / "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
SPEC = importlib.util.spec_from_file_location("v3_13_design_test", MODULE_PATH)
DESIGN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DESIGN)
CONTRACT = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))


def test_schedule_has_complete_positive_support_without_old_seeds():
    rows = DESIGN.build_schedule(CONTRACT)
    assert len(rows) == 64
    blocks = {}
    for row in rows:
        blocks.setdefault(row["block_index"], []).append(row)
        assert row["behavior_propensity"] == 0.25
        assert not (6801 <= row["seed"] <= 7130)
    assert len(blocks) == 16
    assert all(
        {row["forced_candidate_id"] for row in block}
        == set(DESIGN.CANDIDATE_IDS)
        for block in blocks.values()
    )


def test_schedule_balances_every_scene_policy_pair():
    rows = DESIGN.build_schedule(CONTRACT)
    keys = {
        (
            row["scene_id"],
            tuple(row["actor_policies_evaluator_only"]),
        )
        for row in rows
    }
    assert len(keys) == 16


def test_world_contains_two_independent_frozen_actor_plugins():
    scene = CONTRACT["scene_population"][0]
    world, actors = DESIGN.render_world(
        scene, ("cautious", "hesitant"), 7601
    )
    assert len(actors) == 2
    assert world.count("libv3_reactive_actor_plugin.so") == 2
    assert actors[0]["model_name"] != actors[1]["model_name"]
    assert actors[0]["plugin_seed"] != actors[1]["plugin_seed"]


def test_arrival_matching_handles_later_conflicts():
    scene = CONTRACT["scene_population"][0]
    first = DESIGN.actor_parameters(7601, 1, scene["actor_paths"][0])
    second = DESIGN.actor_parameters(7601, 2, scene["actor_paths"][1])
    assert (
        abs(second["start_coordinate_m"])
        > abs(first["start_coordinate_m"])
    )


def test_oncoming_amendment_is_offset_from_centerline():
    scene = next(
        item
        for item in CONTRACT["scene_population"]
        if item["scene_id"] == "crossing_and_oncoming"
    )
    assert scene["actor_paths"][1]["conflict_y_m"] == 1.35


def test_authority_is_bounded_simulation_only():
    authority = CONTRACT["authority"]
    assert authority["exact_bounded_simulation_collection_authorized"]
    assert authority["authorized_preflight_units"] == 1
    assert authority["authorized_evidence_units"] == 64
    assert authority["authorized_total_units"] == 65
    assert not authority["training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]
    assert not authority["stable_workspace_write_authorized"]


def test_preflight_identity_cannot_enter_evidence():
    preflight = CONTRACT["integration_preflight"]
    assert preflight["seed"] == 7299
    assert preflight["exact_units"] == 1
    assert not preflight["evidence_identity"]
    assert preflight[
        "result_cannot_enter_predictor_fit_or_performance_analysis"
    ]
