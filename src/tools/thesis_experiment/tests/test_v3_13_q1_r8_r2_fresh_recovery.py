#!/usr/bin/env python3
"""Static checks for fresh R8-R2 recovery."""

import ast
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r2_fresh_runner_recovery_contract.yaml"
)
RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_recovery_qualification.py"
)


def test_fresh_population_and_no_retry():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["qualification_population"]
    assert population["seeds"] == list(range(7633, 7641))
    assert population["exact_units"] == 40
    assert contract["failure_inheritance"][
        "seeds7625_through_7632_reused"
    ] is False
    assert contract["failure_inheritance"][
        "retry_or_resume_of_r8_r1"
    ] is False


def test_semantics_and_gates_equal_original_r8():
    repaired = yaml.safe_load(CONTRACT.read_bytes())
    frozen = yaml.safe_load(
        (
            W
            / "config/thesis_experiments/v3/"
            / "v3_13_q1_r8_recovery_predictor_qualification_contract.yaml"
        ).read_bytes()
    )
    for key in ("matching_gates", "predictor_gates"):
        assert repaired[key] == frozen[key]
    for key in (
        "scene_policy_strata",
        "replicas_per_stratum",
        "branch_target_sim_s",
        "nominal_wait_depth_s",
        "actions",
        "actions_per_block",
        "actor_trajectories",
        "behavior_propensity_each",
    ):
        assert repaired["qualification_population"][key] == frozen[
            "qualification_population"
        ][key]


def test_original_runner_definition_and_call_have_four_arguments():
    tree = ast.parse(RUNNER.read_bytes())
    definitions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "run_episode"
    ]
    assert len(definitions) == 1
    assert len(definitions[0].args.args) == 4
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "run_episode"
    ]
    assert len(calls) == 1
    assert len(calls[0].args) == 4
    assert not calls[0].keywords


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
