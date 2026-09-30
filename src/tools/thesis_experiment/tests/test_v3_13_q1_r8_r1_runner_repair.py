#!/usr/bin/env python3
"""Static checks for the pre-execution R8-R1 runner repair."""

import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r1_runner_repair_contract.yaml"
)
RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_r1_recovery_qualification.py"
)


def test_only_runner_arity_is_repaired():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    repair = contract["repair"]
    assert repair["failed_release_execute_invoked"] is False
    assert repair["consumed_units_before_repair"] == 0
    assert repair["semantic_change_count"] == 0
    assert repair["gate_change_count"] == 0
    assert repair["model_change_count"] == 0


def test_population_and_gates_equal_frozen_r8():
    repaired = yaml.safe_load(CONTRACT.read_bytes())
    frozen = yaml.safe_load(
        (
            W
            / "config/thesis_experiments/v3/"
            / "v3_13_q1_r8_recovery_predictor_qualification_contract.yaml"
        ).read_bytes()
    )
    assert repaired["qualification_population"] == frozen[
        "qualification_population"
    ]
    assert repaired["matching_gates"] == frozen["matching_gates"]
    assert repaired["predictor_gates"] == frozen["predictor_gates"]


def test_wrapper_normalizes_duplicate_argument():
    spec = importlib.util.spec_from_file_location(
        "r8_r1_runner", RUNNER
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    assert runner.repaired_run_episode.__doc__.startswith("Normalize")
    assert runner.BASE.run_episode is runner.repaired_run_episode


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
