#!/usr/bin/env python3
"""Static and regression tests for R14-R1."""

import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_r1_progress_predictor_qualification_contract.yaml"
)
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r14_r1_progress_qualification.py"
)


def test_only_stage_compatibility_is_repaired():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    repair = contract["repair_scope"]
    assert repair["only_repaired_factor"] == (
        "released_summary_stage_and_status_compatibility"
    )
    assert repair["predictor_model_changed"] is False
    assert repair["action_bank_changed"] is False
    assert repair["scenes_or_actor_policies_changed"] is False
    assert repair["matching_or_predictor_thresholds_changed"] is False


def test_fresh_population_does_not_reuse_terminal_r14():
    population = yaml.safe_load(CONTRACT.read_bytes())[
        "qualification_population"
    ]
    assert population["seeds"] == list(range(7658, 7666))
    assert set(population["seeds"]).isdisjoint(range(7650, 7658))
    assert population["exact_units"] == 40
    assert population["behavior_propensity_each"] == 0.20


def test_thresholds_equal_terminal_r14_contract():
    current = yaml.safe_load(CONTRACT.read_bytes())
    previous = yaml.safe_load(
        (
            W
            / "config/thesis_experiments/v3/"
            "v3_13_q1_r14_progress_predictor_qualification_contract.yaml"
        ).read_bytes()
    )
    assert current["matching_gates"] == previous["matching_gates"]
    assert current["predictor_gates"] == previous["predictor_gates"]
    left = dict(current["qualification_population"])
    right = dict(previous["qualification_population"])
    left.pop("seeds")
    right.pop("seeds")
    assert left == right


def test_repaired_assessor_replays_terminal_r14_without_summary_mismatch():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r14_r1_assessor_regression", ASSESSOR
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    assessor.BASE.CONTRACT = Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r14_progress_predictor_qualification_contract.yaml"
    )
    assessor.BASE.READINESS = Path(
        "artifacts/v3/multiactor/v3_13_q1_r14_readiness"
    )
    assessor.BASE.EXECUTION = Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r14_progress_qualification"
    )
    outputs = assessor.build_outputs()
    assessment = yaml.safe_load(
        outputs[assessor.BASE.ASSESSMENT]
    )
    assert assessment[
        "released_summary_stage_compatibility_repaired"
    ] is True
    assert assessment["trajectory_count"] == 80


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
