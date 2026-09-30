#!/usr/bin/env python3
"""Tests for the R16-D1 offline diagnosis."""

import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_d1_execution_diagnosis_contract.yaml"
)
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r16_d1_execution_diagnosis.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r16_d1_builder_test", BUILDER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_diagnosis_is_offline_and_does_not_change_r16():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"]["pure_offline_diagnosis_authorized"] is True
    assert contract["authority"]["new_simulation_units_authorized"] == 0
    assert contract["downstream"][
        "actual_TEB_qualification_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False


def test_diagnosis_attributes_goal_and_terminal_execution_defect():
    builder = load_builder()
    value = yaml.safe_load(
        builder.build_outputs()[builder.DIAGNOSIS]
    )
    assert value["attribution_confirmed"] is True
    assert value["minimum_candidate_path_goal_distance_m"] > 0.25
    assert value["minimum_robot_trace_terminal_distance_m"] < 0.05
    assert value["maximum_tracking_error_m"] > 0.45
    assert value["minimum_conservative_clearance_m"] >= 0.0


def test_repair_keeps_thresholds_and_requires_fresh_support():
    builder = load_builder()
    value = yaml.safe_load(
        builder.build_outputs()[builder.DIAGNOSIS]
    )
    requirements = value["repair_requirements"]
    assert requirements["goal_aligned_recovery_candidate"] is True
    assert requirements["terminal_completion_guard"] is True
    assert requirements[
        "goal_timeout_or_safety_threshold_relaxation"
    ] is False
    assert requirements[
        "fresh_candidate_support_and_prediction_qualification"
    ] is True
