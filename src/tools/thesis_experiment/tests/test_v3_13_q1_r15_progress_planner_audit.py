#!/usr/bin/env python3
"""Tests for the R15 offline progress-planner audit."""

import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r15_progress_planner_integration_contract.yaml"
)
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r15_progress_planner_audit.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r15_builder_test", BUILDER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_audit_is_offline_and_training_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"]["pure_offline_audit_authorized"] is True
    assert contract["authority"]["new_simulation_units_authorized"] == 0
    assert contract["authority"]["actual_TEB_execution_authorized"] is False
    assert contract["authority"]["training_authorized"] is False


def test_safety_gates_bind_selected_clone_truth():
    gates = yaml.safe_load(CONTRACT.read_bytes())["integration_gates"]
    assert gates["selected_clone_minimum_dynamic_clearance_m_min"] == 0.0
    assert gates["selected_clone_minimum_static_clearance_m_min"] == 0.0
    assert gates["selected_clone_contact_state_count_max"] == 0
    assert gates["no_safe_candidate_exception_count_max"] == 0


def test_all_eight_rows_materialize_and_select_accepted_progress():
    builder = load_builder()
    output = yaml.safe_load(
        builder.build_outputs()[builder.AUDIT]
    )
    assert output["row_count"] == 8
    assert output["planner_integration_pass"] is True
    assert output["selected_candidate_counts"] == {
        "recovery_evasive_right_v120_r2": 8
    }
    assert all(
        row["selected_candidate_accepted"] for row in output["rows"]
    )


def test_hidden_labels_are_rejected_by_runtime_core():
    builder = load_builder()
    planner_contract = builder.load(builder.D0)
    geometry = builder.load(builder.GEOMETRY)
    model = builder.load(builder.MODEL)
    root = builder.source_roots()[1][builder.PROGRESS_ID]
    report = builder.load(root / "episode_report.yaml")
    scene = builder.load(root / "scene_spec.yaml")
    state = builder.snapshot(report, scene, planner_contract)
    state["actors"][0]["hidden_type"] = "forbidden"
    try:
        builder.core.plan(
            state, model, planner_contract, geometry, True
        )
    except builder.core.base.MultiActorPlannerError as error:
        assert str(error) == "HIDDEN_POLICY_RUNTIME_LEAK"
    else:
        raise AssertionError("hidden actor label was accepted")
