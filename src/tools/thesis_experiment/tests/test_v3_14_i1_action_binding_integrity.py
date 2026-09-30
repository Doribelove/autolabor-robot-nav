#!/usr/bin/env python3
"""Properties for the V3-14-I1 fail-closed action binding."""

import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v3_14_action_binding.py"
)
SPEC = importlib.util.spec_from_file_location("v3_14_action_binding", MODULE)
BINDING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BINDING)
ActionBindingError = BINDING.ActionBindingError
DEFAULT_ALLOWED_CANDIDATES = BINDING.DEFAULT_ALLOWED_CANDIDATES
build_launch_binding_args = BINDING.build_launch_binding_args
verify_preconsumption_binding = BINDING.verify_preconsumption_binding
verify_runtime_report_binding = BINDING.verify_runtime_report_binding
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_14_i1_action_binding_integrity_contract.yaml"
)


def row(candidate="spatial_left_v120"):
    return {
        "sequence": 1,
        "block_index": 1,
        "seed": 0,
        "scene_id": "offline_fixture",
        "forced_candidate_id": candidate,
        "behavior_propensity": 0.20,
    }


@pytest.mark.parametrize("candidate", DEFAULT_ALLOWED_CANDIDATES)
def test_every_candidate_binds_from_row_to_launch_and_report(candidate):
    scheduled = row(candidate)
    command = ["roslaunch"] + build_launch_binding_args(scheduled)
    assert verify_preconsumption_binding(scheduled, command)["binding_pass"]
    assert verify_runtime_report_binding(scheduled, dict(scheduled))[
        "binding_pass"
    ]


def test_schedule_to_launch_candidate_mismatch_fails_closed():
    scheduled = row()
    command = ["roslaunch"] + build_launch_binding_args(scheduled)
    command = [
        (
            "forced_candidate_id:=spatial_right_v120"
            if token.startswith("forced_candidate_id:=")
            else token
        )
        for token in command
    ]
    with pytest.raises(ActionBindingError, match="scheduled/launch"):
        verify_preconsumption_binding(scheduled, command)


def test_missing_and_duplicate_candidate_overrides_fail_closed():
    scheduled = row()
    command = ["roslaunch"] + build_launch_binding_args(scheduled)
    missing = [
        token for token in command
        if not token.startswith("forced_candidate_id:=")
    ]
    duplicate = command + ["forced_candidate_id:=spatial_left_v120"]
    with pytest.raises(ActionBindingError, match="exactly once"):
        verify_preconsumption_binding(scheduled, missing)
    with pytest.raises(ActionBindingError, match="exactly once"):
        verify_preconsumption_binding(scheduled, duplicate)


def test_schedule_to_runtime_candidate_mismatch_fails_closed():
    scheduled = row()
    report = dict(scheduled)
    report["forced_candidate_id"] = "recovery_evasive_right_v120_r3"
    with pytest.raises(ActionBindingError, match="scheduled/runtime"):
        verify_runtime_report_binding(scheduled, report)


def test_historical_failure_bytes_are_bound_and_authority_is_zero():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    historical = contract["historical_failure"]
    for path_key, hash_key in (
        ("terminal_failure_path", "terminal_failure_sha256"),
        ("independent_review_path", "independent_review_sha256"),
        ("matrix_runner_path", "matrix_runner_sha256"),
        ("delegated_preflight_runner_path", "delegated_preflight_runner_sha256"),
    ):
        assert hashlib.sha256(
            (W / historical[path_key]).read_bytes()
        ).hexdigest() == historical[hash_key]
    authority = contract["authority"]
    assert authority["offline_only"] is True
    assert authority["evidence_budget_authorized"] == 0
    assert authority["execution_authorized"] is False
    assert authority["training_authorized"] is False
    assert authority["hardware_device_access_authorized"] is False
    assert authority["real_vehicle_authorized"] is False


def test_historical_delegation_and_hardcoded_action_are_detectable():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    historical = contract["historical_failure"]
    matrix = (W / historical["matrix_runner_path"]).read_text()
    delegated = (W / historical["delegated_preflight_runner_path"]).read_text()
    assert "R5.run_episode(row, scene, root, ros_port)" in matrix
    assert (
        '"forced_candidate_id:=recovery_evasive_right_v120_r3"'
        in delegated
    )
