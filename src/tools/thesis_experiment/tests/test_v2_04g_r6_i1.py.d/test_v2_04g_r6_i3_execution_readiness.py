"""Offline directed tests for the R6-I3 execution-readiness closure."""

import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_dependency import (
    COMPILED_SCENE_INDEX,
    EXECUTION_CLOSURE,
    EXECUTION_RELEASE,
    EXPECTED_RUNTIME_BINDINGS,
    R6I3DependencyError,
    build_dependency_closure,
    verify_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
REVIEWER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_reviewer.py"
)
RUNNER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py"
)
RELEASE_VALIDATOR_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_release.py"
)
ARTIFACT_ROOT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)


def _load_module(path, name):
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def test_review_is_deterministic_and_non_authorizing():
    reviewer = _load_module(REVIEWER_PATH, "r6_i3_readiness_reviewer_test")
    first = reviewer.build_review(WORKSPACE)
    second = reviewer.build_review(WORKSPACE)
    assert first == second
    assert first["review_result"] == "pass"
    assert first["status"] == "execution_readiness_closure_pass_release_absent"
    assert first["execution_ready"] is False
    assert first["separate_execution_release_present"] is False
    assert first["side_effects"]["evidence_units_consumed"] == 0
    release = _load_module(RELEASE_VALIDATOR_PATH, "r6_i3_release_review_contract_test")
    release._validate_machine_review(first, first["status"])


def test_dependency_closure_rebuilds_and_has_actual_runtime_bindings():
    persisted = yaml.safe_load((WORKSPACE / EXECUTION_CLOSURE).read_text())
    assert persisted == build_dependency_closure(WORKSPACE)
    receipt = verify_dependency_closure(WORKSPACE, persisted)
    assert receipt["compiled_scene_child_count"] == 14
    assert receipt["external_runtime_binding_count"] == len(EXPECTED_RUNTIME_BINDINGS)
    assert receipt["unresolved_count"] == 0
    assert persisted["inherited_i2_revalidation"] == {
        "closure_path": (
            "artifacts/v2/integration/v2_04g_r6_i1/"
            "r6_i2_repair_review/execution_dependency_closure.yaml"
        ),
        "closure_sha256": (
            "2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6"
        ),
        "local_file_count": 106,
        "external_file_count": 301,
        "external_python_binding_count": 45,
        "external_runtime_binding_count": 5,
        "unresolved_count": 0,
        "all_targets_mechanically_rehashed": True,
        "pass": True,
    }
    names = [row["binding"] for row in persisted["external"]["runtime_bindings"]]
    assert names == list(EXPECTED_RUNTIME_BINDINGS)
    for name in (
        "command-executable:roslaunch",
        "command-executable:rosservice",
        "command-executable:rostopic",
        "node:gazebo_ros:gzserver",
    ):
        assert name in names


def test_dependency_closure_rejects_logical_drift():
    persisted = yaml.safe_load((WORKSPACE / EXECUTION_CLOSURE).read_text())
    drifted = copy.deepcopy(persisted)
    drifted["evidence_budget_consumed"] = 1
    with pytest.raises(R6I3DependencyError):
        verify_dependency_closure(WORKSPACE, drifted)


def test_fresh_index_binds_fourteen_regular_children():
    index = yaml.safe_load((WORKSPACE / COMPILED_SCENE_INDEX).read_text())
    assert index["scene_count"] == 7
    assert len(index["files"]) == 14
    for row in index["files"]:
        path = WORKSPACE / row["path"]
        assert path.is_file()
        assert not path.is_symlink()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == row["sha256"]


def test_runner_has_stdlib_only_top_level_and_release_first_markers():
    source = RUNNER_PATH.read_text()
    tree = ast.parse(source)
    imports = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "").split(".")[0])
    assert not (
        set(imports)
        & {"rospy", "roslaunch", "yaml", "thesis_experiment", "dynamic_reconfigure"}
    )
    for marker in (
        "caller_supplied_exact_release_sha256",
        "R6I2PositiveClockBarrier",
        "/gazebo/unpause_physics",
        "/move_base/TebLocalPlannerROS/set_parameters",
    ):
        assert marker in source


def test_runner_offline_review_binds_current_validator_and_runtime_keys(capsys):
    runner = _load_module(RUNNER_PATH, "r6_i3_runner_readiness_contract_test")
    assert (
        hashlib.sha256(RELEASE_VALIDATOR_PATH.read_bytes()).hexdigest()
        == runner.EXPECTED_RELEASE_VALIDATOR_SHA256
    )
    assert runner.offline_review() == 0
    receipt = json.loads(capsys.readouterr().out)
    assert receipt["status"] == "runner_offline_review_pass_execution_release_absent"
    closure = yaml.safe_load((WORKSPACE / EXECUTION_CLOSURE).read_text())
    runtime = {
        row["binding"]: row["target_canonical_path"]
        for row in closure["external"]["runtime_bindings"]
    }
    runtime["python_interpreter"] = closure["external"]["python_interpreter"][
        "canonical_path"
    ]
    validation = type("Validation", (), {"runtime_executables": runtime})()
    for binding in (
        runner.ROSLAUNCH_BINDING,
        runner.ROSSERVICE_BINDING,
        runner.ROSTOPIC_BINDING,
        runner.XACRO_BINDING,
    ):
        assert Path(runner._runtime_executable(validation, binding)).is_file()


def test_missing_release_fails_before_execution_state_creation():
    runner = _load_module(RUNNER_PATH, "r6_i3_runner_missing_release_test")
    before = {
        "attempts": (ARTIFACT_ROOT / "attempts").exists(),
        "journals": (ARTIFACT_ROOT / "journals").exists(),
        "stage_report": (ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml").exists(),
    }
    with pytest.raises(Exception):
        runner.execute(
            runner.RELEASE_RELATIVE.as_posix(),
            "0" * 64,
            runner.AUTHORIZATION_RELATIVE.as_posix(),
            runner.EXPECTED_AUTHORIZATION_SHA256,
        )
    after = {
        "attempts": (ARTIFACT_ROOT / "attempts").exists(),
        "journals": (ARTIFACT_ROOT / "journals").exists(),
        "stage_report": (ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml").exists(),
    }
    assert before == after == {"attempts": False, "journals": False, "stage_report": False}


def test_release_attempt_and_journal_state_remain_absent():
    assert not (WORKSPACE / EXECUTION_RELEASE).exists()
    assert not (ARTIFACT_ROOT / "attempts").exists()
    assert not (ARTIFACT_ROOT / "journals").exists()
    assert not (ARTIFACT_ROOT / "receipts").exists()
    assert not (ARTIFACT_ROOT / "raw_evidence").exists()
