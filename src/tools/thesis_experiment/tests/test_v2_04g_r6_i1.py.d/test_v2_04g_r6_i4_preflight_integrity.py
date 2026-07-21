"""Directed offline tests for the R6-I4 preflight-integrity runner."""

import ast
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
RUNNER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_bounded_validation.py"
)
VALIDATOR_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
)
VALIDATOR_TEST_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i4_release_validator.py"
)
FAILED_RELEASE_PATH = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_release.yaml"
)
FUTURE_RELEASE_PATH = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i4_execution_release.yaml"
)
DEPENDENCY_CLOSURE_PATH = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i4_preflight_repair_review/execution_dependency_closure.yaml"
)
MACHINE_REVIEW_PATH = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i4_preflight_repair_review/"
    "v2_04g_r6_i4_preflight_integrity_readiness_review.yaml"
)
REVIEWER_PATH = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_preflight_integrity_reviewer.py"
)
EXPECTED_FAILED_RELEASE_SHA256 = (
    "5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6"
)


def _load_runner(name):
    specification = importlib.util.spec_from_file_location(name, RUNNER_PATH)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _state_paths(runner):
    return {
        "failed_release": FAILED_RELEASE_PATH.exists(),
        "future_release": FUTURE_RELEASE_PATH.exists(),
        "i3_attempts": (WORKSPACE / runner.I3_ROOT_RELATIVE / "attempts").exists(),
        "i3_journals": (WORKSPACE / runner.I3_ROOT_RELATIVE / "journals").exists(),
        "i3_receipts": (WORKSPACE / runner.I3_ROOT_RELATIVE / "receipts").exists(),
        "i3_raw_evidence": (
            WORKSPACE / runner.I3_ROOT_RELATIVE / "raw_evidence"
        ).exists(),
        "i3_semantic_evidence": (
            WORKSPACE / runner.I3_ROOT_RELATIVE / "semantic_evidence"
        ).exists(),
        "i3_report": (
            WORKSPACE
            / runner.I3_ROOT_RELATIVE
            / "v2_04g_r6_i3_stage_report.yaml"
        ).exists(),
        "i3_execution_report": (
            WORKSPACE
            / runner.I3_ROOT_RELATIVE
            / "v2_04g_r6_i3_execution_report.yaml"
        ).exists(),
        "i4_attempts": (WORKSPACE / runner.I4_ROOT_RELATIVE / "attempts").exists(),
        "i4_journals": (WORKSPACE / runner.I4_ROOT_RELATIVE / "journals").exists(),
        "i4_receipts": (WORKSPACE / runner.I4_ROOT_RELATIVE / "receipts").exists(),
        "i4_raw_evidence": (
            WORKSPACE / runner.I4_ROOT_RELATIVE / "raw_evidence"
        ).exists(),
        "i4_semantic_evidence": (
            WORKSPACE / runner.I4_ROOT_RELATIVE / "semantic_evidence"
        ).exists(),
        "i4_ros_home": (WORKSPACE / runner.I4_ROOT_RELATIVE / "ros_home").exists(),
        "i4_ros_logs": (WORKSPACE / runner.I4_ROOT_RELATIVE / "ros_logs").exists(),
        "i4_report": (
            WORKSPACE
            / runner.I4_ROOT_RELATIVE
            / "v2_04g_r6_i4_stage_report.yaml"
        ).exists(),
        "i4_execution_report": (
            WORKSPACE
            / runner.I4_ROOT_RELATIVE
            / "v2_04g_r6_i4_execution_report.yaml"
        ).exists(),
    }


def test_runner_is_stdlib_only_and_has_no_execute_surface():
    source = RUNNER_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append((node.module or "").split(".")[0])
    assert not (
        set(imports)
        & {"yaml", "rospy", "roslaunch", "thesis_experiment", "dynamic_reconfigure"}
    )
    assert 'add_argument("--execute"' not in source
    assert "def execute(" not in source
    assert "subprocess" not in imports
    process_function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_process_matches"
    )
    assert "ancestors" not in ast.get_source_segment(source, process_function)


def test_runner_trust_hashes_match_repaired_validator_and_real_roster_tests():
    runner = _load_runner("r6_i4_runner_trust_hash_test")
    assert _sha256(VALIDATOR_PATH) == runner.EXPECTED_REPAIRED_VALIDATOR_SHA256
    assert (
        _sha256(VALIDATOR_TEST_PATH)
        == runner.EXPECTED_REPAIRED_VALIDATOR_TEST_SHA256
    )
    assert len(runner.EXPECTED_RELEASE_RESOURCE_PATHS) == 22
    assert runner.EXPECTED_FAILED_RELEASE_SHA256 == EXPECTED_FAILED_RELEASE_SHA256


def test_offline_preflight_is_deterministic_read_only_and_zero_allocation():
    runner = _load_runner("r6_i4_runner_determinism_test")
    before = _state_paths(runner)
    before_release_sha = _sha256(FAILED_RELEASE_PATH)
    first = runner.offline_preflight()
    second = runner.offline_preflight()
    after = _state_paths(runner)
    assert first == second
    assert json.loads(json.dumps(first, sort_keys=True)) == first
    assert before == after == {
        "failed_release": True,
        "future_release": False,
        "i3_attempts": False,
        "i3_journals": False,
        "i3_receipts": False,
        "i3_raw_evidence": False,
        "i3_semantic_evidence": False,
        "i3_report": False,
        "i3_execution_report": False,
        "i4_attempts": False,
        "i4_journals": False,
        "i4_receipts": False,
        "i4_raw_evidence": False,
        "i4_semantic_evidence": False,
        "i4_ros_home": False,
        "i4_ros_logs": False,
        "i4_report": False,
        "i4_execution_report": False,
    }
    assert before_release_sha == _sha256(FAILED_RELEASE_PATH)
    assert before_release_sha == EXPECTED_FAILED_RELEASE_SHA256
    assert first["status"] == (
        "offline_preflight_integrity_repair_pass_failed_release_preserved"
    )
    assert first["execution_authorized"] is False
    assert first["execution_ready"] is False
    assert first["failed_i3_release"]["authorized_units"] == 6
    assert first["failed_i3_release"]["consumed_units"] == 0
    assert first["failed_i3_release"]["forfeited_units"] == 0
    assert first["i4_allocation"] == {
        "execution_seeds": [],
        "schedule": [],
        "evidence_units_authorized": 0,
        "evidence_units_consumed": 0,
        "evidence_units_forfeited": 0,
    }
    assert first["authorization_resource_audit"]["resource_count"] == 12
    assert first["authorization_resource_audit"]["parsed_count"] == 2
    assert first["authorization_resource_audit"]["hash_only_count"] == 10
    assert first["host_process_isolation"]["pass"] is True
    assert all(value is False for value in first["side_effects"].values())


def test_runner_rejects_trusted_validator_hash_drift(monkeypatch):
    runner = _load_runner("r6_i4_runner_hash_drift_test")
    monkeypatch.setitem(
        runner.TRUSTED_SOURCE_SHA256,
        runner.REPAIRED_VALIDATOR_RELATIVE,
        "0" * 64,
    )
    with pytest.raises(
        runner.R6I4OfflinePreflightError,
        match="trusted source SHA256 drifted",
    ):
        runner.offline_preflight()


@pytest.mark.parametrize(
    "label",
    [
        "future_i4_release",
        "i3_attempts",
        "i3_journals",
        "i3_receipts",
        "i3_raw_evidence",
        "i3_semantic_evidence",
        "i3_stage_report",
        "i3_execution_report",
        "i4_attempts",
        "i4_journals",
        "i4_receipts",
        "i4_raw_evidence",
        "i4_semantic_evidence",
        "i4_ros_home",
        "i4_ros_logs",
        "i4_stage_report",
        "i4_execution_report",
    ],
)
def test_each_forbidden_execution_state_fails_closed(label):
    runner = _load_runner("r6_i4_state_gate_{}_test".format(label))
    state = runner._state_snapshot()
    state[label] = True
    with pytest.raises(
        runner.R6I4OfflinePreflightError,
        match="offline execution-state gate failed",
    ):
        runner._require_expected_state(state)


def test_failed_release_absence_fails_closed():
    runner = _load_runner("r6_i4_failed_release_absence_test")
    state = runner._state_snapshot()
    state["failed_i3_release"] = False
    with pytest.raises(
        runner.R6I4OfflinePreflightError,
        match="offline execution-state gate failed",
    ):
        runner._require_expected_state(state)


def test_forbidden_process_match_fails_before_validation(monkeypatch):
    runner = _load_runner("r6_i4_process_match_test")
    monkeypatch.setattr(
        runner,
        "_process_matches",
        lambda: [
            {
                "pid": 123,
                "executable_basename": "roslaunch",
                "policy_label": (
                    "forbidden_ros_gazebo_move_base_or_training_process"
                ),
            }
        ],
    )
    with pytest.raises(
        runner.R6I4OfflinePreflightError,
        match="host process isolation failed",
    ):
        runner.offline_preflight()


def test_proc_enumeration_failure_is_fail_closed(monkeypatch):
    runner = _load_runner("r6_i4_proc_failure_test")
    original = runner.Path.iterdir

    def blocked(path):
        if path.as_posix() == "/proc":
            raise PermissionError("synthetic /proc denial")
        return original(path)

    monkeypatch.setattr(runner.Path, "iterdir", blocked)
    with pytest.raises(
        runner.R6I4OfflinePreflightError,
        match="cannot enumerate /proc for host isolation",
    ):
        runner._process_matches()


def test_dependency_closure_rebuild_is_deterministic_and_process_free(
    monkeypatch,
):
    from thesis_experiment import (
        v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency as dependency,
    )
    from thesis_experiment import v2_04g_r6_i1_r6_i2_dependency as i2_dependency

    def forbidden_subprocess(*args, **kwargs):
        raise AssertionError("dependency closure attempted a subprocess")

    monkeypatch.setattr(i2_dependency.subprocess, "run", forbidden_subprocess)
    first = dependency.build_dependency_closure(WORKSPACE)
    second = dependency.build_dependency_closure(WORKSPACE)
    assert first == second
    assert first["execution_seeds"] == []
    assert first["exact_schedule"] == []
    assert first["seed_or_evidence_units_allocated"] == 0
    assert first["authorization_resource_audit"]["resource_count"] == 12
    assert first["inherited_i3_revalidation"]["local_file_count"] == 136
    assert first["inherited_i3_revalidation"]["external_file_count"] == 307
    assert first["inherited_i3_revalidation"]["external_runtime_binding_count"] == 9
    assert first["inherited_i2_revalidation"]["local_file_count"] == 106
    assert first["inherited_i2_revalidation"]["external_file_count"] == 301
    assert first["inherited_i2_revalidation"]["external_runtime_binding_count"] == 5
    assert first["unresolved"] == []
    assert first["state_boundary"]["failed_i3_release_present"] is True
    assert first["state_boundary"]["future_i4_release_present"] is False


def test_persisted_closure_and_machine_review_rebuild_exactly():
    from thesis_experiment import (
        v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency as dependency,
    )

    assert DEPENDENCY_CLOSURE_PATH.is_file()
    assert MACHINE_REVIEW_PATH.is_file()
    closure = yaml.safe_load(
        DEPENDENCY_CLOSURE_PATH.read_text(encoding="utf-8")
    )
    receipt = dependency.verify_dependency_closure(WORKSPACE, closure)
    assert receipt["pass"] is True
    assert receipt["authorization_resource_count"] == 12
    reviewer = importlib.util.spec_from_file_location(
        "r6_i4_persisted_machine_review_test", REVIEWER_PATH
    )
    module = importlib.util.module_from_spec(reviewer)
    reviewer.loader.exec_module(module)
    first = module.build_review(WORKSPACE)
    second = module.build_review(WORKSPACE)
    assert first == second
    persisted = yaml.safe_load(MACHINE_REVIEW_PATH.read_text(encoding="utf-8"))
    assert persisted == first
    assert first["status"] == (
        "preflight_integrity_repair_readiness_closure_pass_"
        "future_release_absent"
    )
    assert first["execution_ready"] is False
    assert first["execution_authorized"] is False
