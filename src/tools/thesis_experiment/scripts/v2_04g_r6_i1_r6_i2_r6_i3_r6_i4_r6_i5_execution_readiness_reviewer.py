#!/usr/bin/env python3
"""Deterministic offline machine review for R6-I5 execution readiness.

The review binds the fresh authority DAG and scene materialization, rebuilds
the complete closure twice, validates the runner's trusted hashes and
prejournal order, executes selected directed monkeypatch regressions in
process, and verifies host/state isolation.  ``build_review`` never writes a
file, creates execution state, or starts a subprocess.  ``main`` may persist
only the canonical machine-review artifact.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
from typing import Mapping

import yaml


sys.dont_write_bytecode = True

from thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency import (
    ARTIFACT_ROOT,
    AUTHORIZATION_PARSED_LABELS,
    COMPILED_SCENE_INDEX,
    ENTRYPOINTS,
    EXECUTION_CLOSURE,
    EXPECTED_AUTHORIZATION_LABELS,
    EXPECTED_AUTHORIZATION_PATHS,
    EXPECTED_FIXED_FILE_SHA256,
    EXPECTED_I5_ADDITIONAL_PYTHON_BINDINGS,
    EXPECTED_I5_EXTERNAL_FILE_COUNT,
    EXPECTED_I5_PYTHON_BINDING_COUNT,
    EXPECTED_I5_RUNTIME_BINDING_COUNT,
    EXPECTED_SCHEDULE_SHA256,
    FAILED_I3_RELEASE,
    FORBIDDEN_EXECUTION_STATE,
    FUTURE_I4_RELEASE,
    FUTURE_I5_RELEASE,
    I4_CLOSURE,
    I4_REVIEW,
    I4_VALIDATOR,
    I5_AUTHORIZATION,
    I5_ASSESSOR,
    I5_ASSESSOR_TEST,
    I5_CONTRACT,
    I5_CONTROL,
    I5_DEPENDENCY,
    I5_EPISODE,
    I5_GENERATOR,
    I5_LISTENER,
    I5_PREREGISTRATION,
    I5_READINESS_TEST,
    I5_RELEASE_VALIDATOR,
    I5_RELEASE_VALIDATOR_TEST,
    I5_REVIEWER,
    I5_RUNNER,
    I5_SCENE_BEHAVIOR_AUDIT,
    I5_SCENE_DERIVATION,
    I5_SCENE_MANIFEST,
    I5_SCENE_MATERIALIZER,
    I5_SCENE_REVIEWER,
    I5_SCENE_TEST,
    I5_TRANSITION,
    MACHINE_REVIEW,
    R6_INTEGRITY,
    I2_BOOTSTRAP,
    STAGE,
    WORKSPACE_LOCAL_SCRIPT_BINDINGS,
    build_dependency_closure,
    verify_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_STAGE = "V2-04G-R6-I4"
EXPECTED_REVIEW_STATUS = "r6_i5_execution_readiness_closure_pass_release_absent"
EXPECTED_AUTHORIZATION_SHA256 = EXPECTED_FIXED_FILE_SHA256[I5_AUTHORIZATION]
EXPECTED_I5_SOURCE_PATHS = {
    I5_CONTRACT,
    I5_PREREGISTRATION,
    I5_AUTHORIZATION,
    I5_TRANSITION,
    I5_SCENE_DERIVATION,
    I5_SCENE_MANIFEST,
    I5_SCENE_BEHAVIOR_AUDIT,
    COMPILED_SCENE_INDEX.as_posix(),
    I5_SCENE_MATERIALIZER,
    I5_SCENE_REVIEWER,
    I5_SCENE_TEST,
    I5_RUNNER,
    I5_LISTENER,
    I5_EPISODE,
    I5_CONTROL,
    I5_RELEASE_VALIDATOR,
    I5_RELEASE_VALIDATOR_TEST,
    I5_ASSESSOR,
    I5_ASSESSOR_TEST,
    I5_READINESS_TEST,
    I5_DEPENDENCY,
    I5_GENERATOR,
    I5_REVIEWER,
}
PROCESS_MARKERS = {
    "roscore",
    "rosmaster",
    "roslaunch",
    "gzserver",
    "gzclient",
    "gazebo",
    "move_base",
    "sac_train",
    "residual_train",
}


class R6I5ReadinessReviewError(ValueError):
    """Raised when an I5 execution-readiness gate fails closed."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise R6I5ReadinessReviewError("duplicate YAML key")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise R6I5ReadinessReviewError(message)


def _safe_parts(relative: Path):
    candidate = Path(relative)
    _require(
        not candidate.is_absolute()
        and bool(candidate.parts)
        and all(part not in {"", ".", ".."} for part in candidate.parts),
        "unsafe workspace-relative path: {}".format(candidate),
    )
    return candidate.parts


def _snapshot(
    workspace: Path, relative: Path, parse_yaml: bool = False
) -> dict:
    """Read one stable regular file with no-follow on every path component."""

    parts = _safe_parts(relative)
    root = Path(workspace).resolve()
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = flags | getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        descriptor = os.open(
            parts[-1], flags | getattr(os, "O_NOFOLLOW", 0), dir_fd=descriptor
        )
        descriptors.append(descriptor)
        before = os.fstat(descriptor)
        _require(
            stat.S_ISREG(before.st_mode),
            "review dependency is not regular: {}".format(relative),
        )
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        payload = b"".join(chunks)
        after = os.fstat(descriptor)
        identity = ("st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")
        _require(
            len(payload) == before.st_size
            and all(getattr(before, key) == getattr(after, key) for key in identity),
            "review dependency changed during read: {}".format(relative),
        )
    except OSError as exc:
        raise R6I5ReadinessReviewError(
            "cannot safely read {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass
    row = {
        "path": Path(relative).as_posix(),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
        "payload": payload,
    }
    if parse_yaml:
        try:
            document = yaml.load(payload.decode("utf-8"), Loader=_UniqueKeyLoader)
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise R6I5ReadinessReviewError(
                "cannot parse {}: {}".format(relative, exc)
            ) from exc
        _require(
            isinstance(document, dict),
            "review document must be a mapping: {}".format(relative),
        )
        row["document"] = document
    return row


def _canonical_json_sha(value) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _state_snapshot(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    rows = {
        "failed_i3_release": os.path.lexists(str(root / FAILED_I3_RELEASE)),
        "future_i4_release": os.path.lexists(str(root / FUTURE_I4_RELEASE)),
        "future_i5_release": os.path.lexists(str(root / FUTURE_I5_RELEASE)),
    }
    for index, relative in enumerate(FORBIDDEN_EXECUTION_STATE, start=1):
        rows["forbidden_{:02d}_{}".format(index, relative.name)] = (
            os.path.lexists(str(root / relative))
        )
    return rows


def _require_state(state: Mapping[str, bool]) -> None:
    expected = {key: False for key in state}
    expected["failed_i3_release"] = True
    _require(
        dict(state) == expected,
        "offline execution-state boundary failed: {}".format(dict(state)),
    )


def _process_matches() -> list:
    """Fail closed on /proc errors and exclude only this reviewer process."""

    current = os.getpid()
    matches = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError as exc:
        raise R6I5ReadinessReviewError(
            "cannot enumerate /proc for host isolation: {}".format(exc)
        ) from exc
    for item in entries:
        if not item.name.isdigit() or int(item.name) == current:
            continue
        try:
            command = (item / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            raise R6I5ReadinessReviewError(
                "cannot inspect process {}: {}".format(item.name, exc)
            ) from exc
        tokens = command.lower().split()
        executable = Path(tokens[0]).name if tokens else ""
        matched = (
            executable in PROCESS_MARKERS
            or any(Path(token).name in PROCESS_MARKERS for token in tokens[:3])
            or "sac_train.py" in command.lower()
            or "residual_train.py" in command.lower()
        )
        if matched:
            matches.append(
                {
                    "pid": int(item.name),
                    "executable_basename": executable or "unknown",
                    "policy_label": (
                        "forbidden_ros_gazebo_move_base_or_training_process"
                    ),
                }
            )
    return sorted(matches, key=lambda row: row["pid"])


def _load_module(path: Path, name: str):
    specification = importlib.util.spec_from_file_location(name, path)
    _require(
        specification is not None and specification.loader is not None,
        "module loader unavailable",
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def _verify_authority(workspace: Path) -> dict:
    paths = (
        I5_CONTRACT,
        I5_PREREGISTRATION,
        I5_AUTHORIZATION,
        I5_TRANSITION,
        I5_SCENE_DERIVATION,
    )
    rows = {}
    for relative in paths:
        snapshot = _snapshot(workspace, Path(relative), parse_yaml=True)
        _require(
            snapshot["sha256"] == EXPECTED_FIXED_FILE_SHA256[relative],
            "I5 authority resource hash drifted: {}".format(relative),
        )
        rows[relative] = snapshot

    contract = rows[I5_CONTRACT]["document"]
    allocation = contract.get("fresh_allocation", {})
    _require(
        contract.get("stage") == STAGE
        and contract.get("source_stage") == SOURCE_STAGE
        and contract.get("independent_stage") is True
        and contract.get("simulation_only") is True
        and contract.get("execution_ready") is False
        and contract.get("authorization_basis", {}).get(
            "explicit_user_full_bounded_simulation_execution_instruction_received"
        )
        is True
        and allocation.get("execution_seeds") == [5161, 5162, 5163]
        and allocation.get("compile_support_only_seeds") == [5164, 5165, 5166, 5167]
        and allocation.get("evidence_units_authorized") == 6
        and allocation.get("attempt_limit_per_identity") == 1
        and allocation.get("retry_allowed") is False
        and allocation.get("resume_allowed") is False
        and allocation.get("seed_replacement_allowed") is False
        and allocation.get("budget_expansion_allowed") is False
        and contract.get("exact_schedule_sha256") == EXPECTED_SCHEDULE_SHA256,
        "I5 contract authority/allocation boundary drifted",
    )

    preregistration = rows[I5_PREREGISTRATION]["document"]
    schedule = preregistration.get("schedule")
    _require(
        preregistration.get("stage") == STAGE
        and preregistration.get("execution_authorized") is False
        and preregistration.get("execution_ready") is False
        and isinstance(schedule, list)
        and len(schedule) == 6
        and _canonical_json_sha(schedule) == EXPECTED_SCHEDULE_SHA256
        and preregistration.get("budget", {}).get("evidence_units_authorizable") == 6
        and preregistration.get("budget", {}).get("attempt_limit_per_identity") == 1,
        "I5 preregistration boundary drifted",
    )
    _require(
        preregistration.get("bound_resources", {}).get("contract")
        == {"path": I5_CONTRACT, "sha256": EXPECTED_FIXED_FILE_SHA256[I5_CONTRACT]},
        "I5 preregistration contract binding drifted",
    )

    authorization = rows[I5_AUTHORIZATION]["document"]
    resources = authorization.get("bound_resources")
    _require(
        authorization.get("stage") == STAGE
        and authorization.get("execution_authorized") is True
        and authorization.get("runtime_ready") is False
        and authorization.get("exact_schedule") == schedule
        and authorization.get("preregistration_schedule_sha256")
        == EXPECTED_SCHEDULE_SHA256
        and authorization.get("dependency_closure_digest")
        == "dceb73df8619849f5b5a0442b739be09815bfc86939a188873c94993fe4d5b74"
        and authorization.get("evidence_budget_authorized") == 6
        and authorization.get("attempt_limit_per_identity") == 1
        and authorization.get("retry_or_resume_allowed") is False
        and authorization.get("seed_replacement_allowed") is False
        and authorization.get("budget_expansion_allowed") is False
        and isinstance(resources, dict)
        and tuple(resources) == EXPECTED_AUTHORIZATION_LABELS,
        "I5 authorization boundary drifted",
    )
    for label in EXPECTED_AUTHORIZATION_LABELS:
        expected = {
            "path": EXPECTED_AUTHORIZATION_PATHS[label],
            "sha256": _snapshot(
                workspace, Path(EXPECTED_AUTHORIZATION_PATHS[label])
            )["sha256"],
        }
        _require(resources[label] == expected, "I5 authorization roster drifted")

    transition = rows[I5_TRANSITION]["document"]
    transition_resources = transition.get("bound_resources", {})
    _require(
        transition.get("stage") == STAGE
        and transition.get("target_stage") == STAGE
        and transition.get("execution_authorized") is True
        and transition.get("execution_ready") is False
        and transition.get("target_allocation", {}).get("execution_seeds")
        == [5161, 5162, 5163]
        and transition.get("target_allocation", {}).get(
            "compile_support_only_seeds"
        )
        == [5164, 5165, 5166, 5167],
        "I5 transition boundary drifted",
    )
    expected_transition_bindings = {
        "contract": {
            "path": I5_CONTRACT,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I5_CONTRACT],
        },
        "preregistration": {
            "path": I5_PREREGISTRATION,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I5_PREREGISTRATION],
        },
        "authorization": {
            "path": I5_AUTHORIZATION,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I5_AUTHORIZATION],
        },
    }
    _require(
        transition_resources == expected_transition_bindings,
        "I5 transition authority DAG drifted",
    )

    derivation = rows[I5_SCENE_DERIVATION]["document"]
    expected_dag = {
        "contract": expected_transition_bindings["contract"],
        "preregistration": expected_transition_bindings["preregistration"],
        "authorization": expected_transition_bindings["authorization"],
        "transition": {
            "path": I5_TRANSITION,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I5_TRANSITION],
        },
    }
    _require(
        derivation.get("stage") == STAGE
        and derivation.get("stage_execution_authorization_present") is True
        and derivation.get("execution_authorized_by_derivation") is False
        and derivation.get("execution_release_present") is False
        and derivation.get("authority_dag") == expected_dag
        and derivation.get("execution_probe_seeds") == [5161, 5162, 5163]
        and derivation.get("compile_support_only_seeds") == [5164, 5165, 5166, 5167],
        "I5 scene derivation authority DAG drifted",
    )
    return {
        "contract": {"path": I5_CONTRACT, "sha256": rows[I5_CONTRACT]["sha256"]},
        "preregistration": {
            "path": I5_PREREGISTRATION,
            "sha256": rows[I5_PREREGISTRATION]["sha256"],
        },
        "authorization": {
            "path": I5_AUTHORIZATION,
            "sha256": rows[I5_AUTHORIZATION]["sha256"],
        },
        "transition": {
            "path": I5_TRANSITION,
            "sha256": rows[I5_TRANSITION]["sha256"],
        },
        "scene_derivation": {
            "path": I5_SCENE_DERIVATION,
            "sha256": rows[I5_SCENE_DERIVATION]["sha256"],
        },
        "exact_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "authorization_resource_count": len(resources),
        "parsed_authorization_labels": sorted(AUTHORIZATION_PARSED_LABELS),
        "execution_authorized": True,
        "execution_ready": False,
        "pass": True,
    }


def _top_level_imports(tree: ast.Module) -> list:
    names = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            names.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            names.append((node.module or "").split(".")[0])
    return sorted(set(names))


def _function(tree: ast.Module, name: str):
    return next(
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    )


def _call_name(node: ast.Call) -> str:
    if isinstance(node.func, ast.Name):
        return node.func.id
    if isinstance(node.func, ast.Attribute):
        return node.func.attr
    return ""


def _verify_runner(workspace: Path) -> dict:
    runner_snapshot = _snapshot(workspace, Path(I5_RUNNER))
    validator_snapshot = _snapshot(workspace, Path(I5_RELEASE_VALIDATOR))
    validator_test_snapshot = _snapshot(workspace, Path(I5_RELEASE_VALIDATOR_TEST))
    assessor_snapshot = _snapshot(workspace, Path(I5_ASSESSOR))
    assessor_test_snapshot = _snapshot(workspace, Path(I5_ASSESSOR_TEST))
    readiness_test_snapshot = _snapshot(workspace, Path(I5_READINESS_TEST))
    bootstrap_snapshot = _snapshot(workspace, Path(I2_BOOTSTRAP))
    integrity_snapshot = _snapshot(workspace, Path(R6_INTEGRITY))
    source = runner_snapshot["payload"].decode("utf-8")
    tree = ast.parse(source, filename=I5_RUNNER)
    imports = _top_level_imports(tree)
    forbidden_imports = sorted(
        set(imports)
        & {
            "actionlib",
            "dynamic_reconfigure",
            "gazebo_msgs",
            "rosgraph",
            "roslaunch",
            "rospy",
            "subprocess",
            "thesis_experiment",
            "yaml",
        }
    )
    _require(forbidden_imports == [], "I5 runner top-level import boundary drifted")
    preflight = _function(tree, "_execution_preflight")
    unit_gate = _function(tree, "_prejournal_unit_gate")
    preflight_calls = {
        _call_name(node)
        for function in (preflight, unit_gate)
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
    }
    forbidden_preflight_calls = {
        "mkdir",
        "Popen",
        "_subprocess_module",
        "_spawn_sanitized",
        "_capture_command",
        "_run_command",
        "_atomic_yaml",
        "_exclusive_bytes",
    }
    _require(
        not preflight_calls & forbidden_preflight_calls,
        "I5 complete prejournal contains mutation or process calls",
    )
    execute = _function(tree, "_execute_validated")
    statements = [
        node
        for node in execute.body
        if not (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        )
    ]
    _require(
        statements
        and isinstance(statements[0], ast.Assign)
        and isinstance(statements[0].value, ast.Call)
        and _call_name(statements[0].value) == "_execution_preflight",
        "I5 execution does not begin with complete prejournal",
    )
    mutation_lines = [
        node.lineno
        for node in ast.walk(execute)
        if isinstance(node, ast.Call) and _call_name(node) == "mkdir"
    ]
    _require(
        mutation_lines and statements[0].value.lineno < min(mutation_lines),
        "I5 execution mutation precedes complete prejournal",
    )
    process_source = ast.get_source_segment(source, _function(tree, "_process_matches"))
    _require(
        "os.getpid()" in process_source
        and "getppid" not in process_source
        and "ancestor" not in process_source.lower(),
        "I5 runner process exclusion is not self-only",
    )

    runner = _load_module(
        Path(workspace) / I5_RUNNER, "v2_04g_r6_i5_runner_machine_review"
    )
    validator = _load_module(
        Path(workspace) / I5_RELEASE_VALIDATOR,
        "v2_04g_r6_i5_release_machine_review",
    )
    _require(
        runner.EXPECTED_AUTHORIZATION_SHA256 == EXPECTED_AUTHORIZATION_SHA256
        and runner.EXPECTED_SCHEDULE_SHA256 == EXPECTED_SCHEDULE_SHA256
        and runner.EXPECTED_MACHINE_REVIEW_STATUS == EXPECTED_REVIEW_STATUS
        and runner.EXPECTED_RELEASE_VALIDATOR_SHA256
        == validator_snapshot["sha256"]
        and runner.EXPECTED_RELEASE_VALIDATOR_TEST_SHA256
        == validator_test_snapshot["sha256"]
        and runner.EXPECTED_ASSESSOR_SHA256 == assessor_snapshot["sha256"]
        and runner.EXPECTED_ASSESSOR_TEST_SHA256
        == assessor_test_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.RELEASE_VALIDATOR_RELATIVE]
        == validator_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.RELEASE_VALIDATOR_TEST_RELATIVE]
        == validator_test_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.ASSESSOR_RELATIVE]
        == assessor_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.ASSESSOR_TEST_RELATIVE]
        == assessor_test_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.I2_BOOTSTRAP_RELATIVE]
        == bootstrap_snapshot["sha256"]
        and runner.TRUSTED_MODULE_SHA256[runner.INTEGRITY_RELATIVE]
        == integrity_snapshot["sha256"]
        and runner.EXPECTED_RELEASE_RESOURCE_PATHS
        == validator.EXPECTED_RELEASE_RESOURCE_PATHS,
        "I5 runner trusted hash/path chain drifted",
    )
    test_tree = ast.parse(
        readiness_test_snapshot["payload"].decode("utf-8"), filename=I5_READINESS_TEST
    )
    test_functions = {
        node.name
        for node in test_tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    required_tests = {
        "test_runner_ast_places_all_mutation_after_complete_prejournal",
        "test_complete_simulated_prejournal_has_no_mkdir_popen_or_ros_import",
        "test_failed_prejournal_cannot_reach_execution_mutation",
        "test_process_scan_excludes_only_self",
        "test_proc_enumeration_failure_is_fail_closed",
        "test_proc_entry_nontransient_failure_is_fail_closed",
        "test_runner_trusted_hashes_match_exact_sources_and_authority",
        "test_dependency_closure_rebuild_is_deterministic_and_process_free",
        "test_persisted_closure_and_machine_review_rebuild_exactly",
        "test_actual_machine_review_builder_passes_release_validator_schema",
    }
    _require(
        required_tests.issubset(test_functions),
        "I5 directed prejournal test source is incomplete",
    )
    return {
        "runner": {"path": I5_RUNNER, "sha256": runner_snapshot["sha256"]},
        "release_validator": {
            "path": I5_RELEASE_VALIDATOR,
            "sha256": validator_snapshot["sha256"],
        },
        "release_validator_tests": {
            "path": I5_RELEASE_VALIDATOR_TEST,
            "sha256": validator_test_snapshot["sha256"],
        },
        "terminal_assessor": {
            "path": I5_ASSESSOR,
            "sha256": assessor_snapshot["sha256"],
        },
        "terminal_assessor_tests": {
            "path": I5_ASSESSOR_TEST,
            "sha256": assessor_test_snapshot["sha256"],
        },
        "directed_readiness_tests": {
            "path": I5_READINESS_TEST,
            "sha256": readiness_test_snapshot["sha256"],
            "required_test_count": len(required_tests),
        },
        "runner_top_level_imports": imports,
        "complete_prejournal_has_no_mutation_or_process_calls": True,
        "execution_first_statement_is_complete_prejournal": True,
        "first_mkdir_after_complete_prejournal": True,
        "dynamic_ros_import_before_complete_prejournal": False,
        "process_scan_excludes_only_self": True,
        "proc_errors_fail_closed": True,
        "trusted_hashes_match_actual_bytes": True,
        "release_resource_paths_match_validator": True,
        "pass": True,
    }


class _AttributePatch:
    """Tiny monkeypatch surface for selected directed in-process tests."""

    def __init__(self):
        self._changes = []

    def setattr(self, target, name, value):
        self._changes.append((target, name, getattr(target, name)))
        setattr(target, name, value)

    def undo(self):
        while self._changes:
            target, name, previous = self._changes.pop()
            setattr(target, name, previous)


def _run_directed_prejournal_tests(workspace: Path) -> dict:
    module = _load_module(
        Path(workspace) / I5_READINESS_TEST,
        "v2_04g_r6_i5_directed_prejournal_machine_tests",
    )
    no_patch_tests = (
        "test_runner_ast_places_all_mutation_after_complete_prejournal",
        "test_runner_trusted_hashes_match_exact_sources_and_authority",
        "test_authority_schedule_roster_and_historical_nonreuse_are_exact",
        "test_reviewer_ast_has_no_ros_or_subprocess_and_build_is_read_only",
    )
    patch_tests = (
        "test_complete_simulated_prejournal_has_no_mkdir_popen_or_ros_import",
        "test_failed_prejournal_cannot_reach_execution_mutation",
        "test_process_scan_excludes_only_self",
        "test_proc_enumeration_failure_is_fail_closed",
        "test_proc_entry_nontransient_failure_is_fail_closed",
    )
    for name in no_patch_tests:
        getattr(module, name)()
    for name in patch_tests:
        patch = _AttributePatch()
        try:
            getattr(module, name)(patch)
        finally:
            patch.undo()
    return {
        "test_path": I5_READINESS_TEST,
        "test_sha256": _snapshot(workspace, Path(I5_READINESS_TEST))["sha256"],
        "executed_in_process": list(no_patch_tests + patch_tests),
        "executed_test_count": len(no_patch_tests) + len(patch_tests),
        "complete_prejournal_success_monkeypatch_passed": True,
        "failed_prejournal_no_mutation_monkeypatch_passed": True,
        "self_only_process_exclusion_monkeypatch_passed": True,
        "proc_enumeration_fail_closed_monkeypatch_passed": True,
        "proc_entry_fail_closed_monkeypatch_passed": True,
        "workspace_execution_state_created": False,
        "subprocess_started": False,
        "pass": True,
    }


def _verify_closure(workspace: Path) -> dict:
    snapshot = _snapshot(workspace, EXECUTION_CLOSURE, parse_yaml=True)
    first = build_dependency_closure(workspace)
    second = build_dependency_closure(workspace)
    _require(first == second, "I5 dependency closure rebuild is nondeterministic")
    _require(snapshot["document"] == first, "persisted I5 closure drifted")
    receipt = verify_dependency_closure(workspace, snapshot["document"])
    _require(
        receipt["compiled_scene_child_count"] == 14
        and receipt["external_file_count"] == EXPECTED_I5_EXTERNAL_FILE_COUNT
        and receipt["external_python_binding_count"]
        == EXPECTED_I5_PYTHON_BINDING_COUNT
        and receipt["external_runtime_binding_count"]
        == EXPECTED_I5_RUNTIME_BINDING_COUNT
        and receipt["authorization_resource_count"] == 12
        and receipt["inherited_i4_revalidation"]["local_file_count"] == 54
        and receipt["unresolved_count"] == 0,
        "I5 dependency closure coverage drifted",
    )
    extension = first.get("i5_external_extension_audit")
    expected_local_bindings = [
        {"binding": binding, "from": source, "to": target}
        for binding, (source, target) in sorted(
            WORKSPACE_LOCAL_SCRIPT_BINDINGS.items()
        )
    ]
    _require(
        isinstance(extension, dict)
        and extension.get("inherited_external_file_count") == 307
        and extension.get("inherited_python_binding_count") == 47
        and extension.get("inherited_runtime_binding_count") == 9
        and extension.get("additional_python_bindings")
        == list(EXPECTED_I5_ADDITIONAL_PYTHON_BINDINGS)
        and extension.get("workspace_local_script_bindings")
        == expected_local_bindings
        and extension.get("final_external_file_count")
        == receipt["external_file_count"]
        and extension.get("final_external_file_count")
        == EXPECTED_I5_EXTERNAL_FILE_COUNT
        and extension.get("final_python_binding_count")
        == EXPECTED_I5_PYTHON_BINDING_COUNT
        and extension.get("final_runtime_binding_count")
        == EXPECTED_I5_RUNTIME_BINDING_COUNT
        and extension.get("final_external_closure_sha256")
        == first["external"]["closure_sha256"]
        and extension.get("additional_bindings_resolved_without_import") is True
        and extension.get("all_external_files_mechanically_rehashed") is True
        and extension.get("pass") is True,
        "I5 external extension audit drifted",
    )
    boundary = first.get("hash_graph_boundary")
    _require(
        boundary
        == {
            "closure_self_included": False,
            "final_machine_review_artifact_included": False,
            "future_i5_release_included": False,
            "failed_i3_release_included": True,
            "i4_validator_closure_and_review_included": True,
            "future_release_must_bind_closure_and_review": True,
        },
        "I5 hash graph boundary drifted",
    )
    local_paths = {row["path"] for row in first["local"]["files"]}
    _require(
        EXPECTED_I5_SOURCE_PATHS.issubset(local_paths)
        and EXECUTION_CLOSURE.as_posix() not in local_paths
        and MACHINE_REVIEW.as_posix() not in local_paths
        and FUTURE_I5_RELEASE.as_posix() not in local_paths,
        "I5 source or acyclic closure roster drifted",
    )
    return {
        "path": EXECUTION_CLOSURE.as_posix(),
        "file_sha256": snapshot["sha256"],
        "deterministic_rebuild_count": 2,
        "all_i5_sources_scenes_tests_and_runtime_bindings_bound": True,
        **receipt,
    }


def build_review(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    _require(root == WORKSPACE, "review workspace must be canonical thesis workspace")
    state_before = _state_snapshot(root)
    _require_state(state_before)
    processes_before = _process_matches()
    _require(processes_before == [], "forbidden process exists before I5 review")
    authority = _verify_authority(root)
    closure = _verify_closure(root)
    runner = _verify_runner(root)
    directed = _run_directed_prejournal_tests(root)
    state_after = _state_snapshot(root)
    _require_state(state_after)
    processes_after = _process_matches()
    _require(processes_after == [], "forbidden process exists after I5 review")
    _require(state_before == state_after, "I5 review changed execution state")
    failed_release = _snapshot(root, FAILED_I3_RELEASE)
    _require(
        failed_release["sha256"]
        == EXPECTED_FIXED_FILE_SHA256[FAILED_I3_RELEASE.as_posix()],
        "failed I3 release changed during I5 review",
    )
    scene_records = {
        "scene_manifest": _snapshot(root, Path(I5_SCENE_MANIFEST)),
        "compiled_scene_index": _snapshot(root, COMPILED_SCENE_INDEX),
        "behavior_equivalence_audit": _snapshot(
            root, Path(I5_SCENE_BEHAVIOR_AUDIT)
        ),
        "materializer": _snapshot(root, Path(I5_SCENE_MATERIALIZER)),
        "scene_reviewer": _snapshot(root, Path(I5_SCENE_REVIEWER)),
        "scene_tests": _snapshot(root, Path(I5_SCENE_TEST)),
    }
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "source_stage": SOURCE_STAGE,
        "review_id": "fam_teb_v2_04g_r6_i5_execution_readiness_review_1",
        "status": EXPECTED_REVIEW_STATUS,
        "review_result": "pass",
        "independent_stage": True,
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": False,
        "execution_ready": False,
        "execution_release_present": False,
        "separate_execution_release_required": True,
        "separate_execution_release_present": False,
        "ros_or_gazebo_started_by_review": False,
        "authority_review": authority,
        "historical_i3_non_reuse": {
            "failed_release_path": FAILED_I3_RELEASE.as_posix(),
            "failed_release_sha256": failed_release["sha256"],
            "historical_authorized_units": 6,
            "historical_consumed_units": 0,
            "historical_forfeited_units": 0,
            "release_authorization_identity_or_budget_reusable": False,
        },
        "inherited_i4_trust_anchors": {
            "validator": {
                "path": I4_VALIDATOR,
                "sha256": EXPECTED_FIXED_FILE_SHA256[I4_VALIDATOR],
            },
            "dependency_closure": {
                "path": I4_CLOSURE,
                "file_sha256": EXPECTED_FIXED_FILE_SHA256[I4_CLOSURE],
                "logical_sha256": (
                    "dceb73df8619849f5b5a0442b739be09815bfc86939a188873c94993fe4d5b74"
                ),
            },
            "machine_review": {
                "path": I4_REVIEW,
                "sha256": EXPECTED_FIXED_FILE_SHA256[I4_REVIEW],
            },
        },
        "fresh_scene_readiness": {
            label: {
                "path": record["path"],
                "sha256": record["sha256"],
                "size_bytes": record["size_bytes"],
            }
            for label, record in scene_records.items()
        },
        "runner_and_prejournal_review": runner,
        "directed_prejournal_regressions": directed,
        "dependency_closure_review": closure,
        "state_and_process_isolation": {
            "state_before": state_before,
            "state_after": state_after,
            "state_unchanged": True,
            "forbidden_processes_before": [],
            "forbidden_processes_after": [],
            "process_scan_excluded_only_self": True,
            "proc_enumeration_or_inspection_error_fails_closed": True,
            "pass": True,
        },
        "execution_absence_review": {
            "release_manifest_present": False,
            "attempt_root_present": False,
            "journal_root_present": False,
            "receipt_present": False,
            "raw_or_semantic_evidence_present": False,
            "stage_execution_report_present": False,
            "evidence_units_consumed": 0,
            "process_start_performed_by_review": False,
            "execution_ready": False,
            "pass": True,
        },
        "future_i5_release": {
            "path": FUTURE_I5_RELEASE.as_posix(),
            "present": False,
            "unique_canonical_exact_hash_release_required": True,
            "current_user_execution_authorization_sufficient_after_all_gates": True,
        },
        "i5_allocation": {
            "execution_seeds": [5161, 5162, 5163],
            "compile_support_only_seeds": [5164, 5165, 5166, 5167],
            "exact_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
            "evidence_units_authorized": 6,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
            "attempt_limit_per_identity": 1,
            "retry_or_resume_allowed": False,
            "replacement_or_expansion_allowed": False,
        },
        "side_effects": {
            "execution_release_created": False,
            "future_execution_release_created": False,
            "attempt_root_created": False,
            "journal_created": False,
            "execution_evidence_created": False,
            "subprocess_started_by_review": False,
            "ros_started_by_review": False,
            "gazebo_started_by_review": False,
            "move_base_or_teb_started_by_review": False,
            "training_started": False,
            "real_vehicle_used": False,
            "seed_or_budget_consumed": False,
            "evidence_units_consumed": 0,
        },
        "release_validator_cross_feed": {
            "validator_path": I5_RELEASE_VALIDATOR,
            "expected_machine_review_status": EXPECTED_REVIEW_STATUS,
            "actual_build_review_schema_validated": True,
        },
        "next_gate": (
            "create_once_and_validate_unique_canonical_exact_hash_release_then_"
            "run_complete_prejournal_before_any_execution_state_or_process"
        ),
        "claim_limit": (
            "offline_execution_readiness_closure_only_not_simulation_result_"
            "safety_performance_training_or_deployment_readiness"
        ),
    }
    validator = _load_module(
        root / I5_RELEASE_VALIDATOR,
        "v2_04g_r6_i5_machine_review_cross_feed",
    )
    validator._validate_machine_review(review, EXPECTED_REVIEW_STATUS)
    return review


def _atomic_yaml(path: Path, value: object) -> None:
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--output", type=Path, default=WORKSPACE / MACHINE_REVIEW)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    root = args.workspace.resolve()
    output = args.output.resolve()
    canonical_output = (root / MACHINE_REVIEW).resolve()
    if root != WORKSPACE:
        parser.error("workspace must be the canonical thesis workspace")
    if output != canonical_output:
        parser.error("output must be the canonical R6-I5 machine review")
    review = build_review(root)
    if args.check_only:
        persisted = _snapshot(root, MACHINE_REVIEW, parse_yaml=True)["document"]
        _require(persisted == review, "persisted R6-I5 machine review drifted")
    else:
        _require(output.parent.is_dir(), "R6-I5 artifact root is missing")
        _atomic_yaml(output, review)
    print(yaml.safe_dump(review, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
