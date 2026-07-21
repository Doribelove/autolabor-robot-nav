#!/usr/bin/env python3
"""Deterministic offline machine review for the R6-I4 repair closure.

The reviewer validates the independent zero-allocation stage documents,
rebuilds the complete dependency closure, verifies the runner's trusted
validator/test hashes, calls the runner's read-only ``offline_preflight`` in
process, and executes the directed real-roster regressions in process.  It has
no execution surface and never starts ROS, Gazebo, move_base, training, or an
experiment subprocess.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
import types
from typing import Mapping

import yaml


sys.dont_write_bytecode = True

from thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency import (
    ARTIFACT_ROOT,
    ENTRYPOINTS,
    EXECUTION_CLOSURE,
    EXPECTED_FIXED_FILE_SHA256,
    FAILED_I3_RELEASE,
    FORBIDDEN_EXECUTION_STATE,
    FUTURE_I4_RELEASE,
    I2_CLOSURE,
    I3_CLOSURE,
    I4_CONTRACT,
    I4_DEPENDENCY,
    I4_GENERATOR,
    I4_PREFLIGHT_TEST,
    I4_PREREGISTRATION,
    I4_REVIEWER,
    I4_RUNNER,
    I4_TRANSITION,
    I4_VALIDATOR,
    I4_VALIDATOR_TEST,
    MACHINE_REVIEW,
    STAGE,
    build_dependency_closure,
    verify_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_STAGE = "V2-04G-R6-I3"
EXPECTED_REVIEW_STATUS = (
    "preflight_integrity_repair_readiness_closure_pass_"
    "future_release_absent"
)
EXPECTED_RUNNER_STATUS = (
    "offline_preflight_integrity_repair_pass_failed_release_preserved"
)
EXPECTED_I4_SOURCE_PATHS = {
    I4_CONTRACT,
    I4_PREREGISTRATION,
    I4_TRANSITION,
    I4_VALIDATOR,
    I4_VALIDATOR_TEST,
    I4_PREFLIGHT_TEST,
    I4_RUNNER,
    I4_DEPENDENCY,
    I4_GENERATOR,
    I4_REVIEWER,
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


class R6I4PreflightReviewError(ValueError):
    """Raised when an R6-I4 offline review gate fails closed."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise R6I4PreflightReviewError("duplicate YAML key")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise R6I4PreflightReviewError(message)


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
    """Read one regular file once with no-follow on every path component."""

    parts = _safe_parts(relative)
    root = Path(workspace).resolve()
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        descriptor = os.open(
            parts[-1],
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=descriptor,
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
            and all(
                getattr(before, field) == getattr(after, field)
                for field in identity
            ),
            "review dependency changed during read: {}".format(relative),
        )
    except OSError as exc:
        raise R6I4PreflightReviewError(
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
            document = yaml.load(
                payload.decode("utf-8"), Loader=_UniqueKeyLoader
            )
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise R6I4PreflightReviewError(
                "cannot parse {}: {}".format(relative, exc)
            ) from exc
        _require(
            isinstance(document, dict),
            "review document must be a mapping: {}".format(relative),
        )
        row["document"] = document
    return row


def _state_snapshot(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    rows = {
        "failed_i3_release": os.path.lexists(str(root / FAILED_I3_RELEASE)),
        "future_i4_release": os.path.lexists(str(root / FUTURE_I4_RELEASE)),
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
    current = os.getpid()
    matches = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError as exc:
        raise R6I4PreflightReviewError(
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
            raise R6I4PreflightReviewError(
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


def _verify_i4_authority(workspace: Path) -> dict:
    rows = {}
    for relative in (I4_CONTRACT, I4_PREREGISTRATION, I4_TRANSITION):
        snapshot = _snapshot(workspace, Path(relative), parse_yaml=True)
        _require(
            snapshot["sha256"] == EXPECTED_FIXED_FILE_SHA256[relative],
            "I4 authority resource hash drifted: {}".format(relative),
        )
        rows[relative] = snapshot
    contract = rows[I4_CONTRACT]["document"]
    allocation = contract.get("i4_allocation")
    _require(
        contract.get("stage") == STAGE
        and contract.get("source_stage") == SOURCE_STAGE
        and contract.get("independent_stage") is True
        and contract.get("offline_only") is True
        and contract.get("execution_ready") is False
        and contract.get("execution_authorized_by_this_contract") is False
        and contract.get("ros_or_gazebo_start_authorized") is False
        and isinstance(allocation, dict)
        and allocation.get("execution_seeds") == []
        and allocation.get("exact_schedule") == []
        and allocation.get("evidence_units_authorized") == 0
        and allocation.get("evidence_units_consumed") == 0
        and allocation.get("evidence_units_forfeited") == 0,
        "I4 contract authority/allocation boundary drifted",
    )
    prereg = rows[I4_PREREGISTRATION]["document"]
    prereg_contract = prereg.get("bound_resources", {}).get("i4_contract")
    _require(
        prereg.get("stage") == STAGE
        and prereg.get("offline_only") is True
        and prereg.get("execution_authorized") is False
        and prereg.get("execution_ready") is False
        and prereg.get("allocation")
        == {
            "execution_seeds": [],
            "schedule": [],
            "evidence_units_authorized": 0,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
        }
        and prereg.get("single_repair_factor", {}).get("parsed_labels")
        == ["preregistration", "inherited_r6_i2_dependency_closure"],
        "I4 preregistration boundary drifted",
    )
    _require(
        prereg_contract
        == {
            "path": I4_CONTRACT,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I4_CONTRACT],
        },
        "I4 preregistration contract binding drifted",
    )
    transition = rows[I4_TRANSITION]["document"]
    transition_resources = transition.get("bound_resources", {})
    _require(
        transition.get("stage") == STAGE
        and transition.get("target_stage") == STAGE
        and transition.get("execution_authorized") is False
        and transition.get("execution_ready") is False
        and transition.get("offline_only") is True
        and transition.get("target_allocation")
        == {
            "execution_seeds": [],
            "compile_support_seeds": [],
            "schedule": [],
            "evidence_units_authorized": 0,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
        },
        "I4 stage-transition boundary drifted",
    )
    _require(
        transition_resources.get("i4_contract")
        == {
            "path": I4_CONTRACT,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I4_CONTRACT],
        }
        and transition_resources.get("i4_preregistration")
        == {
            "path": I4_PREREGISTRATION,
            "sha256": EXPECTED_FIXED_FILE_SHA256[I4_PREREGISTRATION],
        },
        "I4 transition authority bindings drifted",
    )
    return {
        "contract": {
            "path": I4_CONTRACT,
            "sha256": rows[I4_CONTRACT]["sha256"],
        },
        "preregistration": {
            "path": I4_PREREGISTRATION,
            "sha256": rows[I4_PREREGISTRATION]["sha256"],
        },
        "transition": {
            "path": I4_TRANSITION,
            "sha256": rows[I4_TRANSITION]["sha256"],
        },
        "i4_execution_seeds": [],
        "i4_exact_schedule": [],
        "i4_evidence_units_authorized": 0,
        "i4_evidence_units_consumed": 0,
        "i4_evidence_units_forfeited": 0,
        "execution_authorized": False,
        "pass": True,
    }


def _load_module(path: Path, name: str):
    specification = importlib.util.spec_from_file_location(name, path)
    _require(specification is not None and specification.loader is not None, "module loader unavailable")
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


class _AttributePatch:
    """Minimal deterministic patch helper for the one real-roster test."""

    def __init__(self):
        self._changes = []

    def setattr(self, target, name, value) -> None:
        self._changes.append((target, name, getattr(target, name)))
        setattr(target, name, value)

    def undo(self) -> None:
        while self._changes:
            target, name, previous = self._changes.pop()
            setattr(target, name, previous)


class _RaisesContext:
    """Small no-import equivalent of the pytest.raises surface used here."""

    def __init__(self, expected, match=None):
        self.expected = expected
        self.match = match

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        if exception_type is None:
            raise AssertionError("expected exception was not raised")
        if not issubclass(exception_type, self.expected):
            return False
        if self.match is not None and re.search(self.match, str(exception)) is None:
            raise AssertionError(
                "exception message did not match {!r}: {}".format(
                    self.match, exception
                )
            )
        return True


class _MarkSurface:
    def parametrize(self, *args, **kwargs):
        del args, kwargs

        def decorate(function):
            return function

        return decorate


def _minimal_pytest_surface():
    """Return only the inert surfaces used by the directed test module."""

    module = types.ModuleType("pytest")
    module.mark = _MarkSurface()
    module.raises = lambda expected, match=None: _RaisesContext(expected, match)
    return module


def _top_level_imports(tree: ast.Module) -> list:
    names = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            names.extend(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            names.append((node.module or "").split(".")[0])
    return sorted(set(names))


def _run_real_roster_regressions(workspace: Path, test_snapshot: dict) -> dict:
    source = test_snapshot["payload"].decode("utf-8")
    tree = ast.parse(source, filename=I4_VALIDATOR_TEST)
    functions = {
        node.name
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    required = {
        "test_real_twelve_resource_roster_and_legacy_integer_keys_pass_hash_only",
        "test_hash_only_legacy_resource_byte_drift_still_fails",
        "test_hash_only_legacy_resource_symlink_is_rejected",
        "test_parsed_i2_closure_still_rejects_non_string_key",
        "test_authorization_roster_is_closed_and_path_exact",
    }
    _require(required.issubset(functions), "real-roster regression source is incomplete")
    previous_pytest = sys.modules.get("pytest")
    sys.modules["pytest"] = _minimal_pytest_surface()
    try:
        module = _load_module(
            Path(workspace) / I4_VALIDATOR_TEST,
            "v2_04g_r6_i4_real_roster_machine_regression",
        )
    finally:
        if previous_pytest is None:
            sys.modules.pop("pytest", None)
        else:
            sys.modules["pytest"] = previous_pytest
    monkeypatch = _AttributePatch()
    try:
        module.test_real_twelve_resource_roster_and_legacy_integer_keys_pass_hash_only(
            monkeypatch
        )
    finally:
        monkeypatch.undo()
    with tempfile.TemporaryDirectory(prefix="r6_i4_roster_drift_") as temporary:
        module.test_hash_only_legacy_resource_byte_drift_still_fails(
            Path(temporary)
        )
    with tempfile.TemporaryDirectory(prefix="r6_i4_roster_symlink_") as temporary:
        module.test_hash_only_legacy_resource_symlink_is_rejected(
            Path(temporary)
        )
    with tempfile.TemporaryDirectory(prefix="r6_i4_parsed_scope_") as temporary:
        module.test_parsed_i2_closure_still_rejects_non_string_key(
            Path(temporary)
        )
    roster_cases = (
        (
            lambda resources: resources.pop("r6_design_report"),
            "authorization.bound_resources keys drifted",
        ),
        (
            lambda resources: resources.update(
                {
                    "unexpected_resource": {
                        "path": resources["r6_design_report"]["path"],
                        "sha256": resources["r6_design_report"]["sha256"],
                    }
                }
            ),
            "authorization.bound_resources keys drifted",
        ),
        (
            lambda resources: resources["r6_i1_scene_derivation"].update(
                {"path": resources["r6_design_report"]["path"]}
            ),
            "authorization resource path drifted: r6_i1_scene_derivation",
        ),
    )
    for index, (mutate, match) in enumerate(roster_cases, start=1):
        with tempfile.TemporaryDirectory(
            prefix="r6_i4_closed_roster_{:02d}_".format(index)
        ) as temporary:
            module.test_authorization_roster_is_closed_and_path_exact(
                Path(temporary), mutate, match
            )
    return {
        "test_path": I4_VALIDATOR_TEST,
        "test_sha256": test_snapshot["sha256"],
        "real_authorization_resource_count": 12,
        "expected_legacy_integer_keys": list(range(5141, 5148)),
        "legacy_resource_hash_only_in_validator": True,
        "actual_integer_key_positive_case_passed": True,
        "hash_only_byte_drift_negative_case_passed": True,
        "hash_only_symlink_negative_case_passed": True,
        "parsed_i2_non_string_key_negative_case_passed": True,
        "authorization_missing_label_negative_case_passed": True,
        "authorization_extra_label_negative_case_passed": True,
        "authorization_path_swap_negative_case_passed": True,
        "regressions_executed_in_process": True,
        "workspace_execution_state_created": False,
        "subprocess_started": False,
        "pass": True,
    }


def _verify_runner(workspace: Path) -> dict:
    runner_snapshot = _snapshot(workspace, Path(I4_RUNNER))
    validator_snapshot = _snapshot(workspace, Path(I4_VALIDATOR))
    validator_test_snapshot = _snapshot(workspace, Path(I4_VALIDATOR_TEST))
    preflight_test_snapshot = _snapshot(workspace, Path(I4_PREFLIGHT_TEST))
    source = runner_snapshot["payload"].decode("utf-8")
    tree = ast.parse(source, filename=I4_RUNNER)
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
    _require(forbidden_imports == [], "I4 runner imports executable dependencies")
    _require(
        'add_argument("--execute"' not in source
        and "def execute(" not in source,
        "I4 offline runner exposes execution",
    )
    runner = _load_module(
        Path(workspace) / I4_RUNNER,
        "v2_04g_r6_i4_runner_machine_review",
    )
    _require(
        runner.EXPECTED_REPAIRED_VALIDATOR_SHA256
        == validator_snapshot["sha256"]
        == EXPECTED_FIXED_FILE_SHA256[I4_VALIDATOR]
        and runner.EXPECTED_REPAIRED_VALIDATOR_TEST_SHA256
        == validator_test_snapshot["sha256"]
        == EXPECTED_FIXED_FILE_SHA256[I4_VALIDATOR_TEST]
        and runner.TRUSTED_SOURCE_SHA256[runner.REPAIRED_VALIDATOR_RELATIVE]
        == validator_snapshot["sha256"]
        and runner.TRUSTED_SOURCE_SHA256[
            runner.REPAIRED_VALIDATOR_TEST_RELATIVE
        ]
        == validator_test_snapshot["sha256"],
        "runner trusted validator/test hash chain drifted",
    )
    before_state = _state_snapshot(workspace)
    _require_state(before_state)
    before_processes = _process_matches()
    _require(before_processes == [], "forbidden host process exists before runner review")
    first = runner.offline_preflight()
    second = runner.offline_preflight()
    _require(first == second, "runner offline preflight is not deterministic")
    _require(
        first.get("status") == EXPECTED_RUNNER_STATUS
        and first.get("execution_authorized") is False
        and first.get("execution_ready") is False
        and first.get("failed_i3_release", {}).get("authorized_units") == 6
        and first.get("failed_i3_release", {}).get("consumed_units") == 0
        and first.get("failed_i3_release", {}).get("forfeited_units") == 0
        and first.get("authorization_resource_audit", {}).get("resource_count") == 12
        and first.get("authorization_resource_audit", {}).get("parsed_count") == 2
        and first.get("authorization_resource_audit", {}).get("hash_only_count") == 10
        and first.get("i4_allocation")
        == {
            "execution_seeds": [],
            "schedule": [],
            "evidence_units_authorized": 0,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
        }
        and all(value is False for value in first.get("side_effects", {}).values()),
        "runner offline-preflight boundary did not pass",
    )
    regressions = _run_real_roster_regressions(
        workspace, validator_test_snapshot
    )
    after_state = _state_snapshot(workspace)
    _require_state(after_state)
    after_processes = _process_matches()
    _require(after_processes == [], "forbidden host process exists after runner review")
    _require(before_state == after_state, "runner/regression review changed execution state")
    preflight_tree = ast.parse(
        preflight_test_snapshot["payload"].decode("utf-8"),
        filename=I4_PREFLIGHT_TEST,
    )
    preflight_functions = {
        node.name
        for node in preflight_tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    _require(
        {
            "test_runner_is_stdlib_only_and_has_no_execute_surface",
            "test_runner_trust_hashes_match_repaired_validator_and_real_roster_tests",
            "test_offline_preflight_is_deterministic_read_only_and_zero_allocation",
            "test_runner_rejects_trusted_validator_hash_drift",
            "test_each_forbidden_execution_state_fails_closed",
            "test_failed_release_absence_fails_closed",
            "test_forbidden_process_match_fails_before_validation",
            "test_proc_enumeration_failure_is_fail_closed",
            "test_dependency_closure_rebuild_is_deterministic_and_process_free",
            "test_persisted_closure_and_machine_review_rebuild_exactly",
        }.issubset(preflight_functions),
        "I4 preflight-integrity directed tests are incomplete",
    )
    return {
        "runner": {
            "path": I4_RUNNER,
            "sha256": runner_snapshot["sha256"],
        },
        "repaired_validator": {
            "path": I4_VALIDATOR,
            "sha256": validator_snapshot["sha256"],
        },
        "real_roster_validator_tests": {
            "path": I4_VALIDATOR_TEST,
            "sha256": validator_test_snapshot["sha256"],
        },
        "preflight_integrity_tests": {
            "path": I4_PREFLIGHT_TEST,
            "sha256": preflight_test_snapshot["sha256"],
        },
        "runner_top_level_imports": imports,
        "execute_surface_present": False,
        "runner_trusted_hashes_match_actual_bytes": True,
        "offline_preflight_called_in_process": True,
        "offline_preflight_deterministic": True,
        "offline_preflight_receipt": first,
        "real_roster_regression_review": regressions,
        "state_before_equals_state_after": True,
        "forbidden_processes_before": [],
        "forbidden_processes_after": [],
        "pass": True,
    }


def _verify_closure(workspace: Path) -> dict:
    snapshot = _snapshot(workspace, EXECUTION_CLOSURE, parse_yaml=True)
    receipt = verify_dependency_closure(workspace, snapshot["document"])
    second = build_dependency_closure(workspace)
    _require(
        second == snapshot["document"],
        "R6-I4 dependency closure second deterministic rebuild drifted",
    )
    local_paths = {
        row["path"] for row in snapshot["document"]["local"]["files"]
    }
    _require(
        EXPECTED_I4_SOURCE_PATHS.issubset(local_paths),
        "I4 closure does not contain every source/contract/test",
    )
    boundary = snapshot["document"].get("hash_graph_boundary")
    _require(
        boundary
        == {
            "closure_self_included": False,
            "final_machine_review_artifact_included": False,
            "failed_i3_release_included": True,
            "future_i4_release_included": False,
            "future_release_must_bind_closure_and_review": True,
        },
        "I4 closure hash-graph boundary drifted",
    )
    _require(
        receipt["authorization_resource_count"] == 12
        and receipt["inherited_i3_revalidation"]["local_file_count"] == 136
        and receipt["inherited_i3_revalidation"]["external_file_count"] == 307
        and receipt["inherited_i3_revalidation"]["external_runtime_binding_count"] == 9
        and receipt["inherited_i2_revalidation"]["local_file_count"] == 106
        and receipt["inherited_i2_revalidation"]["external_file_count"] == 301
        and receipt["inherited_i2_revalidation"]["external_runtime_binding_count"] == 5
        and receipt["unresolved_count"] == 0,
        "I4 or inherited dependency-closure coverage drifted",
    )
    return {
        "path": EXECUTION_CLOSURE.as_posix(),
        "file_sha256": snapshot["sha256"],
        "deterministic_rebuild_count": 2,
        "all_i4_sources_contracts_and_tests_bound": True,
        **receipt,
    }


def build_review(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    _require(root == WORKSPACE, "review workspace must be the canonical thesis workspace")
    state_before = _state_snapshot(root)
    _require_state(state_before)
    processes_before = _process_matches()
    _require(processes_before == [], "forbidden process exists before I4 review")
    authority = _verify_i4_authority(root)
    closure = _verify_closure(root)
    runner = _verify_runner(root)
    state_after = _state_snapshot(root)
    _require_state(state_after)
    processes_after = _process_matches()
    _require(processes_after == [], "forbidden process exists after I4 review")
    _require(state_before == state_after, "I4 review changed execution state")
    failed_release = _snapshot(root, FAILED_I3_RELEASE)
    _require(
        failed_release["sha256"]
        == EXPECTED_FIXED_FILE_SHA256[FAILED_I3_RELEASE.as_posix()],
        "failed I3 release changed during I4 review",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "source_stage": SOURCE_STAGE,
        "review_id": (
            "fam_teb_v2_04g_r6_i4_preflight_integrity_"
            "repair_readiness_review_1"
        ),
        "status": EXPECTED_REVIEW_STATUS,
        "review_result": "pass",
        "independent_stage": True,
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "ros_or_gazebo_started_by_review": False,
        "authority_review": authority,
        "failed_i3_release_snapshot": {
            "path": FAILED_I3_RELEASE.as_posix(),
            "sha256": failed_release["sha256"],
            "present": True,
            "historical_authorized_units": 6,
            "historical_consumed_units": 0,
            "historical_forfeited_units": 0,
            "executable_or_reusable_under_i4": False,
        },
        "future_i4_release": {
            "path": FUTURE_I4_RELEASE.as_posix(),
            "present": False,
            "new_explicit_simulation_execution_instruction_required": True,
        },
        "runner_trust_and_real_roster_review": runner,
        "dependency_closure_review": closure,
        "state_and_process_isolation": {
            "state_before": state_before,
            "state_after": state_after,
            "state_unchanged": True,
            "forbidden_processes_before": [],
            "forbidden_processes_after": [],
            "pass": True,
        },
        "i4_allocation": {
            "execution_seeds": [],
            "schedule": [],
            "evidence_units_authorized": 0,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
        },
        "side_effects": {
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
        },
        "next_gate": (
            "stop_and_wait_for_new_explicit_simulation_execution_"
            "authorization_before_any_future_release_or_execution_state"
        ),
        "claim_limit": (
            "offline_preflight_integrity_repair_readiness_closure_only_"
            "not_simulation_execution_safety_performance_or_deployment_readiness"
        ),
    }


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
    if output != canonical_output:
        parser.error("output must be the canonical R6-I4 machine review")
    review = build_review(root)
    if args.check_only:
        persisted = _snapshot(root, MACHINE_REVIEW, parse_yaml=True)["document"]
        _require(persisted == review, "persisted R6-I4 machine review drifted")
    else:
        _require(output.parent.is_dir(), "R6-I4 static artifact root is missing")
        _atomic_yaml(output, review)
    print(yaml.safe_dump(review, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
