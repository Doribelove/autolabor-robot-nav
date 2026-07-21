"""Versioned fail-closed release validator for bounded R6-I5 simulation.

This module is deliberately ROS-free and process-free.  Every release and
authorization resource is opened once through a component-wise no-follow
chain and rehashed.  Only closed labels whose documents are semantically
consumed are parsed; all other resources remain exact-byte hash-only inputs.
Validation never creates an execution directory, journal, log, or subprocess.

The reviewed execution entrypoint must verify this module's canonical path and
SHA256 before loading it.  It must then call
``load_and_validate_execution_release`` before importing workspace runtime
modules or creating any execution state.
"""

from dataclasses import dataclass, replace
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Dict, Mapping, MutableMapping, Optional, Sequence, Tuple

import yaml


STAGE = "V2-04G-R6-I5"
BASIS_STAGE = "V2-04G-R6-I4"
CANONICAL_RELEASE_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_release.yaml"
)
CANONICAL_AUTHORIZATION_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
MAX_RESOURCE_BYTES = 32 * 1024 * 1024

EXPECTED_PROFILES = [
    "r6_semantics_legacy_control",
    "r6_semantics_circle_contact",
]
EXPECTED_EXECUTION_SEEDS = [5161, 5162, 5163]
EXPECTED_SCHEDULE = [
    {
        "sequence": 1,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 2,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 3,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 4,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 5,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "legacy_non_none_identifiability",
    },
    {
        "sequence": 6,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "none_iff_no_finite_ttc",
    },
]
EXPECTED_SCHEDULE_SHA256 = (
    "b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402"
)

SCHEDULE_FIELDS = {
    "sequence",
    "profile_id",
    "scene_id",
    "seed",
    "attempt",
    "expected_ttc_status",
    "expected_overlay_semantics",
}
RELEASE_FIELDS = {
    "schema_version",
    "architecture_generation",
    "stage",
    "release_id",
    "status",
    "release_date",
    "release_source",
    "explicit_user_execution_instruction_received",
    "execution_release_authorized",
    "simulation_only",
    "formal_result",
    "runtime_ready",
    "training_allowed",
    "real_vehicle_use_forbidden",
    "scope",
    "evidence_budget_authorized",
    "fresh_execution_seeds",
    "attempt_limit_per_identity",
    "retry_or_resume_allowed",
    "seed_replacement_allowed",
    "budget_expansion_allowed",
    "stop_on_first_terminal_failure",
    "forfeit_unattempted_units_after_terminal_failure",
    "i1_retry_or_resume_authorized",
    "i1_forfeited_units_reused",
    "prior_identity_reuse_allowed",
    "r5_retry_or_resume_authorized",
    "r5_remaining_units_consumed",
    "held_out_5001_5010_accessed",
    "rank_or_freeze_winner_authorized",
    "v2_05_authorized",
    "sac_or_training_authorized",
    "real_vehicle_authorized",
    "real_vehicle_teb_write_authorized",
    "authorization_envelope_alone_sufficient_for_execution",
    "exact_schedule",
    "exact_schedule_sha256",
    "bound_resources",
    "dependency_closure_digest",
    "release_trust_anchor",
    "prejournal_gate",
    "completion_boundary",
}
SCOPE_FIELDS = {
    "purpose",
    "stage_only",
    "profiles",
    "fresh_execution_seeds",
    "exact_identity_count",
    "component_stage_authorized",
    "general_navigation_calibration_authorized",
    "winner_selection_authorized",
}
RELEASE_TRUST_FIELDS = {
    "mechanism",
    "self_hash_embedded",
    "guard_rejects_missing_or_mismatched_cli_hash",
    "authorization_hash_independently_supplied",
}
PREJOURNAL_FIELDS = {
    "release_validation_before_execution_state_creation_required",
    "authorization_revalidation_required",
    "all_bound_resources_rehashed_required",
    "closure_logical_digest_recomputed_required",
    "scene_children_rehashed_required",
    "machine_review_pass_required",
    "existing_execution_state_absent_required",
    "forbidden_processes_absent_required",
    "execution_state_creation_before_validation_allowed",
}
COMPLETION_FIELDS = {
    "maximum_claim",
    "safety_performance_generalization_claim_allowed",
    "formal_result_must_remain_false",
    "runtime_ready_must_remain_false",
    "downstream_authorization_after_completion",
}
AUTHORIZATION_FIELDS = {
    "schema_version",
    "architecture_generation",
    "stage",
    "authorization_id",
    "status",
    "authorization_date",
    "authorization_source",
    "execution_authorized",
    "simulation_only",
    "formal_result",
    "runtime_ready",
    "training_allowed",
    "real_vehicle_use_forbidden",
    "scope",
    "evidence_budget_authorized",
    "fresh_execution_seeds",
    "attempt_limit_per_identity",
    "retry_or_resume_allowed",
    "seed_replacement_allowed",
    "budget_expansion_allowed",
    "stop_on_first_terminal_failure",
    "forfeit_unattempted_units_after_terminal_failure",
    "i1_retry_or_resume_authorized",
    "i1_forfeited_units_reused",
    "prior_identity_reuse_allowed",
    "r5_retry_or_resume_authorized",
    "r5_remaining_units_consumed",
    "held_out_5001_5010_accessed",
    "rank_or_freeze_winner_authorized",
    "v2_05_authorized",
    "sac_or_training_authorized",
    "real_vehicle_authorized",
    "real_vehicle_teb_write_authorized",
    "exact_schedule",
    "preregistration_schedule_sha256",
    "bound_resources",
    "dependency_closure_digest",
    "authorization_trust_anchor",
    "completion_boundary",
}
AUTHORIZATION_TRUST_FIELDS = {
    "mechanism",
    "self_hash_embedded",
    "guard_rejects_missing_or_mismatched_cli_hash",
}
REQUIRED_RESOURCE_LABELS = {
    "execution_contract",
    "preregistration",
    "authorization_envelope",
    "stage_transition",
    "scene_derivation",
    "fresh_scene_index",
    "execution_entrypoint",
    "release_validator",
    "release_validator_tests",
    "execution_dependency_closure",
    "execution_machine_review",
    "i4_repaired_validator",
    "i4_dependency_closure",
    "i4_machine_review",
    "failed_i3_release",
}
SCENE_CHILD_LABEL_PREFIX = "fresh_scene_child_"
EXPECTED_SCENE_CHILD_COUNT = 14
AUTHORIZATION_PREREGISTRATION_LABEL = "preregistration"
AUTHORIZATION_CLOSURE_LABEL = "inherited_i4_dependency_closure"
AUTHORIZATION_PARSED_RESOURCE_LABELS = frozenset(
    {
        AUTHORIZATION_PREREGISTRATION_LABEL,
        AUTHORIZATION_CLOSURE_LABEL,
    }
)
EXPECTED_AUTHORIZATION_RESOURCE_PATHS = {
    "contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_"
        "bounded_simulation_execution_contract.yaml"
    ),
    "preregistration": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i5_execution_preregistration.yaml"
    ),
    "i4_validator": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
    ),
    "inherited_i4_dependency_closure": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i4_preflight_repair_review/"
        "execution_dependency_closure.yaml"
    ),
    "i4_machine_review": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i4_preflight_repair_review/"
        "v2_04g_r6_i4_preflight_integrity_readiness_review.yaml"
    ),
    "failed_i3_release": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i3_execution_release.yaml"
    ),
    "frozen_evaluator": (
        "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py"
    ),
    "legacy_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_legacy_control/supervisor.yaml"
    ),
    "aligned_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_circle_contact/supervisor.yaml"
    ),
    "source_i1_scene_manifest": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "v2_04g_r6_i1_scenes.yaml"
    ),
    "source_i1_compiled_scene_index": (
        "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    "r6_design_report": (
        "artifacts/v2/design_review/v2_04g_r6/"
        "v2_04g_r6_design_review.yaml"
    ),
}

RELEASE_PARSED_RESOURCE_LABELS = frozenset(
    {
        "preregistration",
        "authorization_envelope",
        "fresh_scene_index",
        "execution_dependency_closure",
        "execution_machine_review",
    }
)
EXPECTED_MACHINE_REVIEW_STATUS = (
    "r6_i5_execution_readiness_closure_pass_release_absent"
)
I5_EXECUTION_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution"
)
_SCENE_NAMES = (
    "v2-04g-r6-i5-dynamic-conflict-single-s5161.instance.yaml",
    "v2-04g-r6-i5-dynamic-conflict-single-s5161.world",
    "v2-04g-r6-i5-dynamic-conflict-multi-s5162.instance.yaml",
    "v2-04g-r6-i5-dynamic-conflict-multi-s5162.world",
    "v2-04g-r6-i5-dynamic-semantic-clear-s5163.instance.yaml",
    "v2-04g-r6-i5-dynamic-semantic-clear-s5163.world",
    "v2-04g-r6-i5-compile-support-cruise-s5164.instance.yaml",
    "v2-04g-r6-i5-compile-support-cruise-s5164.world",
    "v2-04g-r6-i5-compile-support-static-s5165.instance.yaml",
    "v2-04g-r6-i5-compile-support-static-s5165.world",
    "v2-04g-r6-i5-compile-support-corridor-s5166.instance.yaml",
    "v2-04g-r6-i5-compile-support-corridor-s5166.world",
    "v2-04g-r6-i5-compile-support-maneuver-s5167.instance.yaml",
    "v2-04g-r6-i5-compile-support-maneuver-s5167.world",
)
EXPECTED_RELEASE_RESOURCE_PATHS = {
    "execution_contract": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "contract"
    ],
    "preregistration": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "preregistration"
    ],
    "authorization_envelope": CANONICAL_AUTHORIZATION_PATH,
    "stage_transition": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i5_stage_transition.yaml"
    ),
    "scene_derivation": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i5_scene_derivation.yaml"
    ),
    "fresh_scene_index": I5_EXECUTION_ROOT
    + "/compiled_scenes/compiled_scene_index.yaml",
    "execution_entrypoint": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_bounded_validation.py"
    ),
    "release_validator": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release.py"
    ),
    "release_validator_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i5_release_validator.py"
    ),
    "execution_dependency_closure": I5_EXECUTION_ROOT
    + "/execution_dependency_closure.yaml",
    "execution_machine_review": I5_EXECUTION_ROOT
    + "/v2_04g_r6_i5_execution_readiness_review.yaml",
    "i4_repaired_validator": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "i4_validator"
    ],
    "i4_dependency_closure": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "inherited_i4_dependency_closure"
    ],
    "i4_machine_review": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "i4_machine_review"
    ],
    "failed_i3_release": EXPECTED_AUTHORIZATION_RESOURCE_PATHS[
        "failed_i3_release"
    ],
}
EXPECTED_RELEASE_RESOURCE_PATHS.update(
    {
        "fresh_scene_child_{:02d}".format(index): (
            I5_EXECUTION_ROOT + "/compiled_scenes/" + name
        )
        for index, name in enumerate(_SCENE_NAMES, start=1)
    }
)


class R6I5ExecutionReleaseError(ValueError):
    """Raised when a future release or any bound input fails closed."""


class _StrictYamlLoader(yaml.SafeLoader):
    """Safe loader rejecting merges and duplicate keys.

    Plain aliases are accepted because inherited closure documents emitted by
    ``yaml.safe_dump`` contains aliases for repeated immutable structures.
    Merge keys remain forbidden because they can conceal effective fields from
    a closed-schema review.
    """

    def flatten_mapping(self, node):
        for key_node, _ in node.value:
            if key_node.tag == "tag:yaml.org,2002:merge":
                raise R6I5ExecutionReleaseError(
                    "YAML merge keys are forbidden"
                )
        return super().flatten_mapping(node)


def _construct_unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        if key_node.tag == "tag:yaml.org,2002:merge":
            raise R6I5ExecutionReleaseError("YAML merge keys are forbidden")
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise R6I5ExecutionReleaseError(
                "YAML mapping key is not hashable"
            ) from exc
        if duplicate:
            raise R6I5ExecutionReleaseError(
                "duplicate YAML key: {!r}".format(key)
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_StrictYamlLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class FileSnapshot:
    """Bytes, digest and optional document from one stable file descriptor."""

    declared_path: str
    path: Path
    sha256: str
    size_bytes: int
    payload: bytes
    document: Optional[Mapping[str, Any]]


@dataclass(frozen=True)
class ExecutionReleaseValidation:
    """Immutable result returned only after every release gate passes."""

    release: FileSnapshot
    authorization: FileSnapshot
    preregistration: FileSnapshot
    bound_resources: Mapping[str, FileSnapshot]
    authorization_bound_resources: Mapping[str, FileSnapshot]
    release_parsed_labels: Tuple[str, ...]
    release_hash_only_labels: Tuple[str, ...]
    authorization_parsed_labels: Tuple[str, ...]
    authorization_hash_only_labels: Tuple[str, ...]
    dependency_closure: FileSnapshot
    machine_review: FileSnapshot
    schedule_sha256: str
    identity_count: int
    execution_seeds: Tuple[int, ...]
    runtime_executables: Mapping[str, str]


def _require(condition, message):
    if not condition:
        raise R6I5ExecutionReleaseError(message)


def _hex_digest(value, label):
    _require(
        type(value) is str
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value),
        "{} must be a lowercase SHA256".format(label),
    )
    return value


def _closed_mapping(value, fields, label):
    _require(isinstance(value, Mapping), "{} must be a mapping".format(label))
    expected = set(fields)
    actual = set(value)
    _require(
        actual == expected,
        "{} keys drifted; missing={} extra={}".format(
            label, sorted(expected - actual), sorted(actual - expected)
        ),
    )
    return value


def _exact_equal(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _exact_equal(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _exact_equal(left, right)
            for left, right in zip(actual, expected)
        )
    return actual == expected


def _validate_data_tree(value, label="document"):
    if isinstance(value, Mapping):
        for key, child in value.items():
            _require(type(key) is str, "{} contains a non-string key".format(label))
            _validate_data_tree(child, "{}.{}".format(label, key))
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _validate_data_tree(child, "{}[{}]".format(label, index))
        return
    _require(
        value is None or type(value) in {str, bool, int, float},
        "{} contains an unsupported YAML scalar type".format(label),
    )
    if type(value) is float:
        _require(math.isfinite(value), "{} must be finite".format(label))


def canonical_document_sha256(value):
    """Return the type-stable canonical JSON SHA256 of parsed data."""

    _validate_data_tree(value)
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _relative_parts(declared_path):
    _require(type(declared_path) is str and declared_path, "path is empty")
    _require("\x00" not in declared_path, "path contains NUL")
    declared = Path(declared_path)
    _require(not declared.is_absolute(), "workspace path must be relative")
    _require(
        declared.as_posix() == declared_path,
        "workspace path is not normalized POSIX form",
    )
    _require(
        declared.parts
        and all(part not in {"", ".", ".."} for part in declared.parts),
        "workspace path contains traversal",
    )
    return declared.parts


def _absolute_parts(declared_path):
    _require(type(declared_path) is str and declared_path, "path is empty")
    _require("\x00" not in declared_path, "path contains NUL")
    declared = Path(declared_path)
    _require(declared.is_absolute(), "external path must be absolute")
    _require(
        declared.as_posix() == declared_path,
        "external path is not normalized POSIX form",
    )
    parts = declared.parts[1:]
    _require(
        parts and all(part not in {"", ".", ".."} for part in parts),
        "external path contains traversal",
    )
    return parts


def _read_descriptor_once(file_descriptor, declared_path):
    before = os.fstat(file_descriptor)
    _require(stat.S_ISREG(before.st_mode), declared_path + " is not a regular file")
    _require(
        before.st_size <= MAX_RESOURCE_BYTES,
        declared_path + " exceeds the resource size limit",
    )
    chunks = []
    remaining = MAX_RESOURCE_BYTES + 1
    while remaining:
        chunk = os.read(file_descriptor, min(1024 * 1024, remaining))
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    payload = b"".join(chunks)
    after = os.fstat(file_descriptor)
    stable_fields = (
        "st_dev",
        "st_ino",
        "st_mode",
        "st_nlink",
        "st_size",
        "st_mtime_ns",
        "st_ctime_ns",
    )
    _require(
        all(getattr(before, field) == getattr(after, field) for field in stable_fields)
        and len(payload) == before.st_size,
        declared_path + " changed during its single read",
    )
    return payload


def _read_workspace_relative_bytes_once(workspace, declared_path):
    root = Path(workspace)
    _require(root.is_absolute(), "workspace must be absolute")
    _require(root == root.resolve(), "workspace must be canonical")
    _require(root.is_dir() and not root.is_symlink(), "workspace root is unsafe")
    _require(hasattr(os, "O_NOFOLLOW"), "O_NOFOLLOW is required")
    parts = _relative_parts(declared_path)
    read_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = read_flags | os.O_DIRECTORY | os.O_NOFOLLOW
    descriptors = []
    try:
        current = os.open(str(root), directory_flags)
        descriptors.append(current)
        for component in parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        file_descriptor = os.open(
            parts[-1], read_flags | os.O_NOFOLLOW, dir_fd=current
        )
        descriptors.append(file_descriptor)
        return _read_descriptor_once(file_descriptor, declared_path)
    except OSError as exc:
        raise R6I5ExecutionReleaseError(
            "cannot safely open {}: {}".format(declared_path, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _read_external_absolute_bytes_once(declared_path):
    _require(hasattr(os, "O_NOFOLLOW"), "O_NOFOLLOW is required")
    parts = _absolute_parts(declared_path)
    read_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = read_flags | os.O_DIRECTORY | os.O_NOFOLLOW
    descriptors = []
    try:
        current = os.open("/", directory_flags)
        descriptors.append(current)
        for component in parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        file_descriptor = os.open(
            parts[-1], read_flags | os.O_NOFOLLOW, dir_fd=current
        )
        descriptors.append(file_descriptor)
        return _read_descriptor_once(file_descriptor, declared_path)
    except OSError as exc:
        raise R6I5ExecutionReleaseError(
            "cannot safely open external {}: {}".format(declared_path, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _parse_yaml_mapping(payload, label):
    try:
        document = yaml.load(payload.decode("utf-8"), Loader=_StrictYamlLoader)
    except R6I5ExecutionReleaseError:
        raise
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6I5ExecutionReleaseError(
            "cannot strictly parse {}: {}".format(label, exc)
        ) from exc
    _require(isinstance(document, Mapping), label + " must contain one mapping")
    _validate_data_tree(document, label)
    return document


def read_workspace_file_once(workspace, declared_path, parse_yaml=False):
    """Read one workspace file without following any path-component symlink."""

    payload = _read_workspace_relative_bytes_once(workspace, declared_path)
    document = _parse_yaml_mapping(payload, declared_path) if parse_yaml else None
    return FileSnapshot(
        declared_path=declared_path,
        path=Path(workspace) / declared_path,
        sha256=hashlib.sha256(payload).hexdigest(),
        size_bytes=len(payload),
        payload=payload,
        document=document,
    )


class _SnapshotCache:
    def __init__(self, workspace, preloaded=None):
        self.workspace = Path(workspace)
        self._items = {}  # type: Dict[str, FileSnapshot]
        for declared_path, snapshot in dict(preloaded or {}).items():
            _require(
                isinstance(snapshot, FileSnapshot)
                and snapshot.declared_path == declared_path
                and snapshot.sha256 == hashlib.sha256(snapshot.payload).hexdigest()
                and snapshot.size_bytes == len(snapshot.payload),
                "preloaded snapshot is invalid: {}".format(declared_path),
            )
            self._items["workspace:" + declared_path] = snapshot

    def workspace_file(self, declared_path, parse_yaml=False):
        key = "workspace:" + declared_path
        snapshot = self._items.get(key)
        if snapshot is None:
            snapshot = read_workspace_file_once(
                self.workspace, declared_path, parse_yaml=False
            )
            self._items[key] = snapshot
        if parse_yaml and snapshot.document is None:
            snapshot = replace(
                snapshot,
                document=_parse_yaml_mapping(snapshot.payload, declared_path),
            )
            self._items[key] = snapshot
        return snapshot

    def external_file(self, declared_path):
        key = "external:" + declared_path
        snapshot = self._items.get(key)
        if snapshot is None:
            payload = _read_external_absolute_bytes_once(declared_path)
            snapshot = FileSnapshot(
                declared_path=declared_path,
                path=Path(declared_path),
                sha256=hashlib.sha256(payload).hexdigest(),
                size_bytes=len(payload),
                payload=payload,
                document=None,
            )
            self._items[key] = snapshot
        return snapshot


def _validate_schedule(schedule, label):
    _require(isinstance(schedule, list), label + " must be a list")
    _require(_exact_equal(schedule, EXPECTED_SCHEDULE), label + " drifted")
    identities = set()
    for index, row in enumerate(schedule, start=1):
        _closed_mapping(row, SCHEDULE_FIELDS, "{}[{}]".format(label, index - 1))
        _require(
            type(row["sequence"]) is int and row["sequence"] == index,
            label + " sequence drifted",
        )
        _require(type(row["seed"]) is int, label + " seed type drifted")
        _require(
            type(row["attempt"]) is int and row["attempt"] == 1,
            label + " attempt drifted",
        )
        for field in (
            "profile_id",
            "scene_id",
            "expected_ttc_status",
            "expected_overlay_semantics",
        ):
            _require(type(row[field]) is str and row[field], label + " text drifted")
        identity = (
            row["profile_id"], row["scene_id"], row["seed"], row["attempt"]
        )
        _require(identity not in identities, label + " identity duplicated")
        identities.add(identity)
    schedule_sha256 = canonical_document_sha256(schedule)
    _require(
        schedule_sha256 == EXPECTED_SCHEDULE_SHA256,
        label + " canonical SHA256 drifted",
    )
    return schedule_sha256


def _expected_scope():
    return {
        "purpose": "runtime_evaluator_semantic_and_execution_integrity_validation",
        "stage_only": STAGE,
        "profiles": list(EXPECTED_PROFILES),
        "fresh_execution_seeds": list(EXPECTED_EXECUTION_SEEDS),
        "exact_identity_count": len(EXPECTED_SCHEDULE),
        "component_stage_authorized": False,
        "general_navigation_calibration_authorized": False,
        "winner_selection_authorized": False,
    }


def _safety_expectations():
    return {
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "evidence_budget_authorized": len(EXPECTED_SCHEDULE),
        "fresh_execution_seeds": list(EXPECTED_EXECUTION_SEEDS),
        "attempt_limit_per_identity": 1,
        "retry_or_resume_allowed": False,
        "seed_replacement_allowed": False,
        "budget_expansion_allowed": False,
        "stop_on_first_terminal_failure": True,
        "forfeit_unattempted_units_after_terminal_failure": True,
        "i1_retry_or_resume_authorized": False,
        "i1_forfeited_units_reused": False,
        "prior_identity_reuse_allowed": False,
        "r5_retry_or_resume_authorized": False,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "rank_or_freeze_winner_authorized": False,
        "v2_05_authorized": False,
        "sac_or_training_authorized": False,
        "real_vehicle_authorized": False,
        "real_vehicle_teb_write_authorized": False,
    }


def _validate_preregistration(document):
    _require(document.get("stage") == STAGE, "preregistration stage drifted")
    _require(
        document.get("execution_authorized") is False,
        "preregistration must remain non-authorizing",
    )
    _require(
        document.get("execution_release_required") is True,
        "preregistration release gate drifted",
    )
    schedule_sha = _validate_schedule(document.get("schedule"), "preregistration.schedule")
    budget = document.get("budget")
    _require(isinstance(budget, Mapping), "preregistration budget is missing")
    _require(
        type(budget.get("evidence_units_authorizable")) is int
        and budget["evidence_units_authorizable"] == len(EXPECTED_SCHEDULE)
        and type(budget.get("attempt_limit_per_identity")) is int
        and budget["attempt_limit_per_identity"] == 1
        and budget.get("retry_allowed") is False
        and budget.get("resume_allowed") is False
        and budget.get("replacement_seed_allowed") is False
        and budget.get("budget_expansion_allowed") is False,
        "preregistration budget boundary drifted",
    )
    return schedule_sha


def _logical_digest(document, label):
    _require(isinstance(document, Mapping), label + " must be a mapping")
    digest = _hex_digest(document.get("closure_sha256"), label + " logical digest")
    payload = {key: value for key, value in document.items() if key != "closure_sha256"}
    _require(canonical_document_sha256(payload) == digest, label + " logical digest drifted")
    return digest


def _validate_authorization(document, preregistration, authorization_closure):
    _closed_mapping(document, AUTHORIZATION_FIELDS, "authorization")
    expected = _safety_expectations()
    expected.update(
        {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "status": "bounded_fresh_seed_simulation_authorized",
            "authorization_source": (
                "explicit_user_instruction_after_independent_integration_review"
            ),
            "execution_authorized": True,
        }
    )
    for key, value in expected.items():
        _require(_exact_equal(document[key], value), "authorization {} drifted".format(key))
    _require(
        type(document["authorization_id"]) is str and document["authorization_id"],
        "authorization_id must be text",
    )
    _require(
        type(document["authorization_date"]) is str and document["authorization_date"],
        "authorization_date must be explicit text",
    )
    scope = _closed_mapping(document["scope"], SCOPE_FIELDS, "authorization.scope")
    _require(_exact_equal(dict(scope), _expected_scope()), "authorization scope drifted")
    schedule_sha = _validate_schedule(document["exact_schedule"], "authorization.exact_schedule")
    _require(
        document["preregistration_schedule_sha256"] == schedule_sha,
        "authorization schedule digest drifted",
    )
    _require(
        _validate_preregistration(preregistration) == schedule_sha,
        "preregistration schedule digest drifted",
    )
    trust = _closed_mapping(
        document["authorization_trust_anchor"],
        AUTHORIZATION_TRUST_FIELDS,
        "authorization.authorization_trust_anchor",
    )
    _require(
        _exact_equal(
            dict(trust),
            {
                "mechanism": "caller_supplied_exact_authorization_file_sha256",
                "self_hash_embedded": False,
                "guard_rejects_missing_or_mismatched_cli_hash": True,
            },
        ),
        "authorization trust anchor drifted",
    )
    completion = _closed_mapping(
        document["completion_boundary"], COMPLETION_FIELDS, "authorization.completion_boundary"
    )
    _require(
        _exact_equal(
            dict(completion),
            {
                "maximum_claim": "fresh_simulation_runtime_evaluator_semantic_integration",
                "safety_performance_generalization_claim_allowed": False,
                "formal_result_must_remain_false": True,
                "runtime_ready_must_remain_false": True,
                "downstream_authorization_after_completion": False,
            },
        ),
        "authorization completion boundary drifted",
    )
    expected_closure_digest = _hex_digest(
        document["dependency_closure_digest"], "authorization dependency closure digest"
    )
    _require(
        authorization_closure.get("stage") == BASIS_STAGE,
        "authorization dependency closure stage drifted",
    )
    _require(
        authorization_closure.get("execution_authorized") is False,
        "authorization dependency closure must remain non-authorizing",
    )
    _require(
        authorization_closure.get("execution_ready") is False,
        "authorization dependency closure readiness drifted",
    )
    _require(
        authorization_closure.get("unresolved") == [],
        "authorization dependency closure is unresolved",
    )
    _require(
        _logical_digest(authorization_closure, "authorization dependency closure")
        == expected_closure_digest,
        "authorization dependency closure digest disagrees",
    )
    return schedule_sha


def _validate_release_document(document, expected_resource_paths):
    _closed_mapping(document, RELEASE_FIELDS, "execution release")
    expected = _safety_expectations()
    expected.update(
        {
            "schema_version": "1.0",
            "architecture_generation": "v2",
            "stage": STAGE,
            "status": "bounded_simulation_execution_released",
            "release_source": (
                "same_turn_explicit_user_instruction_after_i5_execution_readiness_closure"
            ),
            "explicit_user_execution_instruction_received": True,
            "execution_release_authorized": True,
            "authorization_envelope_alone_sufficient_for_execution": False,
        }
    )
    for key, value in expected.items():
        _require(_exact_equal(document[key], value), "execution release {} drifted".format(key))
    _require(
        type(document["release_id"]) is str and document["release_id"],
        "release_id must be text",
    )
    _require(
        type(document["release_date"]) is str and document["release_date"],
        "release_date must be explicit text",
    )
    scope = _closed_mapping(document["scope"], SCOPE_FIELDS, "execution release.scope")
    _require(_exact_equal(dict(scope), _expected_scope()), "execution release scope drifted")
    schedule_sha = _validate_schedule(document["exact_schedule"], "execution release.exact_schedule")
    _require(
        document["exact_schedule_sha256"] == schedule_sha,
        "execution release schedule digest drifted",
    )
    bindings = _closed_mapping(
        document["bound_resources"], set(expected_resource_paths), "execution release.bound_resources"
    )
    for label, expected_path in expected_resource_paths.items():
        _relative_parts(expected_path)
        row = _closed_mapping(bindings[label], {"path", "sha256"}, "release resource " + label)
        _require(row["path"] == expected_path, label + " release path drifted")
        _hex_digest(row["sha256"], label + " release SHA256")
    _hex_digest(document["dependency_closure_digest"], "release dependency closure digest")
    trust = _closed_mapping(
        document["release_trust_anchor"], RELEASE_TRUST_FIELDS, "execution release.release_trust_anchor"
    )
    _require(
        _exact_equal(
            dict(trust),
            {
                "mechanism": "caller_supplied_exact_execution_release_file_sha256",
                "self_hash_embedded": False,
                "guard_rejects_missing_or_mismatched_cli_hash": True,
                "authorization_hash_independently_supplied": True,
            },
        ),
        "execution release trust anchor drifted",
    )
    prejournal = _closed_mapping(
        document["prejournal_gate"], PREJOURNAL_FIELDS, "execution release.prejournal_gate"
    )
    _require(
        _exact_equal(
            dict(prejournal),
            {
                "release_validation_before_execution_state_creation_required": True,
                "authorization_revalidation_required": True,
                "all_bound_resources_rehashed_required": True,
                "closure_logical_digest_recomputed_required": True,
                "scene_children_rehashed_required": True,
                "machine_review_pass_required": True,
                "existing_execution_state_absent_required": True,
                "forbidden_processes_absent_required": True,
                "execution_state_creation_before_validation_allowed": False,
            },
        ),
        "execution release prejournal policy drifted",
    )
    completion = _closed_mapping(
        document["completion_boundary"], COMPLETION_FIELDS, "execution release.completion_boundary"
    )
    _require(
        _exact_equal(
            dict(completion),
            {
                "maximum_claim": "fresh_simulation_runtime_evaluator_semantic_integration",
                "safety_performance_generalization_claim_allowed": False,
                "formal_result_must_remain_false": True,
                "runtime_ready_must_remain_false": True,
                "downstream_authorization_after_completion": False,
            },
        ),
        "execution release completion boundary drifted",
    )
    return schedule_sha


def _validate_scene_index(document, resources, expected_resource_paths):
    child_labels = sorted(
        label for label in expected_resource_paths if label.startswith(SCENE_CHILD_LABEL_PREFIX)
    )
    _require(
        len(child_labels) == EXPECTED_SCENE_CHILD_COUNT,
        "release must bind exactly fourteen fresh scene children",
    )
    _require(document.get("formal_result") is False, "scene index formal_result drifted")
    _require(document.get("runtime_ready") is False, "scene index runtime_ready drifted")
    _require(
        type(document.get("scene_count")) is int and document["scene_count"] == 7,
        "scene index scene_count drifted",
    )
    rows = document.get("files")
    _require(isinstance(rows, list), "scene index files must be a list")
    expected_rows = []
    for label in child_labels:
        expected_rows.append(
            {
                "path": expected_resource_paths[label],
                "sha256": resources[label].sha256,
            }
        )
    for index, row in enumerate(rows):
        _closed_mapping(row, {"path", "sha256"}, "scene index file[{}]".format(index))
    _require(_exact_equal(rows, expected_rows), "scene index child closure drifted")
    paths = [row["path"] for row in rows]
    _require(len(paths) == len(set(paths)), "scene index child path duplicated")
    _require(
        sum(path.endswith(".instance.yaml") for path in paths) == 7
        and sum(path.endswith(".world") for path in paths) == 7,
        "scene index child type count drifted",
    )


def _validate_dependency_closure(cache, snapshot, release_resources):
    document = snapshot.document
    _require(isinstance(document, Mapping), "execution dependency closure is not YAML")
    _require(document.get("stage") == STAGE, "execution dependency closure stage drifted")
    _require(
        document.get("execution_authorized") is False,
        "readiness closure must remain non-authorizing",
    )
    _require(document.get("unresolved") == [], "execution dependency closure is unresolved")
    logical_digest = _logical_digest(document, "execution dependency closure")
    local = document.get("local")
    _require(isinstance(local, Mapping), "execution dependency closure local section missing")
    rows = local.get("files")
    _require(isinstance(rows, list) and rows, "execution dependency closure local files missing")
    local_paths = []
    for index, row in enumerate(rows):
        _closed_mapping(row, {"path", "sha256", "size_bytes"}, "closure local file[{}]".format(index))
        declared_path = row["path"]
        _relative_parts(declared_path)
        expected_sha = _hex_digest(row["sha256"], "closure local file SHA256")
        _require(type(row["size_bytes"]) is int and row["size_bytes"] >= 0, "closure local size invalid")
        item = cache.workspace_file(declared_path, parse_yaml=False)
        _require(
            hmac.compare_digest(item.sha256, expected_sha)
            and item.size_bytes == row["size_bytes"],
            "closure local resource drifted: " + declared_path,
        )
        local_paths.append(declared_path)
    _require(local_paths == sorted(set(local_paths)), "closure local paths are not unique and sorted")
    required_paths = local.get("required_paths")
    if required_paths is not None:
        _require(_exact_equal(required_paths, local_paths), "closure required_paths drifted")
    required_release_paths = {
        snapshot.declared_path
        for label, snapshot in release_resources.items()
        if label not in {"execution_dependency_closure", "execution_machine_review"}
    }
    _require(
        required_release_paths.issubset(set(local_paths)),
        "execution closure omits a release-bound runtime resource",
    )

    external = document.get("external")
    _require(isinstance(external, Mapping), "execution dependency closure external section missing")
    _require(external.get("unresolved") == [], "external dependency closure is unresolved")
    external_rows = external.get("files")
    _require(isinstance(external_rows, list), "external dependency files missing")
    external_paths = []
    for index, row in enumerate(external_rows):
        _closed_mapping(
            row,
            {"canonical_path", "sha256", "size_bytes"},
            "closure external file[{}]".format(index),
        )
        declared_path = row["canonical_path"]
        _absolute_parts(declared_path)
        expected_sha = _hex_digest(row["sha256"], "closure external SHA256")
        _require(type(row["size_bytes"]) is int and row["size_bytes"] >= 0, "closure external size invalid")
        item = cache.external_file(declared_path)
        _require(
            hmac.compare_digest(item.sha256, expected_sha)
            and item.size_bytes == row["size_bytes"],
            "closure external resource drifted: " + declared_path,
        )
        external_paths.append(declared_path)
    _require(
        external_paths == sorted(set(external_paths)),
        "closure external paths are not unique and sorted",
    )
    _logical_digest(external, "external dependency closure")
    runtime_executables = {}
    interpreter = external.get("python_interpreter")
    if isinstance(interpreter, Mapping):
        interpreter_path = interpreter.get("canonical_path")
        _require(interpreter_path in set(external_paths), "Python interpreter is not closed")
        runtime_executables["python_interpreter"] = interpreter_path
    bindings = external.get("runtime_bindings")
    _require(isinstance(bindings, list), "closure runtime bindings missing")
    binding_names = []
    for row in bindings:
        _require(isinstance(row, Mapping), "runtime binding must be a mapping")
        name = row.get("binding")
        target = row.get("target_canonical_path")
        _require(type(name) is str and name, "runtime binding name missing")
        _require(type(target) is str and target in set(external_paths), "runtime target is not closed")
        _require(name not in runtime_executables, "runtime binding duplicated")
        runtime_executables[name] = target
        binding_names.append(name)
    _require(binding_names == sorted(set(binding_names)), "runtime bindings are not unique and sorted")
    return logical_digest, runtime_executables


def _validate_machine_review(document, expected_status):
    _require(document.get("stage") == STAGE, "execution machine review stage drifted")
    _require(document.get("status") == expected_status, "execution machine review status drifted")
    _require(document.get("review_result") == "pass", "execution machine review did not pass")
    expected = {
        "execution_ready": False,
        "separate_execution_release_required": True,
        "separate_execution_release_present": False,
        "formal_result": False,
        "runtime_ready": False,
    }
    for key, value in expected.items():
        _require(
            key in document and _exact_equal(document[key], value),
            "execution machine review {} drifted".format(key),
        )
    absence = document.get("execution_absence_review")
    _require(isinstance(absence, Mapping), "execution absence review is missing")
    for key, value in {
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
    }.items():
        _require(
            key in absence and _exact_equal(absence[key], value),
            "execution absence review {} drifted".format(key),
        )
    side_effects = document.get("side_effects")
    _require(isinstance(side_effects, Mapping), "execution review side effects are missing")
    for key, value in {
        "execution_release_created": False,
        "attempt_root_created": False,
        "journal_created": False,
        "subprocess_started_by_review": False,
        "ros_started_by_review": False,
        "gazebo_started_by_review": False,
        "move_base_or_teb_started_by_review": False,
        "evidence_units_consumed": 0,
        "training_started": False,
    }.items():
        _require(
            key in side_effects and _exact_equal(side_effects[key], value),
            "execution review side effect {} drifted".format(key),
        )


def load_and_validate_execution_release(
    workspace,
    release_path,
    caller_release_sha256,
    authorization_path,
    caller_authorization_sha256,
    *,
    expected_resource_paths,
    expected_machine_review_status,
    preloaded_snapshots=None
):
    """Validate a future release without creating any execution state.

    ``expected_resource_paths`` is a trusted, reviewed label-to-relative-path
    mapping supplied by the entrypoint.  Paths learned only from the release
    never select a file.  Every selected file is opened once through a
    no-follow descriptor chain and the same payload is used for hash and parse.
    """

    root = Path(workspace)
    _require(root.is_absolute() and root == root.resolve(), "workspace is not canonical")
    release_declared = Path(release_path).as_posix()
    authorization_declared = Path(authorization_path).as_posix()
    _require(release_declared == CANONICAL_RELEASE_PATH, "release path drifted")
    _require(authorization_declared == CANONICAL_AUTHORIZATION_PATH, "authorization path drifted")
    caller_release_sha = _hex_digest(caller_release_sha256, "caller release SHA256")
    caller_authorization_sha = _hex_digest(
        caller_authorization_sha256, "caller authorization SHA256"
    )
    _require(
        isinstance(expected_resource_paths, Mapping),
        "expected resource paths must be a mapping",
    )
    expected_paths = dict(expected_resource_paths)
    _require(
        _exact_equal(expected_paths, EXPECTED_RELEASE_RESOURCE_PATHS),
        "trusted release resource roster drifted",
    )
    _require(
        type(expected_machine_review_status) is str
        and expected_machine_review_status == EXPECTED_MACHINE_REVIEW_STATUS,
        "trusted machine review status drifted",
    )
    _require(
        expected_paths["authorization_envelope"] == authorization_declared,
        "trusted authorization path drifted",
    )
    child_labels = {
        label for label in expected_paths if label.startswith(SCENE_CHILD_LABEL_PREFIX)
    }
    _require(
        child_labels
        == {
            "{}{:02d}".format(SCENE_CHILD_LABEL_PREFIX, index)
            for index in range(1, EXPECTED_SCENE_CHILD_COUNT + 1)
        },
        "trusted scene child label set drifted",
    )
    all_paths = list(expected_paths.values())
    _require(
        all(type(path) is str for path in all_paths)
        and len(all_paths) == len(set(all_paths)),
        "trusted release paths must be unique strings",
    )
    for path in all_paths:
        _relative_parts(path)

    cache = _SnapshotCache(root, preloaded=preloaded_snapshots)
    release_snapshot = cache.workspace_file(release_declared, parse_yaml=False)
    _require(
        hmac.compare_digest(release_snapshot.sha256, caller_release_sha),
        "release trust-anchor hash mismatch",
    )
    release_snapshot = cache.workspace_file(release_declared, parse_yaml=True)
    schedule_sha = _validate_release_document(release_snapshot.document, expected_paths)
    release_bindings = release_snapshot.document["bound_resources"]

    resources = {}  # type: Dict[str, FileSnapshot]
    for label, declared_path in expected_paths.items():
        snapshot = cache.workspace_file(declared_path, parse_yaml=False)
        expected_sha = release_bindings[label]["sha256"]
        _require(
            hmac.compare_digest(snapshot.sha256, expected_sha),
            "release-bound resource drifted: " + label,
        )
        if label in RELEASE_PARSED_RESOURCE_LABELS:
            snapshot = cache.workspace_file(declared_path, parse_yaml=True)
        else:
            _require(
                snapshot.document is None,
                "hash-only release resource was parsed: " + label,
            )
        resources[label] = snapshot

    authorization_snapshot = resources["authorization_envelope"]
    _require(
        hmac.compare_digest(authorization_snapshot.sha256, caller_authorization_sha),
        "authorization trust-anchor hash mismatch",
    )
    _require(
        hmac.compare_digest(
            release_bindings["authorization_envelope"]["sha256"],
            caller_authorization_sha,
        ),
        "release authorization binding disagrees with caller hash",
    )
    preregistration_snapshot = resources["preregistration"]

    authorization_bindings = _closed_mapping(
        authorization_snapshot.document.get("bound_resources"),
        set(EXPECTED_AUTHORIZATION_RESOURCE_PATHS),
        "authorization.bound_resources",
    )
    authorization_resources = {}
    for label, expected_path in EXPECTED_AUTHORIZATION_RESOURCE_PATHS.items():
        row = authorization_bindings[label]
        _closed_mapping(row, {"path", "sha256"}, "authorization resource " + str(label))
        declared_path = row["path"]
        _relative_parts(declared_path)
        _require(
            declared_path == expected_path,
            "authorization resource path drifted: " + label,
        )
        snapshot = cache.workspace_file(declared_path, parse_yaml=False)
        _require(
            hmac.compare_digest(snapshot.sha256, _hex_digest(row["sha256"], "authorization resource SHA256")),
            "authorization-bound resource drifted: " + str(label),
        )
        if label in AUTHORIZATION_PARSED_RESOURCE_LABELS:
            snapshot = cache.workspace_file(declared_path, parse_yaml=True)
        else:
            _require(
                snapshot.document is None,
                "hash-only authorization resource was parsed: " + label,
            )
        authorization_resources[label] = snapshot
    _require(
        AUTHORIZATION_PREREGISTRATION_LABEL in authorization_resources
        and authorization_resources[AUTHORIZATION_PREREGISTRATION_LABEL].declared_path
        == preregistration_snapshot.declared_path
        and authorization_resources[AUTHORIZATION_PREREGISTRATION_LABEL].sha256
        == preregistration_snapshot.sha256,
        "authorization preregistration binding drifted",
    )
    _require(
        AUTHORIZATION_CLOSURE_LABEL in authorization_resources
        and authorization_resources[AUTHORIZATION_CLOSURE_LABEL].document is not None,
        "authorization dependency closure binding missing",
    )
    authorization_schedule_sha = _validate_authorization(
        authorization_snapshot.document,
        preregistration_snapshot.document,
        authorization_resources[AUTHORIZATION_CLOSURE_LABEL].document,
    )
    _require(authorization_schedule_sha == schedule_sha, "release and authorization schedule differ")

    _validate_scene_index(
        resources["fresh_scene_index"].document, resources, expected_paths
    )
    closure_snapshot = resources["execution_dependency_closure"]
    closure_digest, runtime_executables = _validate_dependency_closure(
        cache, closure_snapshot, resources
    )
    _require(
        closure_digest == release_snapshot.document["dependency_closure_digest"],
        "release dependency closure logical digest drifted",
    )
    machine_review_snapshot = resources["execution_machine_review"]
    _validate_machine_review(
        machine_review_snapshot.document, expected_machine_review_status
    )
    return ExecutionReleaseValidation(
        release=release_snapshot,
        authorization=authorization_snapshot,
        preregistration=preregistration_snapshot,
        bound_resources=dict(resources),
        authorization_bound_resources=dict(authorization_resources),
        release_parsed_labels=tuple(sorted(RELEASE_PARSED_RESOURCE_LABELS)),
        release_hash_only_labels=tuple(
            sorted(set(EXPECTED_RELEASE_RESOURCE_PATHS) - RELEASE_PARSED_RESOURCE_LABELS)
        ),
        authorization_parsed_labels=tuple(
            sorted(AUTHORIZATION_PARSED_RESOURCE_LABELS)
        ),
        authorization_hash_only_labels=tuple(
            sorted(
                set(EXPECTED_AUTHORIZATION_RESOURCE_PATHS)
                - AUTHORIZATION_PARSED_RESOURCE_LABELS
            )
        ),
        dependency_closure=closure_snapshot,
        machine_review=machine_review_snapshot,
        schedule_sha256=schedule_sha,
        identity_count=len(EXPECTED_SCHEDULE),
        execution_seeds=tuple(EXPECTED_EXECUTION_SEEDS),
        runtime_executables=dict(runtime_executables),
    )


__all__ = [
    "AUTHORIZATION_PARSED_RESOURCE_LABELS",
    "BASIS_STAGE",
    "CANONICAL_AUTHORIZATION_PATH",
    "CANONICAL_RELEASE_PATH",
    "EXPECTED_AUTHORIZATION_RESOURCE_PATHS",
    "EXPECTED_MACHINE_REVIEW_STATUS",
    "EXPECTED_RELEASE_RESOURCE_PATHS",
    "EXPECTED_SCHEDULE",
    "EXPECTED_SCHEDULE_SHA256",
    "ExecutionReleaseValidation",
    "FileSnapshot",
    "RELEASE_PARSED_RESOURCE_LABELS",
    "R6I5ExecutionReleaseError",
    "STAGE",
    "canonical_document_sha256",
    "load_and_validate_execution_release",
    "read_workspace_file_once",
]
