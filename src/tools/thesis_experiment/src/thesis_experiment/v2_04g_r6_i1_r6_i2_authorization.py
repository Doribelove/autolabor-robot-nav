"""Fail-closed authorization handling for the offline-only R6-I2 review.

The helpers in this module do not authorize execution.  They define the
single-open/no-follow and closed-schema checks that a later, separately
authorized executor must call before it creates a journal or subprocess.
"""

from dataclasses import dataclass
import hashlib
import hmac
import os
from pathlib import Path
import stat
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import yaml


MAX_RESOURCE_BYTES = 32 * 1024 * 1024
SCHEDULE_FIELDS = {
    "sequence",
    "profile_id",
    "scene_id",
    "seed",
    "attempt",
    "expected_ttc_status",
    "expected_overlay_semantics",
}
TOP_LEVEL_FIELDS = {
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
TRUST_ANCHOR_FIELDS = {
    "mechanism",
    "self_hash_embedded",
    "guard_rejects_missing_or_mismatched_cli_hash",
}
COMPLETION_FIELDS = {
    "maximum_claim",
    "safety_performance_generalization_claim_allowed",
    "formal_result_must_remain_false",
    "runtime_ready_must_remain_false",
    "downstream_authorization_after_completion",
}


class R6I2AuthorizationError(ValueError):
    """Raised when an authorization or one of its resources fails closed."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise R6I2AuthorizationError(
                "duplicate YAML key: {!r}".format(key)
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class FileSnapshot:
    """Bytes and digest obtained from one stable, no-follow file descriptor."""

    declared_path: str
    path: Path
    sha256: str
    payload: bytes
    document: Optional[Mapping[str, Any]]


@dataclass(frozen=True)
class AuthorizationValidation:
    """Validated authorization plus the exact resource snapshots it bound."""

    authorization: FileSnapshot
    preregistration: FileSnapshot
    bound_resources: Mapping[str, FileSnapshot]
    schedule_sha256: str
    identity_count: int
    execution_seeds: Tuple[int, ...]


def _require(condition, message):
    if not condition:
        raise R6I2AuthorizationError(message)


def _hex_digest(value, label):
    _require(
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value),
        "{} must be a lowercase SHA256".format(label),
    )
    return value


def _closed_mapping(value, fields, label):
    _require(isinstance(value, Mapping), "{} must be a mapping".format(label))
    actual = set(value)
    expected = set(fields)
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
        return (
            set(actual) == set(expected)
            and all(_exact_equal(actual[key], expected[key]) for key in expected)
        )
    if isinstance(expected, list):
        return (
            len(actual) == len(expected)
            and all(
                _exact_equal(left, right)
                for left, right in zip(actual, expected)
            )
        )
    return actual == expected


def canonical_document_sha256(value):
    """Return a type-stable digest for an already parsed YAML value."""

    import json

    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _declared_parts(declared_path):
    _require(
        isinstance(declared_path, str) and declared_path,
        "declared path must be a non-empty string",
    )
    declared = Path(declared_path)
    _require(not declared.is_absolute(), "declared path must be relative")
    _require(
        ".." not in declared.parts and "." not in declared.parts,
        "declared path contains traversal",
    )
    _require(
        all(part not in ("", os.curdir, os.pardir) for part in declared.parts),
        "declared path contains an unsafe component",
    )
    return declared.parts


def _read_relative_bytes_once(workspace, declared_path):
    """Read one workspace file without following any path-component symlink."""

    root = Path(workspace)
    _require(root.is_absolute(), "workspace must be absolute")
    _require(
        root == root.resolve(),
        "workspace must be a canonical path",
    )
    _require(root.is_dir() and not root.is_symlink(), "workspace is unsafe")
    parts = _declared_parts(declared_path)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = (
        flags
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors = []
    try:
        current = os.open(str(root), directory_flags)
        descriptors.append(current)
        for component in parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        file_descriptor = os.open(
            parts[-1],
            flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=current,
        )
        descriptors.append(file_descriptor)
        before = os.fstat(file_descriptor)
        _require(
            stat.S_ISREG(before.st_mode),
            "{} is not a regular file".format(declared_path),
        )
        _require(
            before.st_size <= MAX_RESOURCE_BYTES,
            "{} exceeds the resource size limit".format(declared_path),
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
            "st_size",
            "st_mtime_ns",
            "st_ctime_ns",
        )
        _require(
            all(
                getattr(before, field) == getattr(after, field)
                for field in stable_fields
            )
            and len(payload) == before.st_size,
            "{} changed during its single read".format(declared_path),
        )
        return payload
    except OSError as exc:
        raise R6I2AuthorizationError(
            "cannot safely open {}: {}".format(declared_path, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _parse_yaml_mapping(payload, label):
    try:
        document = yaml.load(
            payload.decode("utf-8"),
            Loader=_UniqueKeyLoader,
        )
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6I2AuthorizationError(
            "cannot strictly parse {}: {}".format(label, exc)
        ) from exc
    _require(
        isinstance(document, Mapping),
        "{} must contain one mapping".format(label),
    )
    return document


def read_workspace_file_once(workspace, declared_path, parse_yaml=False):
    """Return one immutable snapshot used for both hashing and parsing."""

    payload = _read_relative_bytes_once(Path(workspace), declared_path)
    document = (
        _parse_yaml_mapping(payload, declared_path) if parse_yaml else None
    )
    return FileSnapshot(
        declared_path=declared_path,
        path=Path(workspace) / declared_path,
        sha256=hashlib.sha256(payload).hexdigest(),
        payload=payload,
        document=document,
    )


def read_workspace_yaml_once(
    workspace, declared_path, expected_sha256=None
):
    """Single-open/no-follow YAML hash+parse primitive."""

    snapshot = read_workspace_file_once(
        workspace, declared_path, parse_yaml=True
    )
    if expected_sha256 is not None:
        expected = _hex_digest(expected_sha256, "expected SHA256")
        _require(
            hmac.compare_digest(snapshot.sha256, expected),
            "{} hash mismatched".format(declared_path),
        )
    return snapshot


def _validate_schedule(preregistration):
    schedule = preregistration.get("schedule")
    _require(
        isinstance(schedule, list) and schedule,
        "preregistration schedule must be a non-empty list",
    )
    identities = set()
    profiles = []
    seeds = []
    for index, row in enumerate(schedule, start=1):
        _closed_mapping(row, SCHEDULE_FIELDS, "schedule[{}]".format(index - 1))
        _require(
            type(row["sequence"]) is int and row["sequence"] == index,
            "schedule sequence must be contiguous and one-based",
        )
        _require(
            isinstance(row["profile_id"], str) and row["profile_id"],
            "schedule profile_id must be a non-empty string",
        )
        _require(
            isinstance(row["scene_id"], str) and row["scene_id"],
            "schedule scene_id must be a non-empty string",
        )
        _require(
            type(row["seed"]) is int and row["seed"] >= 0,
            "schedule seed must be a non-negative integer",
        )
        _require(
            type(row["attempt"]) is int and row["attempt"] == 1,
            "schedule attempt must be exactly one",
        )
        _require(
            row["expected_ttc_status"]
            in {"OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON"},
            "schedule expected TTC status is invalid",
        )
        _require(
            isinstance(row["expected_overlay_semantics"], str)
            and row["expected_overlay_semantics"],
            "schedule overlay semantics must be a non-empty string",
        )
        identity = (
            row["profile_id"],
            row["scene_id"],
            row["seed"],
            row["attempt"],
        )
        _require(identity not in identities, "schedule identity is duplicated")
        identities.add(identity)
        if row["profile_id"] not in profiles:
            profiles.append(row["profile_id"])
        if row["seed"] not in seeds:
            seeds.append(row["seed"])
    return schedule, profiles, seeds


def _validate_preregistration(preregistration, expected_stage):
    _require(
        preregistration.get("stage") == expected_stage,
        "preregistration stage drifted",
    )
    _require(
        preregistration.get("execution_authorized") is False,
        "preregistration must remain non-authorizing",
    )
    schedule, profiles, seeds = _validate_schedule(preregistration)
    firewall = preregistration.get("fresh_seed_firewall")
    budget = preregistration.get("budget")
    _require(isinstance(firewall, Mapping), "fresh seed firewall is missing")
    _require(isinstance(budget, Mapping), "preregistration budget is missing")
    _require(
        _exact_equal(firewall.get("execution_seeds"), seeds),
        "preregistration execution seed list disagrees with schedule",
    )
    _require(
        type(budget.get("evidence_units_authorizable")) is int
        and budget["evidence_units_authorizable"] == len(schedule),
        "preregistration evidence budget disagrees with schedule",
    )
    _require(
        budget.get("attempt_limit_per_identity") == 1
        and budget.get("retry_allowed") is False
        and budget.get("resume_allowed") is False
        and budget.get("replacement_seed_allowed") is False
        and budget.get("budget_expansion_allowed") is False,
        "preregistration retry/resume/budget boundary drifted",
    )
    return schedule, profiles, seeds


def _validate_authorization_document(
    authorization,
    preregistration,
    expected_stage,
    expected_resource_paths,
    expected_dependency_closure_digest,
):
    _closed_mapping(authorization, TOP_LEVEL_FIELDS, "authorization")
    schedule, profiles, seeds = _validate_preregistration(
        preregistration, expected_stage
    )
    scalar_expectations = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": expected_stage,
        "status": "bounded_fresh_seed_simulation_authorized",
        "authorization_source":
            "explicit_user_instruction_after_independent_integration_review",
        "execution_authorized": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "evidence_budget_authorized": len(schedule),
        "fresh_execution_seeds": seeds,
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
    for key, expected in scalar_expectations.items():
        _require(
            _exact_equal(authorization[key], expected),
            "authorization {} drifted".format(key),
        )
    _require(
        isinstance(authorization["authorization_id"], str)
        and authorization["authorization_id"],
        "authorization_id must be a non-empty string",
    )
    _require(
        isinstance(authorization["authorization_date"], str)
        and authorization["authorization_date"],
        "authorization_date must be an explicit string",
    )
    _require(
        _exact_equal(authorization["exact_schedule"], schedule),
        "authorization exact schedule differs from preregistration",
    )
    schedule_sha256 = canonical_document_sha256(schedule)
    _require(
        authorization["preregistration_schedule_sha256"] == schedule_sha256,
        "authorization schedule digest drifted",
    )

    scope = _closed_mapping(
        authorization["scope"], SCOPE_FIELDS, "authorization.scope"
    )
    expected_scope = {
        "purpose":
            "runtime_evaluator_semantic_and_execution_integrity_validation",
        "stage_only": expected_stage,
        "profiles": profiles,
        "fresh_execution_seeds": seeds,
        "exact_identity_count": len(schedule),
        "component_stage_authorized": False,
        "general_navigation_calibration_authorized": False,
        "winner_selection_authorized": False,
    }
    _require(
        _exact_equal(dict(scope), expected_scope),
        "authorization scope drifted",
    )

    trust = _closed_mapping(
        authorization["authorization_trust_anchor"],
        TRUST_ANCHOR_FIELDS,
        "authorization.authorization_trust_anchor",
    )
    _require(
        _exact_equal(
            dict(trust),
            {
                "mechanism":
                    "caller_supplied_exact_authorization_file_sha256",
                "self_hash_embedded": False,
                "guard_rejects_missing_or_mismatched_cli_hash": True,
            },
        ),
        "authorization trust-anchor policy drifted",
    )
    completion = _closed_mapping(
        authorization["completion_boundary"],
        COMPLETION_FIELDS,
        "authorization.completion_boundary",
    )
    _require(
        _exact_equal(
            dict(completion),
            {
                "maximum_claim":
                    "fresh_simulation_runtime_evaluator_semantic_integration",
                "safety_performance_generalization_claim_allowed": False,
                "formal_result_must_remain_false": True,
                "runtime_ready_must_remain_false": True,
                "downstream_authorization_after_completion": False,
            },
        ),
        "authorization completion boundary drifted",
    )
    expected_digest = _hex_digest(
        expected_dependency_closure_digest,
        "expected dependency closure digest",
    )
    _require(
        authorization["dependency_closure_digest"] == expected_digest,
        "authorization dependency closure digest drifted",
    )

    bindings = authorization["bound_resources"]
    _closed_mapping(
        bindings,
        set(expected_resource_paths),
        "authorization.bound_resources",
    )
    for label, expected_path in expected_resource_paths.items():
        row = _closed_mapping(
            bindings[label], {"path", "sha256"}, "bound resource " + label
        )
        _require(
            row["path"] == expected_path,
            "{} bound path drifted".format(label),
        )
        _hex_digest(row["sha256"], "{} bound SHA256".format(label))
    return schedule_sha256, schedule, seeds


def load_and_validate_authorization(
    workspace,
    authorization_path,
    caller_authorization_sha256,
    expected_stage,
    preregistration_label,
    expected_resource_paths,
    expected_dependency_closure_digest,
    dependency_closure_label="dependency_closure",
):
    """Load and validate a later authorization without any hash/parse reread.

    ``expected_resource_paths`` is a trusted label-to-relative-path mapping
    frozen by the reviewed executor.  The authorization must bind exactly
    those labels, and every resource is opened once without following
    symlinks.  The preregistration snapshot is reused from that cache.
    """

    caller_digest = _hex_digest(
        caller_authorization_sha256, "caller authorization SHA256"
    )
    authorization_snapshot = read_workspace_yaml_once(
        workspace, authorization_path
    )
    _require(
        hmac.compare_digest(authorization_snapshot.sha256, caller_digest),
        "authorization trust-anchor hash mismatch",
    )
    authorization = authorization_snapshot.document
    _closed_mapping(authorization, TOP_LEVEL_FIELDS, "authorization")
    bindings = authorization.get("bound_resources")
    _require(
        isinstance(bindings, Mapping),
        "authorization bound_resources must be a mapping",
    )
    _require(
        set(bindings) == set(expected_resource_paths),
        "authorization bound resource label set drifted",
    )

    snapshots: Dict[str, FileSnapshot] = {}
    for label, expected_path in expected_resource_paths.items():
        row = bindings.get(label)
        _closed_mapping(row, {"path", "sha256"}, "bound resource " + label)
        _require(row["path"] == expected_path, label + " bound path drifted")
        expected_sha = _hex_digest(
            row["sha256"], "{} bound SHA256".format(label)
        )
        parse_yaml = Path(expected_path).suffix in {".yaml", ".yml"}
        snapshot = read_workspace_file_once(
            workspace, expected_path, parse_yaml=parse_yaml
        )
        _require(
            hmac.compare_digest(snapshot.sha256, expected_sha),
            "{} bound hash drifted".format(label),
        )
        snapshots[label] = snapshot
    _require(
        preregistration_label in snapshots
        and snapshots[preregistration_label].document is not None,
        "preregistration binding is missing or is not YAML",
    )
    _require(
        dependency_closure_label in snapshots
        and snapshots[dependency_closure_label].document is not None,
        "dependency closure binding is missing or is not YAML",
    )
    preregistration_snapshot = snapshots[preregistration_label]
    closure_snapshot = snapshots[dependency_closure_label]
    expected_closure_digest = _hex_digest(
        expected_dependency_closure_digest,
        "expected dependency closure digest",
    )
    _require(
        closure_snapshot.document.get("closure_sha256")
        == expected_closure_digest,
        "bound dependency closure logical digest drifted",
    )
    schedule_sha, schedule, seeds = _validate_authorization_document(
        authorization,
        preregistration_snapshot.document,
        expected_stage,
        expected_resource_paths,
        expected_closure_digest,
    )
    return AuthorizationValidation(
        authorization=authorization_snapshot,
        preregistration=preregistration_snapshot,
        bound_resources=snapshots,
        schedule_sha256=schedule_sha,
        identity_count=len(schedule),
        execution_seeds=tuple(seeds),
    )
