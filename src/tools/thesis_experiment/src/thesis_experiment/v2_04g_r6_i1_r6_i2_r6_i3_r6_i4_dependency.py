"""Deterministic offline dependency closure for the R6-I4 repair review.

R6-I4 is an independent, zero-allocation preflight-integrity repair stage.  It
does not create or authorize an execution release.  This module binds every
I4 source and test, rehashes the real twelve-resource R6-I3 authorization
roster, and independently revalidates every target in the frozen R6-I3 and
R6-I2 closures.  Building or verifying the closure never starts a process or
creates execution state.

The failed R6-I3 release is a required historical input.  The future R6-I4
release, this closure itself, and the final machine-review artifact are kept
out of the hash graph so that a later separately authorized release can bind
the completed closure and review without a cycle.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

from thesis_experiment import v2_04g_r6_i1_r6_i2_dependency as i2_dependency


STAGE = "V2-04G-R6-I4"
SOURCE_STAGE = "V2-04G-R6-I3"
SCHEMA_VERSION = "5.0"

ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i4_preflight_repair_review"
)
EXECUTION_CLOSURE = ARTIFACT_ROOT / "execution_dependency_closure.yaml"
MACHINE_REVIEW = (
    ARTIFACT_ROOT
    / "v2_04g_r6_i4_preflight_integrity_readiness_review.yaml"
)
FAILED_I3_RELEASE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_release.yaml"
)
FUTURE_I4_RELEASE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i4_execution_release.yaml"
)
I3_ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)

I4_CONTRACT = (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_preflight_integrity_repair_contract.yaml"
)
I4_PREREGISTRATION = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i4_repair_preregistration.yaml"
)
I4_TRANSITION = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i4_stage_transition.yaml"
)
I4_VALIDATOR = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
)
I4_VALIDATOR_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i4_release_validator.py"
)
I4_PREFLIGHT_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i4_preflight_integrity.py"
)
I4_RUNNER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_bounded_validation.py"
)
I4_DEPENDENCY = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency.py"
)
I4_GENERATOR = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency_generator.py"
)
I4_REVIEWER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_preflight_integrity_reviewer.py"
)

I3_AUTHORIZATION = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml"
)
I3_FAILURE_HANDOFF = (
    "docs/thesis_experiment/"
    "CURRENT_V2_04G_R6_I3_RELEASE_PREFLIGHT_HANDOFF.md"
)
I3_CLOSURE = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/"
    "execution_dependency_closure.yaml"
)
I3_REVIEW = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/"
    "v2_04g_r6_i3_execution_readiness_review.yaml"
)
I2_CLOSURE = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "execution_dependency_closure.yaml"
)
I2_REVIEW = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_integration_review.yaml"
)

ENTRYPOINTS = (I4_RUNNER, I4_GENERATOR, I4_REVIEWER)
MANDATORY_INPUTS = (
    I4_CONTRACT,
    I4_PREREGISTRATION,
    I4_TRANSITION,
    I4_VALIDATOR,
    I4_VALIDATOR_TEST,
    I4_PREFLIGHT_TEST,
    I4_DEPENDENCY,
    FAILED_I3_RELEASE.as_posix(),
    I3_AUTHORIZATION,
    I3_FAILURE_HANDOFF,
    I3_CLOSURE,
    I3_REVIEW,
    I2_CLOSURE,
    I2_REVIEW,
)

EXPECTED_FIXED_FILE_SHA256 = {
    I4_CONTRACT: (
        "a5ac55ecd84a59a847e92e4268c95983312e39720fbd1839871bb25f90e158a4"
    ),
    I4_PREREGISTRATION: (
        "4abd2bdaf50ef5b0100494d5ef23d6628e0db03a7513f8a7aa3803f1224e701a"
    ),
    I4_TRANSITION: (
        "99161770dbc0da0699868ce9a37b53b22d42214edff10bcb2ea191c3f2f49c2d"
    ),
    FAILED_I3_RELEASE.as_posix(): (
        "5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6"
    ),
    I3_AUTHORIZATION: (
        "ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2"
    ),
    I3_FAILURE_HANDOFF: (
        "eeb9734b3d6e6c4d531f60d4bd162f24512f0d549d601395d1f7654169ef1755"
    ),
    I3_CLOSURE: (
        "55f0e343788409301258da96f355e78d9fb689bdcc270ef8f5b9fe54b06a4b37"
    ),
    I3_REVIEW: (
        "3a8e9e466a08f7d3ef65542b285cf5020e6ce53dbb63cbe9b2316afb717a680e"
    ),
    I2_CLOSURE: (
        "63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58"
    ),
    I2_REVIEW: (
        "b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61"
    ),
    I4_VALIDATOR: (
        "9b9dd3fc580d0f880705bf87e120cdb9d30fdce812f585ee99e3f0fdf1fa3994"
    ),
    I4_VALIDATOR_TEST: (
        "663fd2da6a8781e3cc4041aad46141e3ffdfee38883d4bcf2fe0e1eb59cc3a89"
    ),
}

EXPECTED_AUTHORIZATION_LABELS = (
    "preregistration",
    "r6_i2_contract",
    "inherited_r6_i2_dependency_closure",
    "r6_i2_integration_review",
    "r6_i2_authorization_component_review",
    "r6_i2_authorization_module",
    "r6_i1_scene_derivation",
    "source_r6_i1_compiled_scene_index",
    "legacy_supervisor",
    "aligned_supervisor",
    "frozen_evaluator",
    "r6_design_report",
)

EXPECTED_I3_CLOSURE = {
    "stage": "V2-04G-R6-I3",
    "file_sha256": EXPECTED_FIXED_FILE_SHA256[I3_CLOSURE],
    "logical_sha256": (
        "f83beb04dc6e7cd1e43c2611997f86dbd5bf07c36f33d51fc979128f2cb4ed4f"
    ),
    "local_file_count": 136,
    "external_file_count": 307,
    "external_python_binding_count": 47,
    "external_runtime_binding_count": 9,
}
EXPECTED_I2_CLOSURE = {
    "stage": "V2-04G-R6-I2",
    "file_sha256": EXPECTED_FIXED_FILE_SHA256[I2_CLOSURE],
    "logical_sha256": (
        "2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6"
    ),
    "local_file_count": 106,
    "external_file_count": 301,
    "external_python_binding_count": 45,
    "external_runtime_binding_count": 5,
}

FORBIDDEN_EXECUTION_STATE = (
    FUTURE_I4_RELEASE,
    I3_ARTIFACT_ROOT / "attempts",
    I3_ARTIFACT_ROOT / "journals",
    I3_ARTIFACT_ROOT / "receipts",
    I3_ARTIFACT_ROOT / "raw_evidence",
    I3_ARTIFACT_ROOT / "semantic_evidence",
    I3_ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml",
    I3_ARTIFACT_ROOT / "v2_04g_r6_i3_execution_report.yaml",
    ARTIFACT_ROOT / "attempts",
    ARTIFACT_ROOT / "journals",
    ARTIFACT_ROOT / "receipts",
    ARTIFACT_ROOT / "raw_evidence",
    ARTIFACT_ROOT / "semantic_evidence",
    ARTIFACT_ROOT / "ros_home",
    ARTIFACT_ROOT / "ros_logs",
    ARTIFACT_ROOT / "v2_04g_r6_i4_stage_report.yaml",
    ARTIFACT_ROOT / "v2_04g_r6_i4_execution_report.yaml",
)


class R6I4DependencyError(ValueError):
    """Raised when the I4 offline dependency boundary is incomplete."""


def _canonical_json_sha(value: Mapping[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _safe_relative(value: str) -> str:
    candidate = Path(value)
    if (
        candidate.is_absolute()
        or not candidate.parts
        or any(part in {"", ".", ".."} for part in candidate.parts)
    ):
        raise R6I4DependencyError(
            "unsafe workspace-relative dependency: {}".format(value)
        )
    return candidate.as_posix()


def _record(workspace: Path, relative: str) -> dict:
    root = Path(workspace).resolve()
    safe = _safe_relative(relative)
    record = i2_dependency.canonical_file_record(root / safe)
    try:
        canonical_relative = Path(record["canonical_path"]).relative_to(root)
    except ValueError as exc:
        raise R6I4DependencyError(
            "workspace dependency escaped root: {}".format(relative)
        ) from exc
    if canonical_relative.as_posix() != safe:
        raise R6I4DependencyError(
            "workspace dependency canonical path drifted: {}".format(relative)
        )
    return {
        "path": safe,
        "sha256": record["sha256"],
        "size_bytes": record["size_bytes"],
    }


def _verify_fixed_files(workspace: Path) -> list:
    rows = []
    for relative in sorted(EXPECTED_FIXED_FILE_SHA256):
        row = _record(workspace, relative)
        if row["sha256"] != EXPECTED_FIXED_FILE_SHA256[relative]:
            raise R6I4DependencyError(
                "fixed I4 or inherited resource drifted: {}".format(relative)
            )
        rows.append(row)
    return rows


def _verify_state_boundary(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    release = _record(root, FAILED_I3_RELEASE.as_posix())
    expected_release = EXPECTED_FIXED_FILE_SHA256[FAILED_I3_RELEASE.as_posix()]
    if release["sha256"] != expected_release:
        raise R6I4DependencyError("failed R6-I3 release digest drifted")
    present = [
        relative.as_posix()
        for relative in FORBIDDEN_EXECUTION_STATE
        if os.path.lexists(str(root / relative))
    ]
    if present:
        raise R6I4DependencyError(
            "forbidden execution state exists during I4 closure: {}".format(
                present
            )
        )
    return {
        "failed_i3_release_present": True,
        "failed_i3_release_sha256": release["sha256"],
        "future_i4_release_present": False,
        "i3_authorized_units": 6,
        "i3_consumed_units": 0,
        "i3_forfeited_units": 0,
        "i4_authorized_units": 0,
        "i4_consumed_units": 0,
        "i4_forfeited_units": 0,
        "forbidden_state_present": [],
        "pass": True,
    }


def _authorization_roster(workspace: Path) -> Tuple[list, Tuple[str, ...]]:
    root = Path(workspace).resolve()
    document = i2_dependency._load_yaml_mapping(
        root / I3_AUTHORIZATION, "frozen R6-I3 authorization envelope"
    )
    if (
        document.get("stage") != SOURCE_STAGE
        or document.get("execution_authorized") is not True
        or document.get("evidence_budget_authorized") != 6
    ):
        raise R6I4DependencyError("historical authorization boundary drifted")
    resources = document.get("bound_resources")
    if not isinstance(resources, dict):
        raise R6I4DependencyError("historical authorization roster is missing")
    if tuple(resources) != EXPECTED_AUTHORIZATION_LABELS:
        raise R6I4DependencyError(
            "historical authorization roster labels/order drifted"
        )
    rows = []
    paths = []
    for label in EXPECTED_AUTHORIZATION_LABELS:
        row = resources[label]
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256"}
            or not isinstance(row.get("path"), str)
            or not isinstance(row.get("sha256"), str)
        ):
            raise R6I4DependencyError(
                "authorization resource row drifted: {}".format(label)
            )
        actual = _record(root, row["path"])
        if actual["sha256"] != row["sha256"]:
            raise R6I4DependencyError(
                "authorization resource bytes drifted: {}".format(label)
            )
        rows.append(
            {
                "label": label,
                "path": actual["path"],
                "sha256": actual["sha256"],
                "size_bytes": actual["size_bytes"],
                "parsed_by_i4_release_validator": label
                in {"preregistration", "inherited_r6_i2_dependency_closure"},
            }
        )
        paths.append(actual["path"])
    if len(paths) != 12 or len(set(paths)) != 12:
        raise R6I4DependencyError("authorization resource paths are not closed")
    return rows, tuple(paths)


def _verify_frozen_closure(
    workspace: Path,
    relative: str,
    expected: Mapping[str, object],
) -> dict:
    """Verify a frozen closure file, logical hash, and every target."""

    root = Path(workspace).resolve()
    file_record = _record(root, relative)
    if file_record["sha256"] != expected["file_sha256"]:
        raise R6I4DependencyError(
            "frozen closure file digest drifted: {}".format(relative)
        )
    document = i2_dependency._load_yaml_mapping(
        root / relative, "frozen dependency closure {}".format(relative)
    )
    logical_payload = {
        key: value for key, value in document.items() if key != "closure_sha256"
    }
    logical = _canonical_json_sha(logical_payload)
    if (
        document.get("stage") != expected["stage"]
        or document.get("closure_sha256") != expected["logical_sha256"]
        or logical != expected["logical_sha256"]
    ):
        raise R6I4DependencyError(
            "frozen closure logical digest drifted: {}".format(relative)
        )
    local = document.get("local")
    if not isinstance(local, dict) or not isinstance(local.get("files"), list):
        raise R6I4DependencyError(
            "frozen closure local section is missing: {}".format(relative)
        )
    local_paths = []
    for row in local["files"]:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256", "size_bytes"}
        ):
            raise R6I4DependencyError(
                "frozen closure local record drifted: {}".format(relative)
            )
        actual = _record(root, row["path"])
        if (
            actual["sha256"] != row["sha256"]
            or actual["size_bytes"] != row["size_bytes"]
        ):
            raise R6I4DependencyError(
                "frozen closure local target drifted: {}".format(row["path"])
            )
        local_paths.append(row["path"])
    if (
        local_paths != sorted(set(local_paths))
        or len(local_paths) != expected["local_file_count"]
    ):
        raise R6I4DependencyError(
            "frozen closure local roster drifted: {}".format(relative)
        )
    external = i2_dependency.verify_external_files(document.get("external"))
    if (
        external["external_file_count"] != expected["external_file_count"]
        or external["python_binding_count"]
        != expected["external_python_binding_count"]
        or external["runtime_binding_count"]
        != expected["external_runtime_binding_count"]
    ):
        raise R6I4DependencyError(
            "frozen closure external roster drifted: {}".format(relative)
        )
    return {
        "closure_path": relative,
        "file_sha256": file_record["sha256"],
        "logical_sha256": logical,
        "local_file_count": len(local_paths),
        "external_file_count": external["external_file_count"],
        "external_python_binding_count": external["python_binding_count"],
        "external_runtime_binding_count": external["runtime_binding_count"],
        "unresolved_count": 0,
        "all_targets_mechanically_rehashed": True,
        "terminal_frozen_snapshot": True,
        "pass": True,
    }


def _verify_frozen_review(
    workspace: Path,
    relative: str,
    expected_stage: str,
    expected_status: str,
) -> dict:
    root = Path(workspace).resolve()
    record = _record(root, relative)
    if record["sha256"] != EXPECTED_FIXED_FILE_SHA256[relative]:
        raise R6I4DependencyError(
            "frozen review file digest drifted: {}".format(relative)
        )
    document = i2_dependency._load_yaml_mapping(
        root / relative, "terminal frozen review {}".format(relative)
    )
    if (
        document.get("stage") != expected_stage
        or document.get("status") != expected_status
        or document.get("review_result") != "pass"
        or document.get("offline_only") is not True
        or document.get("execution_ready") is not False
    ):
        raise R6I4DependencyError(
            "terminal frozen review boundary drifted: {}".format(relative)
        )
    return {
        "path": relative,
        "sha256": record["sha256"],
        "status": expected_status,
        "terminal_frozen_snapshot": True,
        "current_state_inference_forbidden": True,
        "pass": True,
    }


def _build_local_and_external(
    workspace: Path, mandatory_inputs: Sequence[str]
) -> Tuple[dict, dict]:
    root = Path(workspace).resolve()
    local = i2_dependency._discover_local_closure(
        root, ENTRYPOINTS, mandatory_inputs
    )
    # Reuse the already closed and mechanically rehashed I3 external table.
    # Resolving ROS runtime names afresh would invoke ``rospack`` through the
    # inherited resolver, which is outside this no-subprocess offline stage.
    # The I4 surface is therefore required to be a strict binding subset of
    # the frozen 47-Python/9-runtime table before the complete table is copied.
    i3_document = i2_dependency._load_yaml_mapping(
        root / I3_CLOSURE, "frozen R6-I3 closure for external reuse"
    )
    external = copy.deepcopy(i3_document.get("external"))
    receipt = i2_dependency.verify_external_files(external)
    python_names = [row["binding"] for row in external["python_bindings"]]
    runtime_names = [row["binding"] for row in external["runtime_bindings"]]
    discovered_python = set(local["external_python_names"])
    discovered_runtime = set(local["external_runtime_names"])
    if not discovered_python.issubset(set(python_names)):
        raise R6I4DependencyError(
            "I4 Python binding is outside the frozen external table: {}".format(
                sorted(discovered_python - set(python_names))
            )
        )
    if not discovered_runtime.issubset(set(runtime_names)):
        raise R6I4DependencyError(
            "I4 runtime binding is outside the frozen external table: {}".format(
                sorted(discovered_runtime - set(runtime_names))
            )
        )
    if (
        len(python_names) != 47
        or len(runtime_names) != 9
        or receipt["external_file_count"] != 307
    ):
        raise R6I4DependencyError("frozen I3 external table coverage drifted")
    local["external_python_names"] = python_names
    local["external_runtime_names"] = runtime_names
    return local, external


def _trusted_source_records(workspace: Path) -> list:
    return [
        _record(workspace, relative)
        for relative in (I4_RUNNER, I4_VALIDATOR, I4_VALIDATOR_TEST, I4_PREFLIGHT_TEST)
    ]


def build_dependency_closure(workspace: Path) -> dict:
    """Build the complete acyclic R6-I4 offline-review closure."""

    root = Path(workspace).resolve()
    state = _verify_state_boundary(root)
    fixed = _verify_fixed_files(root)
    authorization_rows, authorization_paths = _authorization_roster(root)
    inherited_i3 = _verify_frozen_closure(
        root, I3_CLOSURE, EXPECTED_I3_CLOSURE
    )
    inherited_i2 = _verify_frozen_closure(
        root, I2_CLOSURE, EXPECTED_I2_CLOSURE
    )
    i3_review = _verify_frozen_review(
        root,
        I3_REVIEW,
        "V2-04G-R6-I3",
        "execution_readiness_closure_pass_release_absent",
    )
    i2_review = _verify_frozen_review(
        root,
        I2_REVIEW,
        "V2-04G-R6-I2",
        "repair_integration_review_pass_execution_not_authorized",
    )
    mandatory = tuple(
        dict.fromkeys(MANDATORY_INPUTS + authorization_paths)
    )
    local, external = _build_local_and_external(root, mandatory)
    document = {
        "schema_version": SCHEMA_VERSION,
        "architecture_generation": "v2",
        "stage": STAGE,
        "source_stage": SOURCE_STAGE,
        "review_scope": "offline_preflight_integrity_repair_readiness_only",
        "independent_stage": True,
        "offline_only": True,
        "simulation_only": True,
        "execution_authorized": False,
        "execution_ready": False,
        "formal_result": False,
        "runtime_ready": False,
        "ros_or_gazebo_start_authorized": False,
        "seed_or_evidence_units_allocated": 0,
        "seed_or_evidence_units_consumed": 0,
        "seed_or_evidence_units_forfeited": 0,
        "execution_seeds": [],
        "exact_schedule": [],
        "generator": (
            "thesis_experiment."
            "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_dependency."
            "build_dependency_closure"
        ),
        "state_boundary": state,
        "fixed_resource_receipts": fixed,
        "authorization_resource_audit": {
            "authorization_path": I3_AUTHORIZATION,
            "authorization_sha256": EXPECTED_FIXED_FILE_SHA256[I3_AUTHORIZATION],
            "resource_count": len(authorization_rows),
            "all_exact_bytes_sha256_rehashed": True,
            "parsed_labels_by_release_validator": [
                "preregistration",
                "inherited_r6_i2_dependency_closure",
            ],
            "hash_only_count": 10,
            "resources": authorization_rows,
            "pass": True,
        },
        "trusted_source_hash_rebuild": {
            "resources": _trusted_source_records(root),
            "runner_hardcode_check_deferred_to_machine_reviewer": True,
            "pass": True,
        },
        "inherited_i3_revalidation": inherited_i3,
        "inherited_i2_revalidation": inherited_i2,
        "terminal_frozen_review_snapshots": [i3_review, i2_review],
        "local": local,
        "external": external,
        "unresolved": [],
        "hash_graph_boundary": {
            "closure_self_included": False,
            "final_machine_review_artifact_included": False,
            "failed_i3_release_included": True,
            "future_i4_release_included": False,
            "future_release_must_bind_closure_and_review": True,
        },
    }
    document["closure_sha256"] = _canonical_json_sha(document)
    return document


def verify_dependency_closure(
    workspace: Path, document: Mapping[str, object]
) -> dict:
    """Rebuild, rehash, and compare one persisted R6-I4 closure."""

    rebuilt = build_dependency_closure(workspace)
    if type(document) is not dict or document != rebuilt:
        raise R6I4DependencyError("persisted R6-I4 dependency closure drifted")
    external = i2_dependency.verify_external_files(rebuilt["external"])
    return {
        "local_file_count": len(rebuilt["local"]["files"]),
        "local_edge_count": len(rebuilt["local"]["edges"]),
        "external_file_count": external["external_file_count"],
        "external_python_binding_count": external["python_binding_count"],
        "external_runtime_binding_count": external["runtime_binding_count"],
        "authorization_resource_count": len(
            rebuilt["authorization_resource_audit"]["resources"]
        ),
        "inherited_i3_revalidation": rebuilt["inherited_i3_revalidation"],
        "inherited_i2_revalidation": rebuilt["inherited_i2_revalidation"],
        "unresolved_count": 0,
        "closure_sha256": rebuilt["closure_sha256"],
        "mechanically_rebuilt_and_rehashed": True,
        "process_started": False,
        "execution_state_created": False,
        "pass": True,
    }
