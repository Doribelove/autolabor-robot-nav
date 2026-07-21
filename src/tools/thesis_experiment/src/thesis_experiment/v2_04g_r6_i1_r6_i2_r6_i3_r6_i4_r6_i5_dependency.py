"""Deterministic dependency closure for R6-I5 execution readiness.

This module is offline-only.  It binds the fresh I5 authority DAG, scene
materialization, reviewed runner/release gate, runtime adapters, and the
frozen I4 preflight-integrity trust anchors.  The complete 307-file/47-Python/
9-runtime external table from the terminal I4 closure is copied only after
every target has been mechanically rehashed.  The two I5-only standard-library
bindings are then resolved without importing them and merged as exact
canonical path+SHA records.  A script-directory import used by the scene
reviewer is classified explicitly as workspace-local, never as external.

The output closure, final machine-review artifact, and future execution
release are deliberately excluded from the graph.  A later unique release
binds the completed closure and review without creating a hash cycle.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

from thesis_experiment import v2_04g_r6_i1_r6_i2_dependency as i2_dependency


STAGE = "V2-04G-R6-I5"
SOURCE_STAGE = "V2-04G-R6-I4"
SCHEMA_VERSION = "6.0"

ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution"
)
EXECUTION_CLOSURE = ARTIFACT_ROOT / "execution_dependency_closure.yaml"
MACHINE_REVIEW = ARTIFACT_ROOT / "v2_04g_r6_i5_execution_readiness_review.yaml"
FUTURE_I5_RELEASE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i5_execution_release.yaml"
)
FUTURE_I4_RELEASE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i4_execution_release.yaml"
)
FAILED_I3_RELEASE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml"
)
I3_ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)
I4_ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i4_preflight_repair_review"
)

I5_CONTRACT = (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_"
    "bounded_simulation_execution_contract.yaml"
)
I5_PREREGISTRATION = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_preregistration.yaml"
)
I5_AUTHORIZATION = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
I5_TRANSITION = (
    "experiments/manifests/v2/integration/v2_04g_r6_i5_stage_transition.yaml"
)
I5_SCENE_DERIVATION = (
    "experiments/manifests/v2/integration/v2_04g_r6_i5_scene_derivation.yaml"
)
I5_SCENE_MANIFEST = (
    ARTIFACT_ROOT / "v2_04g_r6_i5_scenes.yaml"
).as_posix()
I5_SCENE_BEHAVIOR_AUDIT = (
    ARTIFACT_ROOT / "v2_04g_r6_i5_scene_behavior_equivalence.yaml"
).as_posix()
COMPILED_SCENE_INDEX = ARTIFACT_ROOT / "compiled_scenes/compiled_scene_index.yaml"

I5_RUNNER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_bounded_validation.py"
)
I5_LISTENER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_activation_probe_listener.py"
)
I5_EPISODE = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_mechanism_episode.py"
)
I5_CONTROL = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_runtime_control.py"
)
I5_RELEASE_VALIDATOR = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release.py"
)
I5_RELEASE_VALIDATOR_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_release_validator.py"
)
I5_ASSESSOR = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_assessment.py"
)
I5_ASSESSOR_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_assessment.py"
)
I5_READINESS_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_execution_readiness.py"
)
I5_SCENE_MATERIALIZER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_scene_materializer.py"
)
I5_SCENE_REVIEWER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_scene_readiness_reviewer.py"
)
I5_SCENE_TEST = (
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_scene_materialization.py"
)
I5_DEPENDENCY = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency.py"
)
I5_GENERATOR = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency_generator.py"
)
I5_REVIEWER = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_execution_readiness_reviewer.py"
)

I4_VALIDATOR = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
)
I4_CLOSURE = (
    I4_ARTIFACT_ROOT / "execution_dependency_closure.yaml"
).as_posix()
I4_REVIEW = (
    I4_ARTIFACT_ROOT
    / "v2_04g_r6_i4_preflight_integrity_readiness_review.yaml"
).as_posix()

I2_BOOTSTRAP = (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_bootstrap.py"
)
R6_INTEGRITY = (
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_integrity.py"
)
FROZEN_EVALUATOR = (
    "src/tools/thesis_experiment/src/thesis_experiment/v2_evaluator.py"
)
I2_LAUNCH = (
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_execution_integration.launch"
)
I2_SPAWN_LAUNCH = (
    "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch"
)

EXPECTED_SCHEDULE_SHA256 = (
    "b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402"
)
EXPECTED_FIXED_FILE_SHA256 = {
    I5_CONTRACT: (
        "f4ce9115427028e12fce1176176204005989652c57bbf8d00a8c4eda8f33042a"
    ),
    I5_PREREGISTRATION: (
        "602fc1044fb9e3e8ac284e77cadcbaced95c5edfb9721df0da26f876cc42c073"
    ),
    I5_AUTHORIZATION: (
        "bc59820b0140b50503657966d735511a8007d9ec8e14f3f2cf237791ff170592"
    ),
    I5_TRANSITION: (
        "1dcca6b55e3571c8f7511852efde6042af92323bf441b2ac5c761b2afa013d58"
    ),
    I5_SCENE_DERIVATION: (
        "b74f24e169f3ffbe98f0139fc01dd78c1d2a1f8d6040df130719680ce4350145"
    ),
    I5_SCENE_MANIFEST: (
        "e6c15d906c707686fcb3923cee1c075ebb000ab1216900b3578387433fc679fa"
    ),
    COMPILED_SCENE_INDEX.as_posix(): (
        "cf9dc16079c2aa01a80e8d50d00e6e968c3e5da125b25c6d8659d26366f218a6"
    ),
    I5_SCENE_BEHAVIOR_AUDIT: (
        "200382b5a03f86a5ba189779ffb187ffaebb92a71a1d16a6beeadba710921669"
    ),
    I5_SCENE_MATERIALIZER: (
        "ce3f2b80d0f37a61bdff8ac3ca315b1c23288ee51b832a0a26c997a25a80e22b"
    ),
    I5_SCENE_REVIEWER: (
        "aa6f6babf2ab4376430e2ad0cff67bc72e522cac549a5e81bd2d49e30b1ec444"
    ),
    I5_SCENE_TEST: (
        "115964fc0da255e155b392f7e59185f29ef2daf5ea9de6e6a51c7a00fe3d7d64"
    ),
    I5_ASSESSOR: (
        "de584f6e9a94fc4c8b02d1b733688c620bb56c31d89289295f6b8a3472c2fe5a"
    ),
    I5_ASSESSOR_TEST: (
        "c96b51748803e8d1fd6833aae7cf207f59295b6def7ed4d540e35583980ab50c"
    ),
    I4_VALIDATOR: (
        "9b9dd3fc580d0f880705bf87e120cdb9d30fdce812f585ee99e3f0fdf1fa3994"
    ),
    I4_CLOSURE: (
        "e9d27ed1522ca744f1bbf4a91832287ac2e780aa395c1fa1d147e3c587099b0f"
    ),
    I4_REVIEW: (
        "3f183768e657ec17f4fd1045ffd0749d13213cc53017a09c6e166d840d647b12"
    ),
    FAILED_I3_RELEASE.as_posix(): (
        "5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6"
    ),
}

EXPECTED_I4_CLOSURE = {
    "stage": "V2-04G-R6-I4",
    "file_sha256": EXPECTED_FIXED_FILE_SHA256[I4_CLOSURE],
    "logical_sha256": (
        "dceb73df8619849f5b5a0442b739be09815bfc86939a188873c94993fe4d5b74"
    ),
    "local_file_count": 54,
    "external_file_count": 307,
    "external_python_binding_count": 47,
    "external_runtime_binding_count": 9,
}

WORKSPACE_LOCAL_SCRIPT_BINDINGS = {
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_scene_materializer": (
        I5_SCENE_REVIEWER,
        I5_SCENE_MATERIALIZER,
    ),
}
EXPECTED_I5_ADDITIONAL_PYTHON_BINDINGS = ("ctypes", "secrets")
EXPECTED_I5_EXTERNAL_FILE_COUNT = 313
EXPECTED_I5_PYTHON_BINDING_COUNT = 49
EXPECTED_I5_RUNTIME_BINDING_COUNT = 9

EXPECTED_AUTHORIZATION_LABELS = (
    "contract",
    "preregistration",
    "i4_validator",
    "inherited_i4_dependency_closure",
    "i4_machine_review",
    "failed_i3_release",
    "frozen_evaluator",
    "legacy_supervisor",
    "aligned_supervisor",
    "source_i1_scene_manifest",
    "source_i1_compiled_scene_index",
    "r6_design_report",
)
AUTHORIZATION_PARSED_LABELS = frozenset(
    {"preregistration", "inherited_i4_dependency_closure"}
)
EXPECTED_AUTHORIZATION_PATHS = {
    "contract": I5_CONTRACT,
    "preregistration": I5_PREREGISTRATION,
    "i4_validator": I4_VALIDATOR,
    "inherited_i4_dependency_closure": I4_CLOSURE,
    "i4_machine_review": I4_REVIEW,
    "failed_i3_release": FAILED_I3_RELEASE.as_posix(),
    "frozen_evaluator": FROZEN_EVALUATOR,
    "legacy_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_legacy_control/supervisor.yaml"
    ),
    "aligned_supervisor": (
        "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
        "r6_semantics_circle_contact/supervisor.yaml"
    ),
    "source_i1_scene_manifest": (
        "artifacts/v2/integration/v2_04g_r6_i1/v2_04g_r6_i1_scenes.yaml"
    ),
    "source_i1_compiled_scene_index": (
        "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
        "compiled_scene_index.yaml"
    ),
    "r6_design_report": (
        "artifacts/v2/design_review/v2_04g_r6/v2_04g_r6_design_review.yaml"
    ),
}

EXPECTED_SCENE_MANIFEST_ID = "fam_teb_v2_04g_r6_i5_bounded_simulation_scenes_1"
EXPECTED_SCENE_IDS = (
    "v2-04g-r6-i5-dynamic-conflict-single-s5161",
    "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
    "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
    "v2-04g-r6-i5-compile-support-cruise-s5164",
    "v2-04g-r6-i5-compile-support-static-s5165",
    "v2-04g-r6-i5-compile-support-corridor-s5166",
    "v2-04g-r6-i5-compile-support-maneuver-s5167",
)
EXPECTED_SCENE_SEEDS = (5161, 5162, 5163, 5164, 5165, 5166, 5167)
EXPECTED_SCENE_FAMILIES = (
    "DYNAMIC",
    "DYNAMIC",
    "DYNAMIC",
    "CRUISE",
    "STATIC_DENSE",
    "CORRIDOR",
    "MANEUVER",
)

RUNTIME_PROFILE_INPUTS = (
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_legacy_control/anchor_bank.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_legacy_control/mechanism.yaml",
    EXPECTED_AUTHORIZATION_PATHS["legacy_supervisor"],
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_circle_contact/anchor_bank.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_circle_contact/mechanism.yaml",
    EXPECTED_AUTHORIZATION_PATHS["aligned_supervisor"],
)

ENTRYPOINTS = (
    I5_RUNNER,
    I5_LISTENER,
    I5_EPISODE,
    I5_CONTROL,
    I5_SCENE_MATERIALIZER,
    I5_SCENE_REVIEWER,
    I5_GENERATOR,
    I5_REVIEWER,
)
MANDATORY_INPUTS = (
    I5_CONTRACT,
    I5_PREREGISTRATION,
    I5_AUTHORIZATION,
    I5_TRANSITION,
    I5_SCENE_DERIVATION,
    I5_SCENE_MANIFEST,
    I5_SCENE_BEHAVIOR_AUDIT,
    COMPILED_SCENE_INDEX.as_posix(),
    I5_RELEASE_VALIDATOR,
    I5_RELEASE_VALIDATOR_TEST,
    I5_ASSESSOR,
    I5_ASSESSOR_TEST,
    I5_READINESS_TEST,
    I5_SCENE_TEST,
    I5_DEPENDENCY,
    I4_VALIDATOR,
    I4_CLOSURE,
    I4_REVIEW,
    FAILED_I3_RELEASE.as_posix(),
    I2_BOOTSTRAP,
    R6_INTEGRITY,
    FROZEN_EVALUATOR,
    I2_LAUNCH,
    I2_SPAWN_LAUNCH,
    "src/tools/thesis_experiment/scripts/compile_v2_scenes.py",
    "src/tools/thesis_experiment/scripts/derive_v2_04g_scenes.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
    "config/thesis_experiments/v2/simulation_contract.yaml",
    "src/simulation/m2_gazebo/config/simulation_candidates.yaml",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_activation_probe_listener.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_mechanism_episode.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_runtime_control.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_bounded_validation.py",
) + RUNTIME_PROFILE_INPUTS

FORBIDDEN_EXECUTION_STATE = (
    FUTURE_I4_RELEASE,
    FUTURE_I5_RELEASE,
    I3_ARTIFACT_ROOT / "attempts",
    I3_ARTIFACT_ROOT / "journals",
    I3_ARTIFACT_ROOT / "receipts",
    I3_ARTIFACT_ROOT / "raw_evidence",
    I3_ARTIFACT_ROOT / "semantic_evidence",
    I3_ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml",
    I3_ARTIFACT_ROOT / "v2_04g_r6_i3_execution_report.yaml",
    I4_ARTIFACT_ROOT / "attempts",
    I4_ARTIFACT_ROOT / "journals",
    I4_ARTIFACT_ROOT / "receipts",
    I4_ARTIFACT_ROOT / "raw_evidence",
    I4_ARTIFACT_ROOT / "semantic_evidence",
    I4_ARTIFACT_ROOT / "ros_home",
    I4_ARTIFACT_ROOT / "ros_logs",
    I4_ARTIFACT_ROOT / "v2_04g_r6_i4_stage_report.yaml",
    I4_ARTIFACT_ROOT / "v2_04g_r6_i4_execution_report.yaml",
    ARTIFACT_ROOT / "attempts",
    ARTIFACT_ROOT / "journals",
    ARTIFACT_ROOT / "receipts",
    ARTIFACT_ROOT / "raw_evidence",
    ARTIFACT_ROOT / "semantic_evidence",
    ARTIFACT_ROOT / "ros_home",
    ARTIFACT_ROOT / "ros_logs",
    ARTIFACT_ROOT / "v2_04g_r6_i5_stage_report.yaml",
    ARTIFACT_ROOT / "v2_04g_r6_i5_execution_report.yaml",
)


class R6I5DependencyError(ValueError):
    """Raised when the I5 readiness dependency boundary is incomplete."""


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
        raise R6I5DependencyError(
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
        raise R6I5DependencyError(
            "workspace dependency escaped root: {}".format(relative)
        ) from exc
    if canonical_relative.as_posix() != safe:
        raise R6I5DependencyError(
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
            raise R6I5DependencyError(
                "fixed I5 or inherited resource drifted: {}".format(relative)
            )
        rows.append(row)
    return rows


def _verify_state_boundary(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    failed_release = _record(root, FAILED_I3_RELEASE.as_posix())
    if (
        failed_release["sha256"]
        != EXPECTED_FIXED_FILE_SHA256[FAILED_I3_RELEASE.as_posix()]
    ):
        raise R6I5DependencyError("failed R6-I3 release digest drifted")
    present = [
        relative.as_posix()
        for relative in FORBIDDEN_EXECUTION_STATE
        if os.path.lexists(str(root / relative))
    ]
    if present:
        raise R6I5DependencyError(
            "forbidden execution state exists during I5 readiness: {}".format(
                present
            )
        )
    return {
        "failed_i3_release_present": True,
        "failed_i3_release_sha256": failed_release["sha256"],
        "future_i4_release_present": False,
        "future_i5_release_present": False,
        "i3_authorized_units": 6,
        "i3_consumed_units": 0,
        "i3_forfeited_units": 0,
        "i3_release_or_identity_reusable": False,
        "i4_authorized_units": 0,
        "i4_consumed_units": 0,
        "i4_forfeited_units": 0,
        "i5_authorized_units": 6,
        "i5_consumed_units": 0,
        "i5_forfeited_units": 0,
        "forbidden_state_present": [],
        "pass": True,
    }


def _authorization_roster(workspace: Path) -> Tuple[list, Tuple[str, ...], list]:
    """Rehash all twelve resources but parse only the two consumed labels."""

    root = Path(workspace).resolve()
    document = i2_dependency._load_yaml_mapping(
        root / I5_AUTHORIZATION, "frozen R6-I5 authorization envelope"
    )
    if (
        document.get("stage") != STAGE
        or document.get("execution_authorized") is not True
        or document.get("evidence_budget_authorized") != 6
        or document.get("fresh_execution_seeds") != [5161, 5162, 5163]
        or document.get("preregistration_schedule_sha256")
        != EXPECTED_SCHEDULE_SHA256
        or document.get("dependency_closure_digest")
        != EXPECTED_I4_CLOSURE["logical_sha256"]
    ):
        raise R6I5DependencyError("I5 authorization boundary drifted")
    resources = document.get("bound_resources")
    if not isinstance(resources, dict):
        raise R6I5DependencyError("I5 authorization roster is missing")
    if tuple(resources) != EXPECTED_AUTHORIZATION_LABELS:
        raise R6I5DependencyError("I5 authorization roster labels/order drifted")
    rows = []
    paths = []
    parsed_documents = {}
    for label in EXPECTED_AUTHORIZATION_LABELS:
        row = resources[label]
        expected_path = EXPECTED_AUTHORIZATION_PATHS[label]
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256"}
            or row.get("path") != expected_path
            or not isinstance(row.get("sha256"), str)
        ):
            raise R6I5DependencyError(
                "authorization resource row drifted: {}".format(label)
            )
        actual = _record(root, expected_path)
        if actual["sha256"] != row["sha256"]:
            raise R6I5DependencyError(
                "authorization resource bytes drifted: {}".format(label)
            )
        parsed = label in AUTHORIZATION_PARSED_LABELS
        if parsed:
            parsed_documents[label] = i2_dependency._load_yaml_mapping(
                root / expected_path,
                "I5 parsed authorization resource {}".format(label),
            )
        rows.append(
            {
                "label": label,
                "path": actual["path"],
                "sha256": actual["sha256"],
                "size_bytes": actual["size_bytes"],
                "parsed_by_i5_release_validator": parsed,
            }
        )
        paths.append(actual["path"])
    if len(paths) != 12 or len(set(paths)) != 12:
        raise R6I5DependencyError("authorization resource paths are not closed")
    preregistration = parsed_documents["preregistration"]
    schedule = preregistration.get("schedule")
    if (
        preregistration.get("stage") != STAGE
        or preregistration.get("execution_authorized") is not False
        or not isinstance(schedule, list)
        or len(schedule) != 6
        or _canonical_json_sha(schedule) != EXPECTED_SCHEDULE_SHA256
        or document.get("exact_schedule") != schedule
    ):
        raise R6I5DependencyError("I5 preregistration schedule drifted")
    inherited_closure = parsed_documents["inherited_i4_dependency_closure"]
    logical_payload = {
        key: value
        for key, value in inherited_closure.items()
        if key != "closure_sha256"
    }
    if (
        inherited_closure.get("closure_sha256")
        != EXPECTED_I4_CLOSURE["logical_sha256"]
        or _canonical_json_sha(logical_payload)
        != EXPECTED_I4_CLOSURE["logical_sha256"]
    ):
        raise R6I5DependencyError("authorization-bound I4 closure drifted")
    return rows, tuple(paths), schedule


def _verify_i4_closure(workspace: Path) -> Tuple[dict, dict]:
    """Rehash the frozen I4 closure file and every local/external target."""

    root = Path(workspace).resolve()
    file_record = _record(root, I4_CLOSURE)
    if file_record["sha256"] != EXPECTED_I4_CLOSURE["file_sha256"]:
        raise R6I5DependencyError("frozen I4 closure file digest drifted")
    document = i2_dependency._load_yaml_mapping(
        root / I4_CLOSURE, "frozen R6-I4 dependency closure"
    )
    logical_payload = {
        key: value for key, value in document.items() if key != "closure_sha256"
    }
    logical = _canonical_json_sha(logical_payload)
    if (
        document.get("stage") != EXPECTED_I4_CLOSURE["stage"]
        or document.get("closure_sha256")
        != EXPECTED_I4_CLOSURE["logical_sha256"]
        or logical != EXPECTED_I4_CLOSURE["logical_sha256"]
    ):
        raise R6I5DependencyError("frozen I4 closure logical digest drifted")
    local = document.get("local")
    if not isinstance(local, dict) or not isinstance(local.get("files"), list):
        raise R6I5DependencyError("frozen I4 local closure is missing")
    local_paths = []
    for row in local["files"]:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256", "size_bytes"}
        ):
            raise R6I5DependencyError("frozen I4 local record drifted")
        actual = _record(root, row["path"])
        if (
            actual["sha256"] != row["sha256"]
            or actual["size_bytes"] != row["size_bytes"]
        ):
            raise R6I5DependencyError(
                "frozen I4 local target drifted: {}".format(row["path"])
            )
        local_paths.append(row["path"])
    if (
        local_paths != sorted(set(local_paths))
        or len(local_paths) != EXPECTED_I4_CLOSURE["local_file_count"]
    ):
        raise R6I5DependencyError("frozen I4 local roster drifted")
    external = copy.deepcopy(document.get("external"))
    receipt = i2_dependency.verify_external_files(external)
    if (
        receipt["external_file_count"]
        != EXPECTED_I4_CLOSURE["external_file_count"]
        or receipt["python_binding_count"]
        != EXPECTED_I4_CLOSURE["external_python_binding_count"]
        or receipt["runtime_binding_count"]
        != EXPECTED_I4_CLOSURE["external_runtime_binding_count"]
    ):
        raise R6I5DependencyError("frozen I4 external roster drifted")
    return (
        {
            "closure_path": I4_CLOSURE,
            "file_sha256": file_record["sha256"],
            "logical_sha256": logical,
            "local_file_count": len(local_paths),
            "external_file_count": receipt["external_file_count"],
            "external_python_binding_count": receipt["python_binding_count"],
            "external_runtime_binding_count": receipt["runtime_binding_count"],
            "unresolved_count": 0,
            "all_targets_mechanically_rehashed": True,
            "terminal_frozen_snapshot": True,
            "pass": True,
        },
        external,
    )


def _verify_i4_review(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    record = _record(root, I4_REVIEW)
    document = i2_dependency._load_yaml_mapping(
        root / I4_REVIEW, "terminal R6-I4 readiness review"
    )
    expected_status = (
        "preflight_integrity_repair_readiness_closure_pass_"
        "future_release_absent"
    )
    if (
        record["sha256"] != EXPECTED_FIXED_FILE_SHA256[I4_REVIEW]
        or document.get("stage") != "V2-04G-R6-I4"
        or document.get("status") != expected_status
        or document.get("review_result") != "pass"
        or document.get("execution_ready") is not False
        or document.get("execution_authorized") is not False
    ):
        raise R6I5DependencyError("terminal I4 review boundary drifted")
    return {
        "path": I4_REVIEW,
        "sha256": record["sha256"],
        "status": expected_status,
        "terminal_frozen_snapshot": True,
        "current_execution_authority_inference_forbidden": True,
        "pass": True,
    }


def _fresh_scene_children(workspace: Path) -> Tuple[Tuple[str, ...], dict]:
    root = Path(workspace).resolve()
    index = i2_dependency._load_yaml_mapping(
        root / COMPILED_SCENE_INDEX, "R6-I5 compiled scene index"
    )
    if set(index) != {
        "schema_version",
        "manifest_id",
        "formal_result",
        "runtime_ready",
        "scene_count",
        "families",
        "files",
    }:
        raise R6I5DependencyError("fresh compiled scene index schema drifted")
    if (
        index.get("schema_version") != "2.0"
        or index.get("manifest_id") != EXPECTED_SCENE_MANIFEST_ID
        or index.get("scene_count") != 7
        or index.get("formal_result") is not False
        or index.get("runtime_ready") is not False
        or tuple(index.get("families", ())) != EXPECTED_SCENE_FAMILIES
        or not isinstance(index.get("files"), list)
        or len(index["files"]) != 14
    ):
        raise R6I5DependencyError("fresh compiled scene index boundary drifted")
    manifest = i2_dependency._load_yaml_mapping(
        root / I5_SCENE_MANIFEST, "R6-I5 fresh scene manifest"
    )
    scenes = manifest.get("scenes")
    if (
        manifest.get("manifest_id") != EXPECTED_SCENE_MANIFEST_ID
        or not isinstance(scenes, list)
        or len(scenes) != 7
        or tuple(row.get("scene_id") for row in scenes) != EXPECTED_SCENE_IDS
        or tuple(row.get("seed") for row in scenes) != EXPECTED_SCENE_SEEDS
    ):
        raise R6I5DependencyError("fresh scene identity roster drifted")
    expected_paths = tuple(
        (
            COMPILED_SCENE_INDEX.parent / (scene_id + suffix)
        ).as_posix()
        for scene_id in EXPECTED_SCENE_IDS
        for suffix in (".instance.yaml", ".world")
    )
    paths = []
    for row in index["files"]:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256"}
            or not isinstance(row.get("path"), str)
            or not isinstance(row.get("sha256"), str)
        ):
            raise R6I5DependencyError("fresh compiled child row drifted")
        relative = Path(row["path"])
        if relative.as_posix() not in expected_paths:
            raise R6I5DependencyError("fresh compiled child path escaped")
        actual = _record(root, relative.as_posix())
        if actual["sha256"] != row["sha256"]:
            raise R6I5DependencyError(
                "fresh compiled child digest drifted: {}".format(relative)
            )
        paths.append(relative.as_posix())
    if tuple(paths) != expected_paths:
        raise R6I5DependencyError("fresh compiled child roster/order drifted")
    behavior = _record(root, I5_SCENE_BEHAVIOR_AUDIT)
    return tuple(paths), {
        "manifest": _record(root, I5_SCENE_MANIFEST),
        "compiled_index": _record(root, COMPILED_SCENE_INDEX.as_posix()),
        "behavior_equivalence_audit": behavior,
        "compiled_child_count": len(paths),
        "scene_ids": list(EXPECTED_SCENE_IDS),
        "scene_seeds": list(EXPECTED_SCENE_SEEDS),
        "pass": True,
    }


def _build_local_and_external(
    workspace: Path,
    mandatory_inputs: Sequence[str],
    inherited_external: Mapping[str, object],
) -> Tuple[dict, dict]:
    root = Path(workspace).resolve()
    local = i2_dependency._discover_local_closure(
        root, ENTRYPOINTS, mandatory_inputs
    )
    external = copy.deepcopy(inherited_external)
    inherited_receipt = i2_dependency.verify_external_files(external)
    inherited_python_names = [
        row["binding"] for row in external["python_bindings"]
    ]
    runtime_names = [row["binding"] for row in external["runtime_bindings"]]
    discovered_python = set(local["external_python_names"])
    discovered_runtime = set(local["external_runtime_names"])

    local_paths = {row["path"] for row in local["files"]}
    local_edges = list(local["edges"])
    classified_local = []
    for binding, (source, target) in sorted(
        WORKSPACE_LOCAL_SCRIPT_BINDINGS.items()
    ):
        if binding not in discovered_python:
            raise R6I5DependencyError(
                "expected workspace-local script binding was not discovered: "
                + binding
            )
        if source not in local_paths or target not in local_paths:
            raise R6I5DependencyError(
                "workspace-local script binding is not file-bound: " + binding
            )
        discovered_python.remove(binding)
        edge = {"from": source, "to": target, "kind": "python_import"}
        if edge not in local_edges:
            local_edges.append(edge)
        classified_local.append(
            {"binding": binding, "from": source, "to": target}
        )
    local["edges"] = sorted(
        local_edges,
        key=lambda row: (row["from"], row["to"], row["kind"]),
    )
    local["workspace_local_python_bindings"] = classified_local

    inherited_names = set(inherited_python_names)
    additional_python = tuple(sorted(discovered_python - inherited_names))
    if additional_python != EXPECTED_I5_ADDITIONAL_PYTHON_BINDINGS:
        raise R6I5DependencyError(
            "I5 additional Python binding roster drifted: {}".format(
                list(additional_python)
            )
        )
    if not discovered_runtime.issubset(set(runtime_names)):
        raise R6I5DependencyError(
            "I5 runtime binding is outside frozen I4 table: {}".format(
                sorted(discovered_runtime - set(runtime_names))
            )
        )
    if (
        len(inherited_python_names) != 47
        or len(runtime_names) != 9
        or inherited_receipt["external_file_count"] != 307
    ):
        raise R6I5DependencyError("frozen I4 external table coverage drifted")

    interpreter = external["python_interpreter"]
    records = {row["canonical_path"]: row for row in external["files"]}
    for binding in additional_python:
        row, resolved = i2_dependency.resolve_python_binding(
            root, binding, interpreter_record=interpreter
        )
        if row.get("binding") != binding:
            raise R6I5DependencyError(
                "I5 Python binding resolver identity drifted: " + binding
            )
        external["python_bindings"].append(row)
        for canonical_path, record in resolved.items():
            prior = records.get(canonical_path)
            if prior is not None and prior != record:
                raise R6I5DependencyError(
                    "I5 external file record conflicts with I4: "
                    + canonical_path
                )
            records[canonical_path] = record
    external["python_bindings"] = sorted(
        external["python_bindings"], key=lambda row: row["binding"]
    )
    external["files"] = [records[path] for path in sorted(records)]
    external_payload = {
        key: value for key, value in external.items() if key != "closure_sha256"
    }
    external["closure_sha256"] = _canonical_json_sha(external_payload)
    extended_receipt = i2_dependency.verify_external_files(external)
    python_names = [row["binding"] for row in external["python_bindings"]]
    if (
        extended_receipt["external_file_count"]
        != EXPECTED_I5_EXTERNAL_FILE_COUNT
        or extended_receipt["python_binding_count"]
        != EXPECTED_I5_PYTHON_BINDING_COUNT
        or extended_receipt["runtime_binding_count"]
        != EXPECTED_I5_RUNTIME_BINDING_COUNT
    ):
        raise R6I5DependencyError("extended I5 external table coverage drifted")
    local["external_python_names"] = python_names
    local["external_runtime_names"] = runtime_names
    return local, external


def _trusted_source_records(workspace: Path) -> list:
    resources = (
        I5_RUNNER,
        I5_LISTENER,
        I5_EPISODE,
        I5_CONTROL,
        I5_RELEASE_VALIDATOR,
        I5_RELEASE_VALIDATOR_TEST,
        I5_ASSESSOR,
        I5_ASSESSOR_TEST,
        I5_READINESS_TEST,
        I5_SCENE_MATERIALIZER,
        I5_SCENE_REVIEWER,
        I5_SCENE_TEST,
        I5_DEPENDENCY,
        I5_GENERATOR,
        I5_REVIEWER,
    )
    return [_record(workspace, relative) for relative in resources]


def build_dependency_closure(workspace: Path) -> dict:
    """Build the complete acyclic R6-I5 execution-readiness closure."""

    root = Path(workspace).resolve()
    state = _verify_state_boundary(root)
    fixed = _verify_fixed_files(root)
    authorization_rows, authorization_paths, schedule = _authorization_roster(root)
    inherited_i4, inherited_external = _verify_i4_closure(root)
    i4_review = _verify_i4_review(root)
    children, scene_receipt = _fresh_scene_children(root)
    mandatory = tuple(
        dict.fromkeys(MANDATORY_INPUTS + authorization_paths + children)
    )
    local, external = _build_local_and_external(
        root, mandatory, inherited_external
    )
    document = {
        "schema_version": SCHEMA_VERSION,
        "architecture_generation": "v2",
        "stage": STAGE,
        "source_stage": SOURCE_STAGE,
        "review_scope": "offline_execution_readiness_closure_release_absent",
        "independent_stage": True,
        "offline_review_only": True,
        "simulation_only": True,
        "execution_authorized": False,
        "execution_ready": False,
        "execution_release_present": False,
        "formal_result": False,
        "runtime_ready": False,
        "ros_or_gazebo_started": False,
        "evidence_budget_authorized": 6,
        "evidence_budget_consumed": 0,
        "evidence_budget_forfeited": 0,
        "execution_seeds": [5161, 5162, 5163],
        "compile_support_only_seeds": [5164, 5165, 5166, 5167],
        "exact_schedule": schedule,
        "exact_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "generator": (
            "thesis_experiment."
            "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_dependency."
            "build_dependency_closure"
        ),
        "state_boundary": state,
        "fixed_resource_receipts": fixed,
        "authorization_resource_audit": {
            "authorization_path": I5_AUTHORIZATION,
            "authorization_sha256": EXPECTED_FIXED_FILE_SHA256[I5_AUTHORIZATION],
            "resource_count": len(authorization_rows),
            "all_exact_bytes_sha256_rehashed": True,
            "parsed_labels": sorted(AUTHORIZATION_PARSED_LABELS),
            "parsed_count": len(AUTHORIZATION_PARSED_LABELS),
            "hash_only_count": 10,
            "suffix_based_parse_forbidden": True,
            "resources": authorization_rows,
            "pass": True,
        },
        "fresh_scene_revalidation": scene_receipt,
        "trusted_source_hash_rebuild": {
            "resources": _trusted_source_records(root),
            "runner_hardcode_check_deferred_to_machine_reviewer": True,
            "pass": True,
        },
        "inherited_i4_revalidation": inherited_i4,
        "inherited_i4_review_snapshot": i4_review,
        "i5_external_extension_audit": {
            "inherited_external_file_count": inherited_i4[
                "external_file_count"
            ],
            "inherited_python_binding_count": inherited_i4[
                "external_python_binding_count"
            ],
            "inherited_runtime_binding_count": inherited_i4[
                "external_runtime_binding_count"
            ],
            "additional_python_bindings": list(
                EXPECTED_I5_ADDITIONAL_PYTHON_BINDINGS
            ),
            "workspace_local_script_bindings": local[
                "workspace_local_python_bindings"
            ],
            "final_external_file_count": len(external["files"]),
            "final_python_binding_count": len(external["python_bindings"]),
            "final_runtime_binding_count": len(external["runtime_bindings"]),
            "final_external_closure_sha256": external["closure_sha256"],
            "additional_bindings_resolved_without_import": True,
            "all_external_files_mechanically_rehashed": True,
            "pass": True,
        },
        "local": local,
        "external": external,
        "unresolved": [],
        "hash_graph_boundary": {
            "closure_self_included": False,
            "final_machine_review_artifact_included": False,
            "future_i5_release_included": False,
            "failed_i3_release_included": True,
            "i4_validator_closure_and_review_included": True,
            "future_release_must_bind_closure_and_review": True,
        },
    }
    document["closure_sha256"] = _canonical_json_sha(document)
    return document


def verify_dependency_closure(
    workspace: Path, document: Mapping[str, object]
) -> dict:
    """Rebuild, rehash, and compare one persisted R6-I5 closure."""

    rebuilt = build_dependency_closure(workspace)
    if type(document) is not dict or document != rebuilt:
        raise R6I5DependencyError("persisted R6-I5 dependency closure drifted")
    external = i2_dependency.verify_external_files(rebuilt["external"])
    return {
        "local_file_count": len(rebuilt["local"]["files"]),
        "local_edge_count": len(rebuilt["local"]["edges"]),
        "compiled_scene_child_count": len(
            rebuilt["fresh_scene_revalidation"]["scene_ids"]
        )
        * 2,
        "external_file_count": external["external_file_count"],
        "external_python_binding_count": external["python_binding_count"],
        "external_runtime_binding_count": external["runtime_binding_count"],
        "authorization_resource_count": len(
            rebuilt["authorization_resource_audit"]["resources"]
        ),
        "inherited_i4_revalidation": rebuilt["inherited_i4_revalidation"],
        "unresolved_count": 0,
        "closure_sha256": rebuilt["closure_sha256"],
        "mechanically_rebuilt_and_rehashed": True,
        "process_started": False,
        "execution_state_created": False,
        "pass": True,
    }
