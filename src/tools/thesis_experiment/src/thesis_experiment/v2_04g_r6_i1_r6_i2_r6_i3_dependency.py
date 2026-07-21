"""Canonical offline dependency closure for R6-I3 execution readiness.

The R6-I3 closure extends the reviewed R6-I2 local/external dependency
builder.  It adds the fresh scene children, the actual execution entrypoint,
the dedicated release gate, and the command executables that the entrypoint
may use after a future release.  Building or verifying this document never
starts ROS, Gazebo, move_base, or an experiment subprocess.

The generated closure intentionally excludes both the future execution
release and the final readiness-review artifact.  The review binds this
closure, and a later user-authorized release must bind both, which keeps the
hash graph acyclic.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
from typing import Dict, Iterable, Mapping, Sequence, Tuple

from thesis_experiment import v2_04g_r6_i1_r6_i2_dependency as i2_dependency


STAGE = "V2-04G-R6-I3"
SCHEMA_VERSION = "4.0"
ARTIFACT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)
COMPILED_SCENE_INDEX = ARTIFACT_ROOT / "compiled_scenes/compiled_scene_index.yaml"
EXECUTION_CLOSURE = ARTIFACT_ROOT / "execution_dependency_closure.yaml"
READINESS_REVIEW = ARTIFACT_ROOT / "v2_04g_r6_i3_execution_readiness_review.yaml"
EXECUTION_RELEASE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i3_execution_release.yaml"
)

ENTRYPOINTS = (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_dependency_generator.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_reviewer.py",
)

MANDATORY_INPUTS = (
    # Frozen authorization-phase trust anchors.
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml",
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_authorization_review/"
    "v2_04g_r6_i3_authorization_review.yaml",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_authorization_review.py",
    # This turn's explicit phase transition and offline scene materialization.
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_readiness_transition.yaml",
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_contract.yaml",
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_scene_derivation.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/"
    "v2_04g_r6_i3_scenes.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution/"
    "v2_04g_r6_i3_scene_behavior_equivalence.yaml",
    COMPILED_SCENE_INDEX.as_posix(),
    "src/tools/thesis_experiment/scripts/compile_v2_scenes.py",
    "src/tools/thesis_experiment/scripts/derive_v2_04g_scenes.py",
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
    "config/thesis_experiments/v2/simulation_contract.yaml",
    "src/simulation/m2_gazebo/config/simulation_candidates.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i1_scene_derivation.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/v2_04g_r6_i1_scenes.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes/"
    "compiled_scene_index.yaml",
    # R6-I2 bootstrap/integrity trust boundary.
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_bootstrap_integrity_repair_contract.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i2_stage_transition.yaml",
    "experiments/manifests/v2/integration/v2_04g_r6_i2_repair_preregistration.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "execution_dependency_closure.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_integration_review.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "v2_04g_r6_i2_authorization_assessment_review.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/execution_dependency_closure.yaml",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_bootstrap.py",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_authorization.py",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_dependency.py",
    "src/simulation/m2_gazebo/launch/"
    "m2_v2_04g_r6_i2_execution_integration.launch",
    "src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch",
    # Dedicated release gate, runner adapters, and directed tests.
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_release.py",
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_dependency.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_activation_probe_listener.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_mechanism_episode.py",
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_runtime_control.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_activation_probe_listener.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_mechanism_episode.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_runtime_control.py",
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_bounded_validation.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_release_validator.py",
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_execution_readiness.py",
)

RUNTIME_PROFILE_INPUTS = (
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_legacy_control/anchor_bank.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_legacy_control/mechanism.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_legacy_control/supervisor.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_circle_contact/anchor_bank.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_circle_contact/mechanism.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs/"
    "r6_semantics_circle_contact/supervisor.yaml",
)

AUTHORIZATION_RESOURCES = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml",
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml",
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_authorization_review/"
    "v2_04g_r6_i3_authorization_review.yaml",
)

COMMAND_BINDINGS = (
    "command-executable:roslaunch",
    "command-executable:rosservice",
    "command-executable:rostopic",
)
ADDITIONAL_ROS_RUNTIME_BINDINGS = ("node:gazebo_ros:gzserver",)
EXPECTED_RUNTIME_BINDINGS = tuple(
    sorted(
        {
            "$(find gazebo_ros)/launch/empty_world.launch",
            "node:gazebo_ros:spawn_model",
            "node:move_base:move_base",
            "node:robot_state_publisher:robot_state_publisher",
            "package-executable:xacro:xacro",
            *COMMAND_BINDINGS,
            *ADDITIONAL_ROS_RUNTIME_BINDINGS,
        }
    )
)

EXPECTED_SCENE_IDS = (
    "v2-04g-r6-i3-dynamic-conflict-single-s5151",
    "v2-04g-r6-i3-dynamic-conflict-multi-s5152",
    "v2-04g-r6-i3-dynamic-semantic-clear-s5153",
    "v2-04g-r6-i3-compile-support-cruise-s5154",
    "v2-04g-r6-i3-compile-support-static-s5155",
    "v2-04g-r6-i3-compile-support-corridor-s5156",
    "v2-04g-r6-i3-compile-support-maneuver-s5157",
)
EXPECTED_SCENE_SEEDS = (5151, 5152, 5153, 5154, 5155, 5156, 5157)
EXPECTED_SCENE_FAMILIES = (
    "DYNAMIC",
    "DYNAMIC",
    "DYNAMIC",
    "CRUISE",
    "STATIC_DENSE",
    "CORRIDOR",
    "MANEUVER",
)
EXPECTED_SCENE_MANIFEST_ID = "fam_teb_v2_04g_r6_i3_execution_readiness_scenes_1"
EXPECTED_FROZEN_AUTHORIZATION_HASHES = {
    AUTHORIZATION_RESOURCES[0]: (
        "a8295c723c1cf973c2c35c86e5b2d5c07361bdf0e92f36a0e8d12d2364ce6268"
    ),
    AUTHORIZATION_RESOURCES[1]: (
        "ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2"
    ),
    AUTHORIZATION_RESOURCES[2]: (
        "20a058f15a79aebc448497374071c7028363faa185d1ebe820f1102c6b330913"
    ),
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py": (
        "906df1914635fc7d996bb1d1073efba21e09e62d3edbf1759216bd7b31563dfb"
    ),
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_authorization_review.py": (
        "0663dec5c746c627df7b4e919c5dac22254245a3e16f4a03bc25349b9054a955"
    ),
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml": (
        "3f78ffd2ef1f022b97dcb03957b6472030fa0c86446e25bfb5724bbad19df69d"
    ),
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "execution_dependency_closure.yaml": (
        "63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58"
    ),
}

FORBIDDEN_READINESS_STATE_PATHS = (
    EXECUTION_RELEASE,
    ARTIFACT_ROOT / "attempts",
    ARTIFACT_ROOT / "journals",
    ARTIFACT_ROOT / "receipts",
    ARTIFACT_ROOT / "raw_evidence",
    ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml",
    ARTIFACT_ROOT / "v2_04g_r6_i3_execution_report.yaml",
)
INHERITED_I2_CLOSURE = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
    "execution_dependency_closure.yaml"
)
EXPECTED_INHERITED_I2_LOGICAL_SHA256 = (
    "2be410c333b78d707b591fb30bef0b344b40c19e3f957d91fbc6e56f1bd01fe6"
)


class R6I3DependencyError(ValueError):
    """Raised when the R6-I3 dependency graph is incomplete or drifted."""


def _canonical_json_sha(value: Mapping[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _fresh_scene_children(workspace: Path) -> Tuple[str, ...]:
    root = Path(workspace).resolve()
    document = i2_dependency._load_yaml_mapping(
        root / COMPILED_SCENE_INDEX, "R6-I3 compiled scene index"
    )
    if set(document) != {
        "schema_version",
        "manifest_id",
        "formal_result",
        "runtime_ready",
        "scene_count",
        "families",
        "files",
    }:
        raise R6I3DependencyError("fresh compiled scene index schema drifted")
    if (
        document["schema_version"] != "2.0"
        or document["manifest_id"] != EXPECTED_SCENE_MANIFEST_ID
        or document["scene_count"] != 7
        or document["formal_result"] is not False
        or document["runtime_ready"] is not False
        or tuple(document["families"]) != EXPECTED_SCENE_FAMILIES
        or not isinstance(document["files"], list)
        or len(document["files"]) != 14
    ):
        raise R6I3DependencyError("fresh compiled scene index boundary drifted")
    scene_manifest = i2_dependency._load_yaml_mapping(
        root / ARTIFACT_ROOT / "v2_04g_r6_i3_scenes.yaml",
        "R6-I3 fresh scene manifest",
    )
    scenes = scene_manifest.get("scenes")
    if (
        scene_manifest.get("manifest_id") != EXPECTED_SCENE_MANIFEST_ID
        or not isinstance(scenes, list)
        or len(scenes) != 7
        or tuple(row.get("scene_id") for row in scenes) != EXPECTED_SCENE_IDS
        or tuple(row.get("seed") for row in scenes) != EXPECTED_SCENE_SEEDS
    ):
        raise R6I3DependencyError("fresh scene identity roster drifted")
    expected_paths = tuple(
        (
            COMPILED_SCENE_INDEX.parent / (scene_id + suffix)
        ).as_posix()
        for scene_id in EXPECTED_SCENE_IDS
        for suffix in (".instance.yaml", ".world")
    )
    paths = []
    for row in document["files"]:
        if (
            not isinstance(row, dict)
            or set(row) != {"path", "sha256"}
            or not isinstance(row["path"], str)
            or not isinstance(row["sha256"], str)
        ):
            raise R6I3DependencyError("fresh compiled child row drifted")
        relative = Path(row["path"])
        if (
            relative.is_absolute()
            or relative.parent != COMPILED_SCENE_INDEX.parent
            or relative.suffix not in {".yaml", ".world"}
        ):
            raise R6I3DependencyError("fresh compiled child path escaped")
        record = i2_dependency.canonical_file_record(root / relative)
        if record["sha256"] != row["sha256"]:
            raise R6I3DependencyError(
                "fresh compiled child digest drifted: {}".format(relative)
            )
        paths.append(relative.as_posix())
    if tuple(paths) != expected_paths:
        raise R6I3DependencyError("fresh compiled child roster/order drifted")
    return tuple(paths)


def _command_binding(name: str) -> Tuple[dict, Dict[str, dict]]:
    executable = shutil.which(name)
    if not executable:
        raise R6I3DependencyError("required command is unavailable: {}".format(name))
    record = i2_dependency.canonical_file_record(executable)
    target = Path(record["canonical_path"])
    row = {
        "binding": "command-executable:{}".format(name),
        "resolution_kind": "canonical_path_command_executable",
        "package": "PATH",
        "package_root": target.parent.as_posix(),
        "target_canonical_path": target.as_posix(),
        "canonical_paths": [target.as_posix()],
    }
    return row, {target.as_posix(): record}


def _augment_external_closure(workspace: Path, external: Mapping[str, object]) -> dict:
    result = {
        "python_interpreter": dict(external["python_interpreter"]),
        "python_bindings": [dict(row) for row in external["python_bindings"]],
        "runtime_bindings": [dict(row) for row in external["runtime_bindings"]],
        "files": [dict(row) for row in external["files"]],
        "unresolved": [],
    }
    records = {row["canonical_path"]: row for row in result["files"]}
    rows = {row["binding"]: row for row in result["runtime_bindings"]}
    for binding in ADDITIONAL_ROS_RUNTIME_BINDINGS:
        row, resolved = i2_dependency.resolve_runtime_binding(workspace, binding)
        rows[row["binding"]] = row
        records.update(resolved)
    for binding in COMMAND_BINDINGS:
        name = binding.split(":", 1)[1]
        row, resolved = _command_binding(name)
        rows[row["binding"]] = row
        records.update(resolved)
    result["runtime_bindings"] = [rows[name] for name in sorted(rows)]
    result["files"] = [records[path] for path in sorted(records)]
    names = tuple(row["binding"] for row in result["runtime_bindings"])
    if names != EXPECTED_RUNTIME_BINDINGS:
        raise R6I3DependencyError(
            "runtime binding set drifted: {}".format(list(names))
        )
    result["closure_sha256"] = _canonical_json_sha(result)
    return result


def _authorization_records(workspace: Path) -> list:
    root = Path(workspace).resolve()
    rows = []
    for relative in AUTHORIZATION_RESOURCES:
        record = i2_dependency.canonical_file_record(root / relative)
        if record["sha256"] != EXPECTED_FROZEN_AUTHORIZATION_HASHES[relative]:
            raise R6I3DependencyError(
                "frozen authorization-phase resource drifted: {}".format(relative)
            )
        rows.append({"path": relative, "sha256": record["sha256"]})
    return rows


def _verify_frozen_authorization_sources(workspace: Path) -> None:
    root = Path(workspace).resolve()
    for relative, expected in EXPECTED_FROZEN_AUTHORIZATION_HASHES.items():
        record = i2_dependency.canonical_file_record(root / relative)
        if record["sha256"] != expected:
            raise R6I3DependencyError(
                "frozen authorization-phase source drifted: {}".format(relative)
            )


def _verify_readiness_state_absent(workspace: Path) -> None:
    root = Path(workspace).resolve()
    present = [
        relative.as_posix()
        for relative in FORBIDDEN_READINESS_STATE_PATHS
        if os.path.lexists(str(root / relative))
    ]
    if present:
        raise R6I3DependencyError(
            "execution state exists during readiness closure: {}".format(present)
        )


def _verify_inherited_i2_closure(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    document = i2_dependency._load_yaml_mapping(
        root / INHERITED_I2_CLOSURE, "frozen inherited R6-I2 closure"
    )
    logical_payload = {
        key: value for key, value in document.items() if key != "closure_sha256"
    }
    if (
        document.get("closure_sha256") != EXPECTED_INHERITED_I2_LOGICAL_SHA256
        or _canonical_json_sha(logical_payload)
        != EXPECTED_INHERITED_I2_LOGICAL_SHA256
    ):
        raise R6I3DependencyError("inherited R6-I2 logical digest drifted")
    local = document.get("local")
    if not isinstance(local, dict) or not isinstance(local.get("files"), list):
        raise R6I3DependencyError("inherited R6-I2 local closure is missing")
    paths = []
    for row in local["files"]:
        if not isinstance(row, dict) or set(row) != {"path", "sha256", "size_bytes"}:
            raise R6I3DependencyError("inherited R6-I2 local record drifted")
        record = i2_dependency.canonical_file_record(root / row["path"])
        if (
            record["sha256"] != row["sha256"]
            or record["size_bytes"] != row["size_bytes"]
        ):
            raise R6I3DependencyError(
                "inherited R6-I2 local target drifted: {}".format(row["path"])
            )
        paths.append(row["path"])
    if paths != sorted(set(paths)):
        raise R6I3DependencyError("inherited R6-I2 local paths drifted")
    external = i2_dependency.verify_external_files(document.get("external"))
    return {
        "closure_path": INHERITED_I2_CLOSURE,
        "closure_sha256": EXPECTED_INHERITED_I2_LOGICAL_SHA256,
        "local_file_count": len(paths),
        "external_file_count": external["external_file_count"],
        "external_python_binding_count": external["python_binding_count"],
        "external_runtime_binding_count": external["runtime_binding_count"],
        "unresolved_count": 0,
        "all_targets_mechanically_rehashed": True,
        "pass": True,
    }


def build_dependency_closure(workspace: Path) -> dict:
    """Build the complete acyclic R6-I3 readiness dependency closure."""

    root = Path(workspace).resolve()
    _verify_readiness_state_absent(root)
    _verify_frozen_authorization_sources(root)
    inherited_i2 = _verify_inherited_i2_closure(root)
    children = _fresh_scene_children(root)
    mandatory = tuple(
        dict.fromkeys(MANDATORY_INPUTS + RUNTIME_PROFILE_INPUTS + children)
    )
    base = i2_dependency.build_dependency_closure(
        root,
        entrypoints=ENTRYPOINTS,
        mandatory_inputs=mandatory,
    )
    local = dict(base["local"])
    local["external_runtime_names"] = list(EXPECTED_RUNTIME_BINDINGS)
    external = _augment_external_closure(root, base["external"])
    document = {
        "schema_version": SCHEMA_VERSION,
        "stage": STAGE,
        "review_scope": "offline_execution_readiness_closure_only",
        "execution_authorized": False,
        "authorization_envelope_validated": True,
        "execution_release_resource_included": False,
        "execution_ready_claimed": False,
        "evidence_budget_authorized": 6,
        "evidence_budget_consumed": 0,
        "seed_or_evidence_units_allocated": 6,
        "seed_or_evidence_units_consumed": 0,
        "authorization_resources": _authorization_records(root),
        "generator": (
            "thesis_experiment."
            "v2_04g_r6_i1_r6_i2_r6_i3_dependency.build_dependency_closure"
        ),
        "compiled_scene_index": COMPILED_SCENE_INDEX.as_posix(),
        "compiled_scene_children": list(children),
        "inherited_i2_revalidation": inherited_i2,
        "local": local,
        "external": external,
        "unresolved": [],
        "hash_graph_boundary": {
            "closure_self_included": False,
            "future_release_included": False,
            "final_machine_review_artifact_included": False,
            "future_release_must_bind_closure_and_review": True,
        },
    }
    document["closure_sha256"] = _canonical_json_sha(document)
    return document


def verify_dependency_closure(workspace: Path, document: Mapping[str, object]) -> dict:
    """Rebuild, rehash, and compare a persisted R6-I3 closure in process."""

    rebuilt = build_dependency_closure(workspace)
    if type(document) is not dict or document != rebuilt:
        raise R6I3DependencyError("persisted R6-I3 dependency closure drifted")
    external_receipt = i2_dependency.verify_external_files(rebuilt["external"])
    return {
        "local_file_count": len(rebuilt["local"]["files"]),
        "local_edge_count": len(rebuilt["local"]["edges"]),
        "compiled_scene_child_count": len(rebuilt["compiled_scene_children"]),
        "external_file_count": external_receipt["external_file_count"],
        "external_python_binding_count": external_receipt["python_binding_count"],
        "external_runtime_binding_count": external_receipt["runtime_binding_count"],
        "unresolved_count": 0,
        "closure_sha256": rebuilt["closure_sha256"],
        "mechanically_rebuilt_and_rehashed": True,
        "process_started": False,
        "pass": True,
    }
