#!/usr/bin/env python3
"""Deterministic offline machine review for R6-I3 execution readiness.

This reviewer validates the authorized phase transition, fresh scene
materialization, fail-closed runner/release-gate source, and the independently
rebuilt dependency closure.  It cannot create an execution release, attempt,
journal, receipt, or process.
"""

from __future__ import annotations

import argparse
import ast
import copy
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
from typing import Mapping
import xml.etree.ElementTree as ET

import yaml


sys.dont_write_bytecode = True

from thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_dependency import (
    COMPILED_SCENE_INDEX,
    EXECUTION_CLOSURE,
    EXECUTION_RELEASE,
    EXPECTED_RUNTIME_BINDINGS,
    READINESS_REVIEW,
    verify_dependency_closure,
)
from thesis_experiment.v2_scene import compile_v2_manifest, render_v2_scene_sdf


STAGE = "V2-04G-R6-I3"
WORKSPACE = Path("/home/robot/robot_ws_base_rl")
ARTIFACT_ROOT = READINESS_REVIEW.parent
CONTRACT = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_contract.yaml"
)
TRANSITION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_readiness_transition.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_preregistration.yaml"
)
AUTHORIZATION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml"
)
HISTORICAL_AUTHORIZATION_REVIEW = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_authorization_review/"
    "v2_04g_r6_i3_authorization_review.yaml"
)
SCENE_DERIVATION = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i3_scene_derivation.yaml"
)
SCENE_MANIFEST = ARTIFACT_ROOT / "v2_04g_r6_i3_scenes.yaml"
SCENE_AUDIT = ARTIFACT_ROOT / "v2_04g_r6_i3_scene_behavior_equivalence.yaml"
SOURCE_SCENE_MANIFEST = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/v2_04g_r6_i1_scenes.yaml"
)
SOURCE_COMPILED_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py"
)
RELEASE_VALIDATOR = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_release.py"
)
RELEASE_TEST = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_release_validator.py"
)
READINESS_TEST = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i3_execution_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_execution_readiness_reviewer.py"
)
WRAPPERS = (
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_activation_probe_listener.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_mechanism_episode.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_runtime_control.py"
    ),
)

EXPECTED_FROZEN_HASHES = {
    PREREGISTRATION: "a8295c723c1cf973c2c35c86e5b2d5c07361bdf0e92f36a0e8d12d2364ce6268",
    AUTHORIZATION: "ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2",
    HISTORICAL_AUTHORIZATION_REVIEW: "20a058f15a79aebc448497374071c7028363faa185d1ebe820f1102c6b330913",
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_authorization_reviewer.py"
    ): "906df1914635fc7d996bb1d1073efba21e09e62d3edbf1759216bd7b31563dfb",
    Path(
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i3_authorization_review.py"
    ): "0663dec5c746c627df7b4e919c5dac22254245a3e16f4a03bc25349b9054a955",
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
        "v2_04g_r6_i2_integration_review.yaml"
    ): "b23f7384e3c88aa9c3c0a9af50f6fc51159b2091b6b44cb127d034847bbb8a61",
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/r6_i2_repair_review/"
        "execution_dependency_closure.yaml"
    ): "63c4e7ba5d8fd64315040a566aabdb54cd71b034ce76f814aa3f519be6dfac58",
    Path(
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "execution_dependency_closure.yaml"
    ): "3f78ffd2ef1f022b97dcb03957b6472030fa0c86446e25bfb5724bbad19df69d",
}
EXPECTED_SCHEDULE_SHA256 = (
    "ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c"
)
EXPECTED_SOURCE_SCENES = (
    ("v2-04g-r6-i1-dynamic-conflict-single-s5141", 5141),
    ("v2-04g-r6-i1-dynamic-conflict-multi-s5142", 5142),
    ("v2-04g-r6-i1-dynamic-semantic-clear-s5143", 5143),
    ("v2-04g-r6-i1-compile-support-cruise-s5144", 5144),
    ("v2-04g-r6-i1-compile-support-static-s5145", 5145),
    ("v2-04g-r6-i1-compile-support-corridor-s5146", 5146),
    ("v2-04g-r6-i1-compile-support-maneuver-s5147", 5147),
)
EXPECTED_TARGET_SCENES = (
    ("v2-04g-r6-i3-dynamic-conflict-single-s5151", 5151),
    ("v2-04g-r6-i3-dynamic-conflict-multi-s5152", 5152),
    ("v2-04g-r6-i3-dynamic-semantic-clear-s5153", 5153),
    ("v2-04g-r6-i3-compile-support-cruise-s5154", 5154),
    ("v2-04g-r6-i3-compile-support-static-s5155", 5155),
    ("v2-04g-r6-i3-compile-support-corridor-s5156", 5156),
    ("v2-04g-r6-i3-compile-support-maneuver-s5157", 5157),
)
EXPECTED_FAMILIES = (
    "DYNAMIC",
    "DYNAMIC",
    "DYNAMIC",
    "CRUISE",
    "STATIC_DENSE",
    "CORRIDOR",
    "MANEUVER",
)


class R6I3ReadinessReviewError(ValueError):
    """Raised when the offline readiness evidence is incomplete."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise R6I3ReadinessReviewError("duplicate YAML key")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise R6I3ReadinessReviewError(message)


def _safe_relative(relative: Path) -> tuple:
    candidate = Path(relative)
    _require(
        not candidate.is_absolute()
        and bool(candidate.parts)
        and all(part not in {"", ".", ".."} for part in candidate.parts),
        "unsafe workspace-relative path: {}".format(candidate),
    )
    return candidate.parts


def _snapshot(workspace: Path, relative: Path, parse_yaml: bool = False) -> dict:
    """Read one workspace file once, with no-follow on every path component."""

    parts = _safe_relative(relative)
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(Path(workspace).resolve()), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        file_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        file_flags |= getattr(os, "O_NOFOLLOW", 0)
        file_descriptor = os.open(parts[-1], file_flags, dir_fd=descriptors[-1])
        try:
            before = os.fstat(file_descriptor)
            _require(stat.S_ISREG(before.st_mode), "resource is not a regular file")
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            _require(
                (
                    before.st_dev,
                    before.st_ino,
                    before.st_mode,
                    before.st_size,
                    before.st_mtime_ns,
                    before.st_ctime_ns,
                )
                == (
                    after.st_dev,
                    after.st_ino,
                    after.st_mode,
                    after.st_size,
                    after.st_mtime_ns,
                    after.st_ctime_ns,
                ),
                "resource changed during single-open read",
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "resource size drifted")
        finally:
            os.close(file_descriptor)
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)
    result = {
        "path": Path(relative).as_posix(),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "size_bytes": len(payload),
        "payload": payload,
    }
    if parse_yaml:
        try:
            document = yaml.load(payload.decode("utf-8"), Loader=_UniqueKeyLoader)
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise R6I3ReadinessReviewError(
                "cannot parse {}: {}".format(relative, exc)
            ) from exc
        _require(type(document) is dict, "YAML resource must be a mapping")
        result["document"] = document
    return result


def _canonical_sha(value: object) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _verify_frozen(workspace: Path) -> list:
    rows = []
    for path, expected in EXPECTED_FROZEN_HASHES.items():
        snapshot = _snapshot(workspace, path)
        _require(snapshot["sha256"] == expected, "frozen resource drifted: {}".format(path))
        rows.append({"path": path.as_posix(), "sha256": expected})
    return rows


def _verify_authority(workspace: Path) -> dict:
    contract_snapshot = _snapshot(workspace, CONTRACT, parse_yaml=True)
    transition_snapshot = _snapshot(workspace, TRANSITION, parse_yaml=True)
    prereg = _snapshot(workspace, PREREGISTRATION, parse_yaml=True)["document"]
    authorization = _snapshot(workspace, AUTHORIZATION, parse_yaml=True)["document"]
    contract = contract_snapshot["document"]
    transition = transition_snapshot["document"]
    _require(contract.get("stage") == STAGE, "readiness contract stage drifted")
    _require(
        contract.get("status") == "offline_execution_readiness_closure_authorized_release_absent"
        and contract.get("offline_only") is True
        and contract.get("execution_ready") is False
        and contract.get("execution_release_present") is False
        and contract.get("ros_or_gazebo_start_authorized") is False,
        "readiness contract safety boundary drifted",
    )
    _require(transition.get("stage") == STAGE, "readiness transition stage drifted")
    user_boundary = transition.get("user_instruction_boundary")
    _require(
        type(user_boundary) is dict
        and user_boundary.get("explicit_execution_readiness_closure_instruction_received") is True
        and user_boundary.get("next_explicit_simulation_execution_instruction_required") is True,
        "readiness user-instruction boundary drifted",
    )
    _require(
        transition.get("readiness_contract")
        == {"path": CONTRACT.as_posix(), "sha256": contract_snapshot["sha256"]},
        "transition does not bind the exact readiness contract",
    )
    _require(
        transition.get("execution_release_created_by_transition") is False
        and transition.get("attempt_root_or_journal_created_by_transition") is False
        and type(transition.get("seed_or_evidence_units_consumed_by_transition")) is int
        and transition.get("seed_or_evidence_units_consumed_by_transition") == 0,
        "transition side-effect boundary drifted",
    )
    schedule = prereg.get("schedule")
    _require(type(schedule) is list and len(schedule) == 6, "preregistered schedule drifted")
    _require(authorization.get("exact_schedule") == schedule, "authorization schedule drifted")
    _require(_canonical_sha(schedule) == EXPECTED_SCHEDULE_SHA256, "schedule digest drifted")
    contract_schedule = contract.get("exact_schedule")
    _require(
        type(contract_schedule) is dict
        and contract_schedule.get("canonical_sha256") == EXPECTED_SCHEDULE_SHA256
        and contract_schedule.get("identity_count") == 6
        and contract_schedule.get("rows") == schedule,
        "readiness contract full exact schedule drifted",
    )
    _require(
        authorization.get("evidence_budget_authorized") == 6
        and authorization.get("fresh_execution_seeds") == [5151, 5152, 5153]
        and authorization.get("attempt_limit_per_identity") == 1
        and authorization.get("retry_or_resume_allowed") is False,
        "authorization budget or seed boundary drifted",
    )
    return {
        "contract": {"path": CONTRACT.as_posix(), "sha256": contract_snapshot["sha256"]},
        "transition": {"path": TRANSITION.as_posix(), "sha256": transition_snapshot["sha256"]},
        "explicit_readiness_instruction_received": True,
        "separate_simulation_execution_instruction_received": False,
        "schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "schedule_identity_count": 6,
        "evidence_units_authorized": 6,
        "evidence_units_consumed": 0,
        "pass": True,
    }


def _verify_scenes(workspace: Path) -> dict:
    derivation = _snapshot(workspace, SCENE_DERIVATION, parse_yaml=True)
    manifest = _snapshot(workspace, SCENE_MANIFEST, parse_yaml=True)
    source_manifest = _snapshot(
        workspace, SOURCE_SCENE_MANIFEST, parse_yaml=True
    )
    audit = _snapshot(workspace, SCENE_AUDIT, parse_yaml=True)
    index = _snapshot(workspace, COMPILED_SCENE_INDEX, parse_yaml=True)
    derivation_document = derivation["document"]
    manifest_document = manifest["document"]
    source_document = source_manifest["document"]
    audit_document = audit["document"]
    index_document = index["document"]
    _require(
        derivation_document.get("stage") == STAGE
        and derivation_document.get("execution_release_present") is False,
        "scene derivation boundary drifted",
    )
    derivation_rows = derivation_document.get("scene_derivations")
    _require(
        type(derivation_rows) is list and len(derivation_rows) == 7,
        "scene derivation roster is missing",
    )
    for index_number, row in enumerate(derivation_rows):
        source_id, source_seed = EXPECTED_SOURCE_SCENES[index_number]
        target_id, target_seed = EXPECTED_TARGET_SCENES[index_number]
        _require(
            type(row) is dict
            and row.get("source_scene_id") == source_id
            and row.get("source_seed") == source_seed
            and row.get("target_scene_id") == target_id
            and row.get("seed") == target_seed
            and row.get("behavior_fields_changed") == [],
            "scene derivation identity map drifted",
        )
    _require(
        audit_document.get("review_result") == "pass"
        and audit_document.get("evidence_units_consumed") == 0
        and audit_document.get("ros_or_gazebo_started") is False,
        "scene behavior audit did not pass",
    )
    equivalence = audit_document.get("equivalence_rule")
    _require(
        type(equivalence) is dict
        and equivalence.get("observed_behavior_fields_changed") == []
        and equivalence.get("pass") is True,
        "scene behavior equivalence drifted",
    )
    provenance = audit_document.get("provenance")
    _require(type(provenance) is dict, "scene audit provenance is missing")
    for label, path in (
        ("readiness_contract", CONTRACT),
        ("readiness_transition", TRANSITION),
        ("scene_derivation", SCENE_DERIVATION),
        ("source_scene_manifest", SOURCE_SCENE_MANIFEST),
    ):
        expected = _snapshot(workspace, path)
        _require(
            provenance.get(label)
            == {"path": path.as_posix(), "sha256": expected["sha256"]},
            "scene audit provenance drifted: {}".format(label),
        )

    source_scenes = source_document.get("scenes")
    target_scenes = manifest_document.get("scenes")
    _require(
        type(source_scenes) is list
        and type(target_scenes) is list
        and len(source_scenes) == len(target_scenes) == 7,
        "source/target scene manifests do not contain seven scenes",
    )
    source_top = copy.deepcopy(source_document)
    target_top = copy.deepcopy(manifest_document)
    source_top["manifest_id"] = "NORMALIZED_MANIFEST_ID"
    target_top["manifest_id"] = "NORMALIZED_MANIFEST_ID"
    normalized_scene_rows = []
    for index_number, (source_scene, target_scene) in enumerate(
        zip(source_scenes, target_scenes)
    ):
        source_id, source_seed = EXPECTED_SOURCE_SCENES[index_number]
        target_id, target_seed = EXPECTED_TARGET_SCENES[index_number]
        _require(
            source_scene.get("scene_id") == source_id
            and source_scene.get("seed") == source_seed
            and target_scene.get("scene_id") == target_id
            and target_scene.get("seed") == target_seed,
            "source/target scene identity roster drifted",
        )
        normalized_source = copy.deepcopy(source_scene)
        normalized_target = copy.deepcopy(target_scene)
        normalized_source.pop("scene_id")
        normalized_source.pop("seed")
        normalized_target.pop("scene_id")
        normalized_target.pop("seed")
        source_digest = _canonical_sha(normalized_source)
        target_digest = _canonical_sha(normalized_target)
        _require(source_digest == target_digest, "scene behavior spec drifted")
        source_top["scenes"][index_number] = normalized_source
        target_top["scenes"][index_number] = normalized_target
        normalized_scene_rows.append(
            {
                "source_scene_id": source_id,
                "target_scene_id": target_id,
                "normalized_source_sha256": source_digest,
                "normalized_target_sha256": target_digest,
                "pass": True,
            }
        )
    _require(
        _canonical_sha(source_top) == _canonical_sha(target_top),
        "scene manifests differ outside manifest_id/scene_id/seed",
    )

    _require(
        index_document.get("scene_count") == 7
        and index_document.get("manifest_id") == manifest_document.get("manifest_id")
        and tuple(index_document.get("families", [])) == EXPECTED_FAMILIES
        and type(index_document.get("files")) is list
        and len(index_document["files"]) == 14,
        "fresh compiled scene index drifted",
    )
    expected_paths = [
        (COMPILED_SCENE_INDEX.parent / (scene_id + suffix)).as_posix()
        for scene_id, _ in EXPECTED_TARGET_SCENES
        for suffix in (".instance.yaml", ".world")
    ]
    _require(
        [row.get("path") for row in index_document["files"]] == expected_paths,
        "fresh compiled scene exact child roster/order drifted",
    )

    # Recompile all seven scenes in memory.  No directory, subprocess, ROS, or
    # Gazebo is involved; the generated bytes must match every persisted child.
    instances = compile_v2_manifest(manifest_document, workspace)
    _require(len(instances) == 7, "in-memory scene recompile count drifted")
    recompiled_payloads = {}
    for instance in instances:
        scene_id = instance["scene"]["scene_id"]
        recompiled_payloads[
            (COMPILED_SCENE_INDEX.parent / (scene_id + ".instance.yaml")).as_posix()
        ] = yaml.safe_dump(
            instance, sort_keys=False, allow_unicode=True
        ).encode("utf-8")
        recompiled_payloads[
            (COMPILED_SCENE_INDEX.parent / (scene_id + ".world")).as_posix()
        ] = render_v2_scene_sdf(instance).encode("utf-8")
    child_rows = []
    for row in index_document["files"]:
        _require(type(row) is dict and set(row) == {"path", "sha256"}, "compiled child row drifted")
        snapshot = _snapshot(workspace, Path(row["path"]))
        _require(snapshot["sha256"] == row["sha256"], "compiled child hash drifted")
        _require(
            recompiled_payloads[row["path"]] == snapshot["payload"],
            "fresh compiled child is not a deterministic recompile",
        )
        child_rows.append({"path": row["path"], "sha256": row["sha256"]})
    _require(len({row["path"] for row in child_rows}) == 14, "compiled child paths repeat")

    def normalized_instance_digest(snapshot):
        document = copy.deepcopy(snapshot["document"])
        document.pop("instance_sha256", None)
        document["scene"]["scene_id"] = "NORMALIZED_EXECUTION_SCENE"
        document["scene"]["seed"] = 0
        return _canonical_sha(document)

    def normalized_world_digest(snapshot):
        try:
            root = ET.fromstring(snapshot["payload"])
        except ET.ParseError as exc:
            raise R6I3ReadinessReviewError("compiled world XML is invalid") from exc
        world = root.find("world")
        _require(world is not None, "compiled SDF world element is missing")
        world.set("name", "NORMALIZED_EXECUTION_SCENE")
        return hashlib.sha256(ET.tostring(root, encoding="utf-8")).hexdigest()

    compiled_execution_rows = []
    audit_compiled = audit_document.get("compiled_execution_scene_equivalence")
    _require(
        type(audit_compiled) is dict
        and type(audit_compiled.get("rows")) is list
        and len(audit_compiled["rows"]) == 3
        and audit_compiled.get("compile_support_cross_seed_compiled_equivalence_required") is False
        and audit_compiled.get("pass") is True,
        "compiled execution equivalence audit is missing",
    )
    for index_number in range(3):
        source_id, _ = EXPECTED_SOURCE_SCENES[index_number]
        target_id, _ = EXPECTED_TARGET_SCENES[index_number]
        source_instance = _snapshot(
            workspace,
            SOURCE_COMPILED_ROOT / (source_id + ".instance.yaml"),
            parse_yaml=True,
        )
        target_instance = _snapshot(
            workspace,
            COMPILED_SCENE_INDEX.parent / (target_id + ".instance.yaml"),
            parse_yaml=True,
        )
        source_world = _snapshot(
            workspace, SOURCE_COMPILED_ROOT / (source_id + ".world")
        )
        target_world = _snapshot(
            workspace, COMPILED_SCENE_INDEX.parent / (target_id + ".world")
        )
        source_instance_sha = normalized_instance_digest(source_instance)
        target_instance_sha = normalized_instance_digest(target_instance)
        source_world_sha = normalized_world_digest(source_world)
        target_world_sha = normalized_world_digest(target_world)
        _require(
            source_instance_sha == target_instance_sha
            and source_world_sha == target_world_sha,
            "compiled execution scene behavior drifted",
        )
        compiled_execution_rows.append(
            {
                "source_scene_id": source_id,
                "target_scene_id": target_id,
                "normalized_instance_sha256": source_instance_sha,
                "normalized_world_sha256": source_world_sha,
                "pass": True,
            }
        )
        audit_row = audit_compiled.get("rows", [])[index_number]
        _require(
            audit_row
            == {
                "source_scene_id": source_id,
                "target_scene_id": target_id,
                "normalized_source_instance_sha256": source_instance_sha,
                "normalized_target_instance_sha256": target_instance_sha,
                "normalized_source_world_sha256": source_world_sha,
                "normalized_target_world_sha256": target_world_sha,
                "pass": True,
            },
            "persisted compiled execution equivalence row drifted",
        )
    for scene in target_scenes[:3]:
        _require(
            scene.get("randomization")
            == {
                "position_jitter_m": 0.0,
                "yaw_jitter_rad": 0.0,
                "agent_time_jitter_s": 0.0,
            },
            "execution scene randomization must remain zero",
        )
    return {
        "derivation": {"path": SCENE_DERIVATION.as_posix(), "sha256": derivation["sha256"]},
        "manifest": {"path": SCENE_MANIFEST.as_posix(), "sha256": manifest["sha256"]},
        "behavior_audit": {"path": SCENE_AUDIT.as_posix(), "sha256": audit["sha256"]},
        "compiled_index": {"path": COMPILED_SCENE_INDEX.as_posix(), "sha256": index["sha256"]},
        "scene_count": 7,
        "compiled_child_count": 14,
        "execution_seeds": [5151, 5152, 5153],
        "compile_support_only_seeds": [5154, 5155, 5156, 5157],
        "compiled_children": child_rows,
        "normalized_scene_spec_equivalence": normalized_scene_rows,
        "compiled_execution_scene_equivalence": compiled_execution_rows,
        "compile_support_compiled_cross_seed_equivalence_required": False,
        "behavior_equivalent": True,
        "deterministic_recompile_reviewed": True,
        "evidence_units_consumed": 0,
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


def _verify_execution_source(workspace: Path) -> dict:
    runner = _snapshot(workspace, RUNNER)
    validator = _snapshot(workspace, RELEASE_VALIDATOR)
    release_test = _snapshot(workspace, RELEASE_TEST)
    readiness_test = _snapshot(workspace, READINESS_TEST)
    reviewer = _snapshot(workspace, REVIEWER)
    wrapper_rows = [_snapshot(workspace, path) for path in WRAPPERS]
    source = runner["payload"].decode("utf-8")
    tree = ast.parse(source, filename=RUNNER.as_posix())
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
            "thesis_experiment",
            "yaml",
        }
    )
    _require(forbidden_imports == [], "runner imports executable dependencies before release validation")
    required_markers = (
        "R6I2PositiveClockBarrier",
        "mark_unpause_requested",
        "mark_unpause_acknowledged",
        "observe_clock",
        "release_service_wait",
        "/gazebo/unpause_physics",
        "/move_base/TebLocalPlannerROS/set_parameters",
    )
    _require(all(marker in source for marker in required_markers), "runner bootstrap integration is incomplete")
    bootstrap_functions = [
        node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "_bootstrap_clock_and_services"
    ]
    _require(len(bootstrap_functions) == 1, "runner bootstrap function is missing")
    calls = []
    for node in ast.walk(bootstrap_functions[0]):
        if not isinstance(node, ast.Call):
            continue
        target = node.func
        name = (
            target.attr
            if isinstance(target, ast.Attribute)
            else target.id if isinstance(target, ast.Name) else None
        )
        if name:
            calls.append((node.lineno, node.col_offset, name))
    call_names = [row[2] for row in sorted(calls)]
    required_call_order = (
        "mark_base_spawned",
        "mark_unpause_requested",
        "mark_unpause_acknowledged",
        "observe_clock",
        "release_service_wait",
    )
    positions = []
    cursor = 0
    for required in required_call_order:
        try:
            position = call_names.index(required, cursor)
        except ValueError as exc:
            raise R6I3ReadinessReviewError(
                "runner bootstrap call is missing: {}".format(required)
            ) from exc
        positions.append(position)
        cursor = position + 1
    _require(positions == sorted(positions), "runner bootstrap source order drifted")
    namespace = {
        "__name__": "v2_04g_r6_i3_runner_offline_machine_review",
        "__file__": str(workspace / RUNNER),
    }
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        exec(compile(source, str(workspace / RUNNER), "exec"), namespace)
        offline_returncode = namespace["offline_review"]()
    try:
        offline_receipt = json.loads(captured.getvalue().strip())
    except json.JSONDecodeError as exc:
        raise R6I3ReadinessReviewError(
            "runner offline-review receipt is not canonical JSON"
        ) from exc
    _require(
        offline_returncode == 0
        and offline_receipt.get("status")
        == "runner_offline_review_pass_execution_release_absent"
        and offline_receipt.get("execution_ready") is False
        and offline_receipt.get("journal_or_attempt_root_created") is False
        and offline_receipt.get("ros_or_subprocess_started") is False
        and offline_receipt.get("seed_or_evidence_units_consumed") == 0,
        "runner offline-review boundary did not pass",
    )
    return {
        "runner": {"path": RUNNER.as_posix(), "sha256": runner["sha256"]},
        "dedicated_release_validator": {
            "path": RELEASE_VALIDATOR.as_posix(),
            "sha256": validator["sha256"],
        },
        "release_validator_negative_tests": {
            "path": RELEASE_TEST.as_posix(),
            "sha256": release_test["sha256"],
        },
        "readiness_tests": {"path": READINESS_TEST.as_posix(), "sha256": readiness_test["sha256"]},
        "reviewer": {"path": REVIEWER.as_posix(), "sha256": reviewer["sha256"]},
        "wrappers": [
            {"path": row["path"], "sha256": row["sha256"]} for row in wrapper_rows
        ],
        "runner_top_level_imports": imports,
        "import_before_release_validation_forbidden": True,
        "positive_clock_bootstrap_order_present": True,
        "runner_offline_review": offline_receipt,
        "runner_offline_review_executed_in_process": True,
        "release_validation_before_mkdir_journal_or_subprocess_required": True,
        "pass": True,
    }


def _verify_absence(workspace: Path) -> dict:
    forbidden_exact = (
        EXECUTION_RELEASE,
        ARTIFACT_ROOT / "attempts",
        ARTIFACT_ROOT / "journals",
        ARTIFACT_ROOT / "receipts",
        ARTIFACT_ROOT / "raw_evidence",
        ARTIFACT_ROOT / "v2_04g_r6_i3_stage_report.yaml",
        ARTIFACT_ROOT / "v2_04g_r6_i3_execution_report.yaml",
    )
    present = [
        path.as_posix()
        for path in forbidden_exact
        if os.path.lexists(str(workspace / path))
    ]
    _require(present == [], "forbidden execution state exists: {}".format(present))
    forbidden_parts = {
        "attempts",
        "journals",
        "receipts",
        "raw_evidence",
        "ros_home",
        "ros_logs",
        "semantic_evidence",
    }
    unexpected = []
    root = workspace / ARTIFACT_ROOT
    expected_static_paths = {
        SCENE_MANIFEST,
        SCENE_AUDIT,
        EXECUTION_CLOSURE,
        READINESS_REVIEW,
        COMPILED_SCENE_INDEX.parent,
        COMPILED_SCENE_INDEX,
    }
    expected_static_paths.update(
        COMPILED_SCENE_INDEX.parent / (scene_id + suffix)
        for scene_id, _ in EXPECTED_TARGET_SCENES
        for suffix in (".instance.yaml", ".world")
    )
    for candidate in root.rglob("*"):
        relative = candidate.relative_to(workspace)
        _require(not candidate.is_symlink(), "symlink exists in static readiness root")
        if forbidden_parts & set(relative.parts):
            unexpected.append(relative.as_posix())
        if relative not in expected_static_paths:
            unexpected.append(relative.as_posix())
    _require(unexpected == [], "unexpected execution-state subtree exists")
    return {
        "canonical_release_path": EXECUTION_RELEASE.as_posix(),
        "release_manifest_present": False,
        "attempt_root_present": False,
        "journal_root_present": False,
        "receipt_present": False,
        "raw_or_semantic_evidence_present": False,
        "stage_execution_report_present": False,
        "evidence_units_consumed": 0,
        "process_start_performed_by_review": False,
        "host_process_exclusivity_check_deferred_to_future_release_gate": True,
        "execution_ready": False,
        "pass": True,
    }


def build_review(workspace: Path) -> dict:
    root = Path(workspace).resolve()
    frozen = _verify_frozen(root)
    authority = _verify_authority(root)
    scenes = _verify_scenes(root)
    execution_source = _verify_execution_source(root)
    closure_snapshot = _snapshot(root, EXECUTION_CLOSURE, parse_yaml=True)
    closure = verify_dependency_closure(root, closure_snapshot["document"])
    absence = _verify_absence(root)
    _require(
        closure["compiled_scene_child_count"] == 14
        and closure["external_runtime_binding_count"] == len(EXPECTED_RUNTIME_BINDINGS)
        and closure["unresolved_count"] == 0,
        "execution dependency closure is incomplete",
    )
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id": "fam_teb_v2_04g_r6_i3_execution_readiness_review_1",
        "status": "execution_readiness_closure_pass_release_absent",
        "review_result": "pass",
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "authorization_envelope_valid": True,
        "separate_execution_release_required": True,
        "separate_execution_release_present": False,
        "execution_release_manifest_present": False,
        "execution_release_authorized": False,
        "ros_or_gazebo_started_by_review": False,
        "seed_or_evidence_units_consumed": 0,
        "frozen_authorization_phase_integrity": {
            "historical_absence_review_is_terminal_snapshot": True,
            "historical_absence_review_rebuilt_after_materialization": False,
            "resource_count": len(frozen),
            "all_hashes_match": True,
            "resources": frozen,
        },
        "readiness_authority_review": authority,
        "fresh_scene_review": scenes,
        "execution_source_review": execution_source,
        "dependency_closure_review": {
            "path": EXECUTION_CLOSURE.as_posix(),
            "file_sha256": closure_snapshot["sha256"],
            **closure,
        },
        "execution_absence_review": absence,
        "side_effects": {
            "execution_release_created": False,
            "attempt_root_created": False,
            "journal_created": False,
            "subprocess_started_by_review": False,
            "ros_started_by_review": False,
            "gazebo_started_by_review": False,
            "move_base_or_teb_started_by_review": False,
            "evidence_units_consumed": 0,
            "training_started": False,
            "real_vehicle_used": False,
        },
        "next_gate": (
            "new_explicit_user_simulation_execution_instruction_then_"
            "caller_exact_hash_execution_release_validation"
        ),
        "claim_limit": (
            "offline_execution_readiness_closure_only_not_simulation_"
            "execution_evidence_safety_performance_or_deployment_readiness"
        ),
    }


def _atomic_yaml(path: Path, value: object) -> None:
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
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
    parser.add_argument("--output", type=Path, default=WORKSPACE / READINESS_REVIEW)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    root = args.workspace.resolve()
    output = args.output.resolve()
    canonical_output = (root / READINESS_REVIEW).resolve()
    if output != canonical_output:
        parser.error("output must be the canonical R6-I3 readiness review")
    review = build_review(root)
    if args.check_only:
        persisted = _snapshot(root, READINESS_REVIEW, parse_yaml=True)["document"]
        _require(persisted == review, "persisted readiness review drifted")
    else:
        _require(output.parent.is_dir(), "static readiness artifact root is missing")
        _atomic_yaml(output, review)
    print(yaml.safe_dump(review, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
