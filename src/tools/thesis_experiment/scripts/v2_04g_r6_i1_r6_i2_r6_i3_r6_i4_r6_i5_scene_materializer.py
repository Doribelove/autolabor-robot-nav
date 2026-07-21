#!/usr/bin/env python3
"""Fail-closed, offline-only R6-I5 scene materialization.

The entrypoint clones the frozen R6-I1 scene specifications while changing
exactly the manifest identity and seven scene identity/seed pairs.  It does
not import ROS, create an execution release, create attempts or journals, or
start a subprocess.  Materialization is assembled in a private sibling
directory and published with Linux ``RENAME_NOREPLACE`` only after every byte
has been written and fsynced.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import secrets
import stat
import sys
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple
import xml.etree.ElementTree as ET

import yaml


sys.dont_write_bytecode = True
SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_SRC = SCRIPT_DIR.parent / "src"
V2_SCENE_SOURCE = PACKAGE_SRC / "thesis_experiment" / "v2_scene.py"
_V2_SCENE_SPEC = importlib.util.spec_from_file_location(
    "_r6_i5_offline_v2_scene", V2_SCENE_SOURCE
)
if _V2_SCENE_SPEC is None or _V2_SCENE_SPEC.loader is None:
    raise ImportError("cannot load the offline V2 scene compiler")
_V2_SCENE = importlib.util.module_from_spec(_V2_SCENE_SPEC)
_V2_SCENE_SPEC.loader.exec_module(_V2_SCENE)
compile_v2_manifest = _V2_SCENE.compile_v2_manifest
render_v2_scene_sdf = _V2_SCENE.render_v2_scene_sdf


STAGE = "V2-04G-R6-I5"
DEFAULT_WORKSPACE = Path("/home/robot/robot_ws_base_rl")
DERIVATION = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i5_scene_derivation.yaml"
)
# Deliberately invalid until the independently authored canonical derivation
# exists.  Materialization cannot pass while this guard remains unbound.
EXPECTED_DERIVATION_SHA256 = (
    "b74f24e169f3ffbe98f0139fc01dd78c1d2a1f8d6040df130719680ce4350145"
)

CONTRACT = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_bounded_simulation_execution_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_preregistration.yaml"
)
AUTHORIZATION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
TRANSITION = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i5_stage_transition.yaml"
)
RELEASE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i5_execution_release.yaml"
)
SOURCE_MANIFEST = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/v2_04g_r6_i1_scenes.yaml"
)
SOURCE_COMPILED_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/compiled_scenes"
)
SOURCE_INDEX = SOURCE_COMPILED_ROOT / "compiled_scene_index.yaml"
TARGET_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution"
)
TARGET_MANIFEST = TARGET_ROOT / "v2_04g_r6_i5_scenes.yaml"
TARGET_COMPILED_ROOT = TARGET_ROOT / "compiled_scenes"
TARGET_INDEX = TARGET_COMPILED_ROOT / "compiled_scene_index.yaml"
TARGET_AUDIT = TARGET_ROOT / "v2_04g_r6_i5_scene_behavior_equivalence.yaml"
TARGET_DEPENDENCY_CLOSURE = TARGET_ROOT / "execution_dependency_closure.yaml"
TARGET_READINESS_REVIEW = (
    TARGET_ROOT / "v2_04g_r6_i5_execution_readiness_review.yaml"
)
CANONICAL_READINESS_FIREWALL_ALLOWLIST = frozenset(
    {TARGET_DEPENDENCY_CLOSURE, TARGET_READINESS_REVIEW}
)

EXPECTED_SOURCE_MANIFEST_SHA256 = (
    "12c50c598951ab09bf5696b9c3acec97691a4d5ae92063352f2a7e46caf87081"
)
EXPECTED_SOURCE_INDEX_SHA256 = (
    "1f1cdde389dc98687142ca8d8c47c03bc8391b003d9103bde05c0e41cfddc4a0"
)
TARGET_MANIFEST_ID = "fam_teb_v2_04g_r6_i5_bounded_simulation_scenes_1"
EXPECTED_FAMILIES = (
    "DYNAMIC",
    "DYNAMIC",
    "DYNAMIC",
    "CRUISE",
    "STATIC_DENSE",
    "CORRIDOR",
    "MANEUVER",
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
    ("v2-04g-r6-i5-dynamic-conflict-single-s5161", 5161),
    ("v2-04g-r6-i5-dynamic-conflict-multi-s5162", 5162),
    ("v2-04g-r6-i5-dynamic-semantic-clear-s5163", 5163),
    ("v2-04g-r6-i5-compile-support-cruise-s5164", 5164),
    ("v2-04g-r6-i5-compile-support-static-s5165", 5165),
    ("v2-04g-r6-i5-compile-support-corridor-s5166", 5166),
    ("v2-04g-r6-i5-compile-support-maneuver-s5167", 5167),
)
EXPECTED_EXECUTION_ROLES = (
    "paired_semantic_probe",
    "paired_semantic_probe",
    "paired_semantic_probe",
    "compile_support_only_never_execute",
    "compile_support_only_never_execute",
    "compile_support_only_never_execute",
    "compile_support_only_never_execute",
)
EXPECTED_LAYOUT_VARIANTS = (
    "v2_04g_r6_i1_single_circle_contact",
    "v2_04g_r6_i1_multi_circle_contact",
    "v2_04g_r6_i1_time_separated_crossing",
    "v2_04g_r6_i1_compile_support_cruise",
    "v2_04g_r6_i1_compile_support_static",
    "v2_04g_r6_i1_compile_support_corridor",
    "v2_04g_r6_i1_compile_support_maneuver",
)
EXPECTED_EVALUATOR_REASONS = (
    "preregistered_single_track_circle_contact",
    "preregistered_multi_track_circle_contact",
    "preregistered_time_separated_centerline_crossing",
    "compile_support_only_not_evidence",
    "compile_support_only_not_evidence",
    "compile_support_only_not_evidence",
    "compile_support_only_not_evidence",
)
FRESH_SEEDS = frozenset(seed for _, seed in EXPECTED_TARGET_SCENES)
EXECUTION_SEEDS = (5161, 5162, 5163)
COMPILE_SUPPORT_SEEDS = (5164, 5165, 5166, 5167)
EXPECTED_CHANGED_PATHS = tuple(
    ["manifest_id"]
    + [
        "scenes[{}].{}".format(index, field)
        for index in range(7)
        for field in ("scene_id", "seed")
    ]
)
MAX_RESOURCE_BYTES = 32 * 1024 * 1024
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")
FRESH_SCENE_ID = re.compile(r"^v2-04g-r6-i5-[a-z0-9-]+-s516[1-7]$")


class R6I5SceneMaterializationError(ValueError):
    """Raised when any offline materialization invariant is not exact."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader: yaml.SafeLoader, node: yaml.Node, deep: bool = False) -> dict:
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise R6I5SceneMaterializationError("unhashable YAML mapping key") from exc
        if duplicate:
            raise R6I5SceneMaterializationError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise R6I5SceneMaterializationError(message)


def _exact_keys(value: object, expected: Iterable[object], context: str) -> None:
    _require(type(value) is dict, "{} must be a mapping".format(context))
    actual = set(value)
    wanted = set(expected)
    _require(
        actual == wanted,
        "{} keys differ; missing={}, extra={}".format(
            context, sorted(wanted - actual, key=str), sorted(actual - wanted, key=str)
        ),
    )


def _canonical_sha(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: object) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _safe_parts(relative: Path) -> Tuple[str, ...]:
    candidate = Path(relative)
    _require(
        not candidate.is_absolute()
        and bool(candidate.parts)
        and all(part not in {"", ".", ".."} for part in candidate.parts),
        "unsafe workspace-relative path: {}".format(candidate),
    )
    return tuple(candidate.parts)


def _snapshot(
    workspace: Path,
    relative: Path,
    *,
    parse_yaml: bool = False,
    require_unique_keys: bool = True,
    max_bytes: int = MAX_RESOURCE_BYTES,
) -> dict:
    """Read exactly one regular file once, without following any symlink."""

    parts = _safe_parts(relative)
    root = Path(workspace).resolve(strict=True)
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors: List[int] = []
    payload = b""
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        file_descriptor = os.open(parts[-1], flags, dir_fd=descriptors[-1])
        try:
            before = os.fstat(file_descriptor)
            _require(stat.S_ISREG(before.st_mode), "resource is not a regular file: {}".format(relative))
            _require(before.st_nlink == 1, "resource must have exactly one hard link: {}".format(relative))
            _require(before.st_size <= max_bytes, "resource exceeds size limit: {}".format(relative))
            chunks = []
            total = 0
            while True:
                chunk = os.read(file_descriptor, min(1024 * 1024, max_bytes + 1 - total))
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
                _require(total <= max_bytes, "resource exceeds size limit: {}".format(relative))
            after = os.fstat(file_descriptor)
            stable_fields = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")
            _require(
                tuple(getattr(before, field) for field in stable_fields)
                == tuple(getattr(after, field) for field in stable_fields),
                "resource changed during single-open read: {}".format(relative),
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "resource size drifted: {}".format(relative))
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise R6I5SceneMaterializationError(
            "cannot single-open/no-follow {}: {}".format(relative, exc)
        ) from exc
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
            loader = _UniqueKeyLoader if require_unique_keys else yaml.SafeLoader
            document = yaml.load(payload.decode("utf-8"), Loader=loader)
        except (UnicodeDecodeError, yaml.YAMLError) as exc:
            raise R6I5SceneMaterializationError(
                "cannot parse YAML {}: {}".format(relative, exc)
            ) from exc
        _require(type(document) is dict, "YAML root must be a mapping: {}".format(relative))
        result["document"] = document
    return result


def _entry_absent(workspace: Path, relative: Path) -> bool:
    """Check one exact entry without following the final component."""

    parts = _safe_parts(relative)
    root = Path(workspace).resolve(strict=True)
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors: List[int] = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        try:
            os.stat(parts[-1], dir_fd=descriptors[-1], follow_symlinks=False)
        except FileNotFoundError:
            return True
        return False
    except OSError as exc:
        raise R6I5SceneMaterializationError(
            "cannot inspect no-follow path {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _contains_forbidden_key(value: object, forbidden: str) -> bool:
    if type(value) is dict:
        return forbidden in value or any(
            _contains_forbidden_key(item, forbidden) for item in value.values()
        )
    if type(value) is list:
        return any(_contains_forbidden_key(item, forbidden) for item in value)
    return False


def _validate_derivation(document: Mapping[str, Any]) -> None:
    """Validate the closed I5 derivation contract and exact seven-row map."""

    _require(not _contains_forbidden_key(document, "scene_patch"), "scene_patch is forbidden at every depth")
    _exact_keys(
        document,
        (
            "schema_version", "architecture_generation", "stage", "derivation_id",
            "status", "simulation_only", "offline_derivation_only", "formal_result",
            "runtime_ready", "execution_ready", "stage_execution_authorization_present",
            "execution_authorized_by_derivation",
            "execution_release_present", "training_allowed", "real_vehicle_use_forbidden",
            "derivation_rule", "authority_dag", "source_manifest", "source_compiled_index",
            "target_manifest_id", "target_path", "target_compiled_root",
            "target_behavior_audit", "behavior_equivalence_contract",
            "freshness_audit",
            "execution_probe_seeds", "compile_support_only_seeds",
            "compile_support_seeds_are_evidence", "seed_roles", "scene_derivations",
            "expected_compiled_children", "forbidden_seed_sets",
            "materialization_and_execution_boundary",
        ),
        "scene_derivation",
    )
    _require(str(document["schema_version"]) == "2.0", "derivation schema version drifted")
    _require(document["architecture_generation"] == "v2", "derivation architecture drifted")
    _require(document["stage"] == STAGE, "derivation stage drifted")
    _require(
        document["derivation_id"] == "fam_teb_v2_04g_r6_i5_bounded_simulation_scenes_1"
        and document["status"] == "fresh_scene_derivation_frozen_materialization_authorized"
        and document["simulation_only"] is True
        and document["offline_derivation_only"] is True
        and document["formal_result"] is False
        and document["runtime_ready"] is False
        and document["execution_ready"] is False
        and document["stage_execution_authorization_present"] is True
        and document["execution_authorized_by_derivation"] is False
        and document["execution_release_present"] is False
        and document["training_allowed"] is False
        and document["real_vehicle_use_forbidden"] is True,
        "derivation safety boundary drifted",
    )
    _require(
        document["derivation_rule"]
        == "clone_frozen_r6_i1_behavior_replace_only_manifest_scene_id_and_seed",
        "derivation rule drifted",
    )
    authority_dag = document["authority_dag"]
    _exact_keys(
        authority_dag,
        ("contract", "preregistration", "authorization", "transition"),
        "scene_derivation.authority_dag",
    )
    expected_bindings = (
        ("contract", CONTRACT),
        ("preregistration", PREREGISTRATION),
        ("authorization", AUTHORIZATION),
        ("transition", TRANSITION),
    )
    for label, expected_path in expected_bindings:
        binding = authority_dag[label]
        _exact_keys(binding, ("path", "sha256"), "scene_derivation.authority_dag.{}".format(label))
        _require(binding["path"] == expected_path.as_posix(), "{} path drifted".format(label))
        _require(type(binding["sha256"]) is str and HEX_SHA256.fullmatch(binding["sha256"]), "{} hash is invalid".format(label))
    _require(
        document["source_manifest"]
        == {"path": SOURCE_MANIFEST.as_posix(), "sha256": EXPECTED_SOURCE_MANIFEST_SHA256},
        "source manifest binding drifted",
    )
    _require(
        document["source_compiled_index"]
        == {"path": SOURCE_INDEX.as_posix(), "sha256": EXPECTED_SOURCE_INDEX_SHA256},
        "source compiled index binding drifted",
    )
    _require(document["target_manifest_id"] == TARGET_MANIFEST_ID, "target manifest id drifted")
    _require(document["target_path"] == TARGET_MANIFEST.as_posix(), "target manifest path drifted")
    _require(document["target_compiled_root"] == TARGET_COMPILED_ROOT.as_posix(), "target compiled root drifted")
    _require(document["target_behavior_audit"] == TARGET_AUDIT.as_posix(), "target audit path drifted")
    equivalence = document["behavior_equivalence_contract"]
    _exact_keys(
        equivalence,
        (
            "allowed_top_level_changes", "allowed_per_scene_changes",
            "behavior_fields_changed", "preserve_scene_order",
            "preserve_layout_variant_exactly", "preserve_evaluator_only_exactly",
            "preserve_scene_timing_exactly", "preserve_randomization_parameters_exactly",
            "compare_type_sensitively", "compiled_outputs_must_reproduce_deterministically",
        ),
        "behavior_equivalence_contract",
    )
    _require(
        equivalence
        == {
            "allowed_top_level_changes": ["manifest_id"],
            "allowed_per_scene_changes": ["scene_id", "seed"],
            "behavior_fields_changed": [],
            "preserve_scene_order": True,
            "preserve_layout_variant_exactly": True,
            "preserve_evaluator_only_exactly": True,
            "preserve_scene_timing_exactly": True,
            "preserve_randomization_parameters_exactly": True,
            "compare_type_sensitively": True,
            "compiled_outputs_must_reproduce_deterministically": True,
        },
        "behavior equivalence contract drifted",
    )
    _require(
        document["freshness_audit"]
        == {
            "authoritative_pre_i5_seed_and_scene_fields_scanned": True,
            "execution_or_compile_seed_match_found": False,
            "target_scene_identity_match_found": False,
            "sha256_substring_matches_excluded_from_seed_evidence": True,
            "historical_allocated_high_watermark": 5157,
        },
        "canonical derivation freshness audit drifted",
    )
    _require(document["execution_probe_seeds"] == list(EXECUTION_SEEDS), "execution seeds drifted")
    _require(document["compile_support_only_seeds"] == list(COMPILE_SUPPORT_SEEDS), "support seeds drifted")
    _require(document["compile_support_seeds_are_evidence"] is False, "support seeds became evidence")
    _require(
        document["seed_roles"]
        == {
            5161: "single_track_circle_contact",
            5162: "multi_track_circle_contact",
            5163: "time_separated_centerline_crossing",
            5164: "compile_support_only_cruise",
            5165: "compile_support_only_static_dense",
            5166: "compile_support_only_corridor",
            5167: "compile_support_only_maneuver",
        },
        "seed-role roster drifted",
    )
    rows = document["scene_derivations"]
    _require(type(rows) is list and len(rows) == 7, "scene derivation must contain exactly seven rows")
    row_keys = (
        "source_scene_id", "source_seed", "target_scene_id", "seed",
        "execution_role", "layout_variant", "evaluator_reason", "behavior_fields_changed",
    )
    for index, row in enumerate(rows):
        _exact_keys(row, row_keys, "scene_derivations[{}]".format(index))
        source_id, source_seed = EXPECTED_SOURCE_SCENES[index]
        target_id, target_seed = EXPECTED_TARGET_SCENES[index]
        _require(
            row
            == {
                "source_scene_id": source_id,
                "source_seed": source_seed,
                "target_scene_id": target_id,
                "seed": target_seed,
                "execution_role": EXPECTED_EXECUTION_ROLES[index],
                "layout_variant": EXPECTED_LAYOUT_VARIANTS[index],
                "evaluator_reason": EXPECTED_EVALUATOR_REASONS[index],
                "behavior_fields_changed": [],
            },
            "scene derivation row {} drifted".format(index),
        )
    _require(
        document["expected_compiled_children"]
        == [
            Path(path).name
            for path in _expected_child_paths(
                TARGET_COMPILED_ROOT, EXPECTED_TARGET_SCENES
            )
        ],
        "expected compiled child roster/order drifted",
    )
    forbidden = document["forbidden_seed_sets"]
    expected_forbidden = {
        "held_out_5001_5010": list(range(5001, 5011)),
        "r5_terminal_allocation_5111_5135": list(range(5111, 5136)),
        "r6_i1_terminal_allocation_5141_5147": list(range(5141, 5148)),
        "r6_i3_failed_release_allocation_5151_5157": list(range(5151, 5158)),
        "prior_failed_identities": [4902, 4973, 4996, 5073, 5094, 5111, 5141],
    }
    _require(forbidden == expected_forbidden, "forbidden seed sets drifted")
    flattened = []
    for label, seeds in forbidden.items():
        _require(type(label) is str and type(seeds) is list, "forbidden seed set is invalid")
        _require(all(type(seed) is int for seed in seeds), "forbidden seed must be an integer")
        flattened.extend(seeds)
    _require(not FRESH_SEEDS.intersection(flattened), "fresh seeds overlap a forbidden allocation")
    _require(set(range(5001, 5011)).issubset(flattened), "held-out seed firewall is incomplete")
    _require(set(range(5111, 5136)).issubset(flattened), "R5 seed firewall is incomplete")
    _require(set(range(5141, 5148)).issubset(flattened), "R6-I1 seed firewall is incomplete")
    _require(set(range(5151, 5158)).issubset(flattened), "R6-I3 seed firewall is incomplete")
    boundary = document["materialization_and_execution_boundary"]
    _require(type(boundary) is dict, "materialization boundary is missing")
    _require(
        boundary
        == {
            "fresh_scene_materialization_authorized_offline": True,
            "evidence_units_consumed_by_materialization": 0,
            "compile_support_evidence_units": 0,
            "execution_release_created_by_derivation": False,
            "attempt_root_or_journal_created_by_derivation": False,
            "ros_or_gazebo_started_by_derivation": False,
            "execution_may_start_from_derivation_alone": False,
            "unique_exact_hash_release_and_full_prejournal_required": True,
            "separate_future_user_instruction_after_valid_release_required": False,
        },
        "materialization boundary drifted",
    )


def _verify_derivation_bindings(workspace: Path, document: Mapping[str, Any]) -> List[dict]:
    rows = []
    for label in ("contract", "preregistration", "authorization", "transition"):
        binding = document["authority_dag"][label]
        snapshot = _snapshot(workspace, Path(binding["path"]))
        _require(snapshot["sha256"] == binding["sha256"], "derivation binding drifted: {}".format(label))
        rows.append({"label": label, "path": binding["path"], "sha256": binding["sha256"]})
    return rows


def _diff_paths(left: object, right: object, prefix: str = "") -> List[str]:
    """Return deterministic, type-sensitive recursive difference paths."""

    if type(left) is not type(right):
        return [prefix or "$ROOT"]
    if type(left) is dict:
        left_keys = set(left)
        right_keys = set(right)
        differences = []
        for key in sorted(left_keys | right_keys, key=lambda item: (type(item).__name__, repr(item))):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in left or key not in right:
                differences.append(child)
            else:
                differences.extend(_diff_paths(left[key], right[key], child))
        return differences
    if type(left) is list:
        differences = []
        common = min(len(left), len(right))
        for index in range(common):
            child = "{}[{}]".format(prefix, index)
            differences.extend(_diff_paths(left[index], right[index], child))
        for index in range(common, max(len(left), len(right))):
            differences.append("{}[{}]".format(prefix, index))
        return differences
    return [] if left == right else [prefix or "$ROOT"]


def _normalize_scene_spec(scene: Mapping[str, Any]) -> dict:
    result = copy.deepcopy(dict(scene))
    result.pop("scene_id")
    result.pop("seed")
    return result


def _normalized_instance_sha(document: Mapping[str, Any]) -> str:
    normalized = copy.deepcopy(dict(document))
    normalized.pop("instance_sha256", None)
    normalized["scene"]["scene_id"] = "NORMALIZED_EXECUTION_SCENE"
    normalized["scene"]["seed"] = 0
    return _canonical_sha(normalized)


def _normalized_world_sha(payload: bytes) -> str:
    try:
        root = ET.fromstring(payload)
    except ET.ParseError as exc:
        raise R6I5SceneMaterializationError("compiled world XML is invalid") from exc
    world = root.find("world")
    _require(world is not None, "compiled SDF world element is missing")
    world.set("name", "NORMALIZED_EXECUTION_SCENE")
    return hashlib.sha256(ET.tostring(root, encoding="utf-8")).hexdigest()


def _compiled_payloads(
    manifest: Mapping[str, Any], workspace: Path, compiled_root: Path
) -> Tuple[List[dict], Dict[str, bytes]]:
    instances_first = compile_v2_manifest(manifest, workspace)
    instances_second = compile_v2_manifest(copy.deepcopy(manifest), workspace)
    _require(len(instances_first) == len(instances_second) == 7, "compiler did not produce seven instances")
    payloads: Dict[str, bytes] = {}
    repeat_payloads: Dict[str, bytes] = {}
    for output, instances in ((payloads, instances_first), (repeat_payloads, instances_second)):
        for instance in instances:
            scene_id = instance["scene"]["scene_id"]
            output[(compiled_root / (scene_id + ".instance.yaml")).as_posix()] = _yaml_bytes(instance)
            output[(compiled_root / (scene_id + ".world")).as_posix()] = render_v2_scene_sdf(instance).encode("utf-8")
    _require(payloads == repeat_payloads, "fresh scenes did not compile deterministically")
    return instances_first, payloads


def _expected_child_paths(root: Path, scenes: Sequence[Tuple[str, int]]) -> List[str]:
    return [
        (root / (scene_id + suffix)).as_posix()
        for scene_id, _ in scenes
        for suffix in (".instance.yaml", ".world")
    ]


def _verify_source_index_and_children(
    workspace: Path,
    source_document: Mapping[str, Any],
    source_payloads: Mapping[str, bytes],
) -> Tuple[dict, Dict[str, dict]]:
    index = _snapshot(workspace, SOURCE_INDEX, parse_yaml=True)
    _require(index["sha256"] == EXPECTED_SOURCE_INDEX_SHA256, "source compiled index hash drifted")
    document = index["document"]
    _exact_keys(
        document,
        ("schema_version", "manifest_id", "formal_result", "runtime_ready", "scene_count", "families", "files"),
        "source compiled index",
    )
    _require(
        str(document["schema_version"]) == "2.0"
        and document["manifest_id"] == source_document["manifest_id"]
        and document["formal_result"] is False
        and document["runtime_ready"] is False
        and document["scene_count"] == 7
        and type(document["scene_count"]) is int
        and tuple(document["families"]) == EXPECTED_FAMILIES,
        "source compiled index metadata drifted",
    )
    expected_paths = _expected_child_paths(SOURCE_COMPILED_ROOT, EXPECTED_SOURCE_SCENES)
    files = document["files"]
    _require(type(files) is list and len(files) == 14, "source compiled index must list fourteen children")
    _require([row.get("path") for row in files] == expected_paths, "source compiled child roster/order drifted")
    snapshots = {}
    for row in files:
        _exact_keys(row, ("path", "sha256"), "source compiled child row")
        _require(type(row["sha256"]) is str and HEX_SHA256.fullmatch(row["sha256"]), "source child hash is invalid")
        snapshot = _snapshot(workspace, Path(row["path"]), parse_yaml=row["path"].endswith(".instance.yaml"))
        _require(snapshot["sha256"] == row["sha256"], "source child hash drifted: {}".format(row["path"]))
        _require(snapshot["payload"] == source_payloads[row["path"]], "source child is not its deterministic recompile")
        snapshots[row["path"]] = snapshot
    return index, snapshots


def _derive_manifest(source: Mapping[str, Any]) -> dict:
    target = copy.deepcopy(dict(source))
    target["manifest_id"] = TARGET_MANIFEST_ID
    scenes = target.get("scenes")
    _require(type(scenes) is list and len(scenes) == 7, "source manifest must contain seven scenes")
    for index, (target_id, target_seed) in enumerate(EXPECTED_TARGET_SCENES):
        scenes[index]["scene_id"] = target_id
        scenes[index]["seed"] = target_seed
    differences = tuple(_diff_paths(source, target))
    _require(differences == EXPECTED_CHANGED_PATHS, "observed scene diff is not the exact 15-path identity map")
    return target


def _fresh_identity_hits(value: object, trail: str = "$ROOT") -> List[str]:
    hits = []
    if type(value) is int and value in FRESH_SEEDS:
        hits.append(trail)
    elif type(value) is str and FRESH_SCENE_ID.fullmatch(value):
        hits.append(trail)
    elif type(value) is dict:
        for key, item in value.items():
            hits.extend(_fresh_identity_hits(item, "{}.{}".format(trail, key)))
    elif type(value) is list:
        for index, item in enumerate(value):
            hits.extend(_fresh_identity_hits(item, "{}[{}]".format(trail, index)))
    return hits


def _walk_yaml_paths(workspace: Path) -> Iterable[Path]:
    roots = (
        Path("config/thesis_experiments/v2"),
        Path("experiments/manifests/v2"),
        Path("artifacts/v2"),
    )
    workspace = Path(workspace).resolve(strict=True)
    for relative_root in roots:
        absolute_root = workspace / relative_root
        if not absolute_root.is_dir():
            continue
        for current, directories, filenames in os.walk(str(absolute_root), followlinks=False):
            current_path = Path(current)
            kept = []
            for name in directories:
                candidate = current_path / name
                if candidate.is_symlink():
                    continue
                kept.append(name)
            directories[:] = kept
            for name in filenames:
                if name.endswith((".yaml", ".yml")):
                    yield (current_path / name).relative_to(workspace)


def _verify_fresh_firewall(workspace: Path, *, target_allowed: bool) -> dict:
    allowed = {CONTRACT, PREREGISTRATION, AUTHORIZATION, TRANSITION, DERIVATION}
    hit_rows = []
    scanned = 0
    for relative in _walk_yaml_paths(workspace):
        if relative == RELEASE:
            raise R6I5SceneMaterializationError("I5 execution release must remain absent during scene materialization")
        if relative in CANONICAL_READINESS_FIREWALL_ALLOWLIST:
            _require(
                target_allowed,
                "canonical readiness output appeared before authorized materialization",
            )
            # These two exact post-materialization readiness outputs bind the
            # fresh scene roster by design.  Their independent validators own
            # their byte/schema checks; excluding them here also keeps the
            # original scene-equivalence firewall receipt invariant.
            _snapshot(workspace, relative)
            continue
        under_target = (
            relative == TARGET_MANIFEST
            or relative == TARGET_AUDIT
            or TARGET_COMPILED_ROOT in relative.parents
        )
        if under_target:
            _require(target_allowed, "fresh target file appeared before authorized materialization")
            # The exact target roster is checked byte-for-byte by the scene
            # reviewer.  Keep this firewall scoped to authoritative pre-I5
            # evidence so its receipt is invariant across publication.
            continue
        snapshot = _snapshot(
            workspace,
            relative,
            parse_yaml=True,
            require_unique_keys=False,
        )
        scanned += 1
        hits = _fresh_identity_hits(snapshot["document"])
        if not hits:
            continue
        _require(relative in allowed, "fresh identity leaked into unauthorized YAML: {}".format(relative))
        hit_rows.append({"path": relative.as_posix(), "hit_count": len(hits)})
    return {
        "structured_yaml_files_scanned": scanned,
        "authorized_identity_documents": hit_rows,
        "fresh_seeds": list(sorted(FRESH_SEEDS)),
        "sha_substring_matches_counted": False,
        "pass": True,
    }


def _build_audit(
    derivation_snapshot: Mapping[str, Any],
    source_snapshot: Mapping[str, Any],
    source_index: Mapping[str, Any],
    target_manifest_payload: bytes,
    target_manifest: Mapping[str, Any],
    target_index_payload: bytes,
    target_index: Mapping[str, Any],
    scene_rows: List[dict],
    compiled_rows: List[dict],
    child_rows: List[dict],
    binding_rows: List[dict],
    firewall: Mapping[str, Any],
) -> dict:
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "audit_id": "fam_teb_v2_04g_r6_i5_scene_behavior_equivalence_1",
        "status": "fresh_scene_materialization_pass_behavior_equivalent",
        "review_result": "pass",
        "offline_only": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_release_present": False,
        "evidence_units_consumed": 0,
        "ros_or_gazebo_started": False,
        "attempt_root_or_journal_created": False,
        "claim_limit": "fresh_scene_identity_and_deterministic_compilation_only_not_execution_evidence",
        "provenance": {
            "scene_derivation": {"path": DERIVATION.as_posix(), "sha256": derivation_snapshot["sha256"]},
            "source_scene_manifest": {"path": SOURCE_MANIFEST.as_posix(), "sha256": source_snapshot["sha256"]},
            "source_compiled_index": {"path": SOURCE_INDEX.as_posix(), "sha256": source_index["sha256"]},
            "derivation_bindings": binding_rows,
        },
        "target_scene_manifest": {
            "path": TARGET_MANIFEST.as_posix(),
            "sha256": hashlib.sha256(target_manifest_payload).hexdigest(),
            "manifest_id": target_manifest["manifest_id"],
            "scene_count": 7,
            "seeds": [seed for _, seed in EXPECTED_TARGET_SCENES],
            "execution_seeds": list(EXECUTION_SEEDS),
            "compile_support_only_seeds": list(COMPILE_SUPPORT_SEEDS),
            "compile_support_evidence_units": 0,
            "schema_validation_pass": True,
        },
        "equivalence_rule": {
            "comparison": "exact_type_sensitive_recursive_equality_after_identity_normalization",
            "expected_and_observed_changed_paths": list(EXPECTED_CHANGED_PATHS),
            "changed_path_count": 15,
            "behavior_fields_allowed_to_change": [],
            "observed_behavior_fields_changed": [],
            "scene_order_preserved": True,
            "layout_variant_preserved_exactly": True,
            "evaluator_only_preserved_exactly": True,
            "trajectory_and_obstacle_fields_preserved_exactly": True,
            "randomization_parameters_preserved_exactly": True,
            "scene_timing_preserved_exactly": True,
            "pass": True,
        },
        "scene_equivalence": scene_rows,
        "compiled_execution_scene_equivalence": {
            "scope": "three_zero_randomization_execution_scenes_only",
            "execution_scene_randomization_is_zero": True,
            "compile_support_cross_seed_compiled_equivalence_required": False,
            "compile_support_reason": "seeds_5164_5167_have_nonzero_randomization_and_are_never_execution_evidence",
            "rows": compiled_rows,
            "all_three_execution_scenes_compiled_behavior_equivalent": True,
            "pass": True,
        },
        "compiled_scene_index": {
            "path": TARGET_INDEX.as_posix(),
            "sha256": hashlib.sha256(target_index_payload).hexdigest(),
            "schema_version": str(target_index["schema_version"]),
            "manifest_id": target_index["manifest_id"],
            "scene_count": 7,
            "child_count": 14,
            "families": list(EXPECTED_FAMILIES),
            "all_children_regular_non_symlink_files": True,
            "all_child_hashes_match": True,
            "deterministic_recompile_child_bytes_equal": True,
        },
        "compiled_children": child_rows,
        "fresh_seed_firewall": dict(firewall),
        "safety_boundary": {
            "execution_release_manifest_present": False,
            "execution_attempt_root_present": False,
            "execution_journal_present": False,
            "raw_or_semantic_evidence_present": False,
            "execution_may_start_from_scene_materialization": False,
        },
    }


def prepare_bundle(
    workspace: Path,
    supplied_derivation_sha256: str,
    *,
    require_target_absent: bool,
) -> dict:
    """Build and verify the complete materialization in memory without writes."""

    workspace = Path(workspace).resolve(strict=True)
    _require(HEX_SHA256.fullmatch(supplied_derivation_sha256 or "") is not None, "caller must supply an exact lowercase SHA-256")
    _require(
        HEX_SHA256.fullmatch(EXPECTED_DERIVATION_SHA256 or "") is not None,
        "materializer canonical derivation hash is intentionally unbound",
    )
    _require(supplied_derivation_sha256 == EXPECTED_DERIVATION_SHA256, "caller derivation hash differs from the bound canonical hash")
    _require(_entry_absent(workspace, RELEASE), "I5 execution release already exists")
    if require_target_absent:
        _require(_entry_absent(workspace, TARGET_ROOT), "canonical I5 execution root must be entirely new")

    derivation = _snapshot(workspace, DERIVATION, parse_yaml=True)
    _require(derivation["sha256"] == supplied_derivation_sha256, "canonical scene derivation hash mismatch")
    _validate_derivation(derivation["document"])
    binding_rows = _verify_derivation_bindings(workspace, derivation["document"])
    firewall = _verify_fresh_firewall(workspace, target_allowed=not require_target_absent)

    source = _snapshot(workspace, SOURCE_MANIFEST, parse_yaml=True)
    _require(source["sha256"] == EXPECTED_SOURCE_MANIFEST_SHA256, "source scene manifest hash drifted")
    source_document = source["document"]
    source_scenes = source_document.get("scenes")
    _require(type(source_scenes) is list and len(source_scenes) == 7, "source scene count drifted")
    for index, (scene_id, seed) in enumerate(EXPECTED_SOURCE_SCENES):
        scene = source_scenes[index]
        _require(
            scene.get("scene_id") == scene_id
            and scene.get("seed") == seed
            and type(scene.get("seed")) is int
            and scene.get("family") == EXPECTED_FAMILIES[index]
            and scene.get("layout", {}).get("variant") == EXPECTED_LAYOUT_VARIANTS[index]
            and scene.get("evaluator_only", {}).get("reason") == EXPECTED_EVALUATOR_REASONS[index],
            "source scene roster drifted at index {}".format(index),
        )

    source_instances, source_payloads = _compiled_payloads(source_document, workspace, SOURCE_COMPILED_ROOT)
    source_index, source_children = _verify_source_index_and_children(
        workspace, source_document, source_payloads
    )
    target_document = _derive_manifest(source_document)
    target_instances, target_payloads = _compiled_payloads(target_document, workspace, TARGET_COMPILED_ROOT)
    _require(tuple(instance["scene"]["family"] for instance in target_instances) == EXPECTED_FAMILIES, "target family order drifted")

    scene_rows = []
    for index, (source_scene, target_scene) in enumerate(zip(source_scenes, target_document["scenes"])):
        normalized_source = _normalize_scene_spec(source_scene)
        normalized_target = _normalize_scene_spec(target_scene)
        source_sha = _canonical_sha(normalized_source)
        target_sha = _canonical_sha(normalized_target)
        _require(source_sha == target_sha, "scene behavior spec drifted at index {}".format(index))
        source_id, source_seed = EXPECTED_SOURCE_SCENES[index]
        target_id, target_seed = EXPECTED_TARGET_SCENES[index]
        scene_rows.append(
            {
                "source_scene_id": source_id,
                "source_seed": source_seed,
                "target_scene_id": target_id,
                "target_seed": target_seed,
                "execution_role": EXPECTED_EXECUTION_ROLES[index],
                "normalized_source_sha256": source_sha,
                "normalized_target_sha256": target_sha,
                "behavior_fields_changed": [],
                "pass": True,
            }
        )
    source_top = copy.deepcopy(source_document)
    target_top = copy.deepcopy(target_document)
    source_top["manifest_id"] = "NORMALIZED_MANIFEST_ID"
    target_top["manifest_id"] = "NORMALIZED_MANIFEST_ID"
    for index in range(7):
        source_top["scenes"][index] = _normalize_scene_spec(source_top["scenes"][index])
        target_top["scenes"][index] = _normalize_scene_spec(target_top["scenes"][index])
    _require(_canonical_sha(source_top) == _canonical_sha(target_top), "manifest behavior differs after identity normalization")

    zero_randomization = {
        "position_jitter_m": 0.0,
        "yaw_jitter_rad": 0.0,
        "agent_time_jitter_s": 0.0,
    }
    compiled_rows = []
    for index in range(3):
        source_id, _ = EXPECTED_SOURCE_SCENES[index]
        target_id, _ = EXPECTED_TARGET_SCENES[index]
        _require(source_scenes[index]["randomization"] == zero_randomization, "source execution randomization drifted")
        _require(target_document["scenes"][index]["randomization"] == zero_randomization, "target execution randomization drifted")
        source_instance = source_children[(SOURCE_COMPILED_ROOT / (source_id + ".instance.yaml")).as_posix()]["document"]
        target_instance_payload = target_payloads[(TARGET_COMPILED_ROOT / (target_id + ".instance.yaml")).as_posix()]
        target_instance = yaml.load(target_instance_payload.decode("utf-8"), Loader=_UniqueKeyLoader)
        source_world = source_children[(SOURCE_COMPILED_ROOT / (source_id + ".world")).as_posix()]["payload"]
        target_world = target_payloads[(TARGET_COMPILED_ROOT / (target_id + ".world")).as_posix()]
        source_instance_sha = _normalized_instance_sha(source_instance)
        target_instance_sha = _normalized_instance_sha(target_instance)
        source_world_sha = _normalized_world_sha(source_world)
        target_world_sha = _normalized_world_sha(target_world)
        _require(
            source_instance_sha == target_instance_sha and source_world_sha == target_world_sha,
            "compiled execution behavior drifted at index {}".format(index),
        )
        compiled_rows.append(
            {
                "source_scene_id": source_id,
                "target_scene_id": target_id,
                "normalized_source_instance_sha256": source_instance_sha,
                "normalized_target_instance_sha256": target_instance_sha,
                "normalized_source_world_sha256": source_world_sha,
                "normalized_target_world_sha256": target_world_sha,
                "pass": True,
            }
        )

    child_paths = _expected_child_paths(TARGET_COMPILED_ROOT, EXPECTED_TARGET_SCENES)
    _require(list(target_payloads) == child_paths, "target compiled exact child roster/order drifted")
    child_rows = [
        {"path": path, "sha256": hashlib.sha256(target_payloads[path]).hexdigest()}
        for path in child_paths
    ]
    index_document = {
        "schema_version": "2.0",
        "manifest_id": TARGET_MANIFEST_ID,
        "formal_result": False,
        "runtime_ready": False,
        "scene_count": 7,
        "families": list(EXPECTED_FAMILIES),
        "files": child_rows,
    }
    manifest_payload = _yaml_bytes(target_document)
    index_payload = _yaml_bytes(index_document)
    audit_document = _build_audit(
        derivation,
        source,
        source_index,
        manifest_payload,
        target_document,
        index_payload,
        index_document,
        scene_rows,
        compiled_rows,
        child_rows,
        binding_rows,
        firewall,
    )
    audit_payload = _yaml_bytes(audit_document)
    outputs = {
        TARGET_MANIFEST.as_posix(): manifest_payload,
        **target_payloads,
        TARGET_INDEX.as_posix(): index_payload,
        TARGET_AUDIT.as_posix(): audit_payload,
    }
    return {
        "workspace": workspace,
        "derivation": derivation,
        "source": source,
        "source_index": source_index,
        "target_manifest": target_document,
        "target_index": index_document,
        "audit": audit_document,
        "outputs": outputs,
        "changed_paths": list(EXPECTED_CHANGED_PATHS),
        "scene_rows": scene_rows,
        "compiled_execution_rows": compiled_rows,
        "firewall": firewall,
    }


def _open_dir_chain(workspace: Path, relative: Path) -> Tuple[List[int], int]:
    parts = _safe_parts(relative)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    descriptor = os.open(str(Path(workspace).resolve(strict=True)), flags)
    descriptors.append(descriptor)
    try:
        for component in parts:
            descriptor = os.open(component, flags, dir_fd=descriptor)
            descriptors.append(descriptor)
    except Exception:
        for item in reversed(descriptors):
            os.close(item)
        raise
    return descriptors, descriptor


def _write_exclusive(directory_fd: int, relative_name: str, payload: bytes) -> None:
    _require("/" not in relative_name and relative_name not in {"", ".", ".."}, "unsafe output filename")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(relative_name, flags, 0o644, dir_fd=directory_fd)
    try:
        view = memoryview(payload)
        written = 0
        while written < len(view):
            count = os.write(descriptor, view[written:])
            _require(count > 0, "short output write")
            written += count
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _rename_noreplace(parent_fd: int, source_name: str, target_name: str) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    function = getattr(libc, "renameat2", None)
    _require(function is not None, "Linux renameat2 is required for atomic no-replace publication")
    function.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    result = function(
        parent_fd,
        os.fsencode(source_name),
        parent_fd,
        os.fsencode(target_name),
        1,  # RENAME_NOREPLACE
    )
    if result != 0:
        error = ctypes.get_errno()
        raise R6I5SceneMaterializationError(
            "atomic no-replace publication failed: {}".format(os.strerror(error))
        )


def _cleanup_private_root(parent_fd: int, stage_name: str, filenames: Sequence[str]) -> None:
    """Remove only the exact private entries created by this invocation."""

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        stage_fd = os.open(stage_name, flags, dir_fd=parent_fd)
    except OSError:
        return
    try:
        try:
            compiled_fd = os.open("compiled_scenes", flags, dir_fd=stage_fd)
        except OSError:
            compiled_fd = None
        if compiled_fd is not None:
            try:
                for name in filenames:
                    if "/compiled_scenes/" in name:
                        try:
                            os.unlink(Path(name).name, dir_fd=compiled_fd)
                        except FileNotFoundError:
                            pass
            finally:
                os.close(compiled_fd)
            try:
                os.rmdir("compiled_scenes", dir_fd=stage_fd)
            except FileNotFoundError:
                pass
        for name in filenames:
            if "/compiled_scenes/" not in name:
                try:
                    os.unlink(Path(name).name, dir_fd=stage_fd)
                except FileNotFoundError:
                    pass
    finally:
        os.close(stage_fd)
    try:
        os.rmdir(stage_name, dir_fd=parent_fd)
    except FileNotFoundError:
        pass


def materialize_bundle(bundle: Mapping[str, Any]) -> dict:
    """Publish a prepared bundle once, atomically and without replacement."""

    workspace = Path(bundle["workspace"])
    _require(_entry_absent(workspace, TARGET_ROOT), "canonical I5 execution root is no longer fresh")
    _require(_entry_absent(workspace, RELEASE), "I5 execution release appeared before scene publication")
    parent = TARGET_ROOT.parent
    descriptors, parent_fd = _open_dir_chain(workspace, parent)
    stage_name = ".r6_i5_execution.scene-materializing.{}.{}".format(os.getpid(), secrets.token_hex(8))
    output_names = list(bundle["outputs"])
    published = False
    stage_created = False
    try:
        os.mkdir(stage_name, 0o755, dir_fd=parent_fd)
        stage_created = True
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
        stage_fd = os.open(stage_name, flags, dir_fd=parent_fd)
        try:
            os.mkdir("compiled_scenes", 0o755, dir_fd=stage_fd)
            compiled_fd = os.open("compiled_scenes", flags, dir_fd=stage_fd)
            try:
                # The index is written last within compiled_scenes.
                for relative, payload in bundle["outputs"].items():
                    path = Path(relative)
                    if path == TARGET_INDEX or TARGET_COMPILED_ROOT not in path.parents:
                        continue
                    _write_exclusive(compiled_fd, path.name, payload)
                _write_exclusive(compiled_fd, TARGET_INDEX.name, bundle["outputs"][TARGET_INDEX.as_posix()])
                os.fsync(compiled_fd)
            finally:
                os.close(compiled_fd)
            _write_exclusive(stage_fd, TARGET_MANIFEST.name, bundle["outputs"][TARGET_MANIFEST.as_posix()])
            _write_exclusive(stage_fd, TARGET_AUDIT.name, bundle["outputs"][TARGET_AUDIT.as_posix()])
            os.fsync(stage_fd)
        finally:
            os.close(stage_fd)
        _require(_entry_absent(workspace, TARGET_ROOT), "canonical target appeared during private assembly")
        _rename_noreplace(parent_fd, stage_name, TARGET_ROOT.name)
        published = True
        os.fsync(parent_fd)
    except OSError as exc:
        raise R6I5SceneMaterializationError("exclusive materialization failed: {}".format(exc)) from exc
    finally:
        if stage_created and not published:
            _cleanup_private_root(parent_fd, stage_name, output_names)
        for descriptor in reversed(descriptors):
            os.close(descriptor)
    return {
        "status": "materialized_offline_scene_bundle_release_absent",
        "stage": STAGE,
        "target_root": TARGET_ROOT.as_posix(),
        "scene_count": 7,
        "compiled_child_count": 14,
        "changed_path_count": 15,
        "evidence_units_consumed": 0,
        "release_created": False,
        "attempt_or_journal_created": False,
        "ros_or_gazebo_started": False,
    }


def _directory_names(workspace: Path, relative: Path) -> List[str]:
    descriptors, descriptor = _open_dir_chain(workspace, relative)
    try:
        return sorted(os.listdir(descriptor))
    finally:
        for item in reversed(descriptors):
            os.close(item)


def review_materialization(workspace: Path, supplied_derivation_sha256: str) -> dict:
    """Read-only review of the persisted exact bundle."""

    workspace = Path(workspace).resolve(strict=True)
    _require(not _entry_absent(workspace, TARGET_ROOT), "canonical I5 scene root is absent")
    bundle = prepare_bundle(
        workspace, supplied_derivation_sha256, require_target_absent=False
    )
    root_names = _directory_names(workspace, TARGET_ROOT)
    _require(TARGET_MANIFEST.name in root_names, "target manifest is absent")
    _require(TARGET_AUDIT.name in root_names, "target behavior audit is absent")
    _require("compiled_scenes" in root_names, "target compiled root is absent")
    _require("attempts" not in root_names and "journals" not in root_names, "execution state appeared during scene review")
    expected_compiled_names = sorted(
        [Path(path).name for path in _expected_child_paths(TARGET_COMPILED_ROOT, EXPECTED_TARGET_SCENES)]
        + [TARGET_INDEX.name]
    )
    _require(
        _directory_names(workspace, TARGET_COMPILED_ROOT) == expected_compiled_names,
        "compiled directory is not the exact fourteen-child-plus-index roster",
    )
    reviewed = []
    for relative, expected_payload in bundle["outputs"].items():
        snapshot = _snapshot(workspace, Path(relative))
        _require(snapshot["payload"] == expected_payload, "persisted output differs from deterministic bundle: {}".format(relative))
        reviewed.append({"path": relative, "sha256": snapshot["sha256"]})
    return {
        "status": "scene_materialization_review_pass_release_absent",
        "stage": STAGE,
        "derivation": {"path": DERIVATION.as_posix(), "sha256": supplied_derivation_sha256},
        "target_root": TARGET_ROOT.as_posix(),
        "scene_count": 7,
        "compiled_child_count": 14,
        "changed_paths": list(EXPECTED_CHANGED_PATHS),
        "compiled_execution_equivalence_count": 3,
        "compile_support_cross_seed_compiled_equivalence_required": False,
        "outputs": reviewed,
        "fresh_seed_firewall": bundle["firewall"],
        "evidence_units_consumed": 0,
        "release_created": False,
        "attempt_or_journal_created": False,
        "ros_or_gazebo_started": False,
        "pass": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(DEFAULT_WORKSPACE))
    parser.add_argument("--derivation-sha256", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--materialize", action="store_true")
    args = parser.parse_args()
    try:
        bundle = prepare_bundle(
            Path(args.workspace),
            args.derivation_sha256,
            require_target_absent=True,
        )
        if args.check_only:
            receipt = {
                "status": "check_pass_target_absent_no_writes",
                "stage": STAGE,
                "derivation": {
                    "path": DERIVATION.as_posix(),
                    "sha256": bundle["derivation"]["sha256"],
                },
                "scene_count": 7,
                "compiled_child_count": 14,
                "changed_paths": bundle["changed_paths"],
                "compiled_execution_equivalence_count": 3,
                "fresh_seed_firewall": bundle["firewall"],
                "evidence_units_consumed": 0,
                "filesystem_writes": False,
                "release_created": False,
                "attempt_or_journal_created": False,
                "ros_or_gazebo_started": False,
            }
        else:
            receipt = materialize_bundle(bundle)
        print(json.dumps(receipt, ensure_ascii=False, sort_keys=True))
        return 0
    except (R6I5SceneMaterializationError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "status": "invalid",
                    "stage": STAGE,
                    "error": str(exc),
                    "evidence_units_consumed": 0,
                    "release_created": False,
                    "attempt_or_journal_created": False,
                    "ros_or_gazebo_started": False,
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
