#!/usr/bin/env python3
"""Build the deterministic, offline-only V2-04G-P1-R1 scene and schedule bundle.

This program has no execution mode: it does not import ROS, spawn subprocesses,
create a release/authorization, or create attempts/journals.  ``--materialize``
publishes only the new offline-readiness root and refuses to overwrite it.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import itertools
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, List, Mapping, Tuple

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
DERIVATION = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p1_r1_scene_derivation.yaml"
)
EXPECTED_DERIVATION_SHA256 = (
    "63ef206eb7bad279c3f2e3da19e9f38a1179445b9b57b50873420e8ec86c9d1d"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness"
)
MANIFEST_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_scenes.yaml"
COMPILED_RELATIVE = OUTPUT_ROOT / "compiled_scenes"
INDEX_RELATIVE = COMPILED_RELATIVE / "compiled_scene_index.yaml"
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_exact_schedule.yaml"
SCENE_AUDIT_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_scene_schedule_audit.yaml"
RESOURCE_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_resource_time_budget.yaml"
EMPTY_ASSESSMENT_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_assessment_not_run.yaml"

METHODS = (
    "r6_semantics_legacy_control",
    "r6_semantics_circle_contact",
    "fixed_teb_reference",
)
ROLES = (
    "single_true_conflict",
    "multi_true_conflict",
    "time_separated_semantic_clear",
)
ROLE_SEED_FIRST = dict(zip(ROLES, (5201, 5231, 5261)))
ROLE_EXPECTED = {
    ROLES[0]: "OBSERVED_CONFLICT",
    ROLES[1]: "OBSERVED_CONFLICT",
    ROLES[2]: "NO_CONFLICT_IN_HORIZON",
}


class P1OfflineBuildError(ValueError):
    """Raised when any offline freeze invariant differs."""


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise P1OfflineBuildError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


V2_SCENE = _load_module(
    "_p1_r1_v2_scene",
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
)
P1_STATS = _load_module(
    "_p1_r1_statistics",
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p1_statistics.py",
)


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise P1OfflineBuildError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1OfflineBuildError(message)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, default_flow_style=False
    ).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise P1OfflineBuildError("cannot load {}: {}".format(path, exc)) from exc
    _require(type(value) is dict, "YAML root must be a mapping: {}".format(path))
    return value


def _canonical_sha(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _changed_paths(source: Any, target: Any, prefix: str = "") -> List[str]:
    if type(source) is not type(target):
        return [prefix]
    if isinstance(source, dict):
        if set(source) != set(target):
            return [prefix]
        output = []
        for key in source:
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            output.extend(_changed_paths(source[key], target[key], child))
        return output
    if isinstance(source, list):
        if len(source) != len(target):
            return [prefix]
        output = []
        for index, (left, right) in enumerate(zip(source, target)):
            output.extend(_changed_paths(left, right, "{}[{}]".format(prefix, index)))
        return output
    return [] if source == target else [prefix]


def _build_scene_manifest(
    derivation: Mapping[str, Any], source: Mapping[str, Any]
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    source_by_id = {scene["scene_id"]: scene for scene in source["scenes"]}
    target_scenes = []
    equivalence = []
    for role in derivation["evidence_roles"]:
        source_scene = source_by_id[role["source_scene_id"]]
        _require(source_scene["seed"] == role["source_seed"], "source evidence seed differs")
        for seed in range(role["seed_first"], role["seed_last"] + 1):
            scene = copy.deepcopy(source_scene)
            scene["scene_id"] = role["target_scene_id_format"].format(seed=seed)
            scene["seed"] = seed
            scene["split"] = "test_id"
            changes = _changed_paths(source_scene, scene)
            _require(changes == ["scene_id", "split", "seed"], "evidence clone fields differ")
            target_scenes.append(scene)
            equivalence.append({
                "target_scene_id": scene["scene_id"],
                "seed": seed,
                "role": role["role"],
                "source_scene_id": source_scene["scene_id"],
                "changed_fields": changes,
                "behavioral_template_canonical_sha256": _canonical_sha({
                    key: value for key, value in scene.items()
                    if key not in {"scene_id", "seed", "split"}
                }),
            })
    for support in derivation["compile_support"]["scenes"]:
        source_scene = source_by_id[support["source_scene_id"]]
        _require(source_scene["seed"] == support["source_seed"], "source support seed differs")
        scene = copy.deepcopy(source_scene)
        scene["scene_id"] = support["target_scene_id"]
        scene["seed"] = support["target_seed"]
        changes = _changed_paths(source_scene, scene)
        _require(changes == ["scene_id", "seed"], "support clone fields differ")
        target_scenes.append(scene)
        equivalence.append({
            "target_scene_id": scene["scene_id"],
            "seed": scene["seed"],
            "role": "compile_support_only",
            "source_scene_id": source_scene["scene_id"],
            "changed_fields": changes,
            "behavioral_template_canonical_sha256": _canonical_sha({
                key: value for key, value in scene.items()
                if key not in {"scene_id", "seed"}
            }),
        })
    target = copy.deepcopy(source)
    target["manifest_id"] = derivation["target_bundle"]["manifest_id"]
    target["scenes"] = target_scenes
    _require(len(target_scenes) == 94, "target scene count differs")
    _require(len({scene["scene_id"] for scene in target_scenes}) == 94, "scene IDs differ")
    _require({scene["seed"] for scene in target_scenes} == set(range(5201, 5295)), "seed set differs")
    V2_SCENE.validate_v2_scene_manifest(target, WORKSPACE)
    return target, equivalence


def _build_compiled(
    manifest: Mapping[str, Any]
) -> Tuple[Dict[Path, bytes], Dict[str, Dict[str, str]], Dict[str, Any]]:
    files: Dict[Path, bytes] = {}
    scene_bindings = {}
    index_files = []
    instances = V2_SCENE.compile_v2_manifest(manifest, WORKSPACE)
    _require(len(instances) == 94, "compiled instance count differs")
    for instance in instances:
        scene_id = instance["scene"]["scene_id"]
        instance_relative = COMPILED_RELATIVE / (scene_id + ".instance.yaml")
        world_relative = COMPILED_RELATIVE / (scene_id + ".world")
        instance_payload = _yaml_bytes(instance)
        world_payload = V2_SCENE.render_v2_scene_sdf(instance).encode("utf-8")
        files[instance_relative] = instance_payload
        files[world_relative] = world_payload
        instance_binding = {
            "path": instance_relative.as_posix(),
            "sha256": _sha256_bytes(instance_payload),
        }
        world_binding = {
            "path": world_relative.as_posix(),
            "sha256": _sha256_bytes(world_payload),
        }
        scene_bindings[scene_id] = {
            "instance": instance_binding,
            "world": world_binding,
        }
        index_files.extend([instance_binding, world_binding])
    index = {
        "schema_version": "2.0",
        "manifest_id": manifest["manifest_id"],
        "formal_result": False,
        "runtime_ready": False,
        "scene_count": len(instances),
        "families": [instance["scene"]["family"] for instance in instances],
        "files": index_files,
    }
    files[INDEX_RELATIVE] = _yaml_bytes(index)
    return files, scene_bindings, index


def _construction() -> Dict[str, Any]:
    role_permutations = [list(item) for item in itertools.permutations(ROLES)]
    method_permutations = [list(item) for item in itertools.permutations(METHODS)]
    return {
        "algorithm": "six_permutation_three_role_wave_balance_v1",
        "waves": 30,
        "role_permutation_cycle": role_permutations,
        "role_permutation_index": "zero_based_wave_mod_6",
        "method_permutation_cycle": method_permutations,
        "method_permutation_index": "(zero_based_wave_plus_2_times_canonical_role_index)_mod_6",
        "block_rows_contiguous": True,
        "retry_resume_replacement_allowed": False,
    }


def _build_schedule(scene_bindings: Mapping[str, Mapping[str, Mapping[str, str]]]) -> Dict[str, Any]:
    construction = _construction()
    role_permutations = construction["role_permutation_cycle"]
    method_permutations = construction["method_permutation_cycle"]
    episodes = []
    for wave_index in range(30):
        role_order = role_permutations[wave_index % 6]
        for role_position, role in enumerate(role_order, 1):
            role_index = ROLES.index(role)
            seed = ROLE_SEED_FIRST[role] + wave_index
            scene_id = "v2-04g-p1-s{}".format(seed)
            method_order = method_permutations[(wave_index + 2 * role_index) % 6]
            block_id = "p1-block-s{}".format(seed)
            for method_position, method in enumerate(method_order, 1):
                binding = scene_bindings[scene_id]
                episodes.append({
                    "sequence": len(episodes) + 1,
                    "wave": wave_index + 1,
                    "role_order_position": role_position,
                    "block_id": block_id,
                    "within_block_position": method_position,
                    "scene_role": role,
                    "seed": seed,
                    "scene_id": scene_id,
                    "instance": dict(binding["instance"]),
                    "world": dict(binding["world"]),
                    "method_id": method,
                    "expected_ttc_status": ROLE_EXPECTED[role],
                    "attempt": 1,
                })
    schedule = {
        "schema_version": "1.0",
        "stage": "V2-04G-P1-R1",
        "status": "offline_schedule_frozen_not_authorized",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "schedule_id": "fam_teb_v2_04g_p1_r1_270_episode_counterbalanced_1",
        "construction": construction,
        "methods": list(METHODS),
        "scene_roles": list(ROLES),
        "episode_count": 270,
        "block_count": 90,
        "episodes": episodes,
        "canonical_episode_digest": P1_STATS.canonical_sha256(episodes),
    }
    P1_STATS.validate_schedule(schedule)
    return schedule


def _directory_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def build_bundle(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    derivation_path = workspace / DERIVATION
    _require(_sha256_file(derivation_path) == EXPECTED_DERIVATION_SHA256, "derivation SHA differs")
    derivation = _load_yaml(derivation_path)
    source_binding = derivation["source_bundle"]["manifest"]
    source_path = workspace / source_binding["path"]
    _require(_sha256_file(source_path) == source_binding["sha256"], "source manifest SHA differs")
    source_index_binding = derivation["source_bundle"]["compiled_index"]
    _require(
        _sha256_file(workspace / source_index_binding["path"]) == source_index_binding["sha256"],
        "source compiled index SHA differs",
    )
    compiler_binding = derivation["source_bundle"]["compiler"]
    _require(
        _sha256_file(workspace / compiler_binding["path"]) == compiler_binding["sha256"],
        "compiler SHA differs",
    )
    source = _load_yaml(source_path)
    manifest, equivalence = _build_scene_manifest(derivation, source)
    files: Dict[Path, bytes] = {MANIFEST_RELATIVE: _yaml_bytes(manifest)}
    compiled_files, scene_bindings, index = _build_compiled(manifest)
    files.update(compiled_files)
    schedule = _build_schedule(scene_bindings)
    files[SCHEDULE_RELATIVE] = _yaml_bytes(schedule)

    permutation_counts = {}
    for role in ROLES:
        blocks = {}
        for row in schedule["episodes"]:
            if row["scene_role"] == role:
                blocks.setdefault(row["block_id"], []).append(row)
        for rows in blocks.values():
            order = tuple(item["method_id"] for item in sorted(rows, key=lambda item: item["within_block_position"]))
            key = " > ".join(order)
            permutation_counts.setdefault(role, {})[key] = permutation_counts.setdefault(role, {}).get(key, 0) + 1
    audit = {
        "schema_version": "1.0",
        "stage": "V2-04G-P1-R1",
        "status": "offline_scene_schedule_audit_pass",
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "scene_counts": {
            "total": 94,
            "evidence": 90,
            "compile_support": 4,
            "compiled_children": len(index["files"]),
        },
        "seed_lifecycle": {
            "reserved_by_i6": list(range(5201, 5295)),
            "materialized_evidence": list(range(5201, 5291)),
            "materialized_compile_support": list(range(5291, 5295)),
            "authorized": [],
            "consumed": [],
        },
        "template_equivalence": equivalence,
        "schedule": {
            "blocks": 90,
            "episodes": 270,
            "confirmatory_episodes": 180,
            "contextual_episodes": 90,
            "canonical_episode_digest": schedule["canonical_episode_digest"],
            "method_permutation_counts_by_role": permutation_counts,
            "all_balance_invariants_pass": True,
        },
        "claim_population": "three_frozen_dynamic_templates_under_fresh_simulator_stochastic_seeds",
        "geometry_or_timing_generalization_allowed": False,
        "runtime_role_label_access_allowed": False,
    }
    files[SCENE_AUDIT_RELATIVE] = _yaml_bytes(audit)

    attempt_root = workspace / (
        "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/attempts"
    )
    attempt_sizes = sorted(
        _directory_bytes(path) for path in attempt_root.iterdir() if path.is_dir()
    )
    _require(len(attempt_sizes) == 6, "I5 attempt size roster differs")
    resource = {
        "schema_version": "1.0",
        "stage": "V2-04G-P1-R1",
        "status": "offline_estimate_only_not_authorized",
        "execution_authorized": False,
        "authorized_resource_budget": 0,
        "serial_concurrency": 1,
        "planned_episode_count": 270,
        "i5_measured_attempt_bytes_sorted": attempt_sizes,
        "i5_largest_attempt_bytes": max(attempt_sizes),
        "linear_projection_bytes": max(attempt_sizes) * 270,
        "canonical_artifact_cap_bytes": 512 * 1024 * 1024,
        "transient_workspace_cap_bytes": 2 * 1024 * 1024 * 1024,
        "minimum_free_disk_before_future_release_bytes": 4 * 1024 * 1024 * 1024,
        "scene_timeout_s": 80,
        "future_per_identity_wall_cap_s": 240,
        "expected_execution_window_h": [4, 6],
        "all_timeout_planning_window_h": [7, 10],
        "future_stage_wall_cap_h": 20,
        "estimate_is_evidence_budget": False,
        "future_release_must_remeasure_resources": True,
    }
    files[RESOURCE_RELATIVE] = _yaml_bytes(resource)
    empty_assessment = {
        "schema_version": "1.0",
        "stage": "V2-04G-P1-R1",
        "status": "assessment_not_run_no_execution_evidence",
        "formal_study_complete": False,
        "formal_result": False,
        "performance_improvement_demonstrated": False,
        "claim_authorized": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "evidence_units_consumed": 0,
        "four_gate_assessment_run": False,
        "reason": "no P1 release, attempts, journals, or terminal ledger exists",
    }
    files[EMPTY_ASSESSMENT_RELATIVE] = _yaml_bytes(empty_assessment)
    return files


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "offline output root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".p1_r1_offline.", dir=str(target.parent)))
    try:
        for relative, payload in sorted(files.items(), key=lambda item: item[0].as_posix()):
            local = temporary / relative.relative_to(OUTPUT_ROOT)
            local.parent.mkdir(parents=True, exist_ok=True)
            with local.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        os.rename(str(temporary), str(target))
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def _check_only(files: Mapping[Path, bytes], workspace: Path) -> None:
    for relative, expected in files.items():
        path = workspace / relative
        _require(path.is_file(), "missing persisted output: {}".format(relative))
        _require(path.read_bytes() == expected, "persisted output differs: {}".format(relative))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    files = build_bundle(workspace)
    if args.materialize:
        _materialize(files, workspace)
    else:
        _check_only(files, workspace)
    print("P1-R1 offline bundle {}: {} files".format(
        "materialized" if args.materialize else "verified", len(files)
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

