#!/usr/bin/env python3
"""Materialize the zero-authority V2-04G-P2-Q1 offline readiness bundle.

This builder has no ROS imports and no execute mode.  It deterministically
creates single-factor candidate files, fresh calibration/qualification scenes,
the exact 342-row schedule, and a command plan that a later release must bind.
It refuses to overwrite an existing bundle.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness"
)
METHOD_BINDINGS_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_method_bindings.yaml"
SCENES_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_scenes.yaml"
COMPILED_ROOT = OUTPUT_ROOT / "compiled_scenes"
COMPILED_INDEX_RELATIVE = COMPILED_ROOT / "compiled_scene_index.yaml"
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_exact_342_schedule.yaml"
COMMAND_PLAN_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_exact_342_command_plan.yaml"
AUDIT_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_construction_audit.yaml"
FAILURE_INDEX_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_failure_index.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p2_q1_source_freeze.sha256"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_readiness_report.yaml"

BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_offline_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_exact_runner.py"
)
NAVIGATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_navigation_episode.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_activation_probe_listener.py"
)
FIXED_SNAPSHOT_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_fixed_snapshot.py"
)
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
P1_EPISODE_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_episode.py"
)

METHODS = (
    "circle_contact_v120",
    "circle_contact_v125",
    "circle_contact_v130",
    "circle_contact_v135",
    "circle_contact_v140",
    "fixed_teb_v120__TEB-Default_alias",
    "fixed_teb_v125",
    "fixed_teb_v130",
    "fixed_teb_v135",
    "fixed_teb_v140",
    "TEB-Tuned-V1",
)
ROLES = (
    "single_true_conflict",
    "multi_true_conflict",
    "time_separated_semantic_clear",
)
ROLE_FIRST_SEED = dict(zip(ROLES, (5301, 5311, 5321)))
ROLE_EXPECTED = {
    "single_true_conflict": "OBSERVED_CONFLICT",
    "multi_true_conflict": "OBSERVED_CONFLICT",
    "time_separated_semantic_clear": "NO_CONFLICT_IN_HORIZON",
}
SPEED_GRID = (1.20, 1.25, 1.30, 1.35, 1.40)


class Q1OfflineBuildError(ValueError):
    """Raised when a P2-Q1 construction invariant differs."""


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise Q1OfflineBuildError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


V2_SCENE = _load_module(
    "_p2_q1_v2_scene",
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py",
)


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise Q1OfflineBuildError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise Q1OfflineBuildError(message)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(
        value,
        sort_keys=False,
        allow_unicode=True,
        default_flow_style=False,
    ).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise Q1OfflineBuildError("cannot load {}: {}".format(path, exc)) from exc
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
    return _sha256_bytes(payload)


def _leaf_differences(source: Any, target: Any, prefix: str = "") -> List[str]:
    if type(source) is not type(target):
        return [prefix]
    if isinstance(source, dict):
        output = []
        for key in sorted(set(source) | set(target), key=str):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in source or key not in target:
                output.append(child)
            else:
                output.extend(_leaf_differences(source[key], target[key], child))
        return output
    if isinstance(source, list):
        if len(source) != len(target):
            return [prefix]
        output = []
        for index, (left, right) in enumerate(zip(source, target)):
            output.extend(
                _leaf_differences(
                    left, right, "{}[{}]".format(prefix, index)
                )
            )
        return output
    return [] if source == target else [prefix]


def _receipt(relative: Path, payload: bytes) -> Dict[str, Any]:
    return {
        "path": relative.as_posix(),
        "sha256": _sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def _find_algorithm(registry: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    matches = [
        item
        for item in registry.get("algorithms", [])
        if type(item) is dict and item.get("name") == name
    ]
    _require(len(matches) == 1, "algorithm {} is not unique".format(name))
    return matches[0]


def _max_vel_parameter(bank: Mapping[str, Any]) -> Tuple[int, Mapping[str, Any]]:
    matches = [
        (index, item)
        for index, item in enumerate(bank.get("parameters", []))
        if type(item) is dict and item.get("name") == "max_vel_x"
    ]
    _require(len(matches) == 1, "max_vel_x registry row is not unique")
    return matches[0]


def _speed_suffix(speed: float) -> str:
    return "v{:03d}".format(int(round(speed * 100.0)))


def _build_methods(
    contract: Mapping[str, Any],
) -> Tuple[Dict[Path, bytes], Dict[str, Any], List[Dict[str, Any]]]:
    bindings = contract["source_bindings"]
    source_bank_path = WORKSPACE / bindings["circle_contact_anchor_bank"]["path"]
    fixed_path = WORKSPACE / bindings["fixed_teb_reference"]["path"]
    registry_path = WORKSPACE / bindings["t08_baseline_registry"]["path"]
    source_bank = _load_yaml(source_bank_path)
    fixed = _load_yaml(fixed_path)
    registry = _load_yaml(registry_path)
    _, velocity_parameter = _max_vel_parameter(source_bank)
    _require(
        velocity_parameter["bounds"] == [0.1, 1.2],
        "source max_vel_x bounds differ",
    )
    _require(
        source_bank["anchors"]["anchor_cruise"]["values"]["max_vel_x"] == 1.2,
        "source cruise speed differs",
    )
    tuned_theta = _find_algorithm(registry, "TEB-Tuned")["theta"]
    _require(
        list(registry["theta_order"]) == list(tuned_theta),
        "T08 tuned theta order differs",
    )

    files: Dict[Path, bytes] = {}
    method_rows: List[Dict[str, Any]] = []
    diff_audit = []
    generated_fixed: Dict[float, Dict[str, Any]] = {}

    for speed in SPEED_GRID:
        suffix = _speed_suffix(speed)
        candidate = copy.deepcopy(fixed)
        candidate["TebLocalPlannerROS"]["max_vel_x"] = speed
        relative = OUTPUT_ROOT / "candidate_configs" / (
            "fixed_teb_{}.yaml".format(suffix)
        )
        payload = _yaml_bytes(candidate)
        files[relative] = payload
        generated_fixed[speed] = _receipt(relative, payload)
        differences = _leaf_differences(fixed, candidate)
        expected = [] if speed == 1.2 else ["TebLocalPlannerROS.max_vel_x"]
        _require(differences == expected, "fixed TEB candidate diff differs")
        method_id = (
            "fixed_teb_v120__TEB-Default_alias"
            if speed == 1.2
            else "fixed_teb_{}".format(suffix)
        )
        method_rows.append(
            {
                "method_id": method_id,
                "family": "independently_tuned_fixed_teb_grid",
                "candidate_speed_mps": speed,
                "runtime_kind": "fixed_teb",
                "materialized_config": generated_fixed[speed],
                "source_config": {
                    "path": bindings["fixed_teb_reference"]["path"],
                    "sha256": bindings["fixed_teb_reference"]["sha256"],
                },
                "single_behavioral_field": "TebLocalPlannerROS.max_vel_x",
                "primary_baseline_selection_eligible_after_calibration": True,
                "transparent_teb_default_alias": speed == 1.2,
                "runtime_ready": False,
                "parameter_write_enabled": False,
            }
        )
        diff_audit.append(
            {
                "method_id": method_id,
                "source": bindings["fixed_teb_reference"]["path"],
                "changed_fields": differences,
                "changed_behavioral_field_count": len(differences),
            }
        )

    for speed in SPEED_GRID:
        suffix = _speed_suffix(speed)
        candidate = copy.deepcopy(source_bank)
        candidate["bank_id"] = "fam_teb_v2_04g_p2_q1_circle_contact_{}".format(
            suffix
        )
        candidate["status"] = "offline_materialized_not_runtime_ready"
        candidate["source_provenance"]["p2_q1"] = {
            "stage": "V2-04G-P2-Q1",
            "factor": "p2_cruise_forward_velocity_ceiling_v1",
            "method_id": "circle_contact_{}".format(suffix),
            "source_bank_sha256": bindings["circle_contact_anchor_bank"]["sha256"],
        }
        parameter_index, _ = _max_vel_parameter(candidate)
        candidate["parameters"][parameter_index]["bounds"][1] = 1.4
        cruise = candidate["anchors"]["anchor_cruise"]
        cruise["profile_id"] = "profile_cruise_p2_q1_{}_offline".format(suffix)
        cruise["values"]["max_vel_x"] = speed
        relative = OUTPUT_ROOT / "candidate_configs" / (
            "circle_contact_{}_anchor_bank.yaml".format(suffix)
        )
        payload = _yaml_bytes(candidate)
        files[relative] = payload
        differences = _leaf_differences(source_bank, candidate)
        allowed = {
            "bank_id",
            "status",
            "source_provenance.p2_q1",
            "parameters[{}].bounds[1]".format(parameter_index),
            "anchors.anchor_cruise.profile_id",
            "anchors.anchor_cruise.values.max_vel_x",
        }
        _require(set(differences) <= allowed, "circle candidate diff escaped allowlist")
        behavioral = [
            item
            for item in differences
            if item == "anchors.anchor_cruise.values.max_vel_x"
        ]
        _require(
            len(behavioral) == (0 if speed == 1.2 else 1),
            "circle candidate behavioral diff count differs",
        )
        method_id = "circle_contact_{}".format(suffix)
        method_rows.append(
            {
                "method_id": method_id,
                "family": "fam_teb_circle_contact_single_factor_grid",
                "candidate_speed_mps": speed,
                "runtime_kind": "rule_multi_anchor",
                "materialized_anchor_bank": _receipt(relative, payload),
                "fixed_startup_config": generated_fixed[1.2],
                "supervisor": dict(bindings["circle_contact_supervisor"]),
                "mechanism": dict(bindings["circle_contact_mechanism"]),
                "single_behavioral_field": "anchors.anchor_cruise.values.max_vel_x",
                "primary_baseline_selection_eligible_after_calibration": False,
                "transparent_teb_default_alias": False,
                "runtime_ready": False,
                "parameter_write_enabled": False,
            }
        )
        diff_audit.append(
            {
                "method_id": method_id,
                "source": bindings["circle_contact_anchor_bank"]["path"],
                "changed_fields": differences,
                "registry_metadata_diff_count": len(differences) - len(behavioral),
                "changed_behavioral_field_count": len(behavioral),
            }
        )

    tuned = copy.deepcopy(fixed)
    for field in registry["theta_order"]:
        tuned["TebLocalPlannerROS"][field] = tuned_theta[field]
    tuned_relative = (
        OUTPUT_ROOT / "candidate_configs" / "teb_tuned_v1_exact_t08.yaml"
    )
    tuned_payload = _yaml_bytes(tuned)
    files[tuned_relative] = tuned_payload
    tuned_differences = _leaf_differences(fixed, tuned)
    expected_tuned = {
        "TebLocalPlannerROS.{}".format(field)
        for field in registry["theta_order"]
        if fixed["TebLocalPlannerROS"][field] != tuned_theta[field]
    }
    _require(
        set(tuned_differences) == expected_tuned and len(tuned_differences) == 9,
        "TEB-Tuned-V1 exact nine-field diff differs",
    )
    method_rows.append(
        {
            "method_id": "TEB-Tuned-V1",
            "family": "frozen_engineering_baseline",
            "candidate_speed_mps": float(tuned_theta["max_vel_x"]),
            "runtime_kind": "fixed_teb",
            "materialized_config": _receipt(tuned_relative, tuned_payload),
            "source_registry": dict(bindings["t08_baseline_registry"]),
            "exact_t08_theta": {
                field: tuned_theta[field] for field in registry["theta_order"]
            },
            "primary_baseline_selection_eligible_after_calibration": True,
            "transparent_teb_default_alias": False,
            "runtime_ready": False,
            "parameter_write_enabled": False,
        }
    )
    diff_audit.append(
        {
            "method_id": "TEB-Tuned-V1",
            "source": bindings["fixed_teb_reference"]["path"],
            "changed_fields": tuned_differences,
            "changed_behavioral_field_count": 9,
            "exception": "preexisting_frozen_engineering_baseline_not_a_P2_candidate_amendment",
        }
    )

    by_id = {row["method_id"]: row for row in method_rows}
    _require(set(by_id) == set(METHODS), "materialized method roster differs")
    ordered = [by_id[method] for method in METHODS]
    document = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-Q1",
        "status": "offline_method_bindings_frozen_not_authorized",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "sole_p2_behavioral_field": "TebLocalPlannerROS.max_vel_x",
        "methods": ordered,
        "fair_baseline_rule": {
            "transparent_reference": "TEB-Default",
            "transparent_reference_is_automatically_primary": False,
            "frozen_engineering_candidate": "TEB-Tuned-V1",
            "independent_fixed_grid_candidates": [
                row["method_id"]
                for row in ordered
                if row["family"] == "independently_tuned_fixed_teb_grid"
            ],
            "future_primary_baseline": (
                "strongest_safety_qualified_calibration_frozen_candidate_between_"
                "TEB-Tuned-V1_and_independent_fixed_grid"
            ),
            "intentional_weakening_or_hidden_advantage_allowed": False,
        },
        "method_binding_digest": _canonical_sha(ordered),
    }
    return files, document, diff_audit


def _build_scenes(
    contract: Mapping[str, Any],
) -> Tuple[Dict[str, Any], List[Dict[str, Any]], Dict[int, str]]:
    source_binding = contract["source_bindings"]["i5_source_scenes"]
    source = _load_yaml(WORKSPACE / source_binding["path"])
    source_by_id = {scene["scene_id"]: scene for scene in source["scenes"]}
    template_ids = contract["scene_derivation"]["source_templates"]
    target_scenes = []
    equivalence = []
    seed_to_id: Dict[int, str] = {}

    def clone(
        template_key: str,
        scene_id: str,
        seed: int,
        role: str,
        reason: str,
        scheduled: bool,
    ) -> None:
        source_scene = source_by_id[template_ids[template_key]]
        scene = copy.deepcopy(source_scene)
        scene["scene_id"] = scene_id
        scene["seed"] = seed
        scene["split"] = "calibration"
        scene["evaluator_only"]["reason"] = reason
        differences = _leaf_differences(source_scene, scene)
        _require(
            set(differences)
            <= {"scene_id", "seed", "split", "evaluator_only.reason"},
            "scene clone escaped allowlist",
        )
        _require(seed not in seed_to_id, "scene seed duplicated")
        target_scenes.append(scene)
        seed_to_id[seed] = scene_id
        equivalence.append(
            {
                "target_scene_id": scene_id,
                "seed": seed,
                "role": role,
                "scheduled": scheduled,
                "source_scene_id": source_scene["scene_id"],
                "source_seed": source_scene["seed"],
                "changed_fields": differences,
                "behavioral_template_sha256": _canonical_sha(
                    {
                        key: value
                        for key, value in scene.items()
                        if key not in {"scene_id", "seed", "split", "evaluator_only"}
                    }
                ),
            }
        )

    role_slug = {
        "single_true_conflict": "single",
        "multi_true_conflict": "multi",
        "time_separated_semantic_clear": "semantic-clear",
    }
    for role in ROLES:
        first = ROLE_FIRST_SEED[role]
        for seed in range(first, first + 10):
            clone(
                role,
                "v2-04g-p2-q1-cal-{}-s{}".format(role_slug[role], seed),
                seed,
                role,
                "p2_q1_fresh_calibration_{}".format(role),
                True,
            )

    probe_template = {
        "braking": "cruise_probe",
        "high_speed_to_conflict_overlay_transition": "single_true_conflict",
        "ackermann_circle": "cruise_probe",
    }
    for row in contract["scene_derivation"]["qualification"]["speed_probe_rows"]:
        speed = float(row["speed_mps"])
        suffix = _speed_suffix(speed)
        for probe, seed_key in (
            ("braking", "braking_seed"),
            ("high_speed_to_conflict_overlay_transition", "overlay_transition_seed"),
            ("ackermann_circle", "ackermann_circle_seed"),
        ):
            seed = int(row[seed_key])
            clone(
                probe_template[probe],
                "v2-04g-p2-q1-qual-{}-{}-s{}".format(
                    probe.replace("_", "-"), suffix, seed
                ),
                seed,
                "qualification_{}".format(probe),
                "p2_q1_evaluator_only_qualification_{}_{}".format(probe, suffix),
                True,
            )

    support_templates = contract["scene_derivation"]["compile_support"]["templates"]
    support_seeds = contract["scene_derivation"]["compile_support"]["seeds"]
    for template_key, seed in zip(support_templates, support_seeds):
        clone(
            template_key,
            "v2-04g-p2-q1-support-{}-s{}".format(
                template_key.replace("_", "-"), seed
            ),
            int(seed),
            "compile_support_only",
            "p2_q1_compile_support_only_{}".format(template_key),
            False,
        )

    _require(len(target_scenes) == 48, "P2-Q1 scene count differs")
    _require(set(seed_to_id) == set(range(5301, 5349)), "P2-Q1 seed set differs")
    _require(
        not set(range(5401, 5491)) & set(seed_to_id),
        "held-out test seed leaked into Q1 scenes",
    )
    manifest = copy.deepcopy(source)
    manifest["manifest_id"] = "fam_teb_v2_04g_p2_q1_offline_scenes_1"
    manifest["scenes"] = target_scenes
    V2_SCENE.validate_v2_scene_manifest(manifest, WORKSPACE)
    return manifest, equivalence, seed_to_id


def _build_compiled(
    manifest: Mapping[str, Any],
) -> Tuple[Dict[Path, bytes], Dict[str, Dict[str, Any]], Dict[str, Any]]:
    files: Dict[Path, bytes] = {}
    bindings: Dict[str, Dict[str, Any]] = {}
    index_files = []
    instances = V2_SCENE.compile_v2_manifest(manifest, WORKSPACE)
    _require(len(instances) == 48, "compiled scene instance count differs")
    for instance in instances:
        scene_id = instance["scene"]["scene_id"]
        instance_relative = COMPILED_ROOT / (scene_id + ".instance.yaml")
        world_relative = COMPILED_ROOT / (scene_id + ".world")
        instance_payload = _yaml_bytes(instance)
        world_payload = V2_SCENE.render_v2_scene_sdf(instance).encode("utf-8")
        files[instance_relative] = instance_payload
        files[world_relative] = world_payload
        bindings[scene_id] = {
            "instance": _receipt(instance_relative, instance_payload),
            "world": _receipt(world_relative, world_payload),
        }
        index_files.extend(
            [bindings[scene_id]["instance"], bindings[scene_id]["world"]]
        )
    index = {
        "schema_version": "2.0",
        "manifest_id": manifest["manifest_id"],
        "stage": "V2-04G-P2-Q1",
        "status": "offline_compiled_scenes_frozen_not_authorized",
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": False,
        "scene_count": 48,
        "compiled_child_count": 96,
        "files": index_files,
    }
    files[COMPILED_INDEX_RELATIVE] = _yaml_bytes(index)
    return files, bindings, index


def _role_order(wave_index: int) -> List[str]:
    base = list(ROLES if (wave_index // 3) % 2 == 0 else tuple(reversed(ROLES)))
    offset = wave_index % 3
    return base[offset:] + base[:offset]


def _method_order(block_index: int) -> List[str]:
    cycle, offset = divmod(block_index, len(METHODS))
    base = list(METHODS if cycle % 2 == 0 else tuple(reversed(METHODS)))
    return base[offset:] + base[:offset]


def _build_schedule(
    contract: Mapping[str, Any],
    scene_bindings: Mapping[str, Mapping[str, Mapping[str, Any]]],
    seed_to_id: Mapping[int, str],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    rows = []
    block_index = 0
    for wave_index in range(10):
        for role_position, role in enumerate(_role_order(wave_index), 1):
            seed = ROLE_FIRST_SEED[role] + wave_index
            scene_id = seed_to_id[seed]
            block_id = "p2-q1-cal-block-s{}".format(seed)
            for method_position, method_id in enumerate(
                _method_order(block_index), 1
            ):
                binding = scene_bindings[scene_id]
                rows.append(
                    {
                        "sequence": len(rows) + 1,
                        "row_type": "calibration_navigation",
                        "wave": wave_index + 1,
                        "role_order_position": role_position,
                        "block_id": block_id,
                        "within_block_position": method_position,
                        "scene_role": role,
                        "seed": seed,
                        "scene_id": scene_id,
                        "instance": dict(binding["instance"]),
                        "world": dict(binding["world"]),
                        "method_id": method_id,
                        "expected_ttc_status": ROLE_EXPECTED[role],
                        "timeout_s": 80.0,
                        "attempt": 1,
                    }
                )
            block_index += 1
    _require(len(rows) == 330 and block_index == 30, "calibration schedule differs")

    qualification_lookup = {}
    for item in contract["scene_derivation"]["qualification"]["speed_probe_rows"]:
        qualification_lookup[float(item["speed_mps"])] = {
            "braking": int(item["braking_seed"]),
            "high_speed_to_conflict_overlay_transition": int(
                item["overlay_transition_seed"]
            ),
            "ackermann_circle": int(item["ackermann_circle_seed"]),
        }
    for speed in (1.25, 1.30, 1.35, 1.40):
        suffix = _speed_suffix(speed)
        for probe in (
            "braking",
            "high_speed_to_conflict_overlay_transition",
            "ackermann_circle",
        ):
            seed = qualification_lookup[speed][probe]
            scene_id = seed_to_id[seed]
            binding = scene_bindings[scene_id]
            rows.append(
                {
                    "sequence": len(rows) + 1,
                    "row_type": "qualification_probe",
                    "qualification_block_id": "p2-q1-qual-{}".format(suffix),
                    "probe_type": probe,
                    "candidate_speed_mps": speed,
                    "candidate_method_id": "circle_contact_{}".format(suffix),
                    "paired_fixed_method_id": "fixed_teb_{}".format(suffix),
                    "seed": seed,
                    "scene_id": scene_id,
                    "instance": dict(binding["instance"]),
                    "world": dict(binding["world"]),
                    "timeout_s": 80.0,
                    "attempt": 1,
                }
            )
    _require(len(rows) == 342, "total schedule row count differs")

    positions = {
        method: {position: 0 for position in range(1, 12)}
        for method in METHODS
    }
    for row in rows[:330]:
        positions[row["method_id"]][row["within_block_position"]] += 1
    _require(
        all(max(counts.values()) - min(counts.values()) <= 1 for counts in positions.values()),
        "method-position balance differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in rows},
        "held-out seed leaked into schedule",
    )
    construction = {
        "algorithm": (
            "cyclic_11_method_rotation_with_alternating_reversal_and_"
            "three_role_wave_balance_v1"
        ),
        "role_waves": 10,
        "method_cycles": 3,
        "method_cycle_orientation": ["forward", "reverse", "forward_partial"],
        "block_rows_contiguous": True,
        "common_random_numbers_within_calibration_block": True,
        "attempt_per_identity": 1,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
    }
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-Q1",
        "status": "offline_exact_schedule_frozen_not_authorized",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "schedule_id": "fam_teb_v2_04g_p2_q1_exact_342_attempt1_1",
        "construction": construction,
        "methods": list(METHODS),
        "calibration_block_count": 30,
        "calibration_row_count": 330,
        "qualification_row_count": 12,
        "total_row_count": 342,
        "held_out_test_row_count": 0,
        "rows": rows,
        "canonical_row_digest": _canonical_sha(rows),
    }
    audit = {
        "method_position_counts": positions,
        "maximum_method_position_count_difference": max(
            max(counts.values()) - min(counts.values())
            for counts in positions.values()
        ),
        "every_method_runs_every_calibration_seed_once": all(
            len(
                {
                    row["method_id"]
                    for row in rows[:330]
                    if row["seed"] == seed
                }
            )
            == 11
            for seed in range(5301, 5331)
        ),
        "qualification_speed_probe_pairs": 12,
        "held_out_test_rows": 0,
    }
    return schedule, audit


def _method_map(method_bindings: Mapping[str, Any]) -> Dict[str, Mapping[str, Any]]:
    return {row["method_id"]: row for row in method_bindings["methods"]}


def _build_command_plan(
    schedule: Mapping[str, Any],
    method_bindings: Mapping[str, Any],
) -> Dict[str, Any]:
    methods = _method_map(method_bindings)
    plan_rows = []
    for row in schedule["rows"]:
        if row["row_type"] == "calibration_navigation":
            method = methods[row["method_id"]]
            semantic = method["runtime_kind"] == "rule_multi_anchor"
            fixed_config = (
                method["fixed_startup_config"]
                if semantic
                else method["materialized_config"]
            )
            base = [
                "{ROSLAUNCH_EXECUTABLE}",
                "m2_gazebo",
                LAUNCH_RELATIVE.name,
                "world:={}".format(row["world"]["path"]),
                "seed:={}".format(row["seed"]),
                "xacro_executable:={XACRO_EXECUTABLE}",
                "attempt_stage:=V2-04G-P2-Q1",
                "attempt_profile_id:={}".format(row["method_id"]),
                "attempt_scene_id:={}".format(row["scene_id"]),
                "attempt_number:=1",
                "fixed_teb_config:={}".format(fixed_config["path"]),
                "start_move_base:=true",
                "start_world_model:=true",
                "start_rule_supervisor:={}".format(
                    "true" if semantic else "false"
                ),
                "start_typed_transaction:=false",
                "allow_simulation_teb_parameter_write:=false",
                "allow_unfrozen_simulation_candidate:=true",
                "gui:=false",
                "paused:=true",
            ]
            transaction = None
            if semantic:
                transaction = [
                    "{ROSLAUNCH_EXECUTABLE}",
                    "teb_mode_manager",
                    "v2_04g_r6_simulation_typed_anchor.launch",
                    "allow_simulation_teb_parameter_write:=true",
                    "allow_unfrozen_simulation_candidate:=true",
                    "anchor_bank:={}".format(
                        method["materialized_anchor_bank"]["path"]
                    ),
                    "mechanism_config:={}".format(method["mechanism"]["path"]),
                    "attempt_stage:=V2-04G-P2-Q1",
                    "attempt_profile_id:={}".format(row["method_id"]),
                    "attempt_scene_id:={}".format(row["scene_id"]),
                    "attempt_seed:={}".format(row["seed"]),
                    "attempt_number:=1",
                    "supervisor_config_sha256:={}".format(
                        method["supervisor"]["sha256"]
                    ),
                ]
            episode = [
                "{PYTHON_EXECUTABLE}",
                NAVIGATION_RELATIVE.as_posix(),
                "--instance",
                row["instance"]["path"],
                "--instance-sha256",
                row["instance"]["sha256"],
                "--method-id",
                row["method_id"],
                "--output-dir",
                "{FUTURE_ATTEMPT_ROOT}/episode",
                "--stage",
                "V2-04G-P2-Q1",
                "--sequence",
                str(row["sequence"]),
                "--attempt",
                "1",
            ]
            plan_rows.append(
                {
                    "sequence": row["sequence"],
                    "identity": {
                        key: row[key]
                        for key in (
                            "row_type",
                            "method_id",
                            "scene_id",
                            "seed",
                            "attempt",
                            "block_id",
                        )
                    },
                    "base_launch_argv": base,
                    "semantic_transaction_launch_argv": transaction,
                    "activation_listener_required": semantic,
                    "runtime_control_required": semantic,
                    "fixed_readback_capture_verify_required": not semantic,
                    "episode_argv": episode,
                }
            )
        else:
            suffix = _speed_suffix(float(row["candidate_speed_mps"]))
            semantic = (
                row["probe_type"]
                == "high_speed_to_conflict_overlay_transition"
            )
            method = methods["circle_contact_{}".format(suffix)]
            base = [
                "{ROSLAUNCH_EXECUTABLE}",
                "m2_gazebo",
                LAUNCH_RELATIVE.name,
                "world:={}".format(row["world"]["path"]),
                "seed:={}".format(row["seed"]),
                "xacro_executable:={XACRO_EXECUTABLE}",
                "attempt_stage:=V2-04G-P2-Q1",
                "attempt_profile_id:=qualification_{}_{}".format(
                    row["probe_type"], suffix
                ),
                "attempt_scene_id:={}".format(row["scene_id"]),
                "attempt_number:=1",
                "fixed_teb_config:={}".format(
                    method["fixed_startup_config"]["path"]
                ),
                "start_move_base:={}".format("true" if semantic else "false"),
                "start_world_model:={}".format("true" if semantic else "false"),
                "start_rule_supervisor:={}".format("true" if semantic else "false"),
                "start_typed_transaction:=false",
                "allow_simulation_teb_parameter_write:=false",
                "allow_unfrozen_simulation_candidate:=true",
                "gui:=false",
                "paused:=true",
            ]
            transaction = None
            if semantic:
                transaction = [
                    "{ROSLAUNCH_EXECUTABLE}",
                    "teb_mode_manager",
                    "v2_04g_r6_simulation_typed_anchor.launch",
                    "allow_simulation_teb_parameter_write:=true",
                    "allow_unfrozen_simulation_candidate:=true",
                    "anchor_bank:={}".format(
                        method["materialized_anchor_bank"]["path"]
                    ),
                    "mechanism_config:={}".format(method["mechanism"]["path"]),
                    "attempt_stage:=V2-04G-P2-Q1",
                    "attempt_profile_id:=qualification_{}_{}".format(
                        row["probe_type"], suffix
                    ),
                    "attempt_scene_id:={}".format(row["scene_id"]),
                    "attempt_seed:={}".format(row["seed"]),
                    "attempt_number:=1",
                    "supervisor_config_sha256:={}".format(
                        method["supervisor"]["sha256"]
                    ),
                ]
            probe = [
                "{PYTHON_EXECUTABLE}",
                QUALIFICATION_RELATIVE.as_posix(),
                "--probe",
                row["probe_type"],
                "--candidate-speed-mps",
                str(row["candidate_speed_mps"]),
                "--sequence",
                str(row["sequence"]),
                "--scene-id",
                row["scene_id"],
                "--seed",
                str(row["seed"]),
                "--output",
                "{FUTURE_ATTEMPT_ROOT}/raw_trace.yaml",
            ]
            plan_rows.append(
                {
                    "sequence": row["sequence"],
                    "identity": {
                        key: row[key]
                        for key in (
                            "row_type",
                            "probe_type",
                            "candidate_speed_mps",
                            "scene_id",
                            "seed",
                            "attempt",
                        )
                    },
                    "base_launch_argv": base,
                    "semantic_transaction_launch_argv": transaction,
                    "activation_listener_required": semantic,
                    "runtime_control_required": semantic,
                    "fixed_readback_capture_verify_required": False,
                    "probe_argv": probe,
                }
            )
    _require(len(plan_rows) == 342, "command plan row count differs")
    return {
        "schema_version": "1.0",
        "stage": "V2-04G-P2-Q1",
        "status": "offline_command_plan_frozen_not_executable",
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "placeholder_contract": {
            "{ROSLAUNCH_EXECUTABLE}": "future_release_bound_absolute_executable",
            "{XACRO_EXECUTABLE}": "future_release_bound_absolute_executable",
            "{PYTHON_EXECUTABLE}": "future_release_bound_absolute_executable",
            "{FUTURE_ATTEMPT_ROOT}": "future_nonexistent_identity_specific_attempt_root",
        },
        "bootstrap_sequence": [
            "credential_safe_child_environment",
            "spawn_base_roslaunch_and_commit_consumption",
            "unpause_gazebo",
            "observe_strictly_positive_clock_progress",
            "wait_move_base_or_direct_probe_readiness",
            "start_and_arm_semantic_transaction_if_required",
            "run_raw_trace_adapter",
            "two_phase_restore_if_required",
            "terminate_process_group_and_verify_isolation",
            "write_raw_receipts_and_terminal_journal",
        ],
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "rows": plan_rows,
        "canonical_plan_digest": _canonical_sha(plan_rows),
    }


def _source_freeze(
    contract: Mapping[str, Any],
) -> Tuple[bytes, List[Dict[str, str]]]:
    paths = {
        CONTRACT_RELATIVE,
        BUILDER_RELATIVE,
        RUNNER_RELATIVE,
        NAVIGATION_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
        FIXED_SNAPSHOT_RELATIVE,
        REPLAY_RELATIVE,
        LAUNCH_RELATIVE,
        P1_EPISODE_RELATIVE,
    }
    paths.update(
        Path(binding["path"]) for binding in contract["source_bindings"].values()
    )
    paths.update(
        Path(binding["path"])
        for binding in contract["historical_freeze_bindings"].values()
    )
    rows = []
    lines = []
    for relative in sorted(paths, key=lambda item: item.as_posix()):
        path = WORKSPACE / relative
        _require(path.is_file(), "source freeze input absent: {}".format(relative))
        digest = _sha256_file(path)
        rows.append({"path": relative.as_posix(), "sha256": digest})
        lines.append("{}  {}\n".format(digest, relative.as_posix()))
    return "".join(lines).encode("utf-8"), rows


def _validate_contract(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == "V2-04G-P2-Q1", "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline-only")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    _require(contract.get("evidence_budget_authorized") == 0, "contract evidence authority differs")
    _require(
        contract.get("training_budget_steps_authorized") == 0,
        "contract training authority differs",
    )
    _require(
        contract["candidate_materialization"]["speed_grid_mps"]
        == [1.2, 1.25, 1.3, 1.35, 1.4],
        "speed grid differs",
    )
    _require(
        contract["exact_schedule"]["total_row_count"] == 342,
        "contract schedule size differs",
    )
    _require(
        contract["future_calibration_release_boundary"]["release_present"] is False
        and contract["future_calibration_release_boundary"]["current_authorized_budget"] == 0,
        "future release boundary differs",
    )
    _require(
        all(contract["forbidden_side_effects"].values()),
        "contract forbidden side-effect boundary differs",
    )
    for label, binding in contract["source_bindings"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "source binding absent: {}".format(label))
        _require(
            _sha256_file(path) == binding["sha256"],
            "source binding drifted: {}".format(label),
        )
    for label, binding in contract["historical_freeze_bindings"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "historical freeze absent: {}".format(label))
        _require(
            _sha256_file(path) == binding["sha256"],
            "historical freeze drifted: {}".format(label),
        )


def build_bundle(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _validate_contract(contract)
    files, method_bindings, method_diff_audit = _build_methods(contract)
    files[METHOD_BINDINGS_RELATIVE] = _yaml_bytes(method_bindings)
    manifest, scene_equivalence, seed_to_id = _build_scenes(contract)
    files[SCENES_RELATIVE] = _yaml_bytes(manifest)
    compiled_files, scene_bindings, compiled_index = _build_compiled(manifest)
    files.update(compiled_files)
    schedule, schedule_audit = _build_schedule(
        contract, scene_bindings, seed_to_id
    )
    files[SCHEDULE_RELATIVE] = _yaml_bytes(schedule)
    command_plan = _build_command_plan(schedule, method_bindings)
    files[COMMAND_PLAN_RELATIVE] = _yaml_bytes(command_plan)
    source_freeze, source_rows = _source_freeze(contract)
    files[SOURCE_FREEZE_RELATIVE] = source_freeze
    construction_audit = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-Q1",
        "status": "offline_construction_audit_pass",
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "candidate_diff_audit": method_diff_audit,
        "scene_template_equivalence": scene_equivalence,
        "scene_counts": {
            "total": 48,
            "calibration": 30,
            "qualification": 12,
            "compile_support": 6,
            "compiled_children": compiled_index["compiled_child_count"],
        },
        "seed_firewall": {
            "calibration": [5301, 5330],
            "qualification": [5331, 5342],
            "compile_support": [5343, 5348],
            "held_out_test_forbidden": [5401, 5490],
            "held_out_test_materialized": False,
        },
        "schedule_audit": schedule_audit,
        "source_freeze_count": len(source_rows),
        "active_runtime_profile_modified": False,
        "historical_frozen_evidence_modified": False,
    }
    files[AUDIT_RELATIVE] = _yaml_bytes(construction_audit)
    failure_index = {
        "schema_version": "1.0",
        "stage": "V2-04G-P2-Q1",
        "status": "no_execution_no_failures_observed",
        "execution_authorized": False,
        "evidence_units_consumed": 0,
        "failures": [],
        "future_every_failure_must_be_retained": True,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
    }
    files[FAILURE_INDEX_RELATIVE] = _yaml_bytes(failure_index)
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-Q1",
        "status": "p2_q1_offline_implementation_readiness_pass_release_absent",
        "offline_implementation_review_ready": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "planned_future_evidence_units": 342,
        "authorized_evidence_units": 0,
        "consumed_evidence_units": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "contract": {
            "path": CONTRACT_RELATIVE.as_posix(),
            "sha256": _sha256_file(workspace / CONTRACT_RELATIVE),
        },
        "materialization": {
            "method_count": 11,
            "circle_single_factor_candidates": 5,
            "independent_fixed_grid_candidates": 5,
            "frozen_engineering_baselines": 1,
            "scene_count": 48,
            "compiled_child_count": 96,
            "schedule_rows": 342,
            "command_plan_rows": 342,
            "sole_p2_behavioral_field": "TebLocalPlannerROS.max_vel_x",
        },
        "fairness": method_bindings["fair_baseline_rule"],
        "artifacts": {
            "method_bindings": _receipt(
                METHOD_BINDINGS_RELATIVE, files[METHOD_BINDINGS_RELATIVE]
            ),
            "scenes": _receipt(SCENES_RELATIVE, files[SCENES_RELATIVE]),
            "compiled_index": _receipt(
                COMPILED_INDEX_RELATIVE, files[COMPILED_INDEX_RELATIVE]
            ),
            "exact_schedule": _receipt(
                SCHEDULE_RELATIVE, files[SCHEDULE_RELATIVE]
            ),
            "exact_command_plan": _receipt(
                COMMAND_PLAN_RELATIVE, files[COMMAND_PLAN_RELATIVE]
            ),
            "construction_audit": _receipt(
                AUDIT_RELATIVE, files[AUDIT_RELATIVE]
            ),
            "failure_index": _receipt(
                FAILURE_INDEX_RELATIVE, files[FAILURE_INDEX_RELATIVE]
            ),
            "source_freeze": _receipt(
                SOURCE_FREEZE_RELATIVE, files[SOURCE_FREEZE_RELATIVE]
            ),
        },
        "execution_gap": {
            "exact_release_present": False,
            "corresponding_simulation_authorization_present": False,
            "authorized_budget": 0,
            "transitive_runtime_dependency_closure_frozen": False,
            "future_executable_and_environment_bindings_frozen": False,
            "runner_execute_entry_hard_disabled": True,
            "held_out_test_open": False,
            "future_release_must_close_all_items": True,
        },
        "claim_state": {
            "candidate_safety_qualified": False,
            "calibration_result_exists": False,
            "primary_baseline_selected": False,
            "performance_improvement_demonstrated": False,
            "held_out_test_accessed": False,
        },
        "next_safe_entry": (
            "independent_P2_Q1_offline_review_then_new_exact_calibration_release_"
            "and_corresponding_separate_simulation_authorization"
        ),
    }
    files[REPORT_RELATIVE] = _yaml_bytes(report)
    return files


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P2-Q1 offline output root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=".p2_q1_offline.", dir=str(target.parent))
    )
    try:
        for relative, payload in sorted(
            files.items(), key=lambda item: item[0].as_posix()
        ):
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
        _require(path.is_file(), "persisted output absent: {}".format(relative))
        _require(
            path.read_bytes() == expected,
            "persisted output differs: {}".format(relative),
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    files = build_bundle(workspace)
    if args.materialize:
        _materialize(files, workspace)
    else:
        _check_only(files, workspace)
    print(
        "P2-Q1 offline bundle {}: {} files".format(
            "materialized" if args.materialize else "verified", len(files)
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
