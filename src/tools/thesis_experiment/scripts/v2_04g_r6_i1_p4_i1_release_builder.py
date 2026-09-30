#!/usr/bin/env python3
"""Build the one-unit P4-I1 semantic preflight release bundle offline."""

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
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P4-I1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p4_i1_semantic_preflight_execution_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p4_i1_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p4_i1_execution_release.yaml"
)
INHERITED_CLOSURE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_release_readiness/"
    "p2_c1_r1_execution_dependency_closure.yaml"
)
P4_DESIGN_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_d1_final_round_design"
)
D1_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p3_i1_d1_failure_attribution"
)
P4_Q1_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p4_q1_runtime_bank_readiness"
)
SOURCE_SCENES_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_scenes.yaml"
)
METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_q1_runtime_bank_readiness/"
    "v2_04g_p4_q1_method_bindings.yaml"
)
Q1_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_i1_release_readiness"
)
SCENES_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_i1_scene.yaml"
COMPILED_ROOT = OUTPUT_ROOT / "compiled_scene"
INSTANCE_RELATIVE = (
    COMPILED_ROOT / "v2-04g-p4-i1-qual-overlay-v125-s5924.instance.yaml"
)
WORLD_RELATIVE = (
    COMPILED_ROOT / "v2-04g-p4-i1-qual-overlay-v125-s5924.world"
)
COMPILED_INDEX_RELATIVE = COMPILED_ROOT / "compiled_scene_index.yaml"
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_i1_exact_one_schedule.yaml"
PLAN_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_i1_exact_one_command_plan.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p4_i1_execution_dependency_closure.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_i1_release_readiness_report.yaml"
FAILURE_INDEX_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_i1_preexecution_failure_index.yaml"

LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_release_builder.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_offline_reviewer.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_exact_runner.py"
)
RELEASE_MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p4_i1_release.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_activation_probe_listener.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_i1_assess.py"
)
Q1_REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
FROZEN_C1_RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
V2_SCENE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/v2_scene.py"
)


class P4I1BuildError(ValueError):
    """Raised when the one-unit preflight bundle differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4I1BuildError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_sha(value: Any) -> str:
    return _sha(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    )


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs")
    return value


def _receipt(relative: Path, payload: bytes) -> Dict[str, Any]:
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _load_v2_scene():
    path = WORKSPACE / V2_SCENE_RELATIVE
    spec = importlib.util.spec_from_file_location("_p4_i1_v2_scene", path)
    _require(spec is not None and spec.loader is not None, "cannot load scene compiler")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


V2_SCENE = _load_v2_scene()


def _verify_inputs(
    contract: Mapping[str, Any], authorization: Mapping[str, Any]
) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(
        contract.get("execution_authorized") is True
        and contract.get("authorized_evidence_units_max") == 1,
        "contract authority differs",
    )
    _require(contract.get("training_budget_steps") == 0, "training budget differs")
    for binding in contract["frozen_sources"].values():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "frozen source absent")
        _require(_sha(path.read_bytes()) == binding["sha256"], "frozen source drifted")
    _require(
        authorization.get("stage") == STAGE
        and authorization.get("explicit_user_authorization_recorded") is True
        and authorization.get("execution_authorized") is True
        and authorization.get("authorized_evidence_units_max") == 1,
        "authorization differs",
    )
    _require(not (WORKSPACE / RELEASE_RELATIVE).exists(), "release already exists")


def _build_scene_files():
    source = _load_yaml(WORKSPACE / SOURCE_SCENES_RELATIVE)
    V2_SCENE.validate_v2_scene_manifest(source, WORKSPACE)
    template = next(scene for scene in source["scenes"] if scene["seed"] == 5332)
    _require(
        template["evaluator_only"]["reason"].startswith(
            "p2_q1_evaluator_only_qualification_high_speed"
        ),
        "source overlay template differs",
    )
    scene = copy.deepcopy(template)
    scene["scene_id"] = "v2-04g-p4-i1-qual-overlay-v125-s5924"
    scene["seed"] = 5924
    scene["split"] = "calibration"
    scene["evaluator_only"]["reason"] = (
        "p4_i1_fresh_semantic_overlay_identity_alignment_preflight"
    )
    manifest = copy.deepcopy(source)
    manifest["manifest_id"] = "fam_teb_v2_04g_p4_i1_fresh_scene_1"
    manifest["scenes"] = [scene]
    instance = V2_SCENE.compile_v2_scene(scene, manifest["generator"])
    _require(
        instance["scene"]["scene_id"] == scene["scene_id"]
        and instance["scene"]["seed"] == 5924
        and instance["scene"]["timeout_s"] == 80.0,
        "compiled scene identity differs",
    )
    scene_payload = _yaml_bytes(manifest)
    instance_payload = _yaml_bytes(instance)
    world_payload = V2_SCENE.render_v2_scene_sdf(instance).encode("utf-8")
    index = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_compiled_preflight_scene_not_yet_released",
        "scene_count": 1,
        "compiled_child_count": 2,
        "files": [
            _receipt(INSTANCE_RELATIVE, instance_payload),
            _receipt(WORLD_RELATIVE, world_payload),
        ],
    }
    return {
        SCENES_RELATIVE: scene_payload,
        INSTANCE_RELATIVE: instance_payload,
        WORLD_RELATIVE: world_payload,
        COMPILED_INDEX_RELATIVE: _yaml_bytes(index),
    }, instance


def _build_schedule(
    scene_files: Mapping[Path, bytes], instance: Mapping[str, Any]
) -> Dict[str, Any]:
    row = {
        "sequence": 1,
        "row_type": "qualification_probe",
        "qualification_block_id": "p4-i1-qual-v125",
        "probe_type": "high_speed_to_conflict_overlay_transition",
        "candidate_speed_mps": 1.25,
        "candidate_method_id": "circle_contact_v125",
        "paired_fixed_method_id": "fixed_teb_v125",
        "seed": 5924,
        "scene_id": instance["scene"]["scene_id"],
        "instance": _receipt(INSTANCE_RELATIVE, scene_files[INSTANCE_RELATIVE]),
        "world": _receipt(WORLD_RELATIVE, scene_files[WORLD_RELATIVE]),
        "timeout_s": float(instance["scene"]["timeout_s"]),
        "attempt": 1,
        "source_P2_qualification_role": "speed_1_25_overlay",
        "stage": STAGE,
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "exact_one_unit_preflight_schedule_not_yet_released",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "global_round_3_maximum_units": 704,
        "future_units_not_authorized": 703,
        "held_out_test_rows": 0,
        "attempt_per_identity": 1,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "rows": [row],
        "canonical_row_digest": _canonical_sha([row]),
    }


def _build_plan(
    schedule: Mapping[str, Any],
    instance: Mapping[str, Any],
    methods_document: Mapping[str, Any],
) -> Dict[str, Any]:
    row = schedule["rows"][0]
    method = next(
        item
        for item in methods_document["methods"]
        if item["method_id"] == "circle_contact_v125"
    )
    runtime_profile = "r6_semantics_circle_contact"
    scene = instance["scene"]
    base = [
        "{ROSLAUNCH_EXECUTABLE}",
        "m2_gazebo",
        LAUNCH_RELATIVE.name,
        "world:={FUTURE_SCENE_SNAPSHOT_WORLD}",
        "seed:=5924",
        "xacro_executable:={XACRO_EXECUTABLE}",
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false",
        "paused:=true",
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(runtime_profile),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_number:=1",
        "fixed_teb_config:={}".format(method["fixed_startup_config"]["path"]),
        "start_move_base:=true",
        "start_world_model:=true",
        "start_rule_supervisor:=true",
        "start_typed_transaction:=false",
        "allow_simulation_teb_parameter_write:=false",
        "allow_unfrozen_simulation_candidate:=true",
        "load_balanced_anchor:=false",
        "publish_teb_obstacles:=true",
        "rule_supervisor_config:={}".format(method["supervisor"]["path"]),
        "rule_supervisor_config_sha256:={}".format(method["supervisor"]["sha256"]),
        "anchor_bank:={}".format(method["materialized_anchor_bank"]["path"]),
        "mechanism_config:={}".format(method["mechanism"]["path"]),
    ]
    transaction = [
        "{ROSLAUNCH_EXECUTABLE}",
        "teb_mode_manager",
        "v2_04g_r6_simulation_typed_anchor.launch",
        "allow_simulation_teb_parameter_write:=true",
        "allow_unfrozen_simulation_candidate:=true",
        "anchor_bank:={}".format(method["materialized_anchor_bank"]["path"]),
        "mechanism_config:={}".format(method["mechanism"]["path"]),
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(runtime_profile),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_seed:=5924",
        "attempt_number:=1",
        "supervisor_config_sha256:={}".format(method["supervisor"]["sha256"]),
    ]
    probe = [
        "{PYTHON_EXECUTABLE}",
        QUALIFICATION_RELATIVE.as_posix(),
        "--probe",
        row["probe_type"],
        "--candidate-speed-mps",
        "1.25",
        "--sequence",
        "1",
        "--scene-id",
        row["scene_id"],
        "--seed",
        "5924",
        "--output",
        "{FUTURE_ATTEMPT_ROOT}/raw_trace.yaml",
    ]
    planned = {
        "sequence": 1,
        "evidence_identity": {
            "sequence": 1,
            "row_type": "qualification_probe",
            "scene_id": row["scene_id"],
            "seed": 5924,
            "attempt": 1,
            "probe_type": row["probe_type"],
            "candidate_speed_mps": 1.25,
        },
        "runtime_profile_id": runtime_profile,
        "evidence_candidate_method_id": "circle_contact_v125",
        "evidence_qualification_profile_id": "qualification_overlay_v125",
        "scene_execution_binding": {
            "source_instance": copy.deepcopy(row["instance"]),
            "source_world": copy.deepcopy(row["world"]),
            "snapshot_instance": "{FUTURE_SCENE_SNAPSHOT_INSTANCE}",
            "snapshot_world": "{FUTURE_SCENE_SNAPSHOT_WORLD}",
            "start": copy.deepcopy(scene["start"]),
            "timeout_s": float(scene["timeout_s"]),
        },
        "base_launch_argv": base,
        "semantic_transaction_launch_argv": transaction,
        "probe_argv": probe,
        "runtime_control_adapter": CONTROL_RELATIVE.as_posix(),
        "activation_listener_adapter": LISTENER_RELATIVE.as_posix(),
        "per_identity_hard_wall_s": 240,
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "exact_one_unit_resolved_command_plan_not_yet_released",
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "single_factor_id": (
            "promote_offline_candidate_bank_to_strict_runtime_candidate_schema_v1"
        ),
        "rows": [planned],
        "canonical_plan_digest": _canonical_sha([planned]),
    }


def _add_resource(
    resources: Dict[str, Dict[str, Any]], relative: Path, payload: bytes
) -> None:
    receipt = _receipt(relative, payload)
    key = relative.as_posix()
    if key in resources:
        _require(resources[key] == receipt, "duplicate dependency differs")
    else:
        resources[key] = receipt


def _build_closure(generated: Mapping[Path, bytes]) -> Dict[str, Any]:
    inherited = _load_yaml(WORKSPACE / INHERITED_CLOSURE_RELATIVE)
    _require(inherited.get("unresolved") == [], "inherited closure unresolved")
    resources: Dict[str, Dict[str, Any]] = {}
    for receipt in inherited["local_resources"]:
        relative = Path(receipt["path"])
        payload = (WORKSPACE / relative).read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "inherited dependency drifted",
        )
        _add_resource(resources, relative, payload)
    for root in (P4_DESIGN_ROOT, D1_ROOT, P4_Q1_ROOT):
        for path in sorted((WORKSPACE / root).rglob("*")):
            if path.is_file():
                _add_resource(resources, path.relative_to(WORKSPACE), path.read_bytes())
    explicit = {
        CONTRACT_RELATIVE,
        AUTHORIZATION_RELATIVE,
        INHERITED_CLOSURE_RELATIVE,
        SOURCE_SCENES_RELATIVE,
        METHODS_RELATIVE,
        Q1_CONTRACT_RELATIVE,
        LAUNCH_RELATIVE,
        BUILDER_RELATIVE,
        REVIEWER_RELATIVE,
        RUNNER_RELATIVE,
        RELEASE_MODULE_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
        ASSESSOR_RELATIVE,
        Q1_REPLAY_RELATIVE,
        FROZEN_C1_RUNNER_RELATIVE,
        V2_SCENE_RELATIVE,
    }
    for relative in sorted(explicit, key=lambda value: value.as_posix()):
        path = WORKSPACE / relative
        _require(path.is_file(), "P4-I1 dependency absent: {}".format(relative))
        _add_resource(resources, relative, path.read_bytes())
    for relative, payload in generated.items():
        _add_resource(resources, relative, payload)
    external = []
    for receipt in inherited["external_resources"]:
        path = Path(receipt["path"])
        payload = path.read_bytes()
        _require(
            path.is_absolute()
            and _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external dependency differs",
        )
        external.append(dict(receipt))
    document = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "one_unit_execution_dependency_closure_release_absent",
        "simulation_only": True,
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "evidence_units_consumed": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "inherited_closure": {
            "path": INHERITED_CLOSURE_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / INHERITED_CLOSURE_RELATIVE).read_bytes()),
        },
        "local_resources": [resources[key] for key in sorted(resources)],
        "local_resource_count": len(resources),
        "external_resources": external,
        "external_resource_count": len(external),
        "schedule_row_count": 1,
        "held_out_test_resource_or_seed_added": False,
        "unresolved": [],
    }
    logical = dict(document)
    document["logical_sha256"] = _canonical_sha(logical)
    return document


def build_bundle(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    authorization = _load_yaml(workspace / AUTHORIZATION_RELATIVE)
    _verify_inputs(contract, authorization)
    scene_files, instance = _build_scene_files()
    schedule = _build_schedule(scene_files, instance)
    methods = _load_yaml(workspace / METHODS_RELATIVE)
    plan = _build_plan(schedule, instance, methods)
    generated = dict(scene_files)
    generated[SCHEDULE_RELATIVE] = _yaml_bytes(schedule)
    generated[PLAN_RELATIVE] = _yaml_bytes(plan)
    closure = _build_closure(generated)
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "one_unit_release_readiness_built_review_pending_release_absent",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "fresh_seed": 5924,
        "runtime_profile_id": "r6_semantics_circle_contact",
        "evidence_candidate_method_id": "circle_contact_v125",
        "behavioral_TEB_parameter_change": False,
        "local_dependencies": closure["local_resource_count"],
        "external_dependencies": closure["external_resource_count"],
        "held_out_test_accessed": False,
        "ROS_or_Gazebo_started": False,
        "next_safe_entry": "independent_release_absent_review",
    }
    failure_index = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "preexecution_no_failures",
        "release_present": False,
        "authorized_evidence_units_max": 1,
        "consumed_evidence_units": 0,
        "failures": [],
        "retry_resume_replacement_or_seed_substitution_allowed": False,
    }
    generated[CLOSURE_RELATIVE] = _yaml_bytes(closure)
    generated[REPORT_RELATIVE] = _yaml_bytes(report)
    generated[FAILURE_INDEX_RELATIVE] = _yaml_bytes(failure_index)
    return generated


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P4-I1 readiness root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=target.name + ".tmp.", dir=target.parent))
    try:
        for relative, payload in sorted(files.items(), key=lambda item: item[0].as_posix()):
            local = temporary / relative.relative_to(OUTPUT_ROOT)
            local.parent.mkdir(parents=True, exist_ok=True)
            with local.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        os.rename(str(temporary), str(target))
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def _check_only(files: Mapping[Path, bytes], workspace: Path) -> None:
    for relative, expected in files.items():
        path = workspace / relative
        _require(path.is_file() and path.read_bytes() == expected, "P4-I1 bytes differ")


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
        "P4-I1 release-readiness {}: {} files".format(
            "materialized" if args.materialize else "verified", len(files)
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
