#!/usr/bin/env python3
"""Build the P2-C1-R1 release-readiness bundle without starting ROS.

R1 preserves the terminally unused C1 evidence identities and repairs one
eight-row qualification binding class: stale schedule timeouts are aligned to
the frozen compiled scenes, and direct circle probes resolve the frozen
fixed-startup config key instead of a fixed-method-only key.  It also
materializes the command surface actually resolved by the runner so an
independent reviewer can compare all 342 rows before a release exists.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1-R1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_r1_exact_calibration_execution_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_r1_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_r1_execution_release.yaml"
)
OLD_RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_execution_release.yaml"
)
OLD_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_release_readiness"
)
OLD_SCHEDULE_RELATIVE = OLD_ROOT / "v2_04g_p2_c1_exact_342_schedule.yaml"
OLD_PLAN_RELATIVE = OLD_ROOT / "v2_04g_p2_c1_exact_342_command_plan.yaml"
OLD_CLOSURE_RELATIVE = OLD_ROOT / "p2_c1_execution_dependency_closure.yaml"
OLD_AUDIT_RELATIVE = OLD_ROOT / "v2_04g_p2_c1_postrelease_preexecution_audit.yaml"
OLD_EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_release_readiness"
)
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_r1_exact_342_schedule.yaml"
PLAN_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_r1_exact_342_command_plan.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p2_c1_r1_execution_dependency_closure.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_r1_release_readiness_report.yaml"
FAILURE_INDEX_RELATIVE = (
    OUTPUT_ROOT / "v2_04g_p2_c1_r1_preexecution_failure_index.yaml"
)

METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)
Q1_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
D2_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d2_offline_single_factor_design_contract.yaml"
)
LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_release_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_exact_runner.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_offline_reviewer.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_assess.py"
)
RELEASE_MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_c1_r1_release.py"
)
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
NAVIGATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_navigation_episode.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_activation_probe_listener.py"
)
FIXED_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_fixed_snapshot.py"
)
FROZEN_C1_RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
FROZEN_C1_ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_assess.py"
)

EXPECTED_TIMEOUT_REPAIRS = {
    1: (80.0, 55.0),
    3: (80.0, 55.0),
    4: (80.0, 55.0),
    6: (80.0, 55.0),
    7: (80.0, 55.0),
    9: (80.0, 55.0),
    10: (80.0, 55.0),
    12: (80.0, 55.0),
}
CIRCLE_METHODS = {
    "circle_contact_v120",
    "circle_contact_v125",
    "circle_contact_v130",
    "circle_contact_v135",
    "circle_contact_v140",
}


class C1R1BuildError(ValueError):
    """Raised when the frozen R1 bundle differs from its contract."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1R1BuildError(message)


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
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _receipt(relative: Path, payload: bytes) -> Dict[str, Any]:
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _verify_binding(binding: Mapping[str, Any], label: str) -> bytes:
    path = WORKSPACE / binding["path"]
    _require(path.is_file(), "{} absent".format(label))
    payload = path.read_bytes()
    _require(_sha(payload) == binding["sha256"], "{} SHA differs".format(label))
    return payload


def _validate_inputs(
    contract: Mapping[str, Any], authorization: Mapping[str, Any]
) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("execution_authorized") is True, "contract lacks authority")
    _require(
        contract.get("authorized_evidence_units_max") == 342,
        "contract budget differs",
    )
    _require(contract.get("training_budget_steps") == 0, "training budget differs")
    _require(contract.get("real_vehicle_use_forbidden") is True, "real vehicle allowed")
    _require(
        contract["single_integrity_amendment"]["affected_sequences"]
        == list(EXPECTED_TIMEOUT_REPAIRS),
        "timeout repair roster differs",
    )
    for label, binding in contract["predecessor_terminal_freeze"].items():
        if type(binding) is dict and set(binding) == {"path", "sha256"}:
            _verify_binding(binding, label)
    audit = _load_yaml(WORKSPACE / OLD_AUDIT_RELATIVE)
    _require(
        audit.get("status") == "release_terminally_unused_preexecution_binding_mismatch"
        and audit.get("execute_called") is False
        and audit.get("consumed_evidence_units") == 0
        and audit.get("forfeited_authority_units") == 342,
        "old C1 terminal audit differs",
    )
    _require(not (WORKSPACE / OLD_EXECUTION_ROOT).exists(), "old C1 execution state exists")
    _require(authorization.get("stage") == STAGE, "authorization stage differs")
    _require(
        authorization.get("explicit_user_authorization_recorded") is True
        and authorization.get("execution_authorized") is True,
        "explicit authorization absent",
    )
    _require(
        authorization.get("authorized_evidence_units_max") == 342,
        "authorization budget differs",
    )
    _require(
        authorization.get("retry_resume_replacement_or_seed_substitution_allowed")
        is False,
        "authorization permits identity changes",
    )
    _require(not (WORKSPACE / RELEASE_RELATIVE).exists(), "R1 release already exists")


def _scene_for_row(row: Mapping[str, Any]) -> Mapping[str, Any]:
    instance_path = WORKSPACE / row["instance"]["path"]
    world_path = WORKSPACE / row["world"]["path"]
    instance_payload = instance_path.read_bytes()
    world_payload = world_path.read_bytes()
    _require(
        _sha(instance_payload) == row["instance"]["sha256"]
        and len(instance_payload) == row["instance"]["size_bytes"],
        "instance binding differs at sequence {}".format(row["sequence"]),
    )
    _require(
        _sha(world_payload) == row["world"]["sha256"]
        and len(world_payload) == row["world"]["size_bytes"],
        "world binding differs at sequence {}".format(row["sequence"]),
    )
    instance = yaml.safe_load(instance_payload)
    _require(type(instance) is dict, "instance root differs")
    scene = instance.get("scene", {})
    _require(
        scene.get("scene_id") == row["scene_id"]
        and scene.get("seed") == row["seed"]
        and scene.get("split") == "calibration",
        "scene identity differs at sequence {}".format(row["sequence"]),
    )
    _require(
        type(scene.get("start")) is dict
        and set(scene["start"]) == {"x_m", "y_m", "yaw_rad"},
        "scene start differs at sequence {}".format(row["sequence"]),
    )
    return scene


def _build_schedule(old: Mapping[str, Any]) -> Dict[str, Any]:
    _require(old.get("stage") == "V2-04G-P2-C1", "old schedule stage differs")
    _require(len(old.get("rows", [])) == 342, "old schedule row count differs")
    rows = copy.deepcopy(old["rows"])
    repaired = {}
    for row in rows:
        scene = _scene_for_row(row)
        old_timeout = float(row["timeout_s"])
        compiled_timeout = float(scene["timeout_s"])
        if old_timeout != compiled_timeout:
            repaired[row["sequence"]] = (old_timeout, compiled_timeout)
            row["timeout_s"] = compiled_timeout
        row["stage"] = STAGE
    _require(repaired == EXPECTED_TIMEOUT_REPAIRS, "exact timeout repair set differs")
    _require(
        [row["sequence"] for row in rows] == list(range(1, 343)),
        "schedule sequence differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in rows},
        "held-out seed leaked",
    )
    result = copy.deepcopy(old)
    result.update(
        {
            "stage": STAGE,
            "status": "exact_repaired_release_candidate_schedule_not_yet_released",
            "schedule_id": "fam_teb_v2_04g_p2_c1_r1_timeout_bound_exact_342_1",
            "source_c1_schedule": {
                "path": OLD_SCHEDULE_RELATIVE.as_posix(),
                "sha256": _sha((WORKSPACE / OLD_SCHEDULE_RELATIVE).read_bytes()),
                "canonical_row_digest": old["canonical_row_digest"],
            },
            "repair_class": "single_execution_integrity_timeout_binding_only",
            "behavioral_factor_count_changed": 0,
            "timeout_binding_repair": {
                "affected_sequences": list(EXPECTED_TIMEOUT_REPAIRS),
                "old_timeout_s": 80.0,
                "compiled_and_repaired_timeout_s": 55.0,
                "all_342_rows_equal_compiled_instance_timeout": True,
            },
            "rows": rows,
            "canonical_row_digest": _canonical_sha(rows),
        }
    )
    return result


def _profile_id(row: Mapping[str, Any]) -> str:
    if row["row_type"] == "calibration_navigation":
        return row["method_id"]
    return "qualification_{}_v{:03d}".format(
        row["probe_type"].replace(
            "high_speed_to_conflict_overlay_transition", "overlay"
        ),
        int(round(float(row["candidate_speed_mps"]) * 100.0)),
    )


def _method_for_row(
    row: Mapping[str, Any], methods: Mapping[str, Mapping[str, Any]]
) -> Mapping[str, Any]:
    method_id = (
        row["method_id"]
        if row["row_type"] == "calibration_navigation"
        else row["candidate_method_id"]
    )
    _require(method_id in methods, "method binding absent")
    return methods[method_id]


def _semantic(row: Mapping[str, Any]) -> bool:
    return (
        row["row_type"] == "calibration_navigation"
        and row["method_id"] in CIRCLE_METHODS
    ) or (
        row["row_type"] == "qualification_probe"
        and row["probe_type"] == "high_speed_to_conflict_overlay_transition"
    )


def _identity(row: Mapping[str, Any]) -> Dict[str, Any]:
    common = {
        "sequence": row["sequence"],
        "row_type": row["row_type"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }
    if row["row_type"] == "calibration_navigation":
        common.update(
            {
                "method_id": row["method_id"],
                "block_id": row["block_id"],
                "scene_role": row["scene_role"],
            }
        )
    else:
        common.update(
            {
                "probe_type": row["probe_type"],
                "candidate_speed_mps": row["candidate_speed_mps"],
            }
        )
    return common


def _build_plan(
    old_plan: Mapping[str, Any],
    schedule: Mapping[str, Any],
    method_document: Mapping[str, Any],
) -> Dict[str, Any]:
    _require(old_plan.get("stage") == "V2-04G-P2-C1", "old plan stage differs")
    _require(len(old_plan.get("rows", [])) == 342, "old plan row count differs")
    methods = {row["method_id"]: row for row in method_document["methods"]}
    output = []
    for row in schedule["rows"]:
        scene = _scene_for_row(row)
        method = _method_for_row(row, methods)
        semantic = _semantic(row)
        start_move_base = (
            row["row_type"] == "calibration_navigation"
            or row.get("probe_type")
            == "high_speed_to_conflict_overlay_transition"
        )
        fixed_binding = (
            method["fixed_startup_config"]
            if "fixed_startup_config" in method
            else method["materialized_config"]
        )
        profile = _profile_id(row)
        base = [
            "{ROSLAUNCH_EXECUTABLE}",
            "m2_gazebo",
            LAUNCH_RELATIVE.name,
            "world:={FUTURE_SCENE_SNAPSHOT_WORLD}",
            "seed:={}".format(row["seed"]),
            "xacro_executable:={XACRO_EXECUTABLE}",
            "x:={}".format(scene["start"]["x_m"]),
            "y:={}".format(scene["start"]["y_m"]),
            "yaw:={}".format(scene["start"]["yaw_rad"]),
            "gui:=false",
            "paused:=true",
            "attempt_stage:={}".format(STAGE),
            "attempt_profile_id:={}".format(profile),
            "attempt_scene_id:={}".format(row["scene_id"]),
            "attempt_number:=1",
            "fixed_teb_config:={}".format(fixed_binding["path"]),
            "start_move_base:={}".format("true" if start_move_base else "false"),
            "start_world_model:={}".format("true" if start_move_base else "false"),
            "start_rule_supervisor:={}".format("true" if semantic else "false"),
            "start_typed_transaction:=false",
            "allow_simulation_teb_parameter_write:=false",
            "allow_unfrozen_simulation_candidate:=true",
            "load_balanced_anchor:=false",
            "publish_teb_obstacles:={}".format("true" if semantic else "false"),
        ]
        transaction = None
        if semantic:
            base.extend(
                [
                    "rule_supervisor_config:={}".format(
                        method["supervisor"]["path"]
                    ),
                    "rule_supervisor_config_sha256:={}".format(
                        method["supervisor"]["sha256"]
                    ),
                    "anchor_bank:={}".format(
                        method["materialized_anchor_bank"]["path"]
                    ),
                    "mechanism_config:={}".format(method["mechanism"]["path"]),
                ]
            )
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
                "attempt_stage:={}".format(STAGE),
                "attempt_profile_id:={}".format(profile),
                "attempt_scene_id:={}".format(row["scene_id"]),
                "attempt_seed:={}".format(row["seed"]),
                "attempt_number:=1",
                "supervisor_config_sha256:={}".format(
                    method["supervisor"]["sha256"]
                ),
            ]
        command_key = (
            "episode_argv"
            if row["row_type"] == "calibration_navigation"
            else "probe_argv"
        )
        if row["row_type"] == "calibration_navigation":
            command = [
                "{PYTHON_EXECUTABLE}",
                NAVIGATION_RELATIVE.as_posix(),
                "--instance",
                "{FUTURE_SCENE_SNAPSHOT_INSTANCE}",
                "--instance-sha256",
                row["instance"]["sha256"],
                "--method-id",
                row["method_id"],
                "--output-dir",
                "{FUTURE_ATTEMPT_ROOT}/episode",
                "--stage",
                STAGE,
                "--sequence",
                str(row["sequence"]),
                "--attempt",
                "1",
            ]
        else:
            command = [
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
        planned = {
            "sequence": row["sequence"],
            "identity": _identity(row),
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
            "bootstrap_mode": (
                "move_base_teb_service_after_positive_clock"
                if start_move_base
                else "direct_probe_positive_clock_only"
            ),
            "runtime_control_adapter": (
                CONTROL_RELATIVE.as_posix() if semantic else None
            ),
            "activation_listener_adapter": (
                LISTENER_RELATIVE.as_posix() if semantic else None
            ),
            "fixed_snapshot_adapter": (
                FIXED_RELATIVE.as_posix()
                if row["row_type"] == "calibration_navigation" and not semantic
                else None
            ),
            command_key: command,
            "per_identity_hard_wall_s": 240,
        }
        output.append(planned)
    _require(len(output) == 342, "R1 command plan row count differs")
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "exact_resolved_command_plan_not_yet_released",
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "source_c1_plan": {
            "path": OLD_PLAN_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / OLD_PLAN_RELATIVE).read_bytes()),
            "canonical_plan_digest": old_plan["canonical_plan_digest"],
        },
        "repair_class": "execution_integrity_binding_only",
        "behavioral_factor_count_changed": 0,
        "qualification_first": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "rows": output,
        "canonical_plan_digest": _canonical_sha(output),
    }


def _add_resource(
    resources: Dict[str, Dict[str, Any]], relative: Path, payload: bytes
) -> None:
    key = relative.as_posix()
    receipt = _receipt(relative, payload)
    if key in resources:
        _require(
            resources[key] == receipt,
            "dependency duplicate differs: {}".format(relative),
        )
    else:
        resources[key] = receipt


def _build_closure(schedule_payload: bytes, plan_payload: bytes) -> Dict[str, Any]:
    inherited = _load_yaml(WORKSPACE / OLD_CLOSURE_RELATIVE)
    _require(inherited.get("unresolved") == [], "old C1 closure unresolved")
    resources: Dict[str, Dict[str, Any]] = {}
    for receipt in inherited["local_resources"]:
        relative = Path(receipt["path"])
        payload = (WORKSPACE / relative).read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "old C1 local dependency drifted",
        )
        _add_resource(resources, relative, payload)
    for path in sorted((WORKSPACE / OLD_ROOT).rglob("*")):
        if path.is_file():
            _add_resource(resources, path.relative_to(WORKSPACE), path.read_bytes())
    explicit = {
        CONTRACT_RELATIVE,
        AUTHORIZATION_RELATIVE,
        OLD_RELEASE_RELATIVE,
        OLD_AUDIT_RELATIVE,
        OLD_SCHEDULE_RELATIVE,
        OLD_PLAN_RELATIVE,
        OLD_CLOSURE_RELATIVE,
        METHODS_RELATIVE,
        Q1_CONTRACT_RELATIVE,
        D2_CONTRACT_RELATIVE,
        LAUNCH_RELATIVE,
        BUILDER_RELATIVE,
        RUNNER_RELATIVE,
        REVIEWER_RELATIVE,
        ASSESSOR_RELATIVE,
        RELEASE_MODULE_RELATIVE,
        REPLAY_RELATIVE,
        NAVIGATION_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
        FIXED_RELATIVE,
        FROZEN_C1_RUNNER_RELATIVE,
        FROZEN_C1_ASSESSOR_RELATIVE,
    }
    for relative in sorted(explicit, key=lambda value: value.as_posix()):
        path = WORKSPACE / relative
        _require(path.is_file(), "R1 dependency absent: {}".format(relative))
        _add_resource(resources, relative, path.read_bytes())
    _add_resource(resources, SCHEDULE_RELATIVE, schedule_payload)
    _add_resource(resources, PLAN_RELATIVE, plan_payload)
    external = []
    for receipt in inherited["external_resources"]:
        path = Path(receipt["path"])
        _require(path.is_absolute() and path.is_file(), "external dependency absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external dependency drifted",
        )
        external.append(dict(receipt))
    document = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_execution_dependency_closure_release_absent",
        "simulation_only": True,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "evidence_units_consumed": 0,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "roots": [
            relative.as_posix()
            for relative in sorted(explicit, key=lambda value: value.as_posix())
        ]
        + [SCHEDULE_RELATIVE.as_posix(), PLAN_RELATIVE.as_posix()],
        "inherited_terminal_C1_closure": {
            "path": OLD_CLOSURE_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / OLD_CLOSURE_RELATIVE).read_bytes()),
            "local_resource_count": len(inherited["local_resources"]),
            "external_resource_count": len(inherited["external_resources"]),
        },
        "local_resources": [resources[key] for key in sorted(resources)],
        "local_resource_count": len(resources),
        "external_resources": external,
        "external_resource_count": len(external),
        "schedule_row_count": 342,
        "timeout_repaired_row_count": 8,
        "held_out_test_resource_or_seed_intentionally_added": False,
        "unresolved": [],
        "hash_graph_boundary": {
            "local": "workspace_relative_regular_files",
            "external": "canonical_absolute_runtime_providers_inherited_and_rehashed",
            "single_open_no_follow_revalidation_required_by_release_validator": True,
        },
    }
    logical = dict(document)
    document["logical_sha256"] = _canonical_sha(logical)
    return document


def build_bundle(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    authorization = _load_yaml(workspace / AUTHORIZATION_RELATIVE)
    _validate_inputs(contract, authorization)
    old_schedule = _load_yaml(workspace / OLD_SCHEDULE_RELATIVE)
    old_plan = _load_yaml(workspace / OLD_PLAN_RELATIVE)
    methods = _load_yaml(workspace / METHODS_RELATIVE)
    schedule = _build_schedule(old_schedule)
    plan = _build_plan(old_plan, schedule, methods)
    schedule_payload = _yaml_bytes(schedule)
    plan_payload = _yaml_bytes(plan)
    closure = _build_closure(schedule_payload, plan_payload)
    closure_payload = _yaml_bytes(closure)
    failure_index = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "preexecution_no_new_failures",
        "release_present": False,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "new_failures": [],
        "inherited_terminal_findings": [
            {
                "stage": "V2-04G-P2-C1",
                "kind": "preconsumption_schedule_instance_binding_mismatch",
                "execute_called": False,
                "consumed": 0,
                "old_authority_forfeited": 342,
                "audit": {
                    "path": OLD_AUDIT_RELATIVE.as_posix(),
                    "sha256": _sha((workspace / OLD_AUDIT_RELATIVE).read_bytes()),
                },
            }
        ],
        "retry_resume_replacement_or_seed_substitution_allowed": False,
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p2_c1_r1_release_readiness_built_review_pending_release_absent",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "old_C1_authority_forfeited": 342,
        "additive_budget_expansion": False,
        "behavioral_factor_count_changed": 0,
        "timeout_repaired_sequences": list(EXPECTED_TIMEOUT_REPAIRS),
        "direct_probe_startup_binding_repaired_sequences": list(
            EXPECTED_TIMEOUT_REPAIRS
        ),
        "all_342_scene_bindings_verified": True,
        "resolved_command_plan_row_count": 342,
        "dependency_counts": {
            "local": closure["local_resource_count"],
            "external": closure["external_resource_count"],
            "unresolved": 0,
        },
        "held_out_test_accessed": False,
        "ros_or_gazebo_started": False,
        "next_safe_entry": "independent_release_absent_review",
    }
    return {
        SCHEDULE_RELATIVE: schedule_payload,
        PLAN_RELATIVE: plan_payload,
        CLOSURE_RELATIVE: closure_payload,
        FAILURE_INDEX_RELATIVE: _yaml_bytes(failure_index),
        REPORT_RELATIVE: _yaml_bytes(report),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "R1 readiness root already exists")
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
        _require(path.is_file(), "R1 persisted resource absent: {}".format(relative))
        _require(path.read_bytes() == expected, "R1 resource differs: {}".format(relative))


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
        "P2-C1-R1 release-readiness {}: {} files".format(
            "materialized" if args.materialize else "verified", len(files)
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
