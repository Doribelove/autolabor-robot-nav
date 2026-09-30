#!/usr/bin/env python3
"""Build the P2-C1 exact release-readiness bundle without starting ROS.

The builder preserves all 342 Q1 source rows, moves the 12 safety
qualification rows before calibration, repairs only missing execution bindings,
and closes local/external runtime dependencies.  It never creates the release
or execution state and refuses to overwrite its output root.
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
STAGE = "V2-04G-P2-C1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_exact_calibration_execution_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_simulation_authorization.yaml"
)
Q1_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness"
)
Q1_SCHEDULE_RELATIVE = Q1_ROOT / "v2_04g_p2_q1_exact_342_schedule.yaml"
Q1_PLAN_RELATIVE = Q1_ROOT / "v2_04g_p2_q1_exact_342_command_plan.yaml"
Q1_METHODS_RELATIVE = Q1_ROOT / "v2_04g_p2_q1_method_bindings.yaml"
P1_CLOSURE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/"
    "offline_readiness_dependency_closure.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_release_readiness"
)
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_exact_342_schedule.yaml"
PLAN_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_exact_342_command_plan.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p2_c1_execution_dependency_closure.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_release_readiness_report.yaml"
FAILURE_INDEX_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_preexecution_failure_index.yaml"

LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_release_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_offline_reviewer.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_assess.py"
)
RELEASE_MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_c1_release.py"
)
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
NAVIGATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_navigation_episode.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_activation_probe_listener.py"
)
FIXED_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_fixed_snapshot.py"
)
P1_RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p1_e1_exact_runner.py"
)


class C1ReleaseBuildError(ValueError):
    """Raised when C1 release-readiness construction differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1ReleaseBuildError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root is not a mapping: {}".format(path))
    return value


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


def _receipt(relative: Path, payload: bytes) -> Dict[str, Any]:
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _validate_inputs(
    contract: Mapping[str, Any],
    authorization: Mapping[str, Any],
) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("execution_authorized") is True, "contract lacks authority")
    _require(
        contract.get("authorized_evidence_units_max") == 342,
        "contract budget differs",
    )
    _require(contract.get("training_budget_steps") == 0, "training budget differs")
    _require(contract.get("real_vehicle_use_forbidden") is True, "real vehicle not forbidden")
    _require(
        all(contract["forbidden_side_effects"].values()),
        "forbidden side-effect boundary differs",
    )
    for label, binding in contract["frozen_sources"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "frozen source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "frozen source drifted")
    _require(authorization.get("stage") == STAGE, "authorization stage differs")
    _require(
        authorization.get("explicit_user_authorization_recorded") is True
        and authorization.get("execution_authorized") is True,
        "explicit authorization is absent",
    )
    _require(
        authorization.get("authorized_evidence_units_max") == 342,
        "authorization budget differs",
    )
    _require(
        authorization.get("retry_resume_replacement_or_seed_substitution_allowed")
        is False,
        "authorization permits identity replacement",
    )


def _build_schedule(q1: Mapping[str, Any]) -> Dict[str, Any]:
    source_rows = q1["rows"]
    _require(len(source_rows) == 342, "Q1 source row count differs")
    qualification = [
        copy.deepcopy(row)
        for row in source_rows
        if row["row_type"] == "qualification_probe"
    ]
    calibration = [
        copy.deepcopy(row)
        for row in source_rows
        if row["row_type"] == "calibration_navigation"
    ]
    _require(len(qualification) == 12 and len(calibration) == 330, "Q1 partitions differ")
    ordered = qualification + calibration
    for sequence, row in enumerate(ordered, 1):
        row["q1_source_sequence"] = row["sequence"]
        row["sequence"] = sequence
        row["stage"] = STAGE
    _require(
        [row["candidate_speed_mps"] for row in ordered[:12]]
        == [speed for speed in (1.25, 1.30, 1.35, 1.40) for _ in range(3)],
        "qualification speed order differs",
    )
    _require(
        [row["probe_type"] for row in ordered[:12]]
        == [
            probe
            for _ in range(4)
            for probe in (
                "braking",
                "high_speed_to_conflict_overlay_transition",
                "ackermann_circle",
            )
        ],
        "qualification probe order differs",
    )
    _require(
        {row["q1_source_sequence"] for row in ordered} == set(range(1, 343)),
        "Q1 source row preservation differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in ordered},
        "held-out seed leaked into C1",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "exact_release_candidate_schedule_not_yet_released",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "training_budget_steps": 0,
        "real_vehicle_use_forbidden": True,
        "schedule_id": "fam_teb_v2_04g_p2_c1_qualification_first_exact_342_1",
        "source_q1_schedule": {
            "path": Q1_SCHEDULE_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / Q1_SCHEDULE_RELATIVE).read_bytes()),
            "canonical_row_digest": q1["canonical_row_digest"],
        },
        "repair_class": "execution_integrity_and_safety_order_only",
        "behavioral_factor_count_changed": 0,
        "qualification_first_gate": {
            "row_range": [1, 12],
            "monotone_speed_disqualification": True,
            "skipped_disqualified_identity_consumes_budget": False,
        },
        "calibration_row_range": [13, 342],
        "qualification_row_count": 12,
        "calibration_row_count": 330,
        "total_row_count": 342,
        "held_out_test_row_count": 0,
        "attempt_per_identity": 1,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "rows": ordered,
        "canonical_row_digest": _canonical_sha(ordered),
    }


def _replace_c1_tokens(value: Any) -> Any:
    if isinstance(value, str):
        return (
            value.replace("V2-04G-P2-Q1", STAGE)
            .replace(
                "v2_04g_r6_i1_p2_q1_navigation_episode.py",
                NAVIGATION_RELATIVE.name,
            )
            .replace(
                "v2_04g_r6_i1_p2_q1_qualification_probe.py",
                QUALIFICATION_RELATIVE.name,
            )
        )
    if isinstance(value, list):
        return [_replace_c1_tokens(item) for item in value]
    if isinstance(value, dict):
        return {key: _replace_c1_tokens(item) for key, item in value.items()}
    return value


def _replace_argument(argv, prefix, replacement):
    matches = [index for index, value in enumerate(argv) if value.startswith(prefix)]
    _require(len(matches) == 1, "command argument {} is not unique".format(prefix))
    argv[matches[0]] = replacement


def _build_plan(
    q1_plan: Mapping[str, Any],
    schedule: Mapping[str, Any],
    methods: Mapping[str, Any],
) -> Dict[str, Any]:
    q1_by_sequence = {row["sequence"]: row for row in q1_plan["rows"]}
    method_by_id = {row["method_id"]: row for row in methods["methods"]}
    output = []
    for scheduled in schedule["rows"]:
        source = _replace_c1_tokens(
            copy.deepcopy(q1_by_sequence[scheduled["q1_source_sequence"]])
        )
        source["q1_source_sequence"] = source["sequence"]
        source["sequence"] = scheduled["sequence"]
        source["identity"] = {
            key: scheduled[key]
            for key in scheduled
            if key
            in {
                "row_type",
                "method_id",
                "probe_type",
                "candidate_speed_mps",
                "scene_id",
                "seed",
                "attempt",
                "block_id",
            }
        }
        base = source["base_launch_argv"]
        _replace_argument(
            base,
            "attempt_stage:=",
            "attempt_stage:={}".format(STAGE),
        )
        semantic = source["semantic_transaction_launch_argv"] is not None
        if scheduled["row_type"] == "calibration_navigation":
            method = method_by_id[scheduled["method_id"]]
        else:
            method = method_by_id[scheduled["candidate_method_id"]]
        base.extend(
            [
                "load_balanced_anchor:=false",
                "publish_teb_obstacles:={}".format(
                    "true" if semantic else "false"
                ),
            ]
        )
        if semantic:
            base.extend(
                [
                    "rule_supervisor_config:={}".format(
                        method["supervisor"]["path"]
                    ),
                    "rule_supervisor_config_sha256:={}".format(
                        method["supervisor"]["sha256"]
                    ),
                ]
            )
            transaction = source["semantic_transaction_launch_argv"]
            _replace_argument(
                transaction,
                "attempt_stage:=",
                "attempt_stage:={}".format(STAGE),
            )
        command_key = (
            "episode_argv"
            if scheduled["row_type"] == "calibration_navigation"
            else "probe_argv"
        )
        command = source[command_key]
        _replace_argument(
            command,
            "--sequence",
            "--sequence",
        )
        sequence_index = command.index("--sequence") + 1
        command[sequence_index] = str(scheduled["sequence"])
        source.update(
            {
                "bootstrap_mode": (
                    "move_base_teb_service_after_positive_clock"
                    if "start_move_base:=true" in base
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
                    if scheduled["row_type"] == "calibration_navigation"
                    and not semantic
                    else None
                ),
            }
        )
        output.append(source)
    _require(len(output) == 342, "C1 command plan row count differs")
    return {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "exact_release_candidate_command_plan_not_yet_released",
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "source_q1_plan": {
            "path": Q1_PLAN_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / Q1_PLAN_RELATIVE).read_bytes()),
        },
        "repair_class": "execution_integrity_and_safety_order_only",
        "behavioral_factor_count_changed": 0,
        "qualification_first": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "rows": output,
        "canonical_plan_digest": _canonical_sha(output),
    }


def _add_resource(resources: Dict[str, Dict[str, Any]], relative: Path, payload: bytes):
    key = relative.as_posix()
    receipt = _receipt(relative, payload)
    if key in resources:
        _require(resources[key] == receipt, "dependency duplicate bytes differ: {}".format(key))
    else:
        resources[key] = receipt


def _build_closure(
    schedule_payload: bytes,
    plan_payload: bytes,
) -> Dict[str, Any]:
    inherited = _load_yaml(WORKSPACE / P1_CLOSURE_RELATIVE)
    _require(inherited.get("unresolved") == [], "inherited P1 closure unresolved")
    resources: Dict[str, Dict[str, Any]] = {}
    for receipt in inherited["local_resources"]:
        relative = Path(receipt["path"])
        payload = (WORKSPACE / relative).read_bytes()
        _require(_sha(payload) == receipt["sha256"], "inherited local dependency drifted")
        _require(len(payload) == receipt["size_bytes"], "inherited dependency size drifted")
        _add_resource(resources, relative, payload)
    for path in sorted((WORKSPACE / Q1_ROOT).rglob("*")):
        if path.is_file():
            relative = path.relative_to(WORKSPACE)
            _add_resource(resources, relative, path.read_bytes())
    explicit = {
        CONTRACT_RELATIVE,
        AUTHORIZATION_RELATIVE,
        P1_CLOSURE_RELATIVE,
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
        P1_RUNNER_RELATIVE,
    }
    for relative in sorted(explicit, key=lambda item: item.as_posix()):
        path = WORKSPACE / relative
        _require(path.is_file(), "C1 dependency absent: {}".format(relative))
        _add_resource(resources, relative, path.read_bytes())
    _add_resource(resources, SCHEDULE_RELATIVE, schedule_payload)
    _add_resource(resources, PLAN_RELATIVE, plan_payload)

    external = []
    for receipt in inherited["external_resources"]:
        path = Path(receipt["path"])
        _require(path.is_absolute() and path.is_file(), "external dependency absent")
        payload = path.read_bytes()
        _require(_sha(payload) == receipt["sha256"], "external dependency drifted")
        _require(len(payload) == receipt["size_bytes"], "external dependency size drifted")
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
        "roots": [relative.as_posix() for relative in sorted(explicit, key=lambda item: item.as_posix())]
        + [SCHEDULE_RELATIVE.as_posix(), PLAN_RELATIVE.as_posix()],
        "inherited_p1_closure": {
            "path": P1_CLOSURE_RELATIVE.as_posix(),
            "sha256": _sha((WORKSPACE / P1_CLOSURE_RELATIVE).read_bytes()),
            "local_resource_count": len(inherited["local_resources"]),
            "external_resource_count": len(inherited["external_resources"]),
        },
        "local_resources": [
            resources[key] for key in sorted(resources)
        ],
        "local_resource_count": len(resources),
        "external_resources": external,
        "external_resource_count": len(external),
        "schedule_row_count": 342,
        "held_out_test_resource_or_seed_intentionally_added": False,
        "unresolved": [],
        "hash_graph_boundary": {
            "local": "workspace_relative_regular_files",
            "external": "canonical_absolute_runtime_providers_inherited_and_rehashed",
            "single_open_no_follow_revalidation_required_by_release_validator": True,
        },
    }
    document["logical_sha256"] = _canonical_sha(document)
    return document


def build_bundle(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    authorization = _load_yaml(workspace / AUTHORIZATION_RELATIVE)
    _validate_inputs(contract, authorization)
    q1_schedule = _load_yaml(workspace / Q1_SCHEDULE_RELATIVE)
    q1_plan = _load_yaml(workspace / Q1_PLAN_RELATIVE)
    methods = _load_yaml(workspace / Q1_METHODS_RELATIVE)
    schedule = _build_schedule(q1_schedule)
    plan = _build_plan(q1_plan, schedule, methods)
    schedule_payload = _yaml_bytes(schedule)
    plan_payload = _yaml_bytes(plan)
    closure = _build_closure(schedule_payload, plan_payload)
    closure_payload = _yaml_bytes(closure)
    failure_index = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "preexecution_no_failures",
        "release_present": False,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "failures": [],
        "skipped_identities": [],
        "retry_resume_replacement_or_seed_substitution_allowed": False,
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p2_c1_release_readiness_built_review_pending_release_absent",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized_by_user": True,
        "release_present": False,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "schedule": _receipt(SCHEDULE_RELATIVE, schedule_payload),
        "command_plan": _receipt(PLAN_RELATIVE, plan_payload),
        "dependency_closure": _receipt(CLOSURE_RELATIVE, closure_payload),
        "qualification_first_rows": 12,
        "calibration_rows": 330,
        "held_out_test_rows": 0,
        "behavioral_factor_count_changed_from_q1": 0,
        "execution_binding_repairs": [
            "qualification_first",
            "semantic_supervisor_path_and_sha",
            "semantic_dynamic_obstacle_bridge_flag",
            "direct_probe_positive_clock_bootstrap",
            "C1_evidence_identity_wrappers",
        ],
        "next_gate": "independent_offline_review_with_release_absent",
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
    _require(not target.exists(), "C1 release-readiness root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".p2_c1_release.", dir=str(target.parent)))
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
        _require(path.is_file(), "C1 persisted resource absent: {}".format(relative))
        _require(path.read_bytes() == expected, "C1 persisted resource differs: {}".format(relative))


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
        "P2-C1 release-readiness {}: {} files".format(
            "materialized" if args.materialize else "verified", len(files)
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
