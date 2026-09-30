#!/usr/bin/env python3
"""Freeze and attribute the terminal P2-C1-R1 execution failure offline."""

from __future__ import annotations

import argparse
import ast
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
STAGE = "V2-04G-P2-C1-R1-D1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_r1_d1_offline_failure_attribution_contract.yaml"
)
EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_execution"
)
READINESS_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_release_readiness"
)
SCHEDULE_RELATIVE = (
    READINESS_ROOT / "v2_04g_p2_c1_r1_exact_342_schedule.yaml"
)
PLAN_RELATIVE = (
    READINESS_ROOT / "v2_04g_p2_c1_r1_exact_342_command_plan.yaml"
)
Q1_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_assess.py"
)
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_d1_failure_attribution.py"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p2_c1_r1_d1_failure_attribution"
)
EVIDENCE_RELATIVE = OUTPUT_ROOT / "p2_c1_r1_d1_source_evidence_freeze.yaml"
REPORT_RELATIVE = (
    OUTPUT_ROOT / "v2_04g_p2_c1_r1_d1_failure_attribution_report.yaml"
)
RELEASE_SHA256 = "1926726a2b08faf10a7b460ace53f01c1123b070da8e65a8be4900985f86f9cc"


class C1R1D1Error(ValueError):
    """Raised when frozen C1-R1 evidence differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1R1D1Error(message)


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


def _receipt(relative: Path) -> Dict[str, Any]:
    payload = (WORKSPACE / relative).read_bytes()
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _load_module(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    _require(spec is not None and spec.loader is not None, "cannot load assessor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ASSESSOR = _load_module("_p2_c1_r1_d1_assessor", ASSESSOR_RELATIVE)


def _verify_contract(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline-only")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    for label, binding in contract["frozen_sources"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "frozen source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "frozen source drifted")


def _profile_allowlist(source: str, estimator_source: str) -> Dict[str, str]:
    tree = ast.parse(source)
    constants: Dict[str, str] = {}
    for node in ast.parse(estimator_source).body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if (
            isinstance(target, ast.Name)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        ):
            constants[target.id] = node.value.value
    target_dict = None
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name):
            continue
        if target.id == "EXPECTED_ESTIMATOR_BY_PROFILE":
            target_dict = node.value
    _require(isinstance(target_dict, ast.Dict), "supervisor profile map absent")
    result = {}
    for key_node, value_node in zip(target_dict.keys, target_dict.values):
        _require(
            isinstance(key_node, ast.Constant)
            and isinstance(key_node.value, str)
            and isinstance(value_node, ast.Name)
            and value_node.id in constants,
            "supervisor profile map is not statically closed",
        )
        result[key_node.value] = constants[value_node.id]
    return result


def _active_runtime_processes():
    tokens = (
        b"roscore",
        b"rosmaster",
        b"roslaunch",
        b"gzserver",
        b"gzclient",
        b"move_base",
    )
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        if any(token in command.lower() for token in tokens):
            found.append({"pid": int(entry.name), "command_sha256": _sha(command)})
    return found


def _build_evidence_manifest(contract: Mapping[str, Any]) -> Dict[str, Any]:
    execution_receipts = [
        _receipt(path.relative_to(WORKSPACE))
        for path in sorted((WORKSPACE / EXECUTION_ROOT).rglob("*"))
        if path.is_file()
    ]
    _require(len(execution_receipts) == 45, "execution file count differs")
    bound = {
        receipt["path"]: receipt
        for receipt in execution_receipts
    }
    for binding in contract["frozen_sources"].values():
        relative = Path(binding["path"])
        bound.setdefault(relative.as_posix(), _receipt(relative))
    for relative in (
        CONTRACT_RELATIVE,
        ANALYZER_RELATIVE,
        SCHEDULE_RELATIVE,
        PLAN_RELATIVE,
        Q1_CONTRACT_RELATIVE,
        METHODS_RELATIVE,
        ASSESSOR_RELATIVE,
    ):
        bound.setdefault(relative.as_posix(), _receipt(relative))
    manifest = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "terminal_C1_R1_source_evidence_frozen",
        "offline_only": True,
        "execution_file_count": len(execution_receipts),
        "execution_total_size_bytes": sum(
            receipt["size_bytes"] for receipt in execution_receipts
        ),
        "resources": [bound[key] for key in sorted(bound)],
        "resource_count": len(bound),
        "unresolved": [],
    }
    manifest["logical_sha256"] = _canonical_sha(manifest)
    return manifest


def build_outputs(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _verify_contract(contract)
    report = _load_yaml(workspace / EXECUTION_ROOT / "p2_c1_r1_stage_report.yaml")
    failure_index = _load_yaml(
        workspace / EXECUTION_ROOT / "p2_c1_r1_failure_index.yaml"
    )
    expected = contract["terminal_accounting_expected"]
    _require(
        report.get("status") == "terminal_integrity_failure"
        and report.get("release_sha256") == RELEASE_SHA256
        and report.get("authorized_evidence_units_max") == expected["authorized_units"]
        and report.get("evidence_units_consumed") == expected["consumed_units"]
        and report.get("evidence_units_completed") == expected["completed_units"]
        and report.get("evidence_units_skipped") == expected["skipped_units"]
        and report.get("evidence_units_forfeited") == expected["forfeited_units"]
        and report.get("held_out_test_accessed") is False,
        "terminal accounting differs",
    )
    _require(
        failure_index.get("failure_count") == 1
        and failure_index["failures"][0]["sequence"] == 2,
        "failure index differs",
    )
    schedule = _load_yaml(workspace / SCHEDULE_RELATIVE)
    thresholds = _load_yaml(workspace / Q1_CONTRACT_RELATIVE)[
        "qualification_thresholds"
    ]
    normalized, replay_audit = ASSESSOR.replay_one(
        workspace,
        schedule["rows"][0],
        workspace / EXECUTION_ROOT / "journals/sequence_0001.yaml",
        RELEASE_SHA256,
        thresholds,
    )
    _require(
        normalized["pass"] is True
        and normalized["probe_type"] == "braking"
        and normalized["candidate_speed_mps"] == 1.25
        and normalized["collision"] is False
        and normalized["emergency"] is False
        and normalized["normal_projection_rate"] == 0.0,
        "completed raw replay differs",
    )

    plan = _load_yaml(workspace / PLAN_RELATIVE)
    row2 = plan["rows"][1]
    profile_args = [
        value.split(":=", 1)[1]
        for value in row2["base_launch_argv"]
        if value.startswith("attempt_profile_id:=")
    ]
    _require(profile_args == ["qualification_overlay_v125"], "failed profile differs")
    supervisor_path = Path(
        contract["frozen_sources"]["supervisor_implementation"]["path"]
    )
    supervisor_source = (workspace / supervisor_path).read_text(encoding="utf-8")
    estimator_source = (
        workspace
        / contract["frozen_sources"]["supervisor_estimator_implementation"]["path"]
    ).read_text(encoding="utf-8")
    allowlist = _profile_allowlist(supervisor_source, estimator_source)
    _require(
        set(allowlist)
        == {"r6_semantics_legacy_control", "r6_semantics_circle_contact"},
        "supervisor allowlist differs",
    )
    methods = _load_yaml(workspace / METHODS_RELATIVE)
    method = next(
        row for row in methods["methods"] if row["method_id"] == "circle_contact_v125"
    )
    supervisor_config = _load_yaml(workspace / method["supervisor"]["path"])
    estimator = supervisor_config["dynamic"]["conflict_estimator_id"]
    matching_runtime_profiles = [
        key for key, value in allowlist.items() if value == estimator
    ]
    _require(
        matching_runtime_profiles == ["r6_semantics_circle_contact"],
        "aligned runtime profile inference differs",
    )
    failed_log = (
        workspace
        / contract["frozen_sources"]["failed_base_log"]["path"]
    ).read_text(encoding="utf-8", errors="replace")
    unpause_log = (
        workspace
        / contract["frozen_sources"]["failed_unpause_log"]["path"]
    ).read_text(encoding="utf-8", errors="replace")
    _require(
        "R6 attempt profile identity is invalid" in failed_log
        and "REQUIRED process [r6_rule_context_supervisor-9] has died" in failed_log
        and "Service [/gazebo/unpause_physics] is not available" in unpause_log,
        "failure trace markers differ",
    )
    processes = _active_runtime_processes()
    _require(processes == [], "runtime process remains active")
    evidence = _build_evidence_manifest(contract)
    diagnosis = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_failure_attribution_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "simulation_started_by_D1": False,
        "terminal_C1_R1_accounting": {
            "authorized": 342,
            "consumed": 2,
            "completed": 1,
            "skipped": 0,
            "forfeited": 340,
            "retry_resume_or_reuse_allowed": False,
            "held_out_test_accessed": False,
        },
        "completed_sequence_1_raw_replay": {
            "normalized": normalized,
            "audit": replay_audit,
        },
        "failure_sequence_2": {
            "recorded_terminal_reason": failure_index["failures"][0]["reason"],
            "planned_runtime_profile_id": profile_args[0],
            "supervisor_allowed_profile_to_estimator": allowlist,
            "frozen_supervisor_estimator_id": estimator,
            "required_aligned_runtime_profile_id": matching_runtime_profiles[0],
            "required_supervisor_process_died": True,
            "unpause_service_not_available_after_required_process_exit": True,
        },
        "attribution": {
            "primary_cause": "runtime_evidence_profile_identity_domain_mismatch",
            "primary_mechanism": "",
        },
        "next_single_factor": {
            "factor_id": (
                "separate_runtime_semantic_profile_from_evidence_candidate_identity_v1"
            ),
            "runtime_profile_id": "r6_semantics_circle_contact",
            "evidence_identity_preserved": True,
            "behavioral_TEB_parameter_change": False,
            "other_runtime_scene_threshold_estimator_or_safety_change": False,
            "fresh_seed_required": True,
            "old_seed_reuse_allowed": False,
            "execution_authorized": False,
        },
        "evidence_freeze": {
            "path": EVIDENCE_RELATIVE.as_posix(),
            "logical_sha256": evidence["logical_sha256"],
            "resource_count": evidence["resource_count"],
        },
        "process_isolation": {
            "active_runtime_processes": processes,
            "clean": True,
        },
        "claim_boundary": {
            "performance_effect_or_candidate_selection_allowed": False,
            "held_out_test_or_obvious_improvement_claim_allowed": False,
        },
        "next_safe_entry": (
            "independent_offline_review_then_new_finite_design_with_fresh_seed"
        ),
    }
    diagnosis["attribution"]["primary_mechanism"] = (
        "The evidence-level candidate profile qualification_overlay_v125 was "
        "passed into the frozen supervisor runtime-profile interface. That "
        "interface accepts only estimator-bound semantic aliases; its required "
        "process exited, and the later unpause call failed because roslaunch "
        "was already shutting the attempt down."
    )
    return {
        EVIDENCE_RELATIVE: _yaml_bytes(evidence),
        REPORT_RELATIVE: _yaml_bytes(diagnosis),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "D1 output root already exists")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=target.name + ".tmp.", dir=target.parent))
    try:
        for relative, payload in sorted(files.items(), key=lambda item: item[0].as_posix()):
            local = temporary / relative.relative_to(OUTPUT_ROOT)
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
        _require(path.is_file(), "D1 output absent")
        _require(path.read_bytes() == expected, "D1 output differs")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    files = build_outputs(workspace)
    if args.materialize:
        _materialize(files, workspace)
    else:
        _check_only(files, workspace)
    print(
        "P2-C1-R1-D1 {}: offline_failure_attribution_pass".format(
            "materialized" if args.materialize else "verified"
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
