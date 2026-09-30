#!/usr/bin/env python3
"""Freeze the final FAM-TEB finite-search result and negative conclusion offline."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P4-F1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p4_f1_terminal_program_closure_contract.yaml"
)
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_f1_terminal_program_closure.py"
)
EXECUTION_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p4_i1_execution")
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_f1_terminal_program_closure"
)
EVIDENCE_RELATIVE = OUTPUT_ROOT / "p4_f1_terminal_evidence_freeze.yaml"
ACCOUNTING_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_f1_global_budget_accounting.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_f1_terminal_conclusion.yaml"


class P4F1Error(ValueError):
    """Raised when final evidence, accounting, or conclusion differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4F1Error(message)


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


def _active_runtime_processes() -> list[Dict[str, Any]]:
    tokens = (
        b"roscore",
        b"rosmaster",
        b"roslaunch",
        b"gzserver",
        b"gzclient",
        b"move_base",
    )
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        if any(token in command.lower() for token in tokens):
            result.append({"pid": int(entry.name), "command_sha256": _sha(command)})
    return result


def _verify_contract(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    for label, binding in contract["frozen_sources"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "frozen source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "frozen source drifted")


def build_outputs(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _verify_contract(contract)

    reports = {
        1: _load_yaml(
            workspace
            / contract["frozen_sources"]["round_1_terminal_report"]["path"]
        ),
        2: _load_yaml(
            workspace
            / contract["frozen_sources"]["round_2_terminal_report"]["path"]
        ),
        3: _load_yaml(
            workspace
            / contract["frozen_sources"]["round_3_terminal_report"]["path"]
        ),
    }
    _require(
        reports[1]["status"] == "terminal_integrity_failure"
        and reports[1]["evidence_units_consumed"] == 2
        and reports[1]["evidence_units_completed"] == 1
        and reports[1]["evidence_units_forfeited"] == 340,
        "round-1 terminal report differs",
    )
    _require(
        reports[2]["status"] == "terminal_integrity_failure"
        and reports[2]["evidence_units_consumed"] == 1
        and reports[2]["evidence_units_completed"] == 0
        and reports[2]["held_out_test_accessed"] is False,
        "round-2 terminal report differs",
    )
    expected3 = contract["terminal_round_3_expected"]
    _require(
        reports[3]["status"] == "terminal_integrity_failure"
        and reports[3]["authorized_evidence_units_max"] == expected3["authorized_units"]
        and reports[3]["evidence_units_consumed"] == expected3["consumed_units"]
        and reports[3]["evidence_units_completed"] == expected3["completed_units"]
        and reports[3]["evidence_units_forfeited"] == expected3["stage_forfeited_units"]
        and reports[3]["held_out_test_accessed"] is False
        and reports[3]["attempt_ledger"][0]["identity"]["seed"] == expected3["seed"],
        "round-3 terminal accounting differs",
    )
    journal = _load_yaml(
        workspace
        / contract["frozen_sources"]["round_3_failure_journal"]["path"]
    )
    failure_index = _load_yaml(
        workspace / contract["frozen_sources"]["round_3_failure_index"]["path"]
    )
    _require(
        journal["raw_resources_produced"] == {}
        and journal["consumption_boundary"] == "base_roslaunch_spawn_requested"
        and failure_index["failure_count"] == 1,
        "round-3 failure ledger differs",
    )
    base_log = (
        workspace / contract["frozen_sources"]["round_3_base_log"]["path"]
    ).read_text(encoding="utf-8", errors="replace")
    _require(
        "is neither a launch file in package [m2_gazebo]" in base_log,
        "direct terminal cause differs",
    )
    launch_path = workspace / "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
    _require(launch_path.is_file(), "locally hash-closed launch file absent")
    environment_policy = reports[3]["attempt_ledger"][0]["environment_policy"]
    _require(
        "ROS_PACKAGE_PATH" in environment_policy["inherited_key_names"]
        and environment_policy["source_value_material_recorded"] is False,
        "child environment evidence differs",
    )
    preaudit = _load_yaml(
        workspace
        / contract["frozen_sources"]["round_3_preexecution_audit"]["path"]
    )
    _require(
        "package_resolution_probe_pass" not in preaudit["release_validation"],
        "preexecution package-resolution statement changed",
    )
    promoted_review = _load_yaml(
        workspace
        / contract["frozen_sources"]["promoted_bank_independent_review"]["path"]
    )
    _require(
        promoted_review["strict_parser_pass_count"] == 5
        and promoted_review["behavioral_TEB_field_count_changed"] == 0,
        "promoted bank offline evidence differs",
    )

    global_expected = contract["global_post_P1_budget_expected"]
    rounds = global_expected["rounds"]
    consumed = sum(row["consumed"] for row in rounds.values())
    retired = sum(row["retired"] for row in rounds.values())
    capacity = sum(row["capacity"] for row in rounds.values())
    _require(
        capacity == global_expected["simulation_capacity_total"] == 2200
        and consumed == global_expected["simulation_units_consumed_total"] == 4
        and retired == global_expected["simulation_units_retired_total"] == 2196
        and consumed + retired == capacity
        and global_expected["remaining_simulation_capacity"] == 0,
        "global budget reconciliation differs",
    )
    p1 = _load_yaml(
        workspace
        / contract["frozen_sources"]["historical_P1_four_gate_assessment"]["path"]
    )
    effect = contract["terminal_effect_gate"]
    _require(
        p1["formal_study_complete"] is True
        and p1["performance_improvement_demonstrated"] is False
        and p1["efficiency"]["geometric_time_reduction_fraction"]
        == effect["historical_P1_point_reduction_fraction"]
        and p1["efficiency"]["one_sided_95_percent_lower_fraction"]
        == effect["historical_P1_one_sided_lower_fraction"]
        and effect["historical_P1_point_reduction_fraction"]
        < effect["required_point_reduction_fraction"]
        and effect["historical_P1_one_sided_lower_fraction"]
        < effect["required_one_sided_lower_reduction_fraction"],
        "historical effect or terminal gate differs",
    )
    _require(not _active_runtime_processes(), "ROS/Gazebo process remains active")

    execution_receipts = [
        _receipt(path.relative_to(workspace))
        for path in sorted((workspace / EXECUTION_ROOT).rglob("*"))
        if path.is_file()
    ]
    _require(len(execution_receipts) == 8, "P4-I1 execution file count differs")
    resources = {row["path"]: row for row in execution_receipts}
    for relative in (CONTRACT_RELATIVE, ANALYZER_RELATIVE):
        resources.setdefault(relative.as_posix(), _receipt(relative))
    for binding in contract["frozen_sources"].values():
        relative = Path(binding["path"])
        resources.setdefault(relative.as_posix(), _receipt(relative))
    manifest = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "terminal_global_program_evidence_frozen",
        "offline_only": True,
        "P4_I1_execution_file_count": len(execution_receipts),
        "P4_I1_execution_total_size_bytes": sum(
            row["size_bytes"] for row in execution_receipts
        ),
        "resources": [resources[key] for key in sorted(resources)],
        "resource_count": len(resources),
        "unresolved": [],
    }
    manifest["logical_sha256"] = _canonical_sha(manifest)

    accounting = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "global_post_P1_budget_exhausted_and_closed",
        "simulation_capacity_total": capacity,
        "simulation_units_consumed": consumed,
        "simulation_units_retired": retired,
        "simulation_units_remaining": 0,
        "training_steps_capacity_total": 120000,
        "training_steps_consumed": 0,
        "training_steps_retired": 120000,
        "training_steps_remaining": 0,
        "rounds": rounds,
        "budget_transfer_expansion_recycling_or_new_round_allowed": False,
        "historical_P1_270_episode_formal_study_outside_post_P1_budget": True,
        "new_held_out_test_accessed": False,
    }
    conclusion = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "global_budget_exhausted_obvious_improvement_not_demonstrated",
        "program_terminal": True,
        "stop_performance_search": True,
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "candidate_winner_frozen": False,
        "positive_performance_claim_allowed": False,
        "historical_formal_result": {
            "stage": "V2-04G-P1-E1",
            "formal_result": True,
            "assessment_status": p1["assessment_status"],
            "point_reduction_fraction": p1["efficiency"][
                "geometric_time_reduction_fraction"
            ],
            "one_sided_95_percent_lower_fraction": p1["efficiency"][
                "one_sided_95_percent_lower_fraction"
            ],
            "safety_and_task_noninferiority": p1["four_gate_decision"][
                "safety_and_task_noninferiority"
            ],
            "performance_improvement_demonstrated": False,
        },
        "requested_obvious_effect_gate": {
            "point_reduction_fraction_at_least": 0.10,
            "one_sided_lower_reduction_fraction_at_least": 0.05,
            "historical_point_gate_pass": False,
            "historical_lower_gate_pass": False,
            "post_P1_complete_test_matrix_available": False,
            "post_P1_integrity_gate_pass": False,
        },
        "post_P1_search": {
            "factor": "p2_cruise_forward_velocity_ceiling_v1",
            "single_amendment_sequence": [
                "runtime_profile_identity_alignment",
                "runtime_bank_schema_promotion",
            ],
            "complete_calibration_matrix": False,
            "held_out_test_accessed": False,
            "performance_effect_observed": False,
            "simulation_units_consumed": consumed,
            "simulation_units_retired": retired,
        },
        "terminal_failure_attribution": {
            "direct_cause": "ros_package_resolution_failed_before_service_readiness",
            "inferred_primary_integrity_gap": (
                "workspace_overlay_environment_not_bound_or_resolved_before_consumption"
            ),
            "Gazebo_started": False,
            "parameter_transaction_started": False,
            "raw_trace_produced": False,
            "promoted_bank_strict_offline_parser_passed": True,
            "promoted_bank_live_runtime_behavior_reached": False,
        },
        "global_budget": accounting,
        "terminal_decision": "obvious_performance_improvement_not_demonstrated",
        "chinese_conclusion": "未证明明显提升",
        "required_thesis_wording": (
            "历史 P1 正式配对试验的受限完成时间点估计降低 3.7624%，"
            "单侧 95% 下界为 2.7904%，均未达到本轮预注册的 10%/5% "
            "明显提升门；后续有限预算搜索因执行完整性失败未形成完整校准或"
            "held-out test 矩阵。全局 2200-unit 预算现已耗尽并关闭，"
            "因此本研究未证明 FAM-TEB 相对最强公平固定 TEB 基线有明显性能提升。"
        ),
        "active_runtime_processes": [],
        "real_vehicle_used": False,
        "stable_workspace_touched": False,
        "next_safe_entry": "none_performance_search_terminal",
    }
    return {
        EVIDENCE_RELATIVE: _yaml_bytes(manifest),
        ACCOUNTING_RELATIVE: _yaml_bytes(accounting),
        REPORT_RELATIVE: _yaml_bytes(conclusion),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P4-F1 output root already exists")
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
        _require(path.is_file() and path.read_bytes() == expected, "output drifted")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    files = build_outputs(WORKSPACE)
    if args.check:
        _check_only(files, WORKSPACE)
    else:
        _materialize(files, WORKSPACE)
    print(
        yaml.safe_dump(
            {
                "stage": STAGE,
                "status": "check_pass" if args.check else "materialized",
                "output_count": len(files),
            },
            sort_keys=False,
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
