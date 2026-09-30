#!/usr/bin/env python3
"""Freeze and attribute the terminal P3-I1 Anchor Bank failure offline."""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P3-I1-D1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p3_i1_d1_offline_failure_attribution_contract.yaml"
)
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_d1_failure_attribution.py"
)
EXECUTION_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p3_i1_execution")
Q1_CANDIDATE_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p2_q1_offline_readiness/candidate_configs"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p3_i1_d1_failure_attribution"
)
EVIDENCE_RELATIVE = OUTPUT_ROOT / "p3_i1_d1_source_evidence_freeze.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_i1_d1_failure_attribution_report.yaml"


class P3I1D1Error(ValueError):
    """Raised when frozen P3-I1 evidence or the single factor differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3I1D1Error(message)


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


def _leaf_differences(left: Any, right: Any, prefix: str = "") -> list[str]:
    if type(left) is not type(right):
        return [prefix]
    if isinstance(left, dict):
        differences = []
        for key in sorted(set(left) | set(right)):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in left or key not in right:
                differences.append(child)
            else:
                differences.extend(_leaf_differences(left[key], right[key], child))
        return differences
    if isinstance(left, list):
        if len(left) != len(right):
            return [prefix]
        differences = []
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            child = "{}[{}]".format(prefix, index)
            differences.extend(_leaf_differences(left_item, right_item, child))
        return differences
    return [] if left == right else [prefix]


def _active_runtime_processes() -> list[Dict[str, Any]]:
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


def _load_anchor_bank_class():
    source_root = WORKSPACE / "src/application/teb_mode_manager/src"
    sys.path.insert(0, str(source_root))
    try:
        from teb_mode_manager.action_pipeline import AnchorBank
    finally:
        sys.path.pop(0)
    return AnchorBank


def _verify_contract(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline-only")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    for label, binding in contract["frozen_sources"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "frozen source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "frozen source drifted")


def _candidate_parser_audit(contract: Mapping[str, Any]) -> list[Dict[str, Any]]:
    AnchorBank = _load_anchor_bank_class()
    rows = []
    expected_differences = ["source_provenance.p2_q1", "status"]
    paths = sorted(Q1_CANDIDATE_ROOT.glob("circle_contact_*_anchor_bank.yaml"))
    _require(len(paths) == 5, "circle candidate count differs")
    for path in paths:
        source = _load_yaml(WORKSPACE / path)
        original_error = None
        try:
            AnchorBank(source)
        except Exception as exc:  # exact frozen rejection is part of the evidence
            original_error = "{}: {}".format(type(exc).__name__, exc)
        _require(
            original_error
            == "ActionPipelineError: anchor bank safety boundary drifted",
            "original candidate rejection differs",
        )
        _require(
            source.get("status") == "offline_materialized_not_runtime_ready"
            and set(source["source_provenance"])
            == {"baseline", "mode_deltas", "formal_test_scenes_used", "p2_q1"},
            "candidate lifecycle/provenance differs",
        )
        max_vel = [
            row for row in source["parameters"] if row.get("name") == "max_vel_x"
        ]
        _require(
            len(max_vel) == 1 and max_vel[0]["bounds"] == [0.1, 1.4],
            "max_vel_x bound differs",
        )
        promoted = deepcopy(source)
        promoted["status"] = "uncalibrated_simulation_candidate"
        removed = promoted["source_provenance"].pop("p2_q1")
        differences = _leaf_differences(source, promoted)
        _require(differences == expected_differences, "promotion diff escaped allowlist")
        parsed = AnchorBank(promoted)
        _require(len(parsed.parameter_names) == 20, "promoted candidate parse differs")
        rows.append(
            {
                "source_path": path.as_posix(),
                "source_sha256": _sha((WORKSPACE / path).read_bytes()),
                "bank_id": source["bank_id"],
                "original_rejection": original_error,
                "in_memory_promoted_parse": "pass",
                "allowed_differences": differences,
                "removed_provenance_stage": removed["stage"],
                "max_vel_x_bounds_unchanged": [0.1, 1.4],
                "behavioral_TEB_field_count_changed": 0,
            }
        )
    return rows


def build_outputs(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _verify_contract(contract)

    report = _load_yaml(workspace / EXECUTION_ROOT / "p3_i1_stage_report.yaml")
    failure_index = _load_yaml(workspace / EXECUTION_ROOT / "p3_i1_failure_index.yaml")
    journal = _load_yaml(
        workspace
        / EXECUTION_ROOT
        / "journals/sequence_0001_terminal_integrity_failure.yaml"
    )
    expected = contract["terminal_accounting_expected"]
    _require(
        report.get("status") == "terminal_integrity_failure"
        and report.get("authorized_evidence_units_max") == expected["stage_authorized_units"]
        and report.get("evidence_units_consumed") == expected["actual_consumed_units"]
        and report.get("evidence_units_completed") == expected["completed_units"]
        and report.get("evidence_units_forfeited") == expected["stage_forfeited_units"]
        and report.get("held_out_test_accessed") is False
        and report.get("runtime_profile_id") == "r6_semantics_circle_contact",
        "terminal accounting or runtime identity differs",
    )
    _require(
        failure_index.get("failure_count") == 1
        and failure_index["failures"][0]["sequence"] == 1
        and journal.get("raw_resources_produced") == {},
        "terminal failure ledger differs",
    )

    bootstrap_path = Path(
        contract["frozen_sources"]["P3_I1_bootstrap_receipt"]["path"]
    )
    bootstrap = _load_yaml(workspace / bootstrap_path)
    barrier = bootstrap["barrier"]
    _require(
        barrier["state"] == "service_wait_released"
        and barrier["terminal"] is True
        and barrier["failure_reason"] is None
        and barrier["post_ack_clock_sample_count"] == 2
        and barrier["service_wait_allowed"] is True
        and bootstrap["move_base_teb_service_wait_after_release"] is True,
        "positive-progress bootstrap evidence differs",
    )
    base_log = (
        workspace / contract["frozen_sources"]["P3_I1_base_log"]["path"]
    ).read_text(encoding="utf-8", errors="replace")
    _require(
        "process[r6_rule_context_supervisor-9]: started with pid" in base_log,
        "supervisor startup evidence differs",
    )
    readback_log = (
        workspace / contract["frozen_sources"]["P3_I1_initial_readback_log"]["path"]
    ).read_text(encoding="utf-8", errors="replace")
    _require(
        "AnchorBank.from_file(args.anchor_bank)" in readback_log
        and "ActionPipelineError: anchor bank safety boundary drifted" in readback_log,
        "initial-readback failure differs",
    )
    parser_audit = _candidate_parser_audit(contract)
    _require(not _active_runtime_processes(), "ROS/Gazebo process remains active")

    execution_receipts = [
        _receipt(path.relative_to(workspace))
        for path in sorted((workspace / EXECUTION_ROOT).rglob("*"))
        if path.is_file()
    ]
    _require(len(execution_receipts) == 25, "execution file count differs")
    resources = {row["path"]: row for row in execution_receipts}
    for relative in (CONTRACT_RELATIVE, ANALYZER_RELATIVE):
        resources.setdefault(relative.as_posix(), _receipt(relative))
    for binding in contract["frozen_sources"].values():
        relative = Path(binding["path"])
        resources.setdefault(relative.as_posix(), _receipt(relative))
    for row in parser_audit:
        relative = Path(row["source_path"])
        resources.setdefault(relative.as_posix(), _receipt(relative))
    manifest = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "terminal_P3_I1_source_evidence_frozen",
        "offline_only": True,
        "execution_file_count": len(execution_receipts),
        "execution_total_size_bytes": sum(row["size_bytes"] for row in execution_receipts),
        "resources": [resources[key] for key in sorted(resources)],
        "resource_count": len(resources),
        "unresolved": [],
    }
    manifest["logical_sha256"] = _canonical_sha(manifest)

    attribution = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_failure_attribution_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "terminal_accounting": {
            "round": 2,
            "round_capacity": 704,
            "actual_consumed": 1,
            "completed": 0,
            "program_capacity_retired": 703,
            "total_closed": 704,
            "reusable": 0,
        },
        "prior_factor_boundary_result": {
            "runtime_semantic_profile_alignment": "passed",
            "positive_progress_clock_bootstrap": "passed",
            "supervisor_started": True,
            "move_base_teb_service_wait_released": True,
        },
        "failure_boundary": {
            "phase": "initial_readback_before_parameter_transaction",
            "raw_trace_produced": False,
            "held_out_test_accessed": False,
            "candidate_bank_sha256": contract["frozen_sources"][
                "failed_candidate_bank"
            ]["sha256"],
            "exception": "ActionPipelineError: anchor bank safety boundary drifted",
        },
        "primary_cause": {
            "id": (
                "offline_candidate_bank_lifecycle_and_provenance_"
                "not_runtime_schema_valid"
            ),
            "status_mismatch": {
                "observed": "offline_materialized_not_runtime_ready",
                "required": "uncalibrated_simulation_candidate",
            },
            "extra_provenance_key": "p2_q1",
            "max_vel_x_upper_bound_observed": 1.4,
            "upper_bound_widening_is_not_the_failure": True,
        },
        "all_five_candidates_in_memory_single_factor_parse_audit": parser_audit,
        "next_factor": {
            "factor_count": 1,
            "factor_id": (
                "promote_offline_candidate_bank_to_"
                "strict_runtime_candidate_schema_v1"
            ),
            "allowed_differences": ["source_provenance.p2_q1", "status"],
            "behavioral_TEB_field_count_changed": 0,
            "fresh_seed_required": True,
            "old_seed_reuse_allowed": False,
        },
        "active_runtime_processes_after_review": [],
        "performance_claim_allowed": False,
        "next_safe_entry": "independent_offline_review_then_final_round_design",
    }
    return {
        EVIDENCE_RELATIVE: _yaml_bytes(manifest),
        REPORT_RELATIVE: _yaml_bytes(attribution),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P3-I1-D1 output root already exists")
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
        _require(path.is_file(), "expected output absent: {}".format(relative))
        _require(path.read_bytes() == expected, "output drifted: {}".format(relative))


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
                "outputs": [path.as_posix() for path in sorted(files)],
            },
            sort_keys=False,
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
