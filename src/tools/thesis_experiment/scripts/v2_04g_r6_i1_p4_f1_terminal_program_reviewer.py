#!/usr/bin/env python3
"""Independent offline review of the terminal finite-search conclusion."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P4-F1"
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_f1_terminal_program_closure.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_f1_terminal_program_reviewer.py"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_f1_terminal_program_closure"
)
EVIDENCE_RELATIVE = OUTPUT_ROOT / "p4_f1_terminal_evidence_freeze.yaml"
ACCOUNTING_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_f1_global_budget_accounting.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_f1_terminal_conclusion.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_f1_independent_review.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p4_f1_terminal_closure_freeze.sha256"


class P4F1ReviewError(ValueError):
    """Raised when independent terminal review differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4F1ReviewError(message)


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


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs")
    return value


def _load_analyzer():
    path = WORKSPACE / ANALYZER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p4_f1_analyzer", path)
    _require(spec is not None and spec.loader is not None, "cannot load analyzer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _active_runtime_processes() -> list[int]:
    tokens = (b"roscore", b"rosmaster", b"roslaunch", b"gzserver", b"gzclient", b"move_base")
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").lower()
        except OSError:
            continue
        if any(token in command for token in tokens):
            result.append(int(entry.name))
    return result


def build_review() -> tuple[bytes, bytes]:
    analyzer = _load_analyzer()
    rebuilt = analyzer.build_outputs(WORKSPACE)
    for relative, expected in rebuilt.items():
        path = WORKSPACE / relative
        _require(path.is_file() and path.read_bytes() == expected, "rebuild differs")
    evidence = _load_yaml(WORKSPACE / EVIDENCE_RELATIVE)
    accounting = _load_yaml(WORKSPACE / ACCOUNTING_RELATIVE)
    report = _load_yaml(WORKSPACE / REPORT_RELATIVE)
    _require(
        accounting["simulation_capacity_total"] == 2200
        and accounting["simulation_units_consumed"] == 4
        and accounting["simulation_units_retired"] == 2196
        and accounting["simulation_units_remaining"] == 0
        and accounting["training_steps_remaining"] == 0,
        "global accounting differs",
    )
    _require(
        report["status"]
        == "global_budget_exhausted_obvious_improvement_not_demonstrated"
        and report["program_terminal"] is True
        and report["stop_performance_search"] is True
        and report["candidate_winner_frozen"] is False
        and report["positive_performance_claim_allowed"] is False
        and report["chinese_conclusion"] == "未证明明显提升",
        "terminal conclusion differs",
    )
    gates = report["requested_obvious_effect_gate"]
    _require(
        gates["point_reduction_fraction_at_least"] == 0.10
        and gates["one_sided_lower_reduction_fraction_at_least"] == 0.05
        and gates["historical_point_gate_pass"] is False
        and gates["historical_lower_gate_pass"] is False
        and gates["post_P1_integrity_gate_pass"] is False,
        "terminal effect gate differs",
    )
    _require(
        report["terminal_failure_attribution"]["direct_cause"]
        == "ros_package_resolution_failed_before_service_readiness"
        and report["terminal_failure_attribution"]["promoted_bank_live_runtime_behavior_reached"]
        is False
        and evidence["P4_I1_execution_file_count"] == 8
        and evidence["resource_count"] == len(evidence["resources"])
        and not evidence["unresolved"]
        and not _active_runtime_processes(),
        "terminal evidence or process isolation differs",
    )
    reviewed_paths = [
        analyzer.CONTRACT_RELATIVE,
        ANALYZER_RELATIVE,
        REVIEWER_RELATIVE,
        EVIDENCE_RELATIVE,
        ACCOUNTING_RELATIVE,
        REPORT_RELATIVE,
    ]
    reviewed = [
        {
            "path": path.as_posix(),
            "sha256": _sha((WORKSPACE / path).read_bytes()),
            "size_bytes": (WORKSPACE / path).stat().st_size,
        }
        for path in reviewed_paths
    ]
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p4_f1_independent_terminal_program_closure_pass",
        "offline_only": True,
        "execution_authorized": False,
        "formal_result": False,
        "runtime_ready": False,
        "program_terminal": True,
        "global_simulation_capacity": 2200,
        "simulation_units_consumed": 4,
        "simulation_units_retired": 2196,
        "simulation_units_remaining": 0,
        "training_steps_remaining": 0,
        "obvious_improvement_demonstrated": False,
        "terminal_conclusion": "未证明明显提升",
        "candidate_winner_frozen": False,
        "held_out_test_accessed_after_P1": False,
        "active_runtime_processes": [],
        "reviewed_resources": reviewed,
        "reviewed_resource_count": len(reviewed),
        "next_safe_entry": "none_performance_search_terminal",
    }
    review["logical_sha256"] = _canonical_sha(review)
    review_bytes = yaml.safe_dump(review, sort_keys=False, allow_unicode=True).encode("utf-8")
    closure_rows = [
        "{}  {}".format(_sha((WORKSPACE / path).read_bytes()), path.as_posix())
        for path in reviewed_paths
    ]
    closure_rows.append("{}  {}".format(_sha(review_bytes), REVIEW_RELATIVE.as_posix()))
    closure_bytes = ("\n".join(closure_rows) + "\n").encode("utf-8")
    return review_bytes, closure_bytes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    review_bytes, closure_bytes = build_review()
    outputs = {REVIEW_RELATIVE: review_bytes, CLOSURE_RELATIVE: closure_bytes}
    for relative, payload in outputs.items():
        path = WORKSPACE / relative
        if args.check:
            _require(path.is_file() and path.read_bytes() == payload, "review output drifted")
        else:
            _require(not path.exists(), "review output already exists")
            path.write_bytes(payload)
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
    print(
        yaml.safe_dump(
            {
                "stage": STAGE,
                "status": "check_pass" if args.check else "review_materialized",
                "review_sha256": _sha(review_bytes),
                "closure_sha256": _sha(closure_bytes),
            },
            sort_keys=False,
            allow_unicode=True,
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
