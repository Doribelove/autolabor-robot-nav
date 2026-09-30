#!/usr/bin/env python3
"""Independent offline review for the frozen P3-I1 failure attribution."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P3-I1-D1"
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_d1_failure_attribution.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_d1_offline_reviewer.py"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p3_i1_d1_failure_attribution"
)
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_i1_d1_failure_attribution_report.yaml"
EVIDENCE_RELATIVE = OUTPUT_ROOT / "p3_i1_d1_source_evidence_freeze.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_i1_d1_independent_review.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p3_i1_d1_offline_closure_freeze.sha256"


class P3I1D1ReviewError(ValueError):
    """Raised when independent reconstruction differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3I1D1ReviewError(message)


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
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _load_analyzer():
    path = WORKSPACE / ANALYZER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p3_i1_d1_analyzer", path)
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
    report = _load_yaml(WORKSPACE / REPORT_RELATIVE)
    evidence = _load_yaml(WORKSPACE / EVIDENCE_RELATIVE)
    _require(
        report["status"] == "offline_failure_attribution_pass"
        and report["terminal_accounting"]
        == {
            "round": 2,
            "round_capacity": 704,
            "actual_consumed": 1,
            "completed": 0,
            "program_capacity_retired": 703,
            "total_closed": 704,
            "reusable": 0,
        },
        "terminal round accounting differs",
    )
    _require(
        report["prior_factor_boundary_result"]["runtime_semantic_profile_alignment"]
        == "passed"
        and report["primary_cause"]["upper_bound_widening_is_not_the_failure"] is True
        and report["next_factor"]["factor_count"] == 1
        and report["next_factor"]["behavioral_TEB_field_count_changed"] == 0,
        "attribution or single-factor boundary differs",
    )
    audits = report["all_five_candidates_in_memory_single_factor_parse_audit"]
    _require(
        len(audits) == 5
        and all(row["in_memory_promoted_parse"] == "pass" for row in audits)
        and all(
            row["allowed_differences"] == ["source_provenance.p2_q1", "status"]
            for row in audits
        ),
        "five-candidate parser audit differs",
    )
    _require(
        evidence["execution_file_count"] == 25
        and evidence["resource_count"] == len(evidence["resources"])
        and not evidence["unresolved"],
        "evidence freeze differs",
    )
    _require(not _active_runtime_processes(), "ROS/Gazebo process remains active")

    reviewed = [
        {
            "path": relative.as_posix(),
            "sha256": _sha((WORKSPACE / relative).read_bytes()),
            "size_bytes": (WORKSPACE / relative).stat().st_size,
        }
        for relative in (
            analyzer.CONTRACT_RELATIVE,
            ANALYZER_RELATIVE,
            REVIEWER_RELATIVE,
            EVIDENCE_RELATIVE,
            REPORT_RELATIVE,
        )
    ]
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p3_i1_d1_independent_offline_failure_attribution_pass",
        "offline_only": True,
        "execution_authorized": False,
        "formal_result": False,
        "runtime_ready": False,
        "terminal_round_2_capacity_closed": 704,
        "terminal_round_2_consumed": 1,
        "terminal_round_2_retired": 703,
        "all_five_promoted_candidates_strict_parse_pass": True,
        "factor_count": 1,
        "behavioral_TEB_field_count_changed": 0,
        "held_out_test_accessed": False,
        "active_runtime_processes": [],
        "reviewed_resources": reviewed,
        "reviewed_resource_count": len(reviewed),
        "next_safe_entry": "new_offline_final_round_design_without_execution",
    }
    review["logical_sha256"] = _canonical_sha(review)
    review_bytes = yaml.safe_dump(review, sort_keys=False).encode("utf-8")
    closure_rows = [
        "{}  {}".format(_sha((WORKSPACE / relative).read_bytes()), relative.as_posix())
        for relative in (
            analyzer.CONTRACT_RELATIVE,
            ANALYZER_RELATIVE,
            REVIEWER_RELATIVE,
            EVIDENCE_RELATIVE,
            REPORT_RELATIVE,
        )
    ]
    closure_rows.append("{}  {}".format(_sha(review_bytes), REVIEW_RELATIVE.as_posix()))
    closure_bytes = ("\n".join(closure_rows) + "\n").encode("utf-8")
    return review_bytes, closure_bytes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    review_bytes, closure_bytes = build_review()
    outputs = {
        REVIEW_RELATIVE: review_bytes,
        CLOSURE_RELATIVE: closure_bytes,
    }
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
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
