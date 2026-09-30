#!/usr/bin/env python3
"""Independent offline review of P4 runtime-schema Anchor Banks."""

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
STAGE = "V2-04G-P4-Q1"
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p4_q1_offline_builder.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p4_q1_offline_reviewer.py"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p4_q1_runtime_bank_readiness"
)
METHODS_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_method_bindings.yaml"
AUDIT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_construction_audit.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p4_q1_source_freeze.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_readiness_report.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_q1_independent_review.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p4_q1_offline_closure_freeze.sha256"


class P4Q1ReviewError(ValueError):
    """Raised when independent promotion review differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4Q1ReviewError(message)


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


def _load_builder():
    path = WORKSPACE / BUILDER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p4_q1_builder", path)
    _require(spec is not None and spec.loader is not None, "cannot load builder")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_anchor_bank_class():
    root = WORKSPACE / "src/application/teb_mode_manager/src"
    sys.path.insert(0, str(root))
    try:
        from teb_mode_manager.action_pipeline import AnchorBank
    finally:
        sys.path.pop(0)
    return AnchorBank


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
    builder = _load_builder()
    rebuilt = builder.build_outputs(WORKSPACE)
    for relative, expected in rebuilt.items():
        path = WORKSPACE / relative
        _require(path.is_file() and path.read_bytes() == expected, "rebuild differs")
    methods = _load_yaml(WORKSPACE / METHODS_RELATIVE)
    audit = _load_yaml(WORKSPACE / AUDIT_RELATIVE)
    report = _load_yaml(WORKSPACE / REPORT_RELATIVE)
    source = _load_yaml(WORKSPACE / SOURCE_FREEZE_RELATIVE)
    AnchorBank = _load_anchor_bank_class()
    circle = [
        row for row in methods["methods"] if row["method_id"].startswith("circle_contact_")
    ]
    _require(len(circle) == 5 and len(methods["methods"]) == 11, "method count differs")
    parsed = []
    for method in circle:
        receipt = method["materialized_anchor_bank"]
        path = WORKSPACE / receipt["path"]
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"] and len(payload) == receipt["size_bytes"],
            "promoted receipt differs",
        )
        bank = AnchorBank.from_file(path)
        _require(len(bank.parameter_names) == 20, "strict parser result differs")
        parsed.append(method["method_id"])
    _require(
        audit["factor_count"] == 1
        and audit["behavioral_TEB_field_count_changed"] == 0
        and audit["fixed_and_TEB_Tuned_bindings_unchanged"] is True
        and all(
            row["leaf_differences"] == ["source_provenance.p2_q1", "status"]
            and row["parameter_registry_and_values_equal"] is True
            for row in audit["candidate_audits"]
        ),
        "promotion audit differs",
    )
    _require(
        report["strict_parser_pass_count"] == 5
        and report["execution_authorized"] is False
        and report["held_out_test_accessed"] is False
        and source["resource_count"] == len(source["resources"])
        and not source["unresolved"]
        and not _active_runtime_processes(),
        "readiness boundary differs",
    )
    reviewed_paths = [
        builder.CONTRACT_RELATIVE,
        BUILDER_RELATIVE,
        REVIEWER_RELATIVE,
        METHODS_RELATIVE,
        AUDIT_RELATIVE,
        SOURCE_FREEZE_RELATIVE,
        REPORT_RELATIVE,
    ]
    reviewed_paths.extend(Path(row["materialized_anchor_bank"]["path"]) for row in circle)
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
        "status": "p4_q1_independent_offline_runtime_bank_readiness_pass",
        "offline_only": True,
        "execution_authorized": False,
        "formal_result": False,
        "runtime_ready": False,
        "factor_count": 1,
        "behavioral_TEB_field_count_changed": 0,
        "strict_parser_pass_methods": parsed,
        "strict_parser_pass_count": len(parsed),
        "fair_baseline_rule_unchanged": True,
        "held_out_test_accessed": False,
        "active_runtime_processes": [],
        "reviewed_resources": reviewed,
        "reviewed_resource_count": len(reviewed),
        "next_safe_entry": "exact_fresh_seed_I1_release_construction",
    }
    review["logical_sha256"] = _canonical_sha(review)
    review_bytes = yaml.safe_dump(review, sort_keys=False).encode("utf-8")
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
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
