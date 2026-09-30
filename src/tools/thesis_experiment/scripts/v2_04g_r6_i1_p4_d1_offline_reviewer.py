#!/usr/bin/env python3
"""Independent offline review for the final P4 round-3 design."""

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
STAGE = "V2-04G-P4-D1"
DESIGNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p4_d1_offline_designer.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p4_d1_offline_reviewer.py"
)
OUTPUT_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p4_d1_final_round_design")
LEDGER_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_d1_seed_budget_ledger.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_d1_design_report.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p4_d1_source_freeze.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_d1_independent_review.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p4_d1_offline_closure_freeze.sha256"


class P4D1ReviewError(ValueError):
    """Raised when the final-round design review differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4D1ReviewError(message)


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


def _load_designer():
    path = WORKSPACE / DESIGNER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p4_d1_designer", path)
    _require(spec is not None and spec.loader is not None, "cannot load designer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_review() -> tuple[bytes, bytes]:
    designer = _load_designer()
    rebuilt = designer.build_outputs(WORKSPACE)
    for relative, expected in rebuilt.items():
        path = WORKSPACE / relative
        _require(path.is_file() and path.read_bytes() == expected, "rebuild differs")
    ledger = _load_yaml(WORKSPACE / LEDGER_RELATIVE)
    report = _load_yaml(WORKSPACE / REPORT_RELATIVE)
    source = _load_yaml(WORKSPACE / SOURCE_FREEZE_RELATIVE)
    _require(
        ledger["global_program"]["simulation_capacity_total"] == 2200
        and ledger["global_program"]["round_1"]
        == {"capacity": 792, "consumed": 2, "retired": 790}
        and ledger["global_program"]["round_2"]
        == {"capacity": 704, "consumed": 1, "retired": 703}
        and ledger["global_program"]["round_3"]["capacity"] == 704,
        "global budget ledger differs",
    )
    groups = ledger["seed_partitions"]
    all_seeds = [seed for values in groups.values() for seed in values]
    _require(
        len(all_seeds) == len(set(all_seeds))
        and min(all_seeds) >= 5901
        and max(all_seeds) <= 6080
        and ledger["I1_identity"]["seed"] == 5924,
        "fresh seed ledger differs",
    )
    _require(
        report["factor_count"] == 1
        and report["behavioral_TEB_field_count_changed"] == 0
        and report["round_3_maximum_units"] == 704
        and report["global_program_terminal_after_this_round"] is True
        and report["held_out_test_accessed"] is False,
        "single factor or terminal round differs",
    )
    _require(
        source["resource_count"] == len(source["resources"]) and not source["unresolved"],
        "source freeze differs",
    )
    reviewed_paths = [
        designer.CONTRACT_RELATIVE,
        DESIGNER_RELATIVE,
        REVIEWER_RELATIVE,
        LEDGER_RELATIVE,
        SOURCE_FREEZE_RELATIVE,
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
        "status": "p4_d1_independent_offline_final_round_design_pass",
        "offline_only": True,
        "execution_authorized": False,
        "formal_result": False,
        "runtime_ready": False,
        "global_capacity": 2200,
        "prior_capacity_closed": 1496,
        "final_round_capacity": 704,
        "factor_count": 1,
        "behavioral_TEB_field_count_changed": 0,
        "fresh_partitions_disjoint": True,
        "held_out_test_accessed": False,
        "reviewed_resources": reviewed,
        "reviewed_resource_count": len(reviewed),
        "next_safe_entry": "offline_promoted_bank_implementation_without_execution",
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
