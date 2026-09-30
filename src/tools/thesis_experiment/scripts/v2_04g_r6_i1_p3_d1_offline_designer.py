#!/usr/bin/env python3
"""Materialize the finite P3 round-2 identity-alignment design offline."""

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
STAGE = "V2-04G-P3-D1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p3_d1_offline_round2_design_contract.yaml"
)
DESIGNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_d1_offline_designer.py"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_d1_round2_design"
)
LEDGER_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_d1_seed_budget_ledger.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_d1_design_report.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p3_d1_source_freeze.yaml"


class P3D1DesignError(ValueError):
    """Raised when the finite round-2 design differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3D1DesignError(message)


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


def _receipt(relative: Path) -> Dict[str, Any]:
    payload = (WORKSPACE / relative).read_bytes()
    return {
        "path": relative.as_posix(),
        "sha256": _sha(payload),
        "size_bytes": len(payload),
    }


def _verify_sources(contract: Mapping[str, Any]) -> None:
    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("offline_only") is True, "contract not offline")
    _require(contract.get("execution_authorized") is False, "design authorizes execution")
    for label, binding in contract["source_bindings"].items():
        path = WORKSPACE / binding["path"]
        _require(path.is_file(), "source absent: {}".format(label))
        _require(_sha(path.read_bytes()) == binding["sha256"], "source SHA differs")


def build_outputs(workspace: Path = WORKSPACE) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    _verify_sources(contract)
    budget = contract["global_round_2_budget"]
    phases = budget["phase_allocation"]
    _require(
        budget["simulation_evidence_units_max"] == 704
        and budget["phase_maximum_sum"] == 704
        and sum(
            (
                phases["semantic_preflight_I1"]["maximum_units"],
                phases["qualification_and_calibration_C1"][
                    "maximum_additional_units"
                ],
                phases["held_out_test_T1"]["maximum_units"],
            )
        )
        == 704,
        "phase budget arithmetic differs",
    )
    partitions = contract["round_2_partitions"]
    calibration = [
        *range(5601, 5609),
        *range(5609, 5616),
        *range(5616, 5623),
    ]
    qualification = list(range(5623, 5635))
    compile_support = list(range(5635, 5641))
    held_out = list(range(5691, 5781))
    groups = {
        "calibration": calibration,
        "qualification": qualification,
        "compile_support_only": compile_support,
        "held_out_test": held_out,
    }
    _require(
        len(calibration) == 22
        and len(set(calibration)) == 22
        and len(qualification) == 12
        and len(compile_support) == 6
        and len(held_out) == 90,
        "seed counts differ",
    )
    all_seeds = [seed for values in groups.values() for seed in values]
    _require(len(all_seeds) == len(set(all_seeds)), "seed partitions overlap")
    _require(
        all(5601 <= seed <= 5790 for seed in all_seeds),
        "seed outside round-2 namespace",
    )
    _require(5624 in qualification, "I1 seed not in qualification")
    _require(
        not ({5331, 5332} & set(all_seeds)),
        "terminal C1-R1 seed reused",
    )
    _require(
        partitions["calibration"]["paired_evidence_units"]
        == 22 * len(partitions["calibration"]["methods"])
        == 242,
        "calibration matrix arithmetic differs",
    )
    _require(
        partitions["qualification"]["evidence_units"] == 12
        and partitions["held_out_test"]["maximum_evidence_units"] == 450,
        "qualification/test arithmetic differs",
    )
    factor = contract["single_new_factor"]
    _require(
        factor["factor_count"] == 1
        and factor["behavioral_TEB_field_count_changed"] == 0
        and factor["amendment"]["runtime_profile_id"]
        == "r6_semantics_circle_contact",
        "single factor differs",
    )
    ledger = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "finite_round2_seed_and_budget_ledger_frozen",
        "execution_authorized": False,
        "round_1_closed_capacity": {
            "total": 792,
            "consumed": 2,
            "retired": 790,
            "reusable": 0,
        },
        "round_2": {
            "maximum_simulation_units": 704,
            "training_steps_preregistered": 0,
            "one_sided_familywise_alpha": 0.015,
            "phase_units": {"I1": 1, "C1_additional": 253, "T1_maximum": 450},
            "phase_sum": 704,
            "unused_unit_recycling_allowed": False,
        },
        "seed_partitions": groups,
        "seed_partition_digest": _canonical_sha(groups),
        "I1_identity": {
            "seed": 5624,
            "probe": "high_speed_to_conflict_overlay_transition",
            "candidate_speed_mps": 1.25,
            "counts_toward_qualification_matrix": True,
        },
        "held_out_test_accessed": False,
    }
    source_paths = [CONTRACT_RELATIVE, DESIGNER_RELATIVE]
    source_paths.extend(
        Path(binding["path"]) for binding in contract["source_bindings"].values()
    )
    unique = {path.as_posix(): _receipt(path) for path in source_paths}
    source_freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_design_source_freeze",
        "resources": [unique[key] for key in sorted(unique)],
        "resource_count": len(unique),
        "unresolved": [],
    }
    source_freeze["logical_sha256"] = _canonical_sha(source_freeze)
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_round2_single_factor_design_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "factor_count": 1,
        "factor_id": factor["factor_id"],
        "behavioral_TEB_field_count_changed": 0,
        "fresh_seed_partitions_disjoint": True,
        "round_2_maximum_units": 704,
        "phase_maximum_sum": 704,
        "training_steps": 0,
        "held_out_test_accessed": False,
        "statistical_gate": contract["held_out_statistics"],
        "safety_and_integrity_gate": contract["safety_and_integrity_gate"],
        "next_safe_entry": (
            "independent_offline_review_then_I1_implementation_without_execution"
        ),
    }
    return {
        LEDGER_RELATIVE: _yaml_bytes(ledger),
        SOURCE_FREEZE_RELATIVE: _yaml_bytes(source_freeze),
        REPORT_RELATIVE: _yaml_bytes(report),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P3-D1 output root already exists")
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
        _require(path.is_file() and path.read_bytes() == expected, "P3-D1 output differs")


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
        "P3-D1 design {}: offline_round2_single_factor_design_pass".format(
            "materialized" if args.materialize else "verified"
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
