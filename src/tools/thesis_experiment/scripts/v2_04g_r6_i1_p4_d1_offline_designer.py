#!/usr/bin/env python3
"""Materialize the final finite P4 round-3 single-factor design offline."""

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
STAGE = "V2-04G-P4-D1"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p4_d1_offline_final_round_design_contract.yaml"
)
DESIGNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p4_d1_offline_designer.py"
)
OUTPUT_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p4_d1_final_round_design")
LEDGER_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_d1_seed_budget_ledger.yaml"
REPORT_RELATIVE = OUTPUT_ROOT / "v2_04g_p4_d1_design_report.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p4_d1_source_freeze.yaml"


class P4D1DesignError(ValueError):
    """Raised when the final finite design differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P4D1DesignError(message)


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
    prior = contract["prior_round_terminal_accounting"]
    _require(
        prior["total_prior_capacity_closed"] == 1496
        and prior["total_prior_consumed"] == 3
        and prior["total_prior_retired"] == 1493
        and prior["round_1"]["capacity"] + prior["round_2"]["capacity"] == 1496,
        "prior round accounting differs",
    )
    budget = contract["global_round_3_budget"]
    phases = budget["phase_allocation"]
    _require(
        budget["simulation_evidence_units_max"] == 704
        and budget["phase_maximum_sum"] == 704
        and sum(
            (
                phases["semantic_preflight_I1"]["maximum_units"],
                phases["qualification_and_calibration_C1"]["maximum_additional_units"],
                phases["held_out_test_T1"]["maximum_units"],
            )
        )
        == 704
        and prior["total_prior_capacity_closed"] + 704 == 2200,
        "global budget arithmetic differs",
    )
    partitions = contract["round_3_partitions"]
    calibration = [*range(5901, 5909), *range(5909, 5916), *range(5916, 5923)]
    qualification = list(range(5923, 5935))
    compile_support = list(range(5935, 5941))
    held_out = list(range(5991, 6081))
    groups = {
        "calibration": calibration,
        "qualification": qualification,
        "compile_support_only": compile_support,
        "held_out_test": held_out,
    }
    _require(
        [len(calibration), len(qualification), len(compile_support), len(held_out)]
        == [22, 12, 6, 90],
        "seed counts differ",
    )
    all_seeds = [seed for values in groups.values() for seed in values]
    _require(
        len(all_seeds) == len(set(all_seeds))
        and all(5901 <= seed <= 6090 for seed in all_seeds),
        "seed partitions overlap or escape namespace",
    )
    historical = set(range(5001, 5791))
    _require(not (set(all_seeds) & historical), "historical seed reused")
    _require(
        partitions["calibration"]["paired_evidence_units"]
        == 22 * len(partitions["calibration"]["methods"])
        == 242
        and partitions["qualification"]["evidence_units"] == 12
        and partitions["held_out_test"]["maximum_evidence_units"] == 450,
        "phase matrix arithmetic differs",
    )
    factor = contract["single_new_factor"]
    _require(
        factor["factor_count"] == 1
        and factor["behavioral_TEB_field_count_changed"] == 0
        and factor["TEB_parameter_value_or_bound_changed"] is False
        and set(factor["allowed_differences_per_circle_candidate_bank"])
        == {"status", "source_provenance.p2_q1"},
        "single factor differs",
    )
    ledger = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "finite_final_round_seed_and_budget_ledger_frozen",
        "execution_authorized": False,
        "global_program": {
            "simulation_capacity_total": 2200,
            "round_1": {"capacity": 792, "consumed": 2, "retired": 790},
            "round_2": {"capacity": 704, "consumed": 1, "retired": 703},
            "round_3": {"capacity": 704, "consumed": 0, "retired": 0},
            "capacity_after_round_3": 0,
            "budget_expansion_or_transfer_allowed": False,
        },
        "round_3": {
            "maximum_simulation_units": 704,
            "training_steps": 0,
            "one_sided_familywise_alpha": 0.010,
            "phase_units": {"I1": 1, "C1_additional": 253, "T1_maximum": 450},
            "phase_sum": 704,
            "unused_unit_recycling_allowed": False,
        },
        "seed_partitions": groups,
        "seed_partition_digest": _canonical_sha(groups),
        "I1_identity": {
            "seed": 5924,
            "probe": "high_speed_to_conflict_overlay_transition",
            "candidate_speed_mps": 1.25,
            "counts_toward_qualification_matrix": True,
        },
        "held_out_test_accessed": False,
    }
    source_paths = [CONTRACT_RELATIVE, DESIGNER_RELATIVE]
    source_paths.extend(Path(binding["path"]) for binding in contract["source_bindings"].values())
    unique = {path.as_posix(): _receipt(path) for path in source_paths}
    source_freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_final_round_design_source_freeze",
        "resources": [unique[key] for key in sorted(unique)],
        "resource_count": len(unique),
        "unresolved": [],
    }
    source_freeze["logical_sha256"] = _canonical_sha(source_freeze)
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "offline_final_round_single_factor_design_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "factor_count": 1,
        "factor_id": factor["factor_id"],
        "behavioral_TEB_field_count_changed": 0,
        "fresh_seed_partitions_disjoint": True,
        "global_program_capacity": 2200,
        "prior_capacity_closed": 1496,
        "round_3_maximum_units": 704,
        "phase_maximum_sum": 704,
        "training_steps": 0,
        "held_out_test_accessed": False,
        "global_program_terminal_after_this_round": True,
        "statistical_gate": contract["held_out_statistics"],
        "safety_and_integrity_gate": contract["safety_and_integrity_gate"],
        "next_safe_entry": (
            "independent_offline_review_then_promoted_bank_implementation_without_execution"
        ),
    }
    return {
        LEDGER_RELATIVE: _yaml_bytes(ledger),
        SOURCE_FREEZE_RELATIVE: _yaml_bytes(source_freeze),
        REPORT_RELATIVE: _yaml_bytes(report),
    }


def _materialize(files: Mapping[Path, bytes], workspace: Path) -> None:
    target = workspace / OUTPUT_ROOT
    _require(not target.exists(), "P4-D1 output root already exists")
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
                "outputs": [path.as_posix() for path in sorted(files)],
            },
            sort_keys=False,
        ).strip()
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
