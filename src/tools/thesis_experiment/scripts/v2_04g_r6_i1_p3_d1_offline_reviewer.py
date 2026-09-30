#!/usr/bin/env python3
"""Independent review of the finite P3 round-2 offline design."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P3-D1"
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_d1_round2_design"
)
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_d1_independent_review.yaml"
CLOSURE_RELATIVE = OUTPUT_ROOT / "p3_d1_offline_closure_freeze.sha256"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p3_d1_offline_round2_design_contract.yaml"
)
DESIGNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_d1_offline_designer.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_d1_offline_reviewer.py"
)


class P3D1ReviewError(ValueError):
    """Raised when the P3 round-2 design is inconsistent."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3D1ReviewError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs")
    return value


def _load_designer():
    path = WORKSPACE / DESIGNER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p3_d1_designer_review", path)
    _require(spec is not None and spec.loader is not None, "cannot load designer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DESIGNER = _load_designer()


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


def build_review(workspace: Path = WORKSPACE):
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    first = DESIGNER.build_outputs(workspace)
    second = DESIGNER.build_outputs(workspace)
    _require(first == second, "designer is not deterministic")
    for relative, payload in first.items():
        path = workspace / relative
        _require(path.is_file() and path.read_bytes() == payload, "design output differs")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    ledger = _load_yaml(workspace / DESIGNER.LEDGER_RELATIVE)
    report = _load_yaml(workspace / DESIGNER.REPORT_RELATIVE)
    freeze = _load_yaml(workspace / DESIGNER.SOURCE_FREEZE_RELATIVE)
    _require(
        contract["single_new_factor"]["factor_count"] == 1
        and contract["single_new_factor"]["behavioral_TEB_field_count_changed"] == 0,
        "factor count differs",
    )
    _require(
        ledger["round_1_closed_capacity"]
        == {"total": 792, "consumed": 2, "retired": 790, "reusable": 0},
        "round-1 closure differs",
    )
    _require(
        ledger["round_2"]["maximum_simulation_units"] == 704
        and ledger["round_2"]["phase_sum"] == 704
        and ledger["round_2"]["phase_units"]
        == {"I1": 1, "C1_additional": 253, "T1_maximum": 450},
        "round-2 budget differs",
    )
    groups = ledger["seed_partitions"]
    all_seeds = [seed for values in groups.values() for seed in values]
    _require(
        len(all_seeds) == len(set(all_seeds))
        and all(5601 <= seed <= 5790 for seed in all_seeds)
        and not ({5331, 5332} & set(all_seeds)),
        "seed isolation differs",
    )
    calibration = set(groups["calibration"])
    qualification = set(groups["qualification"])
    held_out = set(groups["held_out_test"])
    _require(
        len(calibration) == 22
        and len(qualification) == 12
        and len(held_out) == 90
        and 5624 in qualification,
        "partition sizes differ",
    )
    _require(
        report["status"] == "offline_round2_single_factor_design_pass"
        and report["execution_authorized"] is False
        and report["held_out_test_accessed"] is False,
        "design report differs",
    )
    _require(freeze["unresolved"] == [], "source freeze unresolved")
    for receipt in freeze["resources"]:
        path = workspace / receipt["path"]
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "source freeze receipt differs",
        )
    program = _load_yaml(
        workspace
        / contract["source_bindings"]["global_program_contract"]["path"]
    )
    round2 = program["global_research_budget"]["rounds"][1]
    _require(
        round2["round"] == 2
        and round2["simulation_evidence_units_max"] == 704
        and round2["training_environment_steps_max"] == 60000
        and round2["one_sided_alpha"] == 0.015
        and round2["seed_namespace"] == [5601, 5790],
        "inherited global round-2 contract differs",
    )
    processes = _active_runtime_processes()
    _require(processes == [], "runtime process active")
    reviewer_payload = (workspace / REVIEWER_RELATIVE).read_bytes()
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "independent_round2_single_factor_design_review_pass",
        "review_result": "pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "factor_count_verified": 1,
        "behavioral_TEB_field_count_changed": 0,
        "trace_attributed_factor_verified": (
            "separate_runtime_semantic_profile_from_evidence_candidate_identity_v1"
        ),
        "round_1_capacity_closed": 792,
        "round_2_maximum_units": 704,
        "round_2_phase_sum": 704,
        "seed_partitions_disjoint_and_fresh": True,
        "I1_fresh_seed": 5624,
        "held_out_test_accessed": False,
        "source_resources_verified": freeze["resource_count"],
        "side_effect_audit": {
            "active_runtime_processes": processes,
            "ROS_or_Gazebo_started_by_review": False,
        },
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": _sha(reviewer_payload),
        },
        "next_safe_entry": (
            "implement_I1_offline_then_independently_review_an_exact_one_unit_release"
        ),
    }


def _closure_payload(review_payload: bytes) -> bytes:
    paths = [
        CONTRACT_RELATIVE,
        DESIGNER_RELATIVE,
        REVIEWER_RELATIVE,
        DESIGNER.LEDGER_RELATIVE,
        DESIGNER.REPORT_RELATIVE,
        DESIGNER.SOURCE_FREEZE_RELATIVE,
    ]
    lines = [
        "{}  {}\n".format(_sha((WORKSPACE / path).read_bytes()), path.as_posix())
        for path in sorted(paths, key=lambda value: value.as_posix())
    ]
    lines.append("{}  {}\n".format(_sha(review_payload), REVIEW_RELATIVE.as_posix()))
    return "".join(lines).encode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    first = build_review(workspace)
    second = build_review(workspace)
    _require(first == second, "review is not deterministic")
    review_payload = _yaml_bytes(first)
    closure_payload = _closure_payload(review_payload)
    review_path = workspace / REVIEW_RELATIVE
    closure_path = workspace / CLOSURE_RELATIVE
    if args.materialize:
        _require(not review_path.exists() and not closure_path.exists(), "review exists")
        for path, payload in ((review_path, review_payload), (closure_path, closure_payload)):
            with path.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
    else:
        _require(review_path.read_bytes() == review_payload, "review differs")
        _require(closure_path.read_bytes() == closure_payload, "closure differs")
    print(
        "P3-D1 independent review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
