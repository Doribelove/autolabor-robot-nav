#!/usr/bin/env python3
"""Independent offline review of the P2-C1-R1-D1 failure attribution."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1-R1-D1"
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "p2_c1_r1_d1_failure_attribution"
)
REVIEW_RELATIVE = (
    OUTPUT_ROOT / "v2_04g_p2_c1_r1_d1_independent_review.yaml"
)
CLOSURE_RELATIVE = OUTPUT_ROOT / "p2_c1_r1_d1_offline_closure_freeze.sha256"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_r1_d1_offline_failure_attribution_contract.yaml"
)
ANALYZER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_d1_failure_attribution.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_d1_offline_reviewer.py"
)


class C1R1D1ReviewError(ValueError):
    """Raised when an attribution or evidence invariant differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1R1D1ReviewError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs")
    return value


def _load_analyzer():
    path = WORKSPACE / ANALYZER_RELATIVE
    spec = importlib.util.spec_from_file_location("_p2_c1_r1_d1_analyzer_review", path)
    _require(spec is not None and spec.loader is not None, "cannot load analyzer")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ANALYZER = _load_analyzer()


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
    first = ANALYZER.build_outputs(workspace)
    second = ANALYZER.build_outputs(workspace)
    _require(first == second, "analyzer is not deterministic")
    for relative, payload in first.items():
        path = workspace / relative
        _require(path.is_file(), "analyzer output absent")
        _require(path.read_bytes() == payload, "analyzer output differs")
    evidence = _load_yaml(workspace / ANALYZER.EVIDENCE_RELATIVE)
    report = _load_yaml(workspace / ANALYZER.REPORT_RELATIVE)
    _require(
        evidence["status"] == "terminal_C1_R1_source_evidence_frozen"
        and evidence["execution_file_count"] == 45
        and evidence["unresolved"] == [],
        "evidence freeze differs",
    )
    for receipt in evidence["resources"]:
        path = workspace / receipt["path"]
        _require(path.is_file(), "evidence resource absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "evidence receipt differs",
        )
    terminal = report["terminal_C1_R1_accounting"]
    _require(
        report["status"] == "offline_failure_attribution_pass"
        and terminal
        == {
            "authorized": 342,
            "consumed": 2,
            "completed": 1,
            "skipped": 0,
            "forfeited": 340,
            "retry_resume_or_reuse_allowed": False,
            "held_out_test_accessed": False,
        },
        "terminal accounting interpretation differs",
    )
    normalized = report["completed_sequence_1_raw_replay"]["normalized"]
    _require(
        normalized["pass"] is True
        and normalized["collision"] is False
        and normalized["emergency"] is False
        and normalized["normal_projection_rate"] == 0.0
        and abs(normalized["stop_time_s"] - 0.85) < 1.0e-9
        and abs(normalized["stopping_distance_m"] - 0.3624205998809642)
        < 1.0e-12,
        "completed raw replay differs",
    )
    failure = report["failure_sequence_2"]
    _require(
        failure["planned_runtime_profile_id"] == "qualification_overlay_v125"
        and failure["required_aligned_runtime_profile_id"]
        == "r6_semantics_circle_contact"
        and failure["required_supervisor_process_died"] is True
        and failure["unpause_service_not_available_after_required_process_exit"]
        is True,
        "failure attribution evidence differs",
    )
    _require(
        report["attribution"]["primary_cause"]
        == "runtime_evidence_profile_identity_domain_mismatch",
        "primary cause differs",
    )
    factor = report["next_single_factor"]
    _require(
        factor["factor_id"]
        == "separate_runtime_semantic_profile_from_evidence_candidate_identity_v1"
        and factor["behavioral_TEB_parameter_change"] is False
        and factor["other_runtime_scene_threshold_estimator_or_safety_change"]
        is False
        and factor["fresh_seed_required"] is True
        and factor["execution_authorized"] is False,
        "next factor boundary differs",
    )
    processes = _active_runtime_processes()
    _require(processes == [], "runtime process active")
    reviewer_payload = (workspace / REVIEWER_RELATIVE).read_bytes()
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "independent_offline_failure_attribution_review_pass",
        "review_result": "pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "source_evidence_resources_verified": evidence["resource_count"],
        "terminal_accounting_verified": terminal,
        "completed_raw_replay_verified": True,
        "primary_cause_verified": (
            "runtime_evidence_profile_identity_domain_mismatch"
        ),
        "unpause_error_classified_as_downstream_symptom": True,
        "next_factor_count": 1,
        "next_factor_execution_authorized": False,
        "side_effect_audit": {
            "active_runtime_processes": processes,
            "ROS_or_Gazebo_started_by_review": False,
            "held_out_test_accessed": False,
        },
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": _sha(reviewer_payload),
        },
        "next_safe_entry": (
            "preregister_one_finite_fresh_seed_runtime_identity_alignment_factor"
        ),
    }


def _closure_payload(review_payload: bytes) -> bytes:
    resources = [
        CONTRACT_RELATIVE,
        ANALYZER_RELATIVE,
        REVIEWER_RELATIVE,
        ANALYZER.EVIDENCE_RELATIVE,
        ANALYZER.REPORT_RELATIVE,
    ]
    lines = []
    for relative in sorted(resources, key=lambda value: value.as_posix()):
        lines.append(
            "{}  {}\n".format(
                _sha((WORKSPACE / relative).read_bytes()), relative.as_posix()
            )
        )
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
        for path, payload in (
            (review_path, review_payload),
            (closure_path, closure_payload),
        ):
            with path.open("xb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
    else:
        _require(review_path.read_bytes() == review_payload, "review bytes differ")
        _require(closure_path.read_bytes() == closure_payload, "closure bytes differ")
    print(
        "P2-C1-R1-D1 independent review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
