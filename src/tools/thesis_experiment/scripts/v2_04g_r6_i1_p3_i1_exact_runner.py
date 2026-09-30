#!/usr/bin/env python3
"""Execute the unique one-unit P3-I1 semantic preflight release."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from datetime import datetime, timezone

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_ROOT = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))

from thesis_experiment.v2_04g_r6_i1_p3_i1_release import (  # noqa: E402
    P3I1ReleaseError,
    validate_release,
)


STAGE = "V2-04G-P3-I1"
OUTPUT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_i1_execution"
)
OUTPUT_ROOT = WORKSPACE / OUTPUT_ROOT_RELATIVE
ATTEMPTS_ROOT = OUTPUT_ROOT / "attempts"
JOURNAL_ROOT = OUTPUT_ROOT / "journals"
CONSUMPTION_ROOT = OUTPUT_ROOT / "consumption"
STAGE_REPORT = OUTPUT_ROOT / "p3_i1_stage_report.yaml"
SKIPPED_LEDGER = OUTPUT_ROOT / "p3_i1_skipped_identities.yaml"
FAILURE_INDEX = OUTPUT_ROOT / "p3_i1_failure_index.yaml"
NORMALIZED_RESULT = OUTPUT_ROOT / "normalized_preflight_result.json"
REPLAY_REPORT = OUTPUT_ROOT / "p3_i1_raw_replay_report.yaml"

METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)
Q1_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_activation_probe_listener.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_assess.py"
)
FROZEN_C1_RUNNER = Path(__file__).with_name(
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)


class P3I1ExecutionError(RuntimeError):
    """Terminal, non-retriable P3-I1 execution failure."""


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise P3I1ExecutionError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = _load("_p3_i1_frozen_c1_runner", FROZEN_C1_RUNNER)
ASSESSOR = _load("_p3_i1_assessor", WORKSPACE / ASSESSOR_RELATIVE)


def _utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _runtime_profile_id(row):
    if not BASE._semantic(row):  # pylint: disable=protected-access
        return "fixed_teb_reference"
    return "r6_semantics_circle_contact"


def _configure_base():
    BASE.STAGE = STAGE
    BASE.OUTPUT_ROOT_RELATIVE = OUTPUT_ROOT_RELATIVE
    BASE.OUTPUT_ROOT = OUTPUT_ROOT
    BASE.ATTEMPTS_ROOT = ATTEMPTS_ROOT
    BASE.JOURNAL_ROOT = JOURNAL_ROOT
    BASE.CONSUMPTION_ROOT = CONSUMPTION_ROOT
    BASE.STAGE_REPORT = STAGE_REPORT
    BASE.SKIPPED_LEDGER = SKIPPED_LEDGER
    BASE.FAILURE_INDEX = FAILURE_INDEX
    BASE.NORMALIZED_MATRIX = NORMALIZED_RESULT
    BASE.REPLAY_REPORT = REPLAY_REPORT
    BASE.LAUNCH_RELATIVE = LAUNCH_RELATIVE
    BASE.QUALIFICATION_RELATIVE = QUALIFICATION_RELATIVE
    BASE.CONTROL_RELATIVE = CONTROL_RELATIVE
    BASE.LISTENER_RELATIVE = LISTENER_RELATIVE
    BASE.ASSESSOR_RELATIVE = ASSESSOR_RELATIVE
    BASE.ASSESSOR = ASSESSOR
    BASE._profile_id = _runtime_profile_id  # pylint: disable=protected-access
    BASE.validate_release = validate_release


_configure_base()
P1 = BASE.P1


def configure_static_bindings(methods, closure):
    _configure_base()
    BASE.METHOD_BINDINGS = {
        row["method_id"]: row for row in methods["methods"]
    }
    BASE._CURRENT_CLOSURE = dict(closure)  # pylint: disable=protected-access


def identity(row):
    return BASE._identity(row)  # pylint: disable=protected-access


def runtime(row, closure):
    return BASE._runtime(row, closure)  # pylint: disable=protected-access


def base_command(row, scene, world_path, runtime_binding, executables):
    return BASE._base_command(  # pylint: disable=protected-access
        row, scene, world_path, runtime_binding, executables
    )


def transaction_command(row, runtime_binding, executables):
    return BASE._transaction_command(  # pylint: disable=protected-access
        row, runtime_binding, executables
    )


def probe_command(row, output, executables):
    return BASE._probe_command(row, output, executables)  # pylint: disable=protected-access


def execute(expected_release_sha256):
    _configure_base()
    source_environment = dict(os.environ)
    validation = validate_release(
        expected_release_sha256, WORKSPACE, require_fresh_state=True
    )
    P1._assert_host_clean()  # pylint: disable=protected-access
    executables = P1._executables()  # pylint: disable=protected-access
    closure = {
        receipt["path"]: receipt["sha256"]
        for receipt in validation["closure"]["local_resources"]
    }
    for relative in (
        LAUNCH_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
        ASSESSOR_RELATIVE,
        METHODS_RELATIVE,
        Q1_CONTRACT_RELATIVE,
    ):
        P1._rehash_relative(relative, closure)  # pylint: disable=protected-access
    methods = yaml.safe_load(
        P1._rehash_relative(METHODS_RELATIVE, closure)  # pylint: disable=protected-access
    )
    BASE.METHOD_BINDINGS = {
        row["method_id"]: row for row in methods["methods"]
    }
    q1_contract = yaml.safe_load(
        P1._rehash_relative(Q1_CONTRACT_RELATIVE, closure)  # pylint: disable=protected-access
    )
    BASE.THRESHOLDS = q1_contract["qualification_thresholds"]
    BASE._CURRENT_CLOSURE = closure  # pylint: disable=protected-access
    BASE._configure_p1_primitives()  # pylint: disable=protected-access

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    ATTEMPTS_ROOT.mkdir()
    JOURNAL_ROOT.mkdir()
    CONSUMPTION_ROOT.mkdir()
    BASE._write_skipped([])  # pylint: disable=protected-access
    BASE._write_failure_index([])  # pylint: disable=protected-access
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "running_unique_one_unit_semantic_preflight",
        "release_sha256": validation["release_sha256"],
        "schedule_sha256": validation["schedule_sha256"],
        "schedule_row_digest": validation["schedule"]["canonical_row_digest"],
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "evidence_units_consumed": 0,
        "evidence_units_completed": 0,
        "evidence_units_skipped": 0,
        "evidence_units_forfeited": 0,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "held_out_test_accessed": False,
        "runtime_profile_id": "r6_semantics_circle_contact",
        "evidence_candidate_method_id": "circle_contact_v125",
        "started_at_utc": _utc_now(),
        "completed_at_utc": None,
        "attempt_ledger": [],
        "terminal_integrity_failures": [],
        "future_round_2_units_authorized": 0,
        "future_C1_implementation_eligible": False,
    }
    P1._atomic_yaml(STAGE_REPORT, report)  # pylint: disable=protected-access
    row = validation["schedule"]["rows"][0]
    try:
        normalized = BASE._run_qualification(  # pylint: disable=protected-access
            row,
            validation,
            report,
            source_environment,
            executables,
        )
    except BaseException as exc:
        reason = P1._safe_reason(exc, source_environment)  # pylint: disable=protected-access
        report.update(
            {
                "status": "terminal_integrity_failure",
                "completed_at_utc": _utc_now(),
                "evidence_units_forfeited": max(
                    0, 1 - report["evidence_units_consumed"]
                ),
                "terminal_integrity_failures": [reason],
                "future_C1_implementation_eligible": False,
            }
        )
        P1._atomic_yaml(STAGE_REPORT, report)  # pylint: disable=protected-access
        BASE._write_failure_index(  # pylint: disable=protected-access
            [
                {
                    "sequence": 1,
                    "identity": identity(row),
                    "kind": "terminal_integrity_failure",
                    "reason": reason,
                }
            ]
        )
        P1._wait_host_clean()  # pylint: disable=protected-access
        raise P3I1ExecutionError(reason) from None
    normalized_payload = (
        json.dumps(
            normalized,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    P1._exclusive_bytes(NORMALIZED_RESULT, normalized_payload)  # pylint: disable=protected-access
    journal_path = JOURNAL_ROOT / "sequence_0001.yaml"
    replay = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "complete_one_unit_raw_replay",
        "release_sha256": validation["release_sha256"],
        "journal_path": str(journal_path.relative_to(WORKSPACE)),
        "journal_sha256": _sha(journal_path.read_bytes()),
        "normalized_sha256": _sha(normalized_payload),
        "probe_pass": bool(normalized["pass"]),
        "runtime_profile_id": "r6_semantics_circle_contact",
        "evidence_candidate_method_id": "circle_contact_v125",
        "held_out_test_accessed": False,
    }
    P1._exclusive_bytes(  # pylint: disable=protected-access
        REPLAY_REPORT,
        yaml.safe_dump(replay, sort_keys=False).encode("utf-8"),
    )
    report.update(
        {
            "status": (
                "semantic_preflight_and_probe_pass"
                if normalized["pass"]
                else "semantic_preflight_integrity_pass_probe_negative"
            ),
            "completed_at_utc": _utc_now(),
            "formal_result": False,
            "normalized_result_sha256": _sha(normalized_payload),
            "raw_replay_report_sha256": _sha(REPLAY_REPORT.read_bytes()),
            "probe_pass": bool(normalized["pass"]),
            "future_C1_implementation_eligible": bool(normalized["pass"]),
        }
    )
    P1._atomic_yaml(STAGE_REPORT, report)  # pylint: disable=protected-access
    P1._assert_host_clean()  # pylint: disable=protected-access
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; use the offline reviewer for validation")
    try:
        report = execute(args.release_sha256)
    except (P3I1ReleaseError, P3I1ExecutionError, BASE.C1ExecutionError) as exc:
        print("P3-I1 terminal failure: {}".format(exc), file=sys.stderr)
        return 2
    print(
        yaml.safe_dump(
            {
                "status": report["status"],
                "consumed": report["evidence_units_consumed"],
                "completed": report["evidence_units_completed"],
                "probe_pass": report["probe_pass"],
                "future_C1_implementation_eligible": report[
                    "future_C1_implementation_eligible"
                ],
            },
            sort_keys=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
