#!/usr/bin/env python3
"""Run the exact six one-attempt R5 TTC activation/coverage episodes."""

import argparse
from collections import Counter
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
ROOT = WORKSPACE / "artifacts/v2/calibration/v2_04g_r5"
READINESS_ROOT = ROOT / "readiness"
SUMMARY_PATH = READINESS_ROOT / "v2_04g_r5_readiness_summary.yaml"
RUNTIME_CONFIG_ROOT = ROOT / "runtime_candidate_configs"
COMPILED_SCENES = ROOT / "ttc_readiness_compiled_scenes"
EPISODE_RUNNER = Path(__file__).with_name("v2_04g_r5_mechanism_episode.py")
LISTENER = Path(__file__).with_name("v2_04g_r5_activation_probe_listener.py")
CANDIDATE_MATERIALIZER = Path(__file__).with_name(
    "v2_04g_r5_candidate_materializer.py"
)
BASE_BATCH = Path(__file__).with_name("v2_04g_r1_calibration_batch.py")
GUARD = Path(__file__).with_name("v2_04g_r5_execution_guard.py")


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_BASE = _load_module("v2_04g_r1_frozen_batch_for_r5_readiness", BASE_BATCH)
_MAT = _load_module("v2_04g_r5_materializer_for_readiness", CANDIDATE_MATERIALIZER)
_GUARD = _load_module("v2_04g_r5_execution_guard_for_readiness", GUARD)


def _identity(row):
    return str(row["identity"])


def _target(row):
    return READINESS_ROOT / "episodes" / (
        "readiness_{:02d}__{}__{}".format(
            row["sequence"], row["profile_id"], row["scene_id"]
        )
    )


def _base_summary(prereg, schedule):
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "split": "calibration",
        "status": "in_progress",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {
            "path": str(_GUARD.PREREGISTRATION.relative_to(WORKSPACE)),
            "sha256": _GUARD.PREREGISTRATION_SHA256,
        },
        "dry_run_audit": {
            "path": str(_GUARD.DRY_RUN_AUDIT.relative_to(WORKSPACE)),
            "sha256": _GUARD.DRY_RUN_AUDIT_SHA256,
        },
        "planned_probe_count": len(schedule),
        "executed_probe_count": 0,
        "attempted_identity_count": 0,
        "valid_probe_count": 0,
        "evidence_unit_count": 0,
        "attempt_limit_per_identity": 1,
        "attempts_per_identity_max": 1,
        "retry_count": 0,
        "resume_used": False,
        "attempt_ledger": [],
        "reports": [],
        "aggregate_fault_taxonomy_counts": {},
        "observed_status_counts_by_profile": {},
        "all_probe_hard_gates_pass": False,
        "readiness_pass": False,
        "component_probe_authorized": False,
        "ttc_component_authorized": False,
        "navigation_authorized": False,
        "terminal_failure": None,
        "resume_forbidden": False,
        "compile_support_scene_execution_count": 0,
        "held_out_seed_consumption": False,
    }


def _write_summary(summary):
    _GUARD.atomic_yaml(SUMMARY_PATH, summary)


def _record_terminal(summary, row, reason):
    for entry in summary["attempt_ledger"]:
        if entry["identity"] == _identity(row):
            entry["status"] = "terminal_failure"
            entry["reason"] = str(reason)
            entry["resume_forbidden"] = True
            break
    summary.update({
        "status": "terminal_failure",
        "attempted_identity_count": len(summary["attempt_ledger"]),
        "executed_probe_count": len(summary["attempt_ledger"]),
        "valid_probe_count": len(summary["reports"]),
        "evidence_unit_count": len(summary["attempt_ledger"]),
        "all_probe_hard_gates_pass": False,
        "readiness_pass": False,
        "component_probe_authorized": False,
        "ttc_component_authorized": False,
        "navigation_authorized": False,
        "terminal_failure": {
            "identity": _identity(row),
            "sequence": row["sequence"],
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "reason": str(reason),
        },
        "resume_forbidden": True,
    })
    _write_summary(summary)


def _validate_design(prereg):
    gate = prereg["ttc_activation_coverage_readiness"]
    schedule = gate["schedule"]
    if not (
        len(schedule) == 6
        and [row["sequence"] for row in schedule] == list(range(1, 7))
        and [row["profile_id"] for row in schedule]
        == ["r5_ttc_h450"] * 3 + ["r5_ttc_h400"] * 3
        and [row["seed"] for row in schedule] == [5111, 5112, 5113] * 2
        and [row["expected_status"] for row in schedule]
        == [
            "OBSERVED_CONFLICT",
            "OBSERVED_CONFLICT",
            "NO_CONFLICT_IN_HORIZON",
        ] * 2
        and all(row["attempt_limit"] == 1 for row in schedule)
        and prereg["budget"]["activation_readiness_probe_count"] == 6
        and prereg["budget"]["attempts_per_readiness_identity_max"] == 1
    ):
        raise RuntimeError("R5 readiness schedule or budget drifted")
    support = set(prereg["readiness_compile_support_boundary"]["scene_ids"])
    if support.intersection(row["scene_id"] for row in schedule):
        raise RuntimeError("compile-support-only readiness scene entered schedule")
    if set(row["seed"] for row in schedule).intersection(
        prereg["seed_firewall"]["reserved_future_held_out_seeds"]
    ):
        raise RuntimeError("R5 readiness schedule consumes held-out seeds")
    instances = _BASE._load_instances(COMPILED_SCENES)
    if set(row["scene_id"] for row in schedule).difference(instances):
        raise RuntimeError("R5 readiness compiled scene is missing")
    for row in schedule:
        scene = instances[row["scene_id"]][0]["scene"]
        if scene["seed"] != row["seed"] or scene["family"] != "DYNAMIC":
            raise RuntimeError("R5 readiness compiled identity drifted")
    return gate, schedule, instances


def _listener_command(row, gate, report_path):
    fractions = {
        gate["minimum_transaction_activated_fraction"],
        gate["minimum_transaction_valid_fraction"],
        gate["minimum_transaction_join_valid_fraction"],
    }
    if len(fractions) != 1:
        raise RuntimeError("frozen listener cannot represent split R5 fractions")
    return [
        sys.executable,
        str(LISTENER),
        "--output",
        str(report_path),
        "--profile-id",
        row["profile_id"],
        "--repeat",
        str(row["sequence"]),
        "--seed",
        str(row["seed"]),
        "--warmup-timeout-s",
        str(gate["warmup_timeout_s"]),
        "--measurement-duration-s",
        str(gate["measurement_duration_s"]),
        "--minimum-message-count",
        str(gate["minimum_message_count_per_stream"]),
        "--minimum-valid-fraction",
        str(next(iter(fractions))),
        "--required-consecutive-stable-count",
        str(gate["required_consecutive_stable_count"]),
        "--maximum-expected-context-hold-count",
        str(gate["maximum_expected_context_hold_count_per_probe"]),
    ]


def _runner_command(row, instance_path, target):
    return [
        sys.executable,
        str(EPISODE_RUNNER),
        "--instance",
        str(instance_path),
        "--method",
        "rule_multi_anchor",
        "--output-dir",
        str(target),
        "--stage",
        STAGE,
        "--split",
        "calibration",
        "--profile-id",
        row["profile_id"],
    ]


def _launch_command(row, scene, world_path, runtime):
    return [
        "roslaunch",
        "m2_gazebo",
        "m2_v2_04g_r2_mechanism_calibration.launch",
        "world:={}".format(world_path),
        "seed:={}".format(row["seed"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false",
        "rule_supervisor_config:={}".format(runtime["supervisor"]),
        "anchor_bank:={}".format(runtime["anchor_bank"]),
        "mechanism_config:={}".format(runtime["mechanism"]),
        "load_balanced_anchor:=true",
        "publish_teb_obstacles:=true",
        "start_rule_supervisor:=true",
        "start_typed_transaction:=true",
        "force_geometry_balanced:=false",
    ]


def _stop_process(process):
    if process is not None:
        _BASE._terminate_group(process)


def _run_pair(row, gate, instances, runtime, environment):
    instance, instance_path, world_path = instances[row["scene_id"]]
    scene = instance["scene"]
    target = _target(row)
    if target.exists():
        raise RuntimeError("R5 readiness output identity already exists")
    target.mkdir(parents=True)
    report_path = target / "activation_report.yaml"
    launch = listener = runner = None
    listener_result = runner_result = None
    listener_deadline = (
        float(gate["warmup_timeout_s"])
        + float(gate["measurement_duration_s"])
        + 30.0
    )
    runner_deadline = float(scene["timeout_s"]) + 75.0
    with (target / "launch.log").open(
        "x", encoding="utf-8"
    ) as launch_log, (target / "listener.log").open(
        "x", encoding="utf-8"
    ) as listener_log, (target / "runner.log").open(
        "x", encoding="utf-8"
    ) as runner_log:
        try:
            launch = subprocess.Popen(
                _launch_command(row, scene, world_path, runtime),
                env=environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            _BASE._ready(launch, environment, "rule_multi_anchor")
            start = time.monotonic()
            listener = subprocess.Popen(
                _listener_command(row, gate, report_path),
                env=environment,
                stdout=listener_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            runner = subprocess.Popen(
                _runner_command(row, instance_path, target),
                env=environment,
                stdout=runner_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            while listener_result is None or runner_result is None:
                now = time.monotonic()
                listener_result = listener.poll()
                runner_result = runner.poll()
                if listener_result not in (None, 0):
                    raise RuntimeError(
                        "readiness listener exited {}".format(listener_result)
                    )
                if runner_result not in (None, 0):
                    raise RuntimeError(
                        "readiness episode runner exited {}".format(runner_result)
                    )
                if listener_result is None and now - start > listener_deadline:
                    raise RuntimeError("readiness listener timeout")
                if runner_result is None and now - start > runner_deadline:
                    raise RuntimeError("readiness episode runner timeout")
                if launch.poll() is not None:
                    raise RuntimeError("roslaunch exited during readiness episode")
                time.sleep(0.1)
        finally:
            _stop_process(listener)
            _stop_process(runner)
            _stop_process(launch)
            time.sleep(1.0)
    if not report_path.is_file():
        raise RuntimeError("readiness listener produced no atomic report")
    listener_report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    evaluation_path = target / "evaluation.yaml"
    validation_row = {
        "stage": STAGE,
        "split": "calibration",
        "method": "rule_multi_anchor",
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "family": scene["family"],
        "seed": row["seed"],
    }
    evaluation = _BASE._validate_completed(validation_row, evaluation_path)
    if not (
        listener_report.get("stage") == STAGE
        and listener_report.get("profile_id") == row["profile_id"]
        and listener_report.get("seed") == row["seed"]
        and listener_report.get("repeat") == row["sequence"]
        and listener_report.get("all_hard_gates_pass") is True
        and listener_report.get("world_model_sequence_mismatch_count") == 0
        and listener_report.get("world_model_input_join_fault_count") == 0
        and evaluation.get("ttc_status") == row["expected_status"]
        and evaluation.get("formal_result") is False
        and evaluation.get("runtime_ready") is False
        and evaluation.get("experiment_manager_calibration_manifest_access")
        is True
        and evaluation.get("experiment_manager_validation_manifest_access")
        is False
        and evaluation.get("runtime_policy_manifest_access") is False
        and evaluation.get("runtime_scene_labels_available") is False
        and evaluation.get("training_used") is False
        and evaluation.get("clearance_audit", {}).get(
            "runtime_policy_received_truth"
        )
        is False
    ):
        raise RuntimeError("R5 readiness activation or TTC coverage gate failed")
    return report_path, listener_report, evaluation_path, evaluation


def _refresh_aggregates(summary):
    taxonomy = Counter()
    status_by_profile = {}
    mismatch_count = 0
    input_join_count = 0
    for record in summary["reports"]:
        listener = yaml.safe_load(
            Path(record["activation_report"]).read_text(encoding="utf-8")
        )
        taxonomy.update(listener.get("fault_taxonomy_counts", {}))
        mismatch_count += listener.get("world_model_sequence_mismatch_count", 0)
        input_join_count += listener.get("world_model_input_join_fault_count", 0)
        profile = record["profile_id"]
        status_by_profile.setdefault(profile, Counter())
        status_by_profile[profile][record["observed_ttc_status"]] += 1
    summary["aggregate_fault_taxonomy_counts"] = dict(taxonomy)
    summary["world_model_sequence_mismatch_count"] = mismatch_count
    summary["world_model_input_join_fault_count"] = input_join_count
    summary["atomic_world_model_input_alignment_pass"] = (
        mismatch_count == 0 and input_join_count == 0
    )
    summary["observed_status_counts_by_profile"] = {
        profile: dict(counts) for profile, counts in status_by_profile.items()
    }


def _complete(summary, gate):
    _refresh_aggregates(summary)
    counts = summary["observed_status_counts_by_profile"]
    coverage = all(
        counts.get(profile, {}).get("OBSERVED_CONFLICT", 0)
        == gate["expected_observed_conflict_count_per_profile"]
        and counts.get(profile, {}).get("NO_CONFLICT_IN_HORIZON", 0)
        == gate["expected_no_conflict_count_per_profile"]
        and counts.get(profile, {}).get("TRACKER_INVALID", 0)
        <= gate["tracker_invalid_count_max_per_profile"]
        for profile in gate["profile_ids"]
    )
    complete = bool(
        len(summary["attempt_ledger"]) == 6
        and len(summary["reports"]) == 6
        and all(
            entry["status"] == "evidence_complete"
            for entry in summary["attempt_ledger"]
        )
        and summary["atomic_world_model_input_alignment_pass"]
        and summary["aggregate_fault_taxonomy_counts"].get(
            "BACKEND_TRANSACTION_FAULT", 0
        )
        == 0
        and summary["aggregate_fault_taxonomy_counts"].get(
            "UNKNOWN_TRANSACTION_FAULT", 0
        )
        == 0
        and coverage
    )
    summary.update({
        "status": "complete" if complete else "terminal_failure",
        "attempted_identity_count": len(summary["attempt_ledger"]),
        "executed_probe_count": len(summary["attempt_ledger"]),
        "valid_probe_count": len(summary["reports"]),
        "evidence_unit_count": len(summary["attempt_ledger"]),
        "ttc_coverage_pass": coverage,
        "all_probe_hard_gates_pass": complete,
        "readiness_pass": complete,
        "component_probe_authorized": complete,
        "ttc_component_authorized": complete,
        "navigation_authorized": False,
        "resume_forbidden": True,
    })
    if not complete:
        summary["terminal_failure"] = {
            "identity": "aggregate-readiness-gate",
            "reason": "R5 readiness aggregate coverage gate failed",
        }
    _write_summary(summary)
    return complete


def execute():
    prereg, _, _ = _GUARD.verify_frozen_start()
    _GUARD.assert_thesis_workspace_environment()
    _GUARD.assert_no_live_runtime_processes()
    gate, schedule, instances = _validate_design(prereg)
    if SUMMARY_PATH.exists() or READINESS_ROOT.exists() or RUNTIME_CONFIG_ROOT.exists():
        raise RuntimeError(
            "R5 readiness has existing state; retry/resume is forbidden"
        )
    bank_path = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    runtime = _MAT.materialize_candidates(bank_path, RUNTIME_CONFIG_ROOT)
    if set(runtime) != set(prereg["candidate_ids"]):
        raise RuntimeError("R5 runtime candidate set drifted")
    summary = _base_summary(prereg, schedule)
    _write_summary(summary)
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in schedule:
        entry = {
            "sequence": row["sequence"],
            "identity": _identity(row),
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "attempt": 1,
            "attempt_limit": 1,
            "status": "attempt_started",
            "resume_forbidden_if_interrupted": True,
        }
        summary["attempt_ledger"].append(entry)
        summary["active_identity"] = _identity(row)
        summary["attempted_identity_count"] = len(summary["attempt_ledger"])
        summary["executed_probe_count"] = len(summary["attempt_ledger"])
        summary["evidence_unit_count"] = len(summary["attempt_ledger"])
        _write_summary(summary)
        print(
            "START readiness {}/6 {} x {}".format(
                row["sequence"], row["profile_id"], row["scene_id"]
            ),
            flush=True,
        )
        try:
            report_path, report, evaluation_path, evaluation = _run_pair(
                row, gate, instances, runtime[row["profile_id"]], environment
            )
            _GUARD.assert_no_live_runtime_processes()
            entry.update({
                "status": "evidence_complete",
                "activation_report": str(report_path),
                "activation_report_sha256": _GUARD.sha256(report_path),
                "report_path": str(report_path),
                "report_sha256": _GUARD.sha256(report_path),
                "evaluation": str(evaluation_path),
                "evaluation_sha256": _GUARD.sha256(evaluation_path),
                "trace_sha256": evaluation["raw_trace_sha256"],
                "expected_ttc_status": row["expected_status"],
                "observed_ttc_status": evaluation["ttc_status"],
            })
            expected_holds = report.get("fault_taxonomy_counts", {}).get(
                "EXPECTED_FAIL_CLOSED_CONTEXT_HOLD", 0
            )
            backend_faults = report.get("fault_taxonomy_counts", {}).get(
                "BACKEND_TRANSACTION_FAULT", 0
            )
            unknown_faults = report.get("fault_taxonomy_counts", {}).get(
                "UNKNOWN_TRANSACTION_FAULT", 0
            )
            summary["reports"].append({
                **report,
                "sequence": row["sequence"],
                "identity": _identity(row),
                "attempt": 1,
                "profile_id": row["profile_id"],
                "scene_id": row["scene_id"],
                "seed": row["seed"],
                "expected_ttc_status": row["expected_status"],
                "observed_ttc_status": evaluation["ttc_status"],
                "tracker_message_count": evaluation["tracker_message_count"],
                "expected_context_hold_count": expected_holds,
                "backend_transaction_fault_count": backend_faults,
                "unknown_transaction_fault_count": unknown_faults,
                "runtime_policy_manifest_access": evaluation[
                    "runtime_policy_manifest_access"
                ],
                "runtime_scene_labels_available": evaluation[
                    "runtime_scene_labels_available"
                ],
                "activation_report": str(report_path),
                "activation_report_sha256": _GUARD.sha256(report_path),
                "evaluation": str(evaluation_path),
                "evaluation_sha256": _GUARD.sha256(evaluation_path),
                "trace_sha256": evaluation["raw_trace_sha256"],
                "all_hard_gates_pass": report["all_hard_gates_pass"],
            })
            summary["valid_probe_count"] = len(summary["reports"])
            summary["active_identity"] = None
            _refresh_aggregates(summary)
            _write_summary(summary)
            print(
                "PASS readiness {}/6 {} status={}".format(
                    row["sequence"], row["profile_id"], evaluation["ttc_status"]
                ),
                flush=True,
            )
        except Exception as exc:
            _record_terminal(summary, row, exc)
            raise RuntimeError(
                "R5 readiness terminal failure; retry/resume forbidden"
            ) from exc
    if not _complete(summary, gate):
        raise RuntimeError("R5 readiness aggregate gate failed; resume forbidden")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


def dry_run():
    prereg, _, _ = _GUARD.verify_frozen_start()
    _, schedule, _ = _validate_design(prereg)
    if SUMMARY_PATH.exists() or READINESS_ROOT.exists() or RUNTIME_CONFIG_ROOT.exists():
        raise RuntimeError("R5 readiness state already exists")
    print("R5 readiness dry-run: exact 6 identities; no files or processes started")
    for row in schedule:
        print(
            "{:02d} {} {} seed={} expected={}".format(
                row["sequence"],
                row["profile_id"],
                row["scene_id"],
                row["seed"],
                row["expected_status"],
            )
        )
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    return dry_run() if args.dry_run else execute()


if __name__ == "__main__":
    raise SystemExit(main())
