#!/usr/bin/env python3
"""Fail-closed assessment for the bounded V2-04G-R5 calibration.

The assessor has two valid outcomes:

* all 6 readiness, 3 component, and 60 navigation identities are complete,
  in which case every preregistered hard gate is evaluated and only the two
  winner-eligible repair candidates are ranked; or
* an execution phase contains a canonical terminal-failure marker, in which
  case a stop report is produced and ranking is explicitly not performed.

This tool never authorizes or performs winner freezing, held-out validation,
V2-05, SAC, or real-vehicle work.  A ranked candidate is only a candidate
report for later review under a separate user instruction.
"""

import argparse
import hashlib
import importlib.util
import math
from pathlib import Path
import statistics

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
PREREGISTRATION_SHA256 = (
    "0adcfd6a7a686b799b6dc55394cdf1e90fa140cee636d4283e0fb807f14134c6"
)
DRY_RUN_AUDIT_SHA256 = (
    "d7a3113c89b08889dc754a72f4e792c422225f19504ab3218d9712cf46dee8e1"
)
NAVIGATION_SCHEDULE_SHA256 = (
    "5daf5a4dcdf0e68c4b034f343e0ae5f85504c25e6158e94699c6f1a8cc80513a"
)
EXECUTION_GUARD = Path(__file__).with_name("v2_04g_r5_execution_guard.py")
FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
TTC_STATUSES = (
    "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID",
)
CONTROL_ID = "r5_ttc_control_h500"
CANDIDATE_IDS = (
    CONTROL_ID, "r5_ttc_h450", "r5_ttc_h400",
)
ELIGIBLE_IDS = ("r5_ttc_h450", "r5_ttc_h400")
TERMINAL_STATUSES = ("terminal_failure", "terminal_failed")


class R5AssessmentError(ValueError):
    """Raised when R5 evidence is incomplete, ambiguous, or has drifted."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise R5AssessmentError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_yaml(path):
    source = Path(path)
    try:
        value = yaml.load(source.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise R5AssessmentError(
            "cannot strictly load YAML {}: {}".format(source, exc)
        ) from exc
    if not isinstance(value, dict):
        raise R5AssessmentError("{} must contain a YAML mapping".format(source))
    return value


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _require(condition, message):
    if not condition:
        raise R5AssessmentError(message)


def _number(value, label, minimum=None):
    _require(
        not isinstance(value, bool) and isinstance(value, (int, float)),
        "{} must be numeric".format(label),
    )
    result = float(value)
    _require(math.isfinite(result), "{} must be finite".format(label))
    if minimum is not None:
        _require(result >= minimum, "{} is below {}".format(label, minimum))
    return result


def _integer(value, label, minimum=0):
    _require(
        not isinstance(value, bool) and isinstance(value, int),
        "{} must be an integer".format(label),
    )
    _require(value >= minimum, "{} is below {}".format(label, minimum))
    return value


def _inside(root, path, label):
    boundary = Path(root).resolve()
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise R5AssessmentError(
            "{} leaves allowed root: {}".format(label, resolved)
        ) from exc
    return resolved


def _resolve_declared_path(workspace, value, label):
    path = Path(value)
    resolved = path.resolve() if path.is_absolute() else (workspace / path).resolve()
    return _inside(workspace, resolved, label)


def _resource_entries(value, prefix=""):
    if isinstance(value, dict):
        if "path" in value or "sha256" in value:
            _require(
                set(value) == {"path", "sha256"},
                "{} resource declaration drifted".format(prefix),
            )
            yield prefix, value
            return
        for key, child in value.items():
            child_prefix = "{}.{}".format(prefix, key) if prefix else str(key)
            yield from _resource_entries(child, child_prefix)


def verify_frozen_inputs(workspace, preregistration_path, dry_run_audit_path):
    """Load and verify the immutable R5 design boundary."""

    root = Path(workspace).resolve()
    prereg_path = _inside(root, preregistration_path, "preregistration")
    audit_path = _inside(root, dry_run_audit_path, "dry-run audit")
    _require(root == WORKSPACE.resolve(), "R5 assessment workspace drifted")
    _require(
        prereg_path == (
            root / "experiments/manifests/v2/calibration/"
            "v2_04g_r5_preregistration.yaml").resolve()
        and audit_path == (
            root / "artifacts/v2/calibration/v2_04g_r5/"
            "v2_04g_r5_dry_run_audit.yaml").resolve(),
        "R5 frozen-start path drifted",
    )
    spec = importlib.util.spec_from_file_location(
        "v2_04g_r5_execution_guard_for_assessment", EXECUTION_GUARD)
    _require(
        spec is not None and spec.loader is not None,
        "cannot load the R5 execution guard",
    )
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    try:
        guarded_prereg, guarded_audit, authorization = guard.verify_frozen_start()
    except Exception as exc:
        raise R5AssessmentError(
            "R5 execution guard rejected the frozen/auth boundary: {}".format(exc)
        ) from exc
    _require(
        _sha256(prereg_path) == PREREGISTRATION_SHA256,
        "R5 preregistration hash drifted",
    )
    _require(
        _sha256(audit_path) == DRY_RUN_AUDIT_SHA256,
        "R5 dry-run audit hash drifted",
    )
    prereg = _load_yaml(prereg_path)
    audit = _load_yaml(audit_path)
    _require(
        prereg == guarded_prereg and audit == guarded_audit,
        "strictly loaded frozen inputs differ from execution-guard inputs",
    )
    _require(
        prereg.get("stage") == STAGE
        and prereg.get("split") == "calibration"
        and prereg.get("simulation_only") is True
        and prereg.get("calibration_only") is True
        and prereg.get("formal_result") is False
        and prereg.get("runtime_ready") is False
        and prereg.get("training_allowed") is False
        and prereg.get("real_vehicle_use_forbidden") is True,
        "R5 preregistration safety boundary drifted",
    )
    _require(
        audit.get("stage") == STAGE
        and audit.get("status") == "dry_run_audit_pass"
        and audit.get("formal_result") is False
        and audit.get("runtime_ready") is False,
        "R5 dry-run audit boundary drifted",
    )
    _require(
        audit.get("preregistration", {}).get("sha256")
        == PREREGISTRATION_SHA256,
        "dry-run audit does not bind the frozen preregistration",
    )
    entries = list(_resource_entries({
        "resources": prereg.get("resources", {}),
        "frozen_r4_r1_boundary": prereg.get("frozen_r4_r1_boundary", {}),
    }))
    _require(len(entries) == 39, "R5 frozen resource closure must contain 39 files")
    seen = set()
    for label, resource in entries:
        path = _resolve_declared_path(root, resource["path"], label)
        _require(path.is_file(), "frozen resource is missing: {}".format(label))
        _require(
            _sha256(path) == resource["sha256"],
            "frozen resource hash drifted: {}".format(label),
        )
        _require(path not in seen, "duplicate frozen resource path: {}".format(path))
        seen.add(path)
    contract_resource = prereg["resources"]["contract"]
    bank_resource = prereg["resources"]["candidate_bank"]
    contract_path = _resolve_declared_path(root, contract_resource["path"], "contract")
    bank_path = _resolve_declared_path(root, bank_resource["path"], "candidate bank")
    contract = _load_yaml(contract_path)
    bank = _load_yaml(bank_path)
    _require(
        contract.get("stage") == STAGE and bank.get("stage") == STAGE,
        "R5 contract or candidate-bank boundary drifted",
    )
    return {
        "preregistration": prereg,
        "contract": contract,
        "candidate_bank": bank,
        "audit": audit,
        "execution_authorization": authorization,
        "paths": {
            "preregistration": prereg_path,
            "dry_run_audit": audit_path,
            "contract": contract_path,
            "candidate_bank": bank_path,
            "execution_authorization": guard.EXECUTION_AUTHORIZATION,
        },
    }


def _attempt_boundary(document, label):
    _require(
        document.get("attempts_per_identity_max") == 1,
        "{} attempt limit drifted".format(label),
    )
    _require(document.get("retry_count") == 0, "{} retried evidence".format(label))
    _require(
        document.get("resume_used") is False,
        "{} must explicitly record resume_used=false".format(label),
    )
    _require(
        document.get("resume_forbidden") is True,
        "{} must explicitly preserve the no-resume boundary".format(label),
    )


def _canonical_schedule_row(row, phase):
    if phase == "readiness":
        return {
            "sequence": row["sequence"],
            "identity": row["identity"],
            "profile_id": row["profile_id"],
            "scene_id": row["scene_id"],
            "seed": row["seed"],
            "expected_status": row["expected_status"],
            "attempt": 1,
        }
    if phase == "component":
        return {
            "sequence": row["sequence"],
            "identity": row["identity"],
            "expected_status": row["expected_status"],
            "attempt": 1,
        }
    return {
        "sequence": row["sequence"],
        "method": row["method"],
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
    }


def _check_record_identity(record, expected, phase, label):
    canonical = _canonical_schedule_row(expected, phase)
    observed_status_key = (
        "expected_ttc_status" if phase == "readiness" else "expected_status"
    )
    aliases = {
        "expected_status": observed_status_key,
    }
    for key, value in canonical.items():
        observed_key = aliases.get(key, key)
        _require(
            record.get(observed_key) == value,
            "{} identity drifted at {}".format(label, key),
        )


def _terminal_marker(document, label):
    marker = document.get("terminal_failure")
    status = document.get("status")
    if status in TERMINAL_STATUSES:
        _require(isinstance(marker, dict), "{} terminal marker is missing".format(label))
        _require(
            isinstance(marker.get("identity"), (str, list, tuple, dict))
            and isinstance(marker.get("reason"), str)
            and bool(marker["reason"].strip()),
            "{} terminal marker is incomplete".format(label),
        )
        return marker
    _require(marker is None, "{} has a terminal marker while complete".format(label))
    return None


def _record_count(record, keys, label):
    for key in keys:
        if key in record:
            return _integer(record[key], "{}.{}".format(label, key))
    raise R5AssessmentError("{} is missing {}".format(label, "/".join(keys)))


def _record_fraction(record, keys, label):
    for key in keys:
        if key in record:
            value = _number(record[key], "{}.{}".format(label, key), 0.0)
            _require(value <= 1.0, "{}.{} exceeds 1".format(label, key))
            return value
    raise R5AssessmentError("{} is missing {}".format(label, "/".join(keys)))


def _readiness_record_gates(record, gate, expected, label):
    observed = record.get("observed_ttc_status", record.get("ttc_status"))
    stream_counts = {
        "tracker": _record_count(record, ("tracker_message_count",), label),
        "context": _record_count(record, ("context_message_count",), label),
        "transaction": _record_count(record, ("transaction_message_count",), label),
        "mechanism": _record_count(record, ("mechanism_message_count",), label),
    }
    minimum_messages = gate["minimum_message_count_per_stream"]
    fractions = {
        "transaction_activated": _record_fraction(
            record, ("transaction_activated_fraction",), label),
        "transaction_valid": _record_fraction(
            record, ("transaction_valid_fraction",), label),
        "transaction_join_valid": _record_fraction(
            record, ("transaction_join_valid_fraction", "join_valid_fraction"), label),
    }
    fault_taxonomy = record.get("fault_taxonomy_counts", {})
    _require(isinstance(fault_taxonomy, dict), "{} fault taxonomy is invalid".format(label))
    gates = {
        "identity_status": observed == expected["expected_status"],
        "report_status": (
            record.get("status") in ("pass", "complete", "evidence_complete")
            and record.get("all_hard_gates_pass") is True
        ),
        "consecutive_stable": _record_count(
            record, ("maximum_consecutive_stable_count",), label
        ) >= gate["required_consecutive_stable_count"],
        "message_counts": all(value >= minimum_messages for value in stream_counts.values()),
        "transaction_activated_fraction": (
            fractions["transaction_activated"]
            >= gate["minimum_transaction_activated_fraction"]
        ),
        "transaction_valid_fraction": (
            fractions["transaction_valid"]
            >= gate["minimum_transaction_valid_fraction"]
        ),
        "transaction_join_valid_fraction": (
            fractions["transaction_join_valid"]
            >= gate["minimum_transaction_join_valid_fraction"]
        ),
        "expected_context_hold_count": _record_count(
            record, ("expected_context_hold_count",), label
        ) <= gate["maximum_expected_context_hold_count_per_probe"],
        "world_model_sequence_mismatch": _record_count(
            record, ("world_model_sequence_mismatch_count",), label
        ) <= gate["maximum_world_model_sequence_mismatch_count"],
        "world_model_input_join_fault": _record_count(
            record, ("world_model_input_join_fault_count",), label
        ) <= gate["maximum_world_model_input_join_fault_count"],
        "backend_transaction_fault": (
            _record_count(
                record, ("backend_transaction_fault_count",), label
            ) <= gate["maximum_backend_transaction_fault_count"]
            and int(fault_taxonomy.get("BACKEND_TRANSACTION_FAULT", 0)) == 0
        ),
        "unknown_transaction_fault": (
            _record_count(
                record, ("unknown_transaction_fault_count",), label
            ) <= gate["maximum_unknown_transaction_fault_count"]
            and int(fault_taxonomy.get("UNKNOWN_TRANSACTION_FAULT", 0)) == 0
        ),
        "training_unused": record.get("training_used") is False,
        "real_vehicle_unused": record.get("real_vehicle_used") is False,
        "runtime_labels_unavailable": (
            record.get("runtime_scene_labels_available", False) is False
            and record.get("runtime_policy_manifest_access", False) is False
        ),
    }
    return {
        "observed_ttc_status": observed,
        "stream_message_counts": stream_counts,
        "fractions": fractions,
        "hard_gates": gates,
        "all_hard_gates_pass": all(gates.values()),
    }


def assess_readiness(preregistration, contract, summary):
    """Validate the exact six-evidence TTC readiness prefix or completion."""

    _require(isinstance(summary, dict), "readiness summary is missing")
    _require(
        summary.get("stage") == STAGE
        and summary.get("simulation_only") is True
        and summary.get("calibration_only") is True
        and summary.get("formal_result") is False
        and summary.get("runtime_ready") is False
        and summary.get("training_started", False) is False
        and summary.get("real_vehicle_used") is False,
        "readiness safety boundary drifted",
    )
    _attempt_boundary(summary, "readiness")
    marker = _terminal_marker(summary, "readiness")
    schedule = preregistration["ttc_activation_coverage_readiness"]["schedule"]
    reports = summary.get("reports", [])
    _require(isinstance(reports, list), "readiness reports must be a list")
    _require(len(reports) <= len(schedule), "readiness evidence exceeds six identities")
    gate = contract["ttc_activation_coverage_readiness_gate"]
    results = []
    for index, report in enumerate(reports):
        label = "readiness.report[{}]".format(index)
        _require(isinstance(report, dict), "{} must be a mapping".format(label))
        _check_record_identity(report, schedule[index], "readiness", label)
        results.append(_readiness_record_gates(report, gate, schedule[index], label))
    if marker is not None:
        attempted = _integer(
            summary.get("attempted_identity_count"),
            "readiness.attempted_identity_count", 1)
        _require(
            attempted <= 6 and len(reports) <= attempted,
            "readiness terminal attempt/evidence counts drifted",
        )
        expected_index = min(len(reports), len(schedule) - 1)
        if reports and not results[-1]["all_hard_gates_pass"]:
            expected_index = len(reports) - 1
        expected = schedule[expected_index]
        _require(
            marker.get("identity") == expected["identity"],
            "readiness terminal marker does not match the stopped identity",
        )
        _require(
            marker.get("sequence") == expected["sequence"],
            "readiness terminal sequence drifted",
        )
        return {
            "complete": False,
            "pass": False,
            "terminal": True,
            "terminal_failure": marker,
            "evidence_count": attempted,
            "valid_evidence_count": len(reports),
            "reports": results,
        }
    _require(summary.get("status") == "complete", "readiness status is not complete")
    _require(len(reports) == 6, "readiness requires exactly six reports")
    _require(
        summary.get("planned_probe_count") == 6
        and summary.get("executed_probe_count") == 6
        and summary.get("valid_probe_count") == 6,
        "readiness count summary drifted",
    )
    coverage = {}
    for profile_id in ELIGIBLE_IDS:
        indices = [
            index for index, row in enumerate(schedule)
            if row["profile_id"] == profile_id
        ]
        statuses = [results[index]["observed_ttc_status"] for index in indices]
        counts = {status: statuses.count(status) for status in TTC_STATUSES}
        coverage[profile_id] = counts
        _require(
            counts == {
                "OBSERVED_CONFLICT": 2,
                "NO_CONFLICT_IN_HORIZON": 1,
                "TRACKER_INVALID": 0,
            },
            "{} readiness TTC coverage failed".format(profile_id),
        )
    readiness_pass = all(row["all_hard_gates_pass"] for row in results)
    _require(readiness_pass, "readiness hard gate failed without terminal stop")
    _require(
        summary.get("all_probe_hard_gates_pass") is True
        and summary.get("ttc_coverage_pass") is True
        and summary.get("ttc_component_authorized") is True,
        "readiness summary authorization/gate drifted",
    )
    return {
        "complete": True,
        "pass": True,
        "terminal": False,
        "terminal_failure": None,
        "evidence_count": 6,
        "coverage": coverage,
        "reports": results,
    }


def assess_component(preregistration, report):
    """Validate the exact three-state deterministic TTC component evidence."""

    _require(isinstance(report, dict), "TTC component report is missing")
    _require(
        report.get("stage") == STAGE
        and report.get("simulation_only") is True
        and report.get("calibration_only") is True
        and report.get("formal_result") is False
        and report.get("runtime_ready") is False
        and report.get("training_used") is False
        and report.get("real_vehicle_used") is False,
        "TTC component safety boundary drifted",
    )
    _attempt_boundary(report, "TTC component")
    marker = _terminal_marker(report, "TTC component")
    schedule = preregistration["ttc_component_probe"]["schedule"]
    probes = report.get("probes", [])
    _require(isinstance(probes, list), "TTC component probes must be a list")
    _require(len(probes) <= 3, "TTC component evidence exceeds three identities")
    observed = []
    probe_passes = []
    for index, probe in enumerate(probes):
        label = "component.probe[{}]".format(index)
        _require(isinstance(probe, dict), "{} must be a mapping".format(label))
        _check_record_identity(probe, schedule[index], "component", label)
        _require(
            "seed" not in probe or probe.get("seed") is None,
            "{} consumed an undeclared seed".format(label),
        )
        value = probe.get("observed_status", probe.get("ttc_status"))
        observed.append(value)
        probe_passes.append(
            value == schedule[index]["expected_status"]
            and probe.get("pass") is True
        )
    if marker is not None:
        attempted = _integer(
            report.get("attempted_identity_count"),
            "component.attempted_identity_count", 1)
        _require(
            attempted <= 3 and len(probes) <= attempted,
            "TTC component terminal attempt/evidence counts drifted",
        )
        expected_index = min(len(probes), 2)
        if probes and not probe_passes[-1]:
            expected_index = len(probes) - 1
        expected = schedule[expected_index]
        _require(
            marker.get("identity") == expected["identity"]
            and marker.get("sequence") == expected["sequence"],
            "TTC component terminal marker drifted",
        )
        return {
            "complete": False,
            "pass": False,
            "terminal": True,
            "terminal_failure": marker,
            "evidence_count": attempted,
            "valid_evidence_count": len(probes),
            "observed_status_order": observed,
        }
    _require(report.get("status") == "complete", "TTC component status is not complete")
    _require(
        len(probes) == 3 and report.get("probe_count") == 3,
        "TTC component requires exactly three probes",
    )
    expected_order = [row["expected_status"] for row in schedule]
    component_pass = observed == expected_order and all(probe_passes)
    _require(component_pass, "TTC component hard gate failed without terminal stop")
    _require(
        report.get("all_three_states_pass") is True
        and report.get("navigation_authorized") is True,
        "TTC component summary gate drifted",
    )
    return {
        "complete": True,
        "pass": True,
        "terminal": False,
        "terminal_failure": None,
        "evidence_count": 3,
        "observed_status_order": observed,
    }


def _evaluation_boundary(evaluation, expected, family, label):
    _require(
        evaluation.get("stage") == STAGE
        and evaluation.get("split") == "calibration"
        and evaluation.get("formal_result") is False
        and evaluation.get("runtime_ready") is False,
        "{} stage/split boundary drifted".format(label),
    )
    for key in ("profile_id", "scene_id", "seed", "method"):
        evaluation_key = "supervisor_profile_id" if key == "profile_id" else key
        _require(
            evaluation.get(evaluation_key) == expected[key],
            "{} {} drifted".format(label, key),
        )
    _require(evaluation.get("family") == family, "{} family drifted".format(label))
    _require(
        evaluation.get("training_used") is False
        and evaluation.get("runtime_policy_manifest_access") is False
        and evaluation.get("runtime_scene_labels_available") is False
        and evaluation.get("experiment_manager_validation_manifest_access") is False,
        "{} runtime/training boundary drifted".format(label),
    )
    audit = evaluation.get("clearance_audit", {})
    _require(
        isinstance(audit, dict)
        and audit.get("evaluator_only_gazebo_truth_used") is True
        and audit.get("runtime_policy_received_truth") is False,
        "{} evaluator/runtime truth boundary drifted".format(label),
    )
    common = evaluation.get("metrics", {}).get("common", {})
    _require(
        isinstance(common.get("success"), bool)
        and isinstance(common.get("collision"), bool),
        "{} success/collision metrics are invalid".format(label),
    )
    _number(common.get("navigation_time_s"), label + ".navigation_time_s", 0.0)
    _number(common.get("minimum_clearance_m"), label + ".minimum_clearance_m")
    _integer(evaluation.get("active_anchor_switch_count"), label + ".switches")
    _require(
        evaluation.get("ttc_status") in TTC_STATUSES,
        "{} TTC status is invalid".format(label),
    )


def _progress_records(preregistration, progress, evaluations, scene_families):
    schedule = preregistration["navigation_schedule"]["schedule"]
    episodes = progress.get("episodes", [])
    _require(isinstance(episodes, list), "navigation episodes must be a list")
    _require(len(episodes) <= 60, "navigation evidence exceeds 60 identities")
    _require(len(evaluations) == len(episodes), "loaded navigation evidence count drifted")
    normalized = []
    identities = set()
    for index, (record, evaluation) in enumerate(zip(episodes, evaluations)):
        label = "navigation.episode[{}]".format(index)
        _require(isinstance(record, dict), "{} record must be a mapping".format(label))
        expected = schedule[index]
        _check_record_identity(record, expected, "navigation", label)
        identity = (expected["profile_id"], expected["method"], expected["scene_id"])
        _require(identity not in identities, "duplicate navigation evidence identity")
        identities.add(identity)
        _require(
            record.get("attempt", 1) == 1,
            "{} attempt must be one".format(label),
        )
        family = scene_families.get(expected["scene_id"])
        _require(family in FAMILIES, "{} scene family is missing".format(label))
        _evaluation_boundary(evaluation, expected, family, label)
        normalized.append(evaluation)
    return schedule, normalized


def _navigation_identity(row):
    return [row["profile_id"], row["method"], row["scene_id"]]


def _navigation_identity_matches(value, row):
    canonical = _navigation_identity(row)
    return value in (
        canonical,
        tuple(canonical),
        {
            "profile_id": row["profile_id"],
            "method": row["method"],
            "scene_id": row["scene_id"],
        },
        "{}__{}__{}".format(*canonical),
        row["scene_id"],
    )


def _validate_navigation_attempt_ledger(progress, schedule, completed_count,
                                        terminal):
    ledger = progress.get("attempt_ledger")
    if ledger is None:
        _require(
            all(record.get("attempt") == 1 for record in progress.get("episodes", [])),
            "navigation evidence must provide attempt=1 or an attempt ledger",
        )
        return
    _require(isinstance(ledger, list), "navigation attempt ledger must be a list")
    allowed_statuses = {"attempt_started", "evidence_complete", "terminal_failure"}
    events = {}
    first_seen = []
    for index, event in enumerate(ledger):
        label = "navigation.attempt_ledger[{}]".format(index)
        _require(isinstance(event, dict), "{} must be a mapping".format(label))
        _require(event.get("attempt") == 1, "{} attempt drifted".format(label))
        _require(
            event.get("status") in allowed_statuses,
            "{} status is invalid".format(label),
        )
        sequence = _integer(event.get("sequence"), label + ".sequence", 1)
        _require(sequence <= 60, "{} sequence exceeds schedule".format(label))
        expected = schedule[sequence - 1]
        _require(
            _navigation_identity_matches(event.get("identity"), expected),
            "{} identity drifted".format(label),
        )
        key = tuple(_navigation_identity(expected))
        if key not in events:
            events[key] = []
            first_seen.append(key)
        _require(
            event["status"] not in events[key],
            "{} duplicates an attempt-state event".format(label),
        )
        events[key].append(event["status"])
    _require(
        first_seen == [
            tuple(_navigation_identity(row)) for row in schedule[:len(first_seen)]
        ],
        "navigation attempts are not an ordered schedule prefix",
    )
    _require(
        progress.get("attempted_identity_count") == len(events),
        "navigation attempted identity count drifted",
    )
    for index in range(completed_count):
        key = tuple(_navigation_identity(schedule[index]))
        _require(
            key in events and "evidence_complete" in events[key],
            "completed navigation evidence lacks a ledger completion",
        )
    if terminal:
        _require(
            any("terminal_failure" in statuses for statuses in events.values()),
            "terminal navigation progress lacks a terminal ledger event",
        )
    else:
        _require(
            len(events) == 60
            and all("evidence_complete" in statuses for statuses in events.values())
            and not any("terminal_failure" in statuses for statuses in events.values()),
            "complete navigation attempt ledger drifted",
        )


def _terminal_navigation_result(schedule, progress, evaluations):
    marker = progress["terminal_failure"]
    completed = len(evaluations)
    attempted = _integer(
        progress.get("attempted_identity_count"),
        "navigation.attempted_identity_count", 1)
    _require(
        progress.get("valid_evidence_episode_count") == completed
        and progress.get("interface_failure_count") == 1
        and progress.get("resume_forbidden") is True,
        "navigation terminal progress counts/boundary drifted",
    )
    expected_indices = []
    if completed:
        expected_indices.append(completed - 1)
    if completed < len(schedule):
        expected_indices.append(completed)
    matches = [
        schedule[index] for index in expected_indices
        if marker.get("sequence") == schedule[index]["sequence"]
        and _navigation_identity_matches(marker.get("identity"), schedule[index])
    ]
    _require(matches, "navigation terminal marker does not match the stopped identity")
    return {
        "complete": False,
        "pass": False,
        "terminal": True,
        "terminal_failure": marker,
        "evidence_count": attempted,
        "valid_evidence_count": completed,
    }


def _counts(values):
    return {status: values.count(status) for status in TTC_STATUSES}


def _median(values):
    _require(bool(values), "cannot take a median of empty evidence")
    return float(statistics.median(values))


def _join_gate(rows, gate):
    fractions = [row.get("mechanism_join_valid_fraction") for row in rows]
    reasons = [set(row.get("mechanism_join_reason_counts", {})) for row in rows]
    allowed = set(gate["allowed_join_reasons"])
    passed = bool(rows) and all(
        value is not None
        and _number(value, "mechanism_join_valid_fraction", 0.0)
        >= gate["minimum_valid_fraction_per_episode"]
        for value in fractions
    ) and all(reason_set.issubset(allowed) for reason_set in reasons)
    aggregate = {}
    for row in rows:
        for reason, count in row.get("mechanism_join_reason_counts", {}).items():
            aggregate[reason] = aggregate.get(reason, 0) + int(count)
    return {
        "minimum_valid_fraction_per_episode": min(fractions) if fractions else None,
        "reason_counts": aggregate,
        "pass": passed,
    }


def _method_ttc(rows):
    dynamic = [row for row in rows if row["family"] == "DYNAMIC"]
    return _counts([row["ttc_status"] for row in dynamic])


def _ttc_gate_pass(counts, gate):
    return (
        counts["OBSERVED_CONFLICT"] >= gate["observed_conflict_count_min_per_method"]
        and counts["NO_CONFLICT_IN_HORIZON"]
        >= gate["no_conflict_count_min_per_method"]
        and counts["TRACKER_INVALID"]
        <= gate["tracker_invalid_navigation_count_max_per_method"]
    )


def _overlay_conflict_count(rows):
    count = 0
    for row in rows:
        if row["family"] != "DYNAMIC" or row["ttc_status"] != "OBSERVED_CONFLICT":
            continue
        overlay_counts = row.get("context_overlay_sample_counts", {})
        _require(isinstance(overlay_counts, dict), "overlay sample counts are invalid")
        non_none = sum(
            _integer(value, "overlay count")
            for key, value in overlay_counts.items() if key != "NONE"
        )
        count += int(non_none > 0)
    return count


def _family_timing(rows, fixed_by_scene, gate):
    family_time = {}
    non_regression = 0
    regression_pass = True
    for family in FAMILIES:
        family_rows = [row for row in rows if row["family"] == family]
        _require(len(family_rows) == 3, "{} evidence count drifted".format(family))
        changes = []
        for row in family_rows:
            fixed = fixed_by_scene[row["scene_id"]]
            reference = _number(
                fixed["metrics"]["common"]["navigation_time_s"],
                "fixed navigation time", 0.0)
            _require(reference > 0.0, "fixed navigation time must be positive")
            candidate = _number(
                row["metrics"]["common"]["navigation_time_s"],
                "candidate navigation time", 0.0)
            changes.append(100.0 * (candidate - reference) / reference)
        median = _median(changes)
        family_time[family] = {
            "paired_relative_changes_percent": changes,
            "median_relative_change_percent": median,
        }
        non_regression += int(median <= 0.0)
        regression_pass &= (
            median <= gate["family_median_navigation_time_regression_percent_max"]
        )
    return family_time, non_regression, regression_pass


def _candidate_summary(candidate_id, rows, fixed_rows, fixed_valid, contract):
    gates = contract["hard_gates"]
    success = sum(row["metrics"]["common"]["success"] for row in rows)
    collision = sum(row["metrics"]["common"]["collision"] for row in rows)
    fixed_success = sum(row["metrics"]["common"]["success"] for row in fixed_rows)
    successful = [row for row in rows if row["metrics"]["common"]["success"]]
    clearance = [
        _number(row["metrics"]["common"]["minimum_clearance_m"], "minimum clearance")
        for row in successful
    ]
    typed_invalid = sum(row.get("typed_transaction_valid") is not True for row in rows)
    ttc_counts = _method_ttc(rows)
    switches = [
        _integer(row["active_anchor_switch_count"], "active anchor switch count")
        for row in rows
    ]
    join = _join_gate(rows, gates["bounded_context_join"])
    maneuver = [row for row in rows if row["family"] == "MANEUVER"]
    successful_maneuver = [
        row for row in maneuver if row["metrics"]["common"]["success"]
    ]
    scan = [
        _number(
            row["clearance_audit"].get("minimum_signed_scan_clearance_m"),
            "signed Maneuver scan clearance")
        for row in successful_maneuver
    ]
    truth = [
        _number(
            row["clearance_audit"].get("minimum_truth_box_clearance_m"),
            "Maneuver truth clearance")
        for row in successful_maneuver
    ]
    contact_pass = all(
        _integer(row["clearance_audit"].get("contact_count"), "Maneuver contact count")
        <= gates["maneuver_clearance_preservation"]["contact_count_max_per_maneuver"]
        for row in maneuver
    )
    reverse_episode_count = sum(
        _integer(
            row.get("mechanism_maneuver_reverse_sample_count", 0),
            "Maneuver reverse samples") > 0
        for row in maneuver
    )
    reverse_sample_count = sum(
        _integer(
            row.get("mechanism_maneuver_reverse_sample_count", 0),
            "Maneuver reverse samples")
        for row in maneuver
    )
    mechanism = {
        "topology_locked_sample_count": sum(
            _integer(row.get("mechanism_topology_locked_sample_count", 0),
                     "topology locked samples") for row in rows),
        "corridor_centerline_sample_count": sum(
            _integer(row.get("mechanism_corridor_centerline_sample_count", 0),
                     "corridor centerline samples") for row in rows),
        "maneuver_reverse_sample_count": sum(
            _integer(row.get("mechanism_maneuver_reverse_sample_count", 0),
                     "maneuver reverse samples") for row in rows),
        "topology_switch_max": max(
            _integer(row.get("mechanism_topology_switch_count", 0),
                     "mechanism topology switches") for row in rows),
    }
    mechanism_pass = True
    if candidate_id in ELIGIBLE_IDS:
        limit = gates["mechanism_activation_for_non_control_candidates"]
        mechanism_pass = (
            mechanism["topology_locked_sample_count"]
            >= limit["topology_locked_sample_count_min"]
            and mechanism["corridor_centerline_sample_count"]
            >= limit["corridor_centerline_sample_count_min"]
            and mechanism["maneuver_reverse_sample_count"]
            >= limit["maneuver_reverse_sample_count_min"]
            and mechanism["topology_switch_max"]
            <= limit["mechanism_topology_switch_count_max_per_episode"]
        )
    reverse_gate = gates["mechanism_activation"]
    reverse_pass = (
        reverse_episode_count >= reverse_gate["maneuver_reverse_episode_count_min"]
        and reverse_sample_count >= reverse_gate["maneuver_reverse_sample_count_min"]
    )
    fixed_by_scene = {row["scene_id"]: row for row in fixed_rows}
    family_time, non_regression, family_regression_pass = _family_timing(
        rows, fixed_by_scene, gates["efficiency_vs_fixed"])
    priority_gate = gates["priority_family_efficiency"]
    priority_changes = {
        family: family_time[family]["median_relative_change_percent"]
        for family in priority_gate["families"]
    }
    priority_pass = all(
        value <= priority_gate["median_navigation_time_regression_percent_max"]
        for value in priority_changes.values())
    total_time = sum(
        _number(row["metrics"]["common"]["navigation_time_s"], "navigation time", 0.0)
        for row in rows)
    fixed_time = sum(
        _number(row["metrics"]["common"]["navigation_time_s"], "fixed time", 0.0)
        for row in fixed_rows)
    _require(fixed_time > 0.0, "fixed total navigation time must be positive")
    ratio = total_time / fixed_time
    overlay_count = _overlay_conflict_count(rows)
    candidate_gate = gates["candidate_safety"]
    clearance_gate = gates["maneuver_clearance_preservation"]
    hard_gates = {
        "complete": len(rows) == 15,
        "success_non_degradation": success >= fixed_success,
        "collision": collision <= candidate_gate["collision_count_max"],
        "minimum_clearance": (
            bool(clearance)
            and min(clearance)
            >= candidate_gate["minimum_clearance_m_min_per_successful_episode"]
        ),
        "typed_transaction": (
            typed_invalid <= candidate_gate["typed_transaction_invalid_count_max"]
        ),
        "boundary": all(
            row.get("training_used") is False
            and row.get("runtime_policy_manifest_access") is False
            and row.get("runtime_scene_labels_available") is False
            and row.get("clearance_audit", {}).get("runtime_policy_received_truth") is False
            for row in rows
        ),
        "ttc": _ttc_gate_pass(ttc_counts, gates["ttc"]),
        "dynamic_overlay_activation": (
            overlay_count
            >= gates["dynamic_overlay_activation"][
                "conflict_episode_with_non_none_overlay_count_min_per_candidate"]
        ),
        "chatter": (
            max(switches) <= gates["chatter"][
                "active_anchor_switch_count_max_per_episode"]
            and sum(switches) / len(switches)
            <= gates["chatter"]["active_anchor_switch_count_mean_max"]
        ),
        "mechanism_activation_for_non_control_candidate": mechanism_pass,
        "maneuver_reverse_observability": reverse_pass,
        "bounded_context_join": join["pass"],
        "maneuver_scan_clearance": (
            bool(scan)
            and min(scan)
            >= clearance_gate[
                "minimum_signed_scan_clearance_m_per_successful_maneuver"]
        ),
        "maneuver_truth_clearance": (
            bool(truth)
            and min(truth)
            >= clearance_gate[
                "minimum_truth_box_clearance_m_per_successful_maneuver"]
        ),
        "maneuver_zero_contact": contact_pass,
        "total_time": (
            ratio <= gates["efficiency_vs_fixed"]["total_navigation_time_ratio_max"]
        ),
        "family_time_regression": family_regression_pass,
        "family_non_regression_count": (
            non_regression
            >= gates["efficiency_vs_fixed"]["family_non_regression_count_min"]
        ),
        "priority_family_efficiency": priority_pass,
    }
    return {
        "episode_count": len(rows),
        "success_count": success,
        "collision_count": collision,
        "winner_eligible": candidate_id in ELIGIBLE_IDS,
        "selection_eligible": candidate_id in ELIGIBLE_IDS,
        "minimum_clearance_m": min(clearance) if clearance else None,
        "minimum_signed_scan_clearance_m": min(scan) if scan else None,
        "minimum_truth_maneuver_clearance_m": min(truth) if truth else None,
        "typed_transaction_invalid_count": typed_invalid,
        "active_anchor_switch_max": max(switches),
        "active_anchor_switch_mean": sum(switches) / len(switches),
        "ttc_counts": ttc_counts,
        "conflict_episode_with_non_none_overlay_count": overlay_count,
        "bounded_context_join": join,
        "mechanism": mechanism,
        "maneuver_reverse_episode_count": reverse_episode_count,
        "family_time": family_time,
        "family_non_regression_count": non_regression,
        "priority_family_median_time_changes_percent": priority_changes,
        "total_navigation_time_s": total_time,
        "total_navigation_time_ratio_vs_fixed": ratio,
        "hard_gates": hard_gates,
        "all_hard_gates_pass": fixed_valid and all(hard_gates.values()),
    }


def assess_navigation(preregistration, contract, candidate_bank, progress,
                      evaluations, scene_families):
    """Assess a terminal prefix or all 60 navigation evaluations."""

    _require(isinstance(progress, dict), "navigation progress is missing")
    _require(
        progress.get("stage") == STAGE
        and progress.get("simulation_only") is True
        and progress.get("calibration_only") is True
        and progress.get("formal_result") is False
        and progress.get("runtime_ready") is False
        and progress.get("training_started") is False
        and progress.get("real_vehicle_used") is False,
        "navigation progress safety boundary drifted",
    )
    _attempt_boundary(progress, "navigation")
    marker = _terminal_marker(progress, "navigation")
    schedule, rows = _progress_records(
        preregistration, progress, evaluations, scene_families)
    _validate_navigation_attempt_ledger(
        progress, schedule, len(rows), marker is not None)
    if marker is not None:
        return _terminal_navigation_result(schedule, progress, rows)
    _require(progress.get("status") == "complete", "navigation status is not complete")
    _require(
        len(rows) == 60
        and progress.get("planned_navigation_episode_count") == 60
        and progress.get("valid_evidence_episode_count") == 60
        and progress.get("attempted_identity_count") == 60
        and progress.get("interface_failure_count") == 0,
        "navigation evidence is not the exact complete 60",
    )
    _require(
        progress.get("schedule_sha256") == NAVIGATION_SCHEDULE_SHA256,
        "navigation schedule hash differs from the frozen dry-run audit",
    )
    eligibility = {
        row["candidate_id"]: bool(row["winner_eligible"])
        for row in candidate_bank["candidates"]
    }
    _require(
        tuple(eligibility) == CANDIDATE_IDS
        and tuple(candidate for candidate, value in eligibility.items() if value)
        == ELIGIBLE_IDS
        and eligibility[CONTROL_ID] is False,
        "candidate eligibility boundary drifted",
    )
    fixed = [
        row for row in rows if row["supervisor_profile_id"] == "fixed_reference"
    ]
    fixed_success = sum(row["metrics"]["common"]["success"] for row in fixed)
    fixed_collision = sum(row["metrics"]["common"]["collision"] for row in fixed)
    fixed_ttc = _method_ttc(fixed)
    fixed_gate = contract["hard_gates"]["fixed_reference_validity"]
    fixed_valid = (
        len(fixed) == 15
        and fixed_success >= fixed_gate["success_count_min"]
        and fixed_collision <= fixed_gate["collision_count_max"]
        and _ttc_gate_pass(fixed_ttc, contract["hard_gates"]["ttc"])
    )
    summaries = {}
    for candidate in CANDIDATE_IDS:
        candidate_rows = [
            row for row in rows if row["supervisor_profile_id"] == candidate
        ]
        summaries[candidate] = _candidate_summary(
            candidate, candidate_rows, fixed, fixed_valid, contract)
    passing = [
        candidate for candidate in ELIGIBLE_IDS
        if summaries[candidate]["all_hard_gates_pass"]
    ]
    passing.sort(key=lambda candidate: (
        summaries[candidate]["total_navigation_time_ratio_vs_fixed"],
        -summaries[candidate]["minimum_truth_maneuver_clearance_m"],
        -summaries[candidate]["minimum_clearance_m"],
        summaries[candidate]["active_anchor_switch_mean"],
        candidate,
    ))
    return {
        "complete": True,
        "pass": bool(passing),
        "terminal": False,
        "terminal_failure": None,
        "evidence_count": 60,
        "fixed_reference": {
            "episode_count": len(fixed),
            "success_count": fixed_success,
            "collision_count": fixed_collision,
            "ttc_counts": fixed_ttc,
            "validity_gate_pass": fixed_valid,
        },
        "candidate_summaries": summaries,
        "qualified_candidate_ids": passing,
        "ranked_qualified_candidate_ids": passing,
        "reported_candidate_id": passing[0] if passing else None,
    }


def _decision_boundary():
    return {
        "winner_declared": False,
        "winner_freeze_attempted": False,
        "winner_freeze_authorized": False,
        "freeze_authorized": False,
        "winner_configuration_generated": False,
        "held_out_validation_authorized": False,
        "generate_fresh_held_out_validation_authorized": False,
        "enter_v2_05_authorized": False,
        "sac_training_authorized": False,
        "real_vehicle_authorized": False,
        "real_vehicle_motion_authorized": False,
        "real_vehicle_parameter_write_authorized": False,
    }


def _terminal_assessment(phase, readiness, component=None, navigation=None,
                         evidence=None):
    counts = {
        "readiness": readiness.get("evidence_count", 0),
        "ttc_component": component.get("evidence_count", 0) if component else 0,
        "navigation": navigation.get("evidence_count", 0) if navigation else 0,
    }
    valid_counts = {
        "readiness": readiness.get(
            "valid_evidence_count", readiness.get("evidence_count", 0)),
        "ttc_component": (
            component.get(
                "valid_evidence_count", component.get("evidence_count", 0))
            if component else 0
        ),
        "navigation": (
            navigation.get(
                "valid_evidence_count", navigation.get("evidence_count", 0))
            if navigation else 0
        ),
    }
    result = navigation or component or readiness
    return {
        "schema_version": "2.0",
        "stage": STAGE,
        "status": "calibration_terminally_stopped",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "evidence_complete": False,
        "evidence_unit_counts": {
            **counts,
            "total": sum(counts.values()),
            "preregistered_budget": 69,
        },
        "valid_evidence_counts": {
            **valid_counts,
            "total": sum(valid_counts.values()),
        },
        "terminal_stop": {
            "phase": phase,
            "failure": result["terminal_failure"],
            "preserved": True,
            "retry_forbidden": True,
            "resume_forbidden": True,
        },
        "ranking_performed": False,
        "qualified_candidate_ids": [],
        "ranked_qualified_candidate_ids": [],
        "reported_candidate_id": None,
        "winner_candidate_id": None,
        "evidence": evidence or {},
        "decision": _decision_boundary(),
    }


def assess_documents(preregistration, contract, candidate_bank, readiness_summary,
                     component_report=None, progress=None, evaluations=None,
                     scene_families=None, evidence=None):
    """Pure-document entry point used by the CLI and offline unit tests."""

    _require(
        preregistration.get("stage") == STAGE
        and contract.get("stage") == STAGE
        and candidate_bank.get("stage") == STAGE,
        "R5 assessment input boundary drifted",
    )
    readiness = assess_readiness(preregistration, contract, readiness_summary)
    if readiness["terminal"]:
        _require(
            component_report is None and progress is None,
            "evidence exists after readiness terminal stop",
        )
        return _terminal_assessment("readiness", readiness, evidence=evidence)
    component = assess_component(preregistration, component_report)
    if component["terminal"]:
        _require(progress is None, "navigation evidence exists after TTC terminal stop")
        return _terminal_assessment(
            "ttc_component", readiness, component=component, evidence=evidence)
    _require(evaluations is not None, "navigation evaluations are missing")
    _require(scene_families is not None, "navigation scene-family map is missing")
    navigation = assess_navigation(
        preregistration, contract, candidate_bank, progress, evaluations,
        scene_families)
    if navigation["terminal"]:
        return _terminal_assessment(
            "navigation", readiness, component=component,
            navigation=navigation, evidence=evidence)
    _require(
        readiness["evidence_count"] + component["evidence_count"]
        + navigation["evidence_count"] == 69,
        "complete R5 assessment must contain exactly 69 evidence units",
    )
    passing = navigation["qualified_candidate_ids"]
    status = (
        "calibration_complete_candidate_report_only"
        if passing else "calibration_complete_no_candidate"
    )
    return {
        "schema_version": "2.0",
        "stage": STAGE,
        "status": status,
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "single_changed_factor": "supervisor.dynamic.predicted_ttc_max_s",
        "evidence_complete": True,
        "evidence_unit_counts": {
            "readiness": 6,
            "ttc_component": 3,
            "navigation": 60,
            "total": 69,
            "preregistered_budget": 69,
        },
        "readiness": readiness,
        "ttc_component": component,
        "fixed_reference": navigation["fixed_reference"],
        "candidate_summaries": navigation["candidate_summaries"],
        "ranking_performed": True,
        "ranking_rule": preregistration["ranking_after_all_hard_gates"],
        "qualified_candidate_ids": passing,
        "ranked_qualified_candidate_ids": passing,
        "reported_candidate_id": navigation["reported_candidate_id"],
        "winner_candidate_id": None,
        "winner_declared": False,
        "evidence": evidence or {},
        "decision": _decision_boundary(),
    }


def _load_embedded_reports(workspace, artifact_root, summary, key):
    loaded = []
    for index, record in enumerate(summary.get(key, [])):
        value = dict(record)
        path_value = record.get("report_path")
        if path_value is not None:
            path = _resolve_declared_path(workspace, path_value, "{} report".format(key))
            _inside(artifact_root, path, "{} report".format(key))
            _require(
                _sha256(path) == record.get("report_sha256"),
                "{} report hash drifted".format(key),
            )
            report = _load_yaml(path)
            report.update(value)
            value = report
        evaluation_value = record.get("evaluation")
        if evaluation_value is not None:
            evaluation_path = _resolve_declared_path(
                workspace, evaluation_value, "{} evaluation".format(key))
            _inside(
                artifact_root, evaluation_path,
                "{} evaluation".format(key))
            _require(
                _sha256(evaluation_path) == record.get("evaluation_sha256"),
                "{} evaluation hash drifted".format(key),
            )
            evaluation = _load_yaml(evaluation_path)
            trace_path = evaluation_path.parent / "trace.csv"
            _inside(artifact_root, trace_path, "{} trace".format(key))
            load_v2_trace(trace_path)
            digest = trace_sha256(trace_path)
            _require(
                digest == evaluation.get("raw_trace_sha256")
                and digest == record.get("trace_sha256"),
                "{} evaluation trace hash drifted".format(key),
            )
            _require(
                evaluation.get("stage") == STAGE
                and evaluation.get("scene_id") == record.get("scene_id")
                and evaluation.get("seed") == record.get("seed")
                and evaluation.get("supervisor_profile_id")
                == record.get("profile_id")
                and evaluation.get("ttc_status")
                == record.get("observed_ttc_status")
                and evaluation.get("tracker_message_count")
                == record.get("tracker_message_count"),
                "{} evaluation identity/TTC evidence drifted".format(key),
            )
        loaded.append(value)
    result = dict(summary)
    result[key] = loaded
    return result


def _load_scene_families(workspace, preregistration):
    resource = preregistration["resources"]["navigation_scene_manifest"]
    path = _resolve_declared_path(workspace, resource["path"], "navigation scene manifest")
    manifest = _load_yaml(path)
    return {
        scene["scene_id"]: scene["family"]
        for scene in manifest["scenes"]
    }


def _load_navigation_evaluations(workspace, artifact_root, progress):
    evaluations = []
    for index, record in enumerate(progress.get("episodes", [])):
        label = "navigation episode {}".format(index + 1)
        evaluation_path = _resolve_declared_path(
            workspace, record["evaluation"], label + " evaluation")
        _inside(artifact_root, evaluation_path, label + " evaluation")
        _require(
            _sha256(evaluation_path) == record["evaluation_sha256"],
            "{} evaluation hash drifted".format(label),
        )
        evaluation = _load_yaml(evaluation_path)
        trace_path = evaluation_path.parent / "trace.csv"
        _inside(artifact_root, trace_path, label + " trace")
        load_v2_trace(trace_path)
        digest = trace_sha256(trace_path)
        _require(
            digest == evaluation["raw_trace_sha256"]
            and digest == record["trace_sha256"],
            "{} raw trace hash drifted".format(label),
        )
        evaluations.append(evaluation)
    return evaluations


def _load_optional(path):
    if path is None or not Path(path).is_file():
        return None
    return _load_yaml(path)


def assess_files(workspace, preregistration_path, dry_run_audit_path,
                 readiness_path, component_path, progress_path):
    """File-backed assessment with frozen hashes and raw-evidence verification."""

    frozen = verify_frozen_inputs(
        workspace, preregistration_path, dry_run_audit_path)
    root = Path(workspace).resolve()
    artifact_root = _inside(
        root, root / "artifacts/v2/calibration/v2_04g_r5", "R5 artifact root")
    readiness_file = _inside(artifact_root, readiness_path, "readiness summary")
    _require(readiness_file.is_file(), "readiness summary is missing")
    readiness_raw = _load_yaml(readiness_file)
    _require(
        readiness_raw.get("preregistration", {}).get("sha256")
        == PREREGISTRATION_SHA256,
        "readiness summary does not bind the frozen preregistration",
    )
    readiness = _load_embedded_reports(
        root, artifact_root, readiness_raw, "reports")
    component_file = (
        _inside(artifact_root, component_path, "TTC component report")
        if component_path is not None else None
    )
    progress_file = (
        _inside(artifact_root, progress_path, "navigation progress")
        if progress_path is not None else None
    )
    component = _load_optional(component_file)
    progress = _load_optional(progress_file)
    if component is not None:
        _require(
            component.get("preregistration", {}).get("sha256")
            == PREREGISTRATION_SHA256
            and component.get("readiness_summary", {}).get("sha256")
            == _sha256(readiness_file),
            "TTC component evidence-chain hash drifted",
        )
    if progress is not None:
        _require(
            progress.get("preregistration", {}).get("sha256")
            == PREREGISTRATION_SHA256
            and progress.get("readiness_summary", {}).get("sha256")
            == _sha256(readiness_file)
            and component_file is not None
            and component_file.is_file()
            and progress.get("ttc_component_probe", {}).get("sha256")
            == _sha256(component_file),
            "navigation evidence-chain hash drifted",
        )
    evaluations = (
        _load_navigation_evaluations(root, artifact_root, progress)
        if progress is not None else None
    )
    evidence = {
        "preregistration": {
            "path": str(frozen["paths"]["preregistration"]),
            "sha256": PREREGISTRATION_SHA256,
        },
        "dry_run_audit": {
            "path": str(frozen["paths"]["dry_run_audit"]),
            "sha256": DRY_RUN_AUDIT_SHA256,
        },
        "contract": {
            "path": str(frozen["paths"]["contract"]),
            "sha256": _sha256(frozen["paths"]["contract"]),
        },
        "candidate_bank": {
            "path": str(frozen["paths"]["candidate_bank"]),
            "sha256": _sha256(frozen["paths"]["candidate_bank"]),
        },
        "execution_authorization": {
            "path": str(frozen["paths"]["execution_authorization"]),
            "sha256": _sha256(frozen["paths"]["execution_authorization"]),
        },
        "readiness": {
            "path": str(readiness_file), "sha256": _sha256(readiness_file),
        },
    }
    if component_file is not None and component_file.is_file():
        evidence["ttc_component"] = {
            "path": str(component_file), "sha256": _sha256(component_file),
        }
    if progress_file is not None and progress_file.is_file():
        evidence["navigation_progress"] = {
            "path": str(progress_file), "sha256": _sha256(progress_file),
        }
    return assess_documents(
        frozen["preregistration"],
        frozen["contract"],
        frozen["candidate_bank"],
        readiness,
        component_report=component,
        progress=progress,
        evaluations=evaluations,
        scene_families=_load_scene_families(root, frozen["preregistration"]),
        evidence=evidence,
    )


def _write_atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument(
        "--preregistration", type=Path,
        default=WORKSPACE / (
            "experiments/manifests/v2/calibration/"
            "v2_04g_r5_preregistration.yaml"))
    parser.add_argument(
        "--dry-run-audit", type=Path,
        default=WORKSPACE / (
            "artifacts/v2/calibration/v2_04g_r5/"
            "v2_04g_r5_dry_run_audit.yaml"))
    parser.add_argument(
        "--readiness", type=Path,
        default=WORKSPACE / (
            "artifacts/v2/calibration/v2_04g_r5/readiness/"
            "v2_04g_r5_readiness_summary.yaml"))
    parser.add_argument(
        "--ttc-component", type=Path,
        default=WORKSPACE / (
            "artifacts/v2/calibration/v2_04g_r5/"
            "v2_04g_r5_ttc_three_state_probe.yaml"))
    parser.add_argument(
        "--progress", type=Path,
        default=WORKSPACE / (
            "artifacts/v2/calibration/v2_04g_r5/"
            "v2_04g_r5_progress.yaml"))
    parser.add_argument(
        "--output", type=Path,
        default=WORKSPACE / (
            "artifacts/v2/calibration/v2_04g_r5/"
            "v2_04g_r5_assessment.yaml"))
    args = parser.parse_args()
    report = assess_files(
        args.workspace, args.preregistration, args.dry_run_audit,
        args.readiness, args.ttc_component, args.progress)
    _write_atomic_yaml(args.output, report)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
