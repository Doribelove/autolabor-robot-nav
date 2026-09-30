#!/usr/bin/env python3
"""Create descriptive tables and paper figures from the Arena Q1 evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

import run_qualification as runner


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROOT = ROOT / "artifacts/arena_pct_teb_dwa_qualification"
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")
LABELS = {
    "pct_scope_rl": "PCT-SCOPE-RL",
    "analytic_prior": "Analytic prior",
    "actual_teb": "Actual TEB",
    "dwa": "DWA",
}
COLORS = {
    "pct_scope_rl": "#0B6E4F",
    "analytic_prior": "#4C78A8",
    "actual_teb": "#F58518",
    "dwa": "#B279A2",
}
CAP_S = 30.0
EXPECTED_SOURCE_WORLD_SHA256 = (
    "818a5eab5a9eedc43050d2fb3a2345d267dd32843c4139f29d58d06b396e2c95"
)
SOURCE_WORLD = ROOT / (
    "src/navigation_arena/arena-rosnav-3D/simulator_setup/worlds/"
    "hospital/worlds/hospital.world"
)


def finite(value):
    return value is not None and math.isfinite(float(value))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json_or_error(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8")), None
    except Exception as error:
        return None, type(error).__name__


def read_yaml_or_error(path):
    try:
        return yaml.safe_load(Path(path).read_text(encoding="utf-8")), None
    except Exception as error:
        return None, type(error).__name__


def validate_raw_evidence_closure(
    output_root, expected_matrix_sha256, expected_seal_sha256
):
    output_root = Path(output_root).resolve()
    seal_path = output_root / "q1_raw_evidence_seal.json"
    manifest_path = output_root / "q1_raw_evidence_manifest.json"
    matrix_path = output_root / "q1/matrix_receipt.json"
    journal_path = output_root / "q1/matrix_journal.json"
    review_path = output_root / "q1_release_review.json"
    release_path = output_root / "q1_execution_release.yaml"
    if sha256(seal_path) != str(expected_seal_sha256):
        raise RuntimeError("RAW_EVIDENCE_SEAL_SHA_ARGUMENT_MISMATCH")
    seal = json.loads(seal_path.read_text(encoding="utf-8"))
    if (
        seal.get("stage") != "ARENA-PCT-Q1-RAW-EVIDENCE-SEAL"
        or seal.get("terminal") is not True
    ):
        raise RuntimeError("RAW_EVIDENCE_SEAL_INVALID")
    if sha256(manifest_path) != seal.get("raw_evidence_manifest_sha256"):
        raise RuntimeError("RAW_EVIDENCE_MANIFEST_HASH_INVALID")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    observed_matrix_sha256 = sha256(matrix_path)
    if (
        observed_matrix_sha256 != str(expected_matrix_sha256)
        or seal.get("matrix_receipt_sha256") != observed_matrix_sha256
        or manifest.get("matrix_receipt_sha256") != observed_matrix_sha256
    ):
        raise RuntimeError("RAW_EVIDENCE_MATRIX_HASH_INVALID")
    observed = runner.raw_evidence_records(output_root)
    if manifest.get("files") != observed:
        raise RuntimeError("RAW_EVIDENCE_PATH_SET_OR_HASH_MISMATCH")
    checks = {
        "release_sha256": sha256(release_path),
        "release_review_sha256": sha256(review_path),
        "terminal_journal_sha256": sha256(journal_path),
        "matrix_receipt_sha256": observed_matrix_sha256,
    }
    for key, value in checks.items():
        if manifest.get(key) != value or seal.get(key) != value:
            raise RuntimeError("RAW_EVIDENCE_CHAIN_HASH_INVALID:" + key)
    if int(seal.get("raw_file_count", -1)) != len(observed):
        raise RuntimeError("RAW_EVIDENCE_FILE_COUNT_INVALID")
    matrix_payload = json.loads(matrix_path.read_text(encoding="utf-8"))
    expected_identities = [
        {
            "seed": int(row["seed"]),
            "scene_id": str(row["scene_id"]),
            "method": str(row["method"]),
            "attempt": int(row["attempt"]),
        }
        for row in matrix_payload.get("episodes", [])
    ]
    if seal.get("episode_identities") != expected_identities:
        raise RuntimeError("RAW_EVIDENCE_IDENTITY_SET_INVALID")
    return {
        **checks,
        "raw_evidence_manifest_sha256": sha256(manifest_path),
        "raw_evidence_seal_sha256": sha256(seal_path),
    }


def resolve_release_resource(row):
    if row.get("scope") == "workspace_relative":
        return (ROOT / str(row["path"])).absolute()
    if row.get("scope") == "absolute_system":
        return Path(str(row["path"])).absolute()
    raise RuntimeError("RELEASE_RESOURCE_SCOPE_INVALID:" + str(row))


def verify_release_resources(release):
    failures = []
    seen = set()
    for name in ("resources", "system_resources", "hospital_assets"):
        for row in release.get(name, []):
            path = resolve_release_resource(row)
            identity = (str(row.get("scope")), str(path))
            if identity in seen:
                failures.append(str(path) + ":duplicate")
                continue
            seen.add(identity)
            if not path.is_file():
                failures.append(str(path) + ":absent")
                continue
            if "symlink_target" in row and (
                not path.is_symlink()
                or os.readlink(str(path)) != str(row["symlink_target"])
            ):
                failures.append(str(path) + ":symlink_target_mismatch")
            if sha256(path) != str(row.get("sha256")):
                failures.append(str(path) + ":hash_mismatch")
            if path.stat().st_size != int(row.get("size_bytes", -1)):
                failures.append(str(path) + ":size_mismatch")
    if failures:
        raise RuntimeError("ANALYSIS_RELEASE_REHASH_FAILED:" + ",".join(failures))
    return len(seen)


def validate_matrix(q1_root):
    matrix_path = q1_root / "matrix_receipt.json"
    release_path = q1_root.parent / "q1_execution_release.yaml"
    journal_path = q1_root / "matrix_journal.json"
    matrix = json.loads(matrix_path.read_text(encoding="utf-8"))
    release = yaml.safe_load(release_path.read_text(encoding="utf-8"))
    journal = json.loads(journal_path.read_text(encoding="utf-8"))
    resource_count = verify_release_resources(release)
    runner.verify_release_resources(
        release_path, expected_release_sha256=sha256(release_path)
    )
    if matrix.get("release_sha256") != sha256(release_path):
        raise RuntimeError("MATRIX_RELEASE_HASH_MISMATCH")
    if (
        matrix.get("source_world_sha256_after")
        != EXPECTED_SOURCE_WORLD_SHA256
        or sha256(SOURCE_WORLD) != EXPECTED_SOURCE_WORLD_SHA256
    ):
        raise RuntimeError("MATRIX_SOURCE_WORLD_HASH_INVALID")
    if (
        matrix.get("stage") != "ARENA-PCT-Q1"
        or not matrix.get("terminal")
        or journal.get("terminal") is not True
        or journal.get("aborted") is not False
        or journal.get("abort_reason") is not None
        or journal.get("in_progress") is not None
        or journal.get("release_sha256") != sha256(release_path)
        or journal.get("release_review_sha256")
        != matrix.get("release_review_sha256")
        or journal.get("episodes") != matrix.get("episodes")
        or matrix.get("terminal_journal_path")
        != str(journal_path.relative_to(ROOT))
        or matrix.get("terminal_journal_sha256") != sha256(journal_path)
    ):
        raise RuntimeError("MATRIX_TERMINAL_JOURNAL_INVALID")
    expected = [
        (int(row["seed"]), str(row["scene"]), method)
        for row in release["schedule"]
        for method in row["method_order"]
    ]
    observed = [
        (int(row["seed"]), str(row["scene_id"]), str(row["method"]))
        for row in matrix["episodes"]
    ]
    if len(matrix["episodes"]) != 12 or observed != expected:
        raise RuntimeError("MATRIX_IDENTITY_SET_INVALID")
    review_path = q1_root.parent / "q1_release_review.json"
    if matrix.get("release_review_path") != str(review_path.relative_to(ROOT)):
        raise RuntimeError("MATRIX_RELEASE_REVIEW_PATH_INVALID")
    review = json.loads(review_path.read_text(encoding="utf-8"))
    if (
        sha256(review_path) != matrix.get("release_review_sha256")
        or review.get("status") != "arena_q1_release_preflight_pass"
        or review.get("release_sha256") != sha256(release_path)
        or not review.get("execution_ready")
        or review.get("exact_release_payload_match") is not True
        or review.get("exact_resource_roster_match") is not True
        or review.get("source_world_hash_pass") is not True
    ):
        raise RuntimeError("MATRIX_RELEASE_REVIEW_INVALID")
    return matrix, release, release_path, sha256(release_path), resource_count


def identity_matches(payload, identity, stage="ARENA-PCT-Q1"):
    return bool(
        payload
        and payload.get("stage") == stage
        and int(payload.get("seed", -1)) == identity[0]
        and str(payload.get("scene_id")) == identity[1]
        and str(payload.get("method")) == identity[2]
    )


def recompute_contact_monitor(report):
    monitor = report.get("contact_monitor") if report else None
    failures = []
    if not isinstance(monitor, dict):
        return False, False, None, ["contact_monitor_absent"]
    try:
        raw_stamps = list(monitor["message_stamps_s"])
        valid_stamps = [
            float(value)
            for value in raw_stamps
            if value is not None and finite(value)
        ]
        invalid_count = sum(
            value is None or not finite(value) for value in raw_stamps
        )
        intervals = [
            right - left for left, right in zip(valid_stamps[:-1], valid_stamps[1:])
        ]
        regression_count = sum(value < -1.0e-9 for value in intervals)
        positive_intervals = [value for value in intervals if value >= 0.0]
        start_s = float(monitor["monitor_start_s"])
        end_s = float(monitor["monitor_end_s"])
        first_gap = valid_stamps[0] - start_s if valid_stamps else None
        last_gap = end_s - valid_stamps[-1] if valid_stamps else None
        gap_candidates = list(positive_intervals)
        if first_gap is not None:
            gap_candidates.append(max(0.0, first_gap))
        if last_gap is not None:
            gap_candidates.append(max(0.0, last_gap))
        maximum_gap = max(gap_candidates) if gap_candidates else None
        observed_rate = (
            (len(valid_stamps) - 1) / (valid_stamps[-1] - valid_stamps[0])
            if len(valid_stamps) >= 2 and valid_stamps[-1] > valid_stamps[0]
            else None
        )
        raw_state_counts = list(monitor["message_state_counts"])
        if any(
            type(value) is not int or value < 0 for value in raw_state_counts
        ):
            raise ValueError("invalid contact message state count")
        message_count = int(monitor["message_count"])
        empty_count = int(monitor["empty_message_count"])
        nonempty_count = int(monitor["nonempty_message_count"])
        state_count = int(monitor["contact_state_sample_count"])
        derived_empty_count = sum(value == 0 for value in raw_state_counts)
        derived_nonempty_count = sum(value > 0 for value in raw_state_counts)
        derived_state_count = sum(raw_state_counts)
        frame_counts = dict(monitor["frame_id_counts"])
        readiness = dict(monitor["readiness"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False, False, None, ["contact_monitor_schema_invalid"]

    def same_optional_float(observed, expected, tolerance=1.0e-8):
        if observed is None or expected is None:
            return observed is None and expected is None
        return finite(observed) and abs(float(observed) - float(expected)) <= tolerance

    if (
        message_count != len(raw_stamps)
        or message_count != len(raw_state_counts)
        or message_count != empty_count + nonempty_count
    ):
        failures.append("contact_message_count_inconsistent")
    if (
        empty_count != derived_empty_count
        or nonempty_count != derived_nonempty_count
        or state_count != derived_state_count
    ):
        failures.append("contact_state_counts_inconsistent")
    if invalid_count != int(monitor.get("invalid_stamp_count", -1)):
        failures.append("contact_invalid_stamp_count_inconsistent")
    if regression_count != int(monitor.get("timestamp_regression_count", -1)):
        failures.append("contact_timestamp_regression_count_inconsistent")
    if sum(int(value) for value in frame_counts.values()) != message_count:
        failures.append("contact_frame_count_inconsistent")
    frame_mismatch_count = sum(
        int(value) for frame, value in frame_counts.items() if frame != "base_link"
    )
    if frame_mismatch_count != int(monitor.get("frame_mismatch_count", -1)):
        failures.append("contact_frame_mismatch_count_inconsistent")
    for key, expected in (
        ("first_stamp_s", valid_stamps[0] if valid_stamps else None),
        ("last_stamp_s", valid_stamps[-1] if valid_stamps else None),
        ("observed_rate_hz", observed_rate),
        ("first_coverage_gap_s", first_gap),
        ("last_coverage_gap_s", last_gap),
        ("maximum_coverage_gap_s", maximum_gap),
    ):
        if not same_optional_float(monitor.get(key), expected):
            failures.append("contact_derived_value_mismatch:" + key)
    publisher_presence_pass = bool(
        monitor.get("topic") == "/m2_gazebo/contacts"
        and monitor.get("expected_type") == "gazebo_msgs/ContactsState"
        and monitor.get("expected_publisher_nodes") == ["/gazebo"]
        and monitor.get("expected_frame_id") == "base_link"
        and float(monitor.get("expected_update_rate_hz", -1.0)) == 100.0
        and float(monitor.get("minimum_observed_rate_hz", -1.0)) == 80.0
        and float(monitor.get("maximum_allowed_coverage_gap_s", -1.0)) == 0.05
        and readiness.get("topic_type") == "gazebo_msgs/ContactsState"
        and readiness.get("publisher_nodes") == ["/gazebo"]
        and int(readiness.get("subscriber_connections", 0)) >= 1
        and int(readiness.get("message_count", 0)) >= 5
        and monitor.get("end_topic_type") == "gazebo_msgs/ContactsState"
        and monitor.get("end_publisher_nodes") == ["/gazebo"]
        and int(monitor.get("end_subscriber_connections", 0)) >= 1
    )
    heartbeat_pass = bool(
        not failures
        and publisher_presence_pass
        and len(valid_stamps) >= 2
        and observed_rate is not None
        and observed_rate >= 80.0
        and maximum_gap is not None
        and maximum_gap <= 0.05
        and invalid_count == 0
        and regression_count == 0
        and frame_mismatch_count == 0
    )
    if monitor.get("publisher_presence_pass") is not publisher_presence_pass:
        failures.append("contact_publisher_pass_mismatch")
    if monitor.get("heartbeat_pass") is not heartbeat_pass:
        failures.append("contact_heartbeat_pass_mismatch")
    if monitor.get("no_contact_state_samples_observed") is not (state_count == 0):
        failures.append("contact_state_boolean_mismatch")
    if monitor.get("collision_sensitivity_positive_control_qualified") is not False:
        failures.append("contact_positive_control_claim_invalid")
    if state_count < 0 or (state_count > 0 and nonempty_count == 0):
        failures.append("contact_state_count_invalid")
    if (state_count == 0) != (nonempty_count == 0):
        failures.append("contact_nonempty_state_equivalence_invalid")
    if state_count < nonempty_count:
        failures.append("contact_state_count_below_nonempty_messages")
    schema_valid = not failures
    return schema_valid, bool(schema_valid and heartbeat_pass), state_count, failures


def collect(q1_root, matrix, release, release_sha256):
    matrix_by_identity = {
        (int(row["seed"]), str(row["scene_id"]), str(row["method"])): row
        for row in matrix["episodes"]
    }
    rows = []
    for scheduled in release["schedule"]:
        seed = int(scheduled["seed"])
        scene = str(scheduled["scene"])
        block = q1_root / "blocks" / "{}_{}".format(seed, scene)
        manifest_path = block / "snapshot/scene_manifest.yaml"
        snapshot_world_path = block / "snapshot/world.sdf"
        snapshot_receipt_path = block / "snapshot/snapshot_receipt.json"
        manifest, manifest_parse_error = (
            read_yaml_or_error(manifest_path)
            if manifest_path.is_file()
            else (None, None)
        )
        snapshot_receipt, snapshot_receipt_parse_error = (
            read_json_or_error(snapshot_receipt_path)
            if snapshot_receipt_path.is_file()
            else (None, None)
        )
        snapshot_failures = []
        if manifest_parse_error:
            snapshot_failures.append(
                "snapshot_manifest_parse_error:" + manifest_parse_error
            )
        if snapshot_receipt_parse_error:
            snapshot_failures.append(
                "snapshot_receipt_parse_error:" + snapshot_receipt_parse_error
            )
        if not manifest or not snapshot_receipt or not snapshot_world_path.is_file():
            snapshot_failures.append("snapshot_evidence_absent")
        else:
            if (
                manifest.get("stage") != "ARENA-PCT-Q1"
                or int(manifest.get("seed", -1)) != seed
                or manifest.get("scene_id") != scene
                or manifest.get("source_world_sha256")
                != EXPECTED_SOURCE_WORLD_SHA256
                or manifest.get("release_sha256") != release_sha256
                or manifest.get("release_review_sha256")
                != matrix.get("release_review_sha256")
            ):
                snapshot_failures.append("snapshot_manifest_identity_invalid")
            if (
                snapshot_receipt.get("source_world_sha256_before")
                != EXPECTED_SOURCE_WORLD_SHA256
                or snapshot_receipt.get("source_world_sha256_after")
                != EXPECTED_SOURCE_WORLD_SHA256
                or snapshot_receipt.get("snapshot_world_sha256")
                != sha256(snapshot_world_path)
                or snapshot_receipt.get("scene_manifest_sha256")
                != sha256(manifest_path)
                or snapshot_receipt.get("release_sha256") != release_sha256
                or snapshot_receipt.get("release_review_sha256")
                != matrix.get("release_review_sha256")
            ):
                snapshot_failures.append("snapshot_receipt_hash_invalid")
        for method in scheduled["method_order"]:
            episode_root = block / method
            report_path = episode_root / "episode_report.yaml"
            receipt_path = episode_root / "receipt.json"
            binding_path = episode_root / "preflight_binding.json"
            log_path = episode_root / "roslaunch.log"
            receipt, receipt_parse_error = (
                read_json_or_error(receipt_path)
                if receipt_path.is_file()
                else (None, None)
            )
            report, report_parse_error = (
                read_yaml_or_error(report_path)
                if report_path.is_file()
                else (None, None)
            )
            binding, binding_parse_error = (
                read_json_or_error(binding_path)
                if binding_path.is_file()
                else (None, None)
            )
            identity = (seed, scene, method)
            matrix_row = matrix_by_identity[identity]
            validation_failures = list(snapshot_failures)
            for label, parse_error in (
                ("receipt", receipt_parse_error),
                ("report", report_parse_error),
                ("binding", binding_parse_error),
            ):
                if parse_error:
                    validation_failures.append(
                        label + "_parse_error:" + parse_error
                    )
            if not receipt:
                validation_failures.append("receipt_absent")
            else:
                if not identity_matches(receipt, identity):
                    validation_failures.append("receipt_identity_invalid")
                if int(receipt.get("attempt", -1)) != 1:
                    validation_failures.append("receipt_attempt_invalid")
                if receipt.get("return_code") != 0:
                    validation_failures.append("runner_return_code_nonzero")
                if receipt.get("runner_timeout") is not False:
                    validation_failures.append("runner_timeout")
                if not receipt.get("post_process_group_clear"):
                    validation_failures.append("post_process_group_not_clear")
                if not receipt.get("post_uri_processes_clear"):
                    validation_failures.append("post_uri_processes_not_clear")
                if not receipt.get("post_uri_scan_complete"):
                    validation_failures.append("post_uri_scan_incomplete")
                if not receipt.get("post_ports_clear"):
                    validation_failures.append("post_ports_not_clear")
                if not receipt.get("post_port_probe_complete"):
                    validation_failures.append("post_port_probe_incomplete")
                if not receipt.get("cleanup_complete"):
                    validation_failures.append("cleanup_incomplete")
                if receipt.get("post_uri_processes"):
                    validation_failures.append("post_uri_processes_present")
                if receipt.get("post_unexpected_uri_scan_errors"):
                    validation_failures.append("unexpected_uri_scan_errors")
                if (
                    receipt.get("source_world_sha256_after")
                    != EXPECTED_SOURCE_WORLD_SHA256
                ):
                    validation_failures.append("receipt_source_hash_invalid")
                if receipt.get("report_sha256") != (
                    sha256(report_path) if report_path.is_file() else None
                ):
                    validation_failures.append("report_receipt_hash_invalid")
                if receipt.get("preflight_binding_sha256") != (
                    sha256(binding_path) if binding_path.is_file() else None
                ):
                    validation_failures.append("binding_receipt_hash_invalid")
                if receipt.get("roslaunch_log_sha256") != (
                    sha256(log_path) if log_path.is_file() else None
                ):
                    validation_failures.append("roslaunch_log_hash_invalid")
                if receipt.get("release_sha256") != release_sha256:
                    validation_failures.append("receipt_release_hash_invalid")
                if receipt.get("release_review_sha256") != matrix.get(
                    "release_review_sha256"
                ):
                    validation_failures.append("receipt_review_hash_invalid")
                if matrix_row.get("receipt_path") != str(
                    receipt_path.relative_to(ROOT)
                ):
                    validation_failures.append("matrix_receipt_path_invalid")
                if matrix_row.get("receipt_sha256") != sha256(receipt_path):
                    validation_failures.append("matrix_receipt_hash_invalid")
                for key, value in receipt.items():
                    if matrix_row.get(key) != value:
                        validation_failures.append("matrix_receipt_mismatch:" + key)
            if not binding:
                validation_failures.append("preflight_binding_absent")
            else:
                if not identity_matches(binding, identity):
                    validation_failures.append("binding_identity_invalid")
                if binding.get("release_sha256") != release_sha256:
                    validation_failures.append("binding_release_hash_invalid")
                if binding.get("release_review_sha256") != matrix.get(
                    "release_review_sha256"
                ):
                    validation_failures.append("binding_review_hash_invalid")
                if int(binding.get("attempt", -1)) != 1:
                    validation_failures.append("binding_attempt_invalid")
                if int(binding.get("ros_port", -1)) != int(
                    matrix_row.get("ros_port", -2)
                ) or int(binding.get("gazebo_port", -1)) != int(
                    matrix_row.get("gazebo_port", -2)
                ):
                    validation_failures.append("binding_ports_invalid")
                if manifest_path.is_file() and binding.get(
                    "scene_manifest_sha256"
                ) != sha256(manifest_path):
                    validation_failures.append("binding_manifest_hash_invalid")
                if snapshot_world_path.is_file() and binding.get(
                    "world_sha256"
                ) != sha256(snapshot_world_path):
                    validation_failures.append("binding_world_hash_invalid")
                if binding.get("credential_like_child_environment_names"):
                    validation_failures.append("credential_like_child_environment")
            if not report:
                validation_failures.append("report_absent")
            else:
                if not identity_matches(report, identity):
                    validation_failures.append("report_identity_invalid")
                if (
                    report.get("release_sha256") != release_sha256
                    or report.get("release_review_sha256")
                    != matrix.get("release_review_sha256")
                ):
                    validation_failures.append("report_release_chain_invalid")
                if (
                    report.get("split") != "pilot"
                    or report.get("arena_split") != "qualification"
                    or report.get("candidate_contract_variant")
                    != "r2_candidate_v060"
                    or report.get("scientific_inclusion") is not True
                    or report.get("policy_time_feature_cap_s") != 22.0
                    or report.get("execution_episode_cap_s") != 30.0
                ):
                    validation_failures.append("report_protocol_invalid")
                if manifest_path.is_file() and report.get(
                    "scene_manifest_sha256"
                ) != sha256(manifest_path):
                    validation_failures.append("report_manifest_hash_invalid")
            (
                contact_schema_valid,
                contact_monitor_pass,
                contact_state_sample_count,
                contact_monitor_failures,
            ) = recompute_contact_monitor(report)
            if report and not contact_schema_valid:
                validation_failures.extend(contact_monitor_failures)
            report_valid = not validation_failures
            result = report.get("result", {}) if report else {}
            if report and contact_schema_valid and int(
                result.get("contact_state_sample_count", -1)
            ) != int(contact_state_sample_count):
                validation_failures.append("result_contact_state_count_mismatch")
                report_valid = False
            if report and (
                matrix_row.get("goal_reached")
                is not bool(result.get("goal_reached"))
                or matrix_row.get("execution_integrity_pass")
                is not bool(report.get("execution_integrity_pass"))
            ):
                validation_failures.append("matrix_report_summary_mismatch")
                report_valid = False
            branch_states = (
                report.get("warmup", {}).get("branch_actor_states", [])
                if report
                else []
            )
            if scene == "hospital_static":
                branch_state_pass = report_valid and not branch_states
            else:
                branch_state_pass = bool(
                    report_valid
                    and len(branch_states) == 1
                    and abs(float(branch_states[0].get("x_m", float("inf"))))
                    <= 0.01
                    and abs(
                        float(branch_states[0].get("y_m", float("inf"))) + 2.7
                    )
                    <= 0.01
                    and abs(
                        float(
                            branch_states[0].get(
                                "physical_speed_mps", float("inf")
                            )
                        )
                    )
                    <= 0.01
                )
            if report_valid and not branch_state_pass:
                validation_failures.append("common_branch_state_invalid")
                report_valid = False
            method_runtime_ready = bool(
                report
                and (
                    int(result.get("local_plan_count", 0)) > 0
                    if method in ("actual_teb", "dwa")
                    else int(result.get("decision_count", 0)) > 0
                )
            )
            platform_integrity_pass = bool(
                report_valid
                and report.get("gate_valid")
                and report.get("message_contract_pass")
                and contact_monitor_pass
                and int(result.get("command_count", 0)) > 0
                and method_runtime_ready
                and branch_state_pass
            )
            raw_goal = bool(report_valid and result.get("goal_reached"))
            contact_free = bool(
                report_valid
                and contact_monitor_pass
                and contact_state_sample_count == 0
            )
            clearance_qualified = bool(
                report_valid
                and finite(result.get("minimum_conservative_clearance_m"))
                and float(result["minimum_conservative_clearance_m"]) >= 0.0
            )
            mask_qualified = bool(
                report_valid and int(result.get("mask_violation_count", 0)) == 0
            )
            zero_contact_arrival = bool(
                platform_integrity_pass and raw_goal and contact_free
            )
            safety_qualified_goal = bool(
                zero_contact_arrival and clearance_qualified and mask_qualified
            )
            safety_specification_pass = bool(
                report_valid
                and contact_state_sample_count == 0
                and clearance_qualified
                and mask_qualified
            )
            semantic_flag_mismatches = []
            if report_valid:
                for key, expected in (
                    ("common_branch_state_pass", branch_state_pass),
                    (
                        "platform_integrity_pass_excluding_algorithm_outcome",
                        platform_integrity_pass,
                    ),
                    ("safety_specification_pass", safety_specification_pass),
                    ("safety_qualified_goal", safety_qualified_goal),
                ):
                    if report.get(key) is not expected:
                        semantic_flag_mismatches.append(
                            "report_semantic_flag_mismatch:" + key
                        )
            if semantic_flag_mismatches:
                validation_failures.extend(semantic_flag_mismatches)
                report_valid = False
                branch_state_pass = False
                platform_integrity_pass = False
                raw_goal = False
                contact_free = False
                clearance_qualified = False
                mask_qualified = False
                zero_contact_arrival = False
                safety_qualified_goal = False
            raw_time = (
                float(result["restricted_elapsed_s"])
                if platform_integrity_pass
                and raw_goal
                and finite(result.get("restricted_elapsed_s"))
                else CAP_S
            )
            safety_time = (
                float(result["restricted_elapsed_s"])
                if safety_qualified_goal
                and finite(result.get("restricted_elapsed_s"))
                else CAP_S
            )
            if not report_valid:
                failure_reason = "evidence_chain_invalid"
            elif not platform_integrity_pass:
                failure_reason = "platform_integrity_failure"
            elif not raw_goal:
                failure_reason = "timeout_or_nonarrival"
            elif not contact_free:
                failure_reason = "contact"
            elif not clearance_qualified:
                failure_reason = "conservative_clearance_violation"
            elif not mask_qualified:
                failure_reason = "mask_violation"
            else:
                failure_reason = "safety_qualified_goal"
            rows.append(
                {
                    "seed": seed,
                    "scene_id": scene,
                    "task_type": (
                        manifest.get("task_type")
                        if manifest
                        else "static" if scene == "hospital_static" else "single_crossing"
                    ),
                    "method": method,
                    "report_present": report_path.is_file(),
                    "evidence_chain_valid": report_valid,
                    "platform_integrity_pass": platform_integrity_pass,
                    "branch_state_pass": branch_state_pass,
                    "contact_monitor_pass": bool(
                        report_valid and contact_monitor_pass
                    ),
                    "raw_goal_reached": raw_goal,
                    "zero_contact_arrival": zero_contact_arrival,
                    "safety_qualified_goal": safety_qualified_goal,
                    "valid_goal": safety_qualified_goal,
                    "reported_execution_integrity_pass": bool(
                        report_valid and report.get("execution_integrity_pass")
                    ),
                    "arrival_failure_inclusive_time_s": raw_time,
                    "safety_failure_inclusive_time_s": safety_time,
                    "failure_inclusive_time_s": safety_time,
                    "failure_reason": failure_reason,
                    "validation_failures": ";".join(validation_failures),
                    "raw_restricted_time_s": (
                        result.get("restricted_elapsed_s") if report_valid else None
                    ),
                    "minimum_clearance_m": (
                        result.get("minimum_conservative_clearance_m")
                        if report_valid
                        else None
                    ),
                    "contact_count": (
                        int(contact_state_sample_count)
                        if report_valid and contact_monitor_pass
                        else None
                    ),
                    "path_length_m": (
                        result.get("path_length_m") if report_valid else None
                    ),
                    "p99_inference_latency_ms": (
                        result.get("p99_inference_latency_ms")
                        if report_valid
                        else None
                    ),
                    "candidate_switch_count": (
                        result.get("candidate_switch_count")
                        if report_valid
                        else None
                    ),
                    "runner_timeout": bool(receipt.get("runner_timeout", False)),
                    "return_code": receipt.get("return_code"),
                    "runner_exception": matrix_row.get("runner_exception"),
                    "report_path": str(report_path.relative_to(ROOT)),
                }
            )
    return rows


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows):
    effect_evidence_complete = bool(
        len(rows) == 12
        and all(row["evidence_chain_valid"] for row in rows)
        and all(row["platform_integrity_pass"] for row in rows)
    )
    by_method = defaultdict(list)
    for row in rows:
        by_method[row["method"]].append(row)
    summary = []
    for method in METHODS:
        group = by_method[method]
        performance_evidence_complete = bool(
            len(group) == 3
            and all(row["evidence_chain_valid"] for row in group)
            and all(row["platform_integrity_pass"] for row in group)
        )
        safety_times = [
            float(row["safety_failure_inclusive_time_s"]) for row in group
        ]
        arrival_times = [
            float(row["arrival_failure_inclusive_time_s"]) for row in group
        ]
        clearances = [
            float(row["minimum_clearance_m"])
            for row in group
            if finite(row["minimum_clearance_m"])
        ]
        summary.append(
            {
                "method": method,
                "label": LABELS[method],
                "episodes": len(group),
                "evidence_valid_episodes": sum(
                    row["evidence_chain_valid"] for row in group
                ),
                "platform_integrity_passes": sum(
                    row["platform_integrity_pass"] for row in group
                ),
                "performance_evidence_complete": performance_evidence_complete,
                "raw_goals": (
                    sum(row["raw_goal_reached"] for row in group)
                    if performance_evidence_complete
                    else None
                ),
                "zero_contact_arrivals": (
                    sum(row["zero_contact_arrival"] for row in group)
                    if performance_evidence_complete
                    else None
                ),
                "valid_goals": (
                    sum(row["safety_qualified_goal"] for row in group)
                    if performance_evidence_complete
                    else None
                ),
                "safety_qualified_goals": (
                    sum(row["safety_qualified_goal"] for row in group)
                    if performance_evidence_complete
                    else None
                ),
                "valid_goal_rate": (
                    sum(row["safety_qualified_goal"] for row in group)
                    / len(group)
                    if performance_evidence_complete
                    else None
                ),
                "raw_goal_rate": (
                    sum(row["raw_goal_reached"] for row in group) / len(group)
                    if performance_evidence_complete
                    else None
                ),
                "mean_arrival_failure_inclusive_time_s": (
                    float(np.mean(arrival_times))
                    if performance_evidence_complete
                    else None
                ),
                "mean_failure_inclusive_time_s": (
                    float(np.mean(safety_times))
                    if performance_evidence_complete
                    else None
                ),
                "mean_safety_failure_inclusive_time_s": (
                    float(np.mean(safety_times))
                    if performance_evidence_complete
                    else None
                ),
                "minimum_clearance_m": (
                    min(clearances)
                    if performance_evidence_complete and clearances
                    else None
                ),
                "contact_count": (
                    sum(row["contact_count"] for row in group)
                    if performance_evidence_complete
                    and all(row["contact_count"] is not None for row in group)
                    else None
                ),
                "integrity_passes": (
                    sum(
                        row["reported_execution_integrity_pass"]
                        for row in group
                    )
                    if performance_evidence_complete
                    else None
                ),
            }
        )
    pct = next(row for row in summary if row["method"] == "pct_scope_rl")
    effects = []
    keyed = {(row["seed"], row["scene_id"], row["method"]): row for row in rows}
    blocks = sorted({(row["seed"], row["scene_id"]) for row in rows})
    for baseline in ("analytic_prior", "actual_teb", "dwa"):
        pct_safety_times = [
            keyed[(seed, scene, "pct_scope_rl")][
                "safety_failure_inclusive_time_s"
            ]
            for seed, scene in blocks
        ]
        baseline_safety_times = [
            keyed[(seed, scene, baseline)][
                "safety_failure_inclusive_time_s"
            ]
            for seed, scene in blocks
        ]
        pct_arrival_times = [
            keyed[(seed, scene, "pct_scope_rl")][
                "arrival_failure_inclusive_time_s"
            ]
            for seed, scene in blocks
        ]
        baseline_arrival_times = [
            keyed[(seed, scene, baseline)][
                "arrival_failure_inclusive_time_s"
            ]
            for seed, scene in blocks
        ]
        pct_mean = (
            float(np.mean(pct_safety_times)) if effect_evidence_complete else None
        )
        baseline_mean = (
            float(np.mean(baseline_safety_times))
            if effect_evidence_complete
            else None
        )
        pct_arrival_mean = (
            float(np.mean(pct_arrival_times))
            if effect_evidence_complete
            else None
        )
        baseline_arrival_mean = (
            float(np.mean(baseline_arrival_times))
            if effect_evidence_complete
            else None
        )
        effects.append(
            {
                "comparison": "pct_scope_rl_vs_" + baseline,
                "blocks": len(blocks),
                "pct_mean_s": pct_mean,
                "baseline_mean_s": baseline_mean,
                "relative_time_reduction_percent": (
                    100.0 * (baseline_mean - pct_mean) / baseline_mean
                    if effect_evidence_complete and baseline_mean > 0.0
                    else None
                ),
                "pct_arrival_mean_s": pct_arrival_mean,
                "baseline_arrival_mean_s": baseline_arrival_mean,
                "arrival_relative_time_reduction_percent": (
                    100.0
                    * (baseline_arrival_mean - pct_arrival_mean)
                    / baseline_arrival_mean
                    if effect_evidence_complete and baseline_arrival_mean > 0.0
                    else None
                ),
                "pct_valid_goal_rate": (
                    pct["valid_goal_rate"] if effect_evidence_complete else None
                ),
                "baseline_valid_goal_rate": (
                    next(
                        row["valid_goal_rate"]
                        for row in summary
                        if row["method"] == baseline
                    )
                    if effect_evidence_complete
                    else None
                ),
                "inferential_status": "not_computed_qualification_n3",
                "effect_evidence_complete": effect_evidence_complete,
            }
        )
    return summary, effects


def plot_times(rows, summary, output):
    figure, axis = plt.subplots(figsize=(9.2, 5.4), constrained_layout=True)
    arrival_means = [
        (
            row["mean_arrival_failure_inclusive_time_s"]
            if finite(row["mean_arrival_failure_inclusive_time_s"])
            else np.nan
        )
        for row in summary
    ]
    safety_means = [
        (
            row["mean_safety_failure_inclusive_time_s"]
            if finite(row["mean_safety_failure_inclusive_time_s"])
            else np.nan
        )
        for row in summary
    ]
    positions = np.arange(len(METHODS))
    axis.bar(
        positions - 0.18,
        arrival_means,
        color=[COLORS[method] for method in METHODS],
        width=0.34,
        alpha=0.38,
        label="Method-native arrival",
    )
    axis.bar(
        positions + 0.18,
        safety_means,
        color=[COLORS[method] for method in METHODS],
        width=0.34,
        alpha=0.90,
        hatch="//",
        label="Safety-qualified arrival",
    )
    rng = np.random.default_rng(20310813)
    for index, method in enumerate(METHODS):
        arrival_values = [
            row["arrival_failure_inclusive_time_s"]
            for row in rows
            if row["method"] == method
            and row["evidence_chain_valid"]
            and row["platform_integrity_pass"]
        ]
        safety_values = [
            row["safety_failure_inclusive_time_s"]
            for row in rows
            if row["method"] == method
            and row["evidence_chain_valid"]
            and row["platform_integrity_pass"]
        ]
        jitter = rng.uniform(-0.04, 0.04, len(arrival_values))
        axis.scatter(
            np.full(len(arrival_values), index - 0.18) + jitter,
            arrival_values,
            s=30,
            facecolors="none",
            edgecolors="black",
            zorder=3,
        )
        axis.scatter(
            np.full(len(safety_values), index + 0.18) + jitter,
            safety_values,
            s=32,
            color="black",
            zorder=3,
        )
    axis.set_xticks(positions, [LABELS[method] for method in METHODS])
    axis.set_ylabel("30 s failure-inclusive traversal time (s)")
    axis.set_title(
        "Arena hospital qualification: arrival vs safety-qualified outcome"
    )
    axis.set_ylim(0.0, CAP_S * 1.08)
    axis.grid(axis="y", alpha=0.25)
    axis.text(
        0.99,
        0.02,
        "Algorithm outcome failure = 30 s cap; evidence/platform-invalid rows omitted",
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
    )
    axis.legend(loc="upper left", fontsize=9)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def plot_trajectories(q1_root, output, rows):
    block = q1_root / "blocks/20310814_hospital_single_crossing"
    manifest = yaml.safe_load(
        (block / "snapshot/scene_manifest.yaml").read_text(encoding="utf-8")
    )
    map_path = ROOT / manifest["map_image_path"]
    map_yaml = yaml.safe_load((ROOT / manifest["map_yaml_path"]).read_text())
    image = np.asarray(Image.open(map_path).convert("L"))
    resolution = float(map_yaml["resolution"])
    origin = map_yaml["origin"]
    extent = (
        float(origin[0]),
        float(origin[0]) + image.shape[1] * resolution,
        float(origin[1]),
        float(origin[1]) + image.shape[0] * resolution,
    )
    route = manifest["route"]
    cosine = math.cos(float(route["yaw_world_rad"]))
    sine = math.sin(float(route["yaw_world_rad"]))

    def route_to_world(x_values, y_values):
        x_values = np.asarray(x_values, dtype=np.float64)
        y_values = np.asarray(y_values, dtype=np.float64)
        return (
            float(route["origin_world_x_m"]) + cosine * x_values - sine * y_values,
            float(route["origin_world_y_m"]) + sine * x_values + cosine * y_values,
        )

    figure, axis = plt.subplots(figsize=(10.5, 6.4), constrained_layout=True)
    axis.imshow(image, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)
    valid_methods = {
        row["method"]
        for row in rows
        if row["seed"] == 20310814
        and row["evidence_chain_valid"]
        and row["platform_integrity_pass"]
    }
    for method in METHODS:
        if method not in valid_methods:
            continue
        report_path = block / method / "episode_report.yaml"
        if not report_path.is_file():
            continue
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        trace = report.get("control_trace", [])
        x_route = [row["ego_pose"]["x_m"] for row in trace]
        y_route = [row["ego_pose"]["y_m"] for row in trace]
        x_world, y_world = route_to_world(x_route, y_route)
        axis.plot(
            x_world,
            y_world,
            linewidth=2.2,
            color=COLORS[method],
            label=LABELS[method],
        )
        actor_x = []
        actor_y = []
        for row in trace:
            poses = row.get("actor_poses", [])
            if poses:
                actor_x.append(poses[0]["x_m"])
                actor_y.append(poses[0]["y_m"])
        if actor_x:
            ax_world, ay_world = route_to_world(actor_x, actor_y)
            axis.plot(
                ax_world,
                ay_world,
                ":",
                color=COLORS[method],
                linewidth=1.2,
                alpha=0.75,
                label=LABELS[method] + " actor response",
            )
    start_world = route_to_world([route["policy_start_route_x_m"]], [0.0])
    goal_world = route_to_world([route["goal_route_x_m"]], [0.0])
    axis.scatter(*start_world, marker="o", s=90, color="#111111", label="Policy gate")
    axis.scatter(*goal_world, marker="*", s=170, color="#FFD700", edgecolor="black", label="Goal")
    axis.set_xlim(-7.0, 7.0)
    axis.set_ylim(6.5, 15.5)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("Arena world x (m)")
    axis.set_ylabel("Arena world y (m)")
    axis.set_title("Hospital route and executed trajectories (seed 20310814)")
    axis.legend(loc="upper right", fontsize=8)
    axis.grid(alpha=0.16)
    figure.savefig(output, dpi=220)
    plt.close(figure)


def render_markdown(summary, effects, rows):
    evidence_complete = len(rows) == 12 and all(
        row["evidence_chain_valid"] for row in rows
    )
    platform_complete = evidence_complete and all(
        row["platform_integrity_pass"] for row in rows
    )
    interface_pass = all(
        any(
            row["method"] == method and row["platform_integrity_pass"]
            for row in rows
        )
        for method in METHODS
    )

    def printable_number(value, digits=3):
        return "{:.{digits}f}".format(float(value), digits=digits) if finite(value) else "NA"

    def printable_count(value):
        return str(int(value)) if value is not None else "NA"

    contact_monitor_passes = sum(row["contact_monitor_pass"] for row in rows)
    lines = [
        "# Arena Hospital 算法对比资格实验结果",
        "",
        "> 证据等级：资格/探索性结果。3 个配对块不足以形成总体性能或先进性统计结论。",
        "",
        "## 方法汇总",
        "",
        "| 方法 | 证据有效 | 原始到达 | 安全合格到达 | 到达计入均时 / s | 安全计入均时 / s | 最小保守净空 / m | 接触状态样本数 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            "| {label} | {valid}/{episodes} | {raw}/{episodes} | {safe}/{episodes} | {arrival} | {safety} | {clearance} | {contacts} |".format(
                label=row["label"],
                valid=row["evidence_valid_episodes"],
                episodes=row["episodes"],
                raw=printable_count(row["raw_goals"]),
                safe=printable_count(row["safety_qualified_goals"]),
                arrival=printable_number(
                    row["mean_arrival_failure_inclusive_time_s"]
                ),
                safety=printable_number(
                    row["mean_safety_failure_inclusive_time_s"]
                ),
                clearance=printable_number(row["minimum_clearance_m"]),
                contacts=printable_count(row["contact_count"]),
            )
        )
    lines.extend(
        [
            "",
            "## 描述性效应",
            "",
            "| 比较 | 配对块 | 安全合格口径降低 | 原始到达口径降低 | 推断状态 |",
            "|---|---:|---:|---:|---|",
        ]
    )
    for row in effects:
        effect = row["relative_time_reduction_percent"]
        arrival_effect = row["arrival_relative_time_reduction_percent"]
        if effect is None or arrival_effect is None:
            lines.append(
                "| {comparison} | {blocks} | NA | NA | 证据链或平台完整性不完整，效应不可解释 |".format(
                    **row
                )
            )
        else:
            lines.append(
                "| {comparison} | {blocks} | {effect:.3f}% ({pct_mean_s:.3f}/{baseline_mean_s:.3f} s) | {arrival_effect:.3f}% ({pct_arrival_mean_s:.3f}/{baseline_arrival_mean_s:.3f} s) | 未进行推断统计 |".format(
                    effect=effect,
                    arrival_effect=arrival_effect,
                    **row,
                )
            )
    pct = next(row for row in summary if row["method"] == "pct_scope_rl")
    if interface_pass:
        interface_sentence = (
            "四种方法均至少完成一个证据链与平台完整性有效的 episode，"
            "说明共享 M2/Arena 执行接口已打通。"
        )
    else:
        interface_sentence = (
            "四方法接口资格未闭合；不得声称共享执行接口已经打通。"
        )
    if contact_monitor_passes == len(rows) == 12:
        contact_sentence = (
            "接触流在 12/12 episode 中通过发布者与心跳完整性检查；"
            "非零接触状态会使 episode 失败。"
        )
    else:
        contact_sentence = (
            "接触流仅在 {}/12 episode 中通过发布者与心跳完整性检查；"
            "未通过的 episode 不进入性能汇总或效应计算。"
        ).format(contact_monitor_passes)
    lines.extend(
        [
            "",
            "## 可写入论文的结论",
            "",
            interface_sentence,
            "本轮是完整导航系统栈比较：PCT/解析方法用冻结地图执行候选硬净空验证，TEB/DWA 通过全局代价地图与 Navfn 使用该图；不能写成四方法内部信息语义完全相同。",
            contact_sentence + "由于本 Q1 未冻结碰撞敏感性正控，零接触状态样本只作诊断，不构成独立的‘零碰撞’或形式安全证明。",
            "",
            "不能据此写成‘PCT 在 Arena 中显著优于 TEB/DWA’。原因是仅有 hospital 的一条宽直路线、3 个描述性配对块、受控圆柱代理，且没有推断统计。正式先进性结论仍需多世界、多任务和新鲜种子的独立矩阵。",
            "",
            "- 证据链完整：{}".format("是" if evidence_complete else "否"),
            "- 平台完整性 12/12：{}".format("是" if platform_complete else "否"),
            "- PCT 原始到达：{}/{}".format(
                printable_count(pct["raw_goals"]), pct["episodes"]
            ),
            "- PCT 安全规范合格到达：{}/{}".format(
                printable_count(pct["safety_qualified_goals"]), pct["episodes"]
            ),
            "- 正式结果：否（qualification only）",
            "- 训练步数：0；实车：未使用",
            "",
            "## 图",
            "",
            "![失败计入时间](figures/failure_inclusive_time.png)",
            "",
            "![医院路线轨迹](figures/hospital_route_trajectories.png)",
            "",
        ]
    )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--matrix-receipt-sha256", required=True)
    parser.add_argument("--raw-evidence-seal-sha256", required=True)
    args = parser.parse_args()
    output_root = args.root.resolve()
    closure = validate_raw_evidence_closure(
        output_root,
        args.matrix_receipt_sha256,
        args.raw_evidence_seal_sha256,
    )
    result_root = output_root / "q1"
    matrix, release, release_path, release_sha256, resource_count = validate_matrix(
        result_root
    )
    rows = collect(result_root, matrix, release, release_sha256)
    if len(rows) != 12:
        raise RuntimeError("EXPECTED_12_Q1_ROWS_OBSERVED_{}".format(len(rows)))
    summary, effects = summarize(rows)
    analysis_root = result_root / "analysis"
    figures_root = analysis_root / "figures"
    figures_root.mkdir(parents=True, exist_ok=False)
    write_csv(analysis_root / "episode_results.csv", rows, list(rows[0]))
    write_csv(analysis_root / "method_summary.csv", summary, list(summary[0]))
    write_csv(analysis_root / "effect_summary.csv", effects, list(effects[0]))
    plot_times(rows, summary, figures_root / "failure_inclusive_time.png")
    plot_trajectories(
        result_root, figures_root / "hospital_route_trajectories.png", rows
    )
    (analysis_root / "RESULTS.md").write_text(
        render_markdown(summary, effects, rows), encoding="utf-8"
    )
    evidence_complete = all(row["evidence_chain_valid"] for row in rows)
    platform_complete = evidence_complete and all(
        row["platform_integrity_pass"] for row in rows
    )
    any_safety_failure = any(
        not row["safety_qualified_goal"] for row in rows
    )
    status = (
        "qualification_incomplete_evidence_chain"
        if not evidence_complete
        else "qualification_complete_platform_failure"
        if not platform_complete
        else "qualification_complete_with_safety_outcome_failures"
        if any_safety_failure
        else "qualification_complete_all_safety_qualified"
    )
    evidence = {
        "schema_version": "1.0",
        "stage": "ARENA-PCT-Q1",
        "status": status,
        "formal_result": False,
        "performance_superiority_claim_authorized": False,
        "rows": len(rows),
        "release_path": str(release_path.relative_to(ROOT)),
        "release_sha256": release_sha256,
        "release_review_sha256": closure["release_review_sha256"],
        "terminal_journal_sha256": closure["terminal_journal_sha256"],
        "matrix_receipt_sha256": closure["matrix_receipt_sha256"],
        "raw_evidence_manifest_sha256": closure[
            "raw_evidence_manifest_sha256"
        ],
        "raw_evidence_seal_sha256": closure["raw_evidence_seal_sha256"],
        "release_resource_count": resource_count,
        "evidence_chain_complete": evidence_complete,
        "platform_integrity_complete": platform_complete,
        "method_summary": summary,
        "descriptive_effects": effects,
        "limitations": [
            "one frozen Arena world",
            "one wide straight hospital route rather than whole-world coverage",
            "three paired blocks",
            "no inferential statistics",
            "controlled reactive cylinder proxy rather than human-crowd realism",
            "no frozen positive collision-sensitivity control; heartbeat completeness is reported separately",
            "system-stack comparison with different internal map semantics",
            "LaserScan/odometry navigation study, not localization or perception",
        ],
    }
    (analysis_root / "evidence_summary.yaml").write_text(
        yaml.safe_dump(evidence, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    manifest_rows = []
    for path in sorted(
        item for item in analysis_root.rglob("*") if item.is_file()
    ):
        manifest_rows.append(
            {
                "path": str(path.relative_to(analysis_root)),
                "size_bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    (analysis_root / "analysis_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "release_sha256": release_sha256,
                "release_review_sha256": closure["release_review_sha256"],
                "terminal_journal_sha256": closure[
                    "terminal_journal_sha256"
                ],
                "matrix_receipt_sha256": closure[
                    "matrix_receipt_sha256"
                ],
                "raw_evidence_manifest_sha256": closure[
                    "raw_evidence_manifest_sha256"
                ],
                "raw_evidence_seal_sha256": closure[
                    "raw_evidence_seal_sha256"
                ],
                "files": manifest_rows,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    print(analysis_root.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
