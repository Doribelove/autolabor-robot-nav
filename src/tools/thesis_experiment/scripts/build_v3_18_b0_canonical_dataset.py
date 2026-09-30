#!/usr/bin/env python3
"""Build a canonical episode-level dataset from immutable V2/V3 evidence."""
import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_b0_canonical_dataset"
CONTRACT = W / "config/thesis_experiments/v3/v3_18_b0_canonical_dataset_contract.yaml"
SCHEMA_SOURCE = W / "src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py"
JSONL = ROOT / "canonical_episodes.jsonl"
CSV = ROOT / "canonical_episodes.csv"
DICTIONARY = ROOT / "data_dictionary.yaml"
SUMMARY = ROOT / "dataset_summary.yaml"
MANIFEST = ROOT / "source_and_output_manifest.sha256"

P1_ROOT = W / "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution"
V305_ROOT = W / "artifacts/v3/formal/v3_05_b0"
V309_ROOT = W / "artifacts/v3/formal/v3_09_f1"
V311_ROOT = W / "artifacts/v3/actual_teb"
V316_ROOT = W / "artifacts/v3/multiactor/v3_16_s1_response_gated_support"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    return sha(path.read_bytes())


def load_yaml(path):
    return yaml.safe_load(path.read_bytes())


def module():
    spec = importlib.util.spec_from_file_location("v3_18_b0_schema", SCHEMA_SOURCE)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def percentile(values, q):
    values = sorted(float(value) for value in values)
    if not values:
        return None
    index = int(math.ceil(q * len(values))) - 1
    return values[max(0, min(index, len(values) - 1))]


def base_row(**values):
    schema = module()
    row = {field: None for field in schema.REQUIRED_FIELDS}
    row.update({
        "schema_version": schema.SCHEMA_VERSION,
        "simulation_only": True,
        "completed": True,
        "terminal_reason": "completed",
        "retry_used": False,
        "resume_used": False,
        "statistical_unit": "scene_seed_block",
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "source_revision": "frozen_historical_artifact",
    })
    row.update(values)
    return row


def v2_p1_rows(sources):
    matrix_path = P1_ROOT / "normalized_terminal_matrix.json"
    stage_report_path = P1_ROOT / "p1_e1_stage_report.yaml"
    schedule_path = W / "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/exact_schedule.yaml"
    if not schedule_path.is_file():
        candidates = list((W / "artifacts/v2/integration/v2_04g_r6_i1").rglob("*schedule*.yaml"))
        schedule_path = next(path for path in candidates if "p1" in path.name.lower() or "p1" in str(path).lower())
    freeze_path = P1_ROOT / "p1_e1_terminal_evidence_freeze.sha256"
    stage_report = load_yaml(stage_report_path)
    matrix = json.loads(matrix_path.read_text())
    sources.update((matrix_path, stage_report_path, schedule_path, freeze_path))
    roles = {
        "r6_semantics_circle_contact": "main_method",
        "r6_semantics_legacy_control": "primary_baseline",
        "fixed_teb_reference": "context_comparator",
    }
    result = []
    for item in matrix:
        journal = P1_ROOT / "journals" / "sequence_{:04d}.yaml".format(int(item["sequence"]))
        sources.add(journal)
        result.append(base_row(
            stage="V2-04G-P1-E1", sequence=item["sequence"],
            block_id=item["block_id"], attempt=item["attempt"], seed=item["seed"],
            scene_id=item["scene_id"], actor_policy=item["scene_role"],
            method=item["method_id"], method_role=roles[item["method_id"]],
            terminal_reason=item["terminal_status"], sim_time_s=item["observation_duration_s"],
            goal_success=item["success"], restricted_time_s=item["navigation_time_s"],
            contact_count=1 if item["collision"] else 0, collision=item["collision"],
            minimum_clearance_m=item["minimum_footprint_clearance_m"],
            emergency_count=1 if item["algorithmic_emergency"] else 0,
            report_sha256=file_sha(journal), release_sha256=stage_report["release_sha256"],
            schedule_sha256=file_sha(schedule_path), raw_manifest_sha256=file_sha(freeze_path),
            scientific_inclusion=True, oracle_track_used=True,
        ))
    return result


def v3_receipt_rows(stage, root, schedule_path, release_path, freeze_path, role_map, sources):
    sources.update((schedule_path, release_path, freeze_path))
    schedule_sha = file_sha(schedule_path)
    release_sha = file_sha(release_path)
    freeze_sha = file_sha(freeze_path)
    result = []
    for report_path in sorted((root / "execution/episodes").rglob("episode_receipt.yaml")):
        sources.add(report_path)
        report = load_yaml(report_path)
        identity = report["row"]
        decisions = report.get("decisions", [])
        latencies = [item["planner_latency_ms"] for item in decisions if item.get("planner_latency_ms") is not None]
        selected = [item.get("selected_candidate_id") for item in decisions if item.get("selected_candidate_id")]
        accepted = [item.get("selected_candidate_accepted") for item in decisions if item.get("selected_candidate_accepted") is not None]
        stop_available = [item.get("certified_stop_available") for item in decisions if item.get("certified_stop_available") is not None]
        prefix = report.get("prefix", {})
        clearance = prefix.get("minimum_conservative_clearance_m")
        final = report.get("final_state", {})
        goal_time = report.get("goal_time_s")
        result.append(base_row(
            stage=stage, sequence=identity["sequence"], block_id=identity["block_id"],
            attempt=identity["attempt"], seed=identity["seed"],
            scene_id="parameterized_crossing", actor_policy=identity["actor_policy_evaluator_only"],
            method=identity["method"], method_role=role_map[identity["method"]],
            sim_time_s=final.get("global_time_s"), goal_success=report.get("goal_reached"),
            restricted_time_s=goal_time, contact_count=0 if clearance is not None and clearance >= 0 else None,
            collision=False if clearance is not None and clearance >= 0 else None,
            minimum_clearance_m=clearance, emergency_count=sum(1 for value in selected if value == "certified_stop_v000"),
            safety_fallback_used=any(value == "certified_stop_v000" for value in selected),
            decision_count=report.get("decision_count"), selected_candidate_id="|".join(sorted(set(selected))) if selected else None,
            selected_accepted=all(accepted) if accepted else None,
            certified_stop_available=all(stop_available) if stop_available else None,
            planner_latency_p99_ms=percentile(latencies, 0.99),
            tracking_error_max_m=prefix.get("maximum_ego_trace_error_m"),
            contract_sha256=report.get("planner_contract_sha256"), release_sha256=release_sha,
            schedule_sha256=schedule_sha, report_sha256=file_sha(report_path),
            raw_manifest_sha256=freeze_sha, scientific_inclusion=True, oracle_track_used=True,
        ))
    return result


def v3_11_rows(sources):
    schedule = V311_ROOT / "v3_11_f1_release/exact_schedule.yaml"
    release = V311_ROOT / "v3_11_f1_release/dependency_freeze.yaml"
    freeze = V311_ROOT / "v3_11_f1_assessment/terminal_freeze.yaml"
    sources.update((schedule, release, freeze))
    result = []
    for report_path in sorted((V311_ROOT / "v3_11_f1_execution/episodes").glob("*/episode_report.yaml")):
        receipt_path = report_path.with_name("matrix_receipt.yaml")
        sources.update((report_path, receipt_path))
        report = load_yaml(report_path)
        receipt = load_yaml(receipt_path)
        row = receipt["row"]
        outcome = report["result"]
        plan = report.get("v3_plan") or {}
        evaluation = plan.get("selected_evaluation") or {}
        result.append(base_row(
            stage="V3-11-F1", sequence=row["sequence"], block_id="v3_11_block_{:02d}".format(row["block_index"]),
            attempt=1, seed=row["seed"], scene_id=row["scene_id"], actor_policy=row["actor_policy"],
            method=row["method"], method_role="main_method" if row["method"] == "v3_spatial_safe" else "primary_baseline",
            wall_time_s=receipt.get("wall_elapsed_s"), sim_time_s=outcome.get("elapsed_s"),
            goal_success=outcome.get("goal_reached"), restricted_time_s=outcome.get("restricted_elapsed_s"),
            path_length_m=outcome.get("path_length_m"), final_position_error_m=outcome.get("final_position_error_m"),
            contact_count=outcome.get("contact_state_count"), collision=(outcome.get("contact_state_count", 0) > 0),
            minimum_clearance_m=outcome.get("minimum_conservative_clearance_m"), emergency_count=0,
            safety_fallback_used=plan.get("safety_fallback_used") if plan else None,
            decision_count=1 if plan else outcome.get("local_plan_count"),
            selected_candidate_id=plan.get("selected_candidate_id"),
            selected_accepted=evaluation.get("accepted"), certified_stop_available=True if plan else None,
            planner_latency_p99_ms=plan.get("latency_ms"), tracking_error_max_m=outcome.get("tracking_error_max_m"),
            contract_sha256=(report.get("planner_contract") or {}).get("sha256"),
            release_sha256=file_sha(release), schedule_sha256=file_sha(schedule),
            report_sha256=file_sha(report_path), raw_manifest_sha256=file_sha(freeze),
            scientific_inclusion=True, oracle_track_used=report.get("shared_oracle_actor_track"),
        ))
    return result


def v3_16_rows(sources):
    schedule = V316_ROOT / "exact_schedule.yaml"
    freeze = V316_ROOT / "terminal_freeze_r1.sha256"
    manifest = V316_ROOT / "evidence_manifest_r1.sha256"
    sources.update((schedule, freeze, manifest))
    result = []
    for report_path in sorted((V316_ROOT / "episodes").glob("*/episode_report.yaml")):
        receipt_path = report_path.with_name("matrix_receipt.yaml")
        sources.update((report_path, receipt_path))
        report = load_yaml(report_path)
        receipt = load_yaml(receipt_path)
        identity = receipt["row"]
        outcome = report["result"]
        inclusion = identity["phase"] == "evidence" and receipt.get("scientific_inclusion_binding_pass") is True
        result.append(base_row(
            stage="V3-16-S1", sequence=identity["sequence"], block_id="v3_16_b{:02d}".format(identity["block_index"]),
            attempt=identity["attempt"], seed=identity["seed"], scene_id=identity["scene_id"],
            actor_policy="+".join(identity["actor_policies_evaluator_only"]),
            method=identity["forced_candidate_id"], method_role="forced_action_mechanism_ablation",
            wall_time_s=receipt.get("wall_elapsed_s"), sim_time_s=outcome.get("trace_duration_s"),
            goal_success=None, restricted_time_s=outcome.get("trace_duration_s"), path_length_m=outcome.get("path_length_m"),
            contact_count=outcome.get("contact_state_count"), collision=(outcome.get("contact_state_count", 0) > 0),
            minimum_clearance_m=outcome.get("minimum_conservative_clearance_m"), emergency_count=0,
            safety_fallback_used=(identity["forced_candidate_id"] == "certified_stop_v000"),
            decision_count=1, selected_candidate_id=identity["forced_candidate_id"], selected_accepted=True,
            certified_stop_available=True, planner_latency_p99_ms=None,
            tracking_error_max_m=outcome.get("tracking_error_max_m"),
            contract_sha256=(report.get("planner_contract") or {}).get("sha256"), release_sha256=file_sha(manifest),
            schedule_sha256=file_sha(schedule), report_sha256=file_sha(report_path), raw_manifest_sha256=file_sha(freeze),
            scientific_inclusion=inclusion, oracle_track_used=True,
        ))
    return result


def csv_bytes(rows):
    fields = ["schema_version"] + list(module().REQUIRED_FIELDS) + ["source_path", "row_digest"]
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def outputs():
    contract = load_yaml(CONTRACT)
    sources = {CONTRACT, SCHEMA_SOURCE, Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_18_b0_canonical_dataset.py"}
    rows = []
    rows.extend(v2_p1_rows(sources))
    rows.extend(v3_receipt_rows(
        "V3-05-B0", V305_ROOT,
        V305_ROOT / "offline_readiness/v3_05_b0_exact_schedule.yaml",
        V305_ROOT / "offline_readiness/v3_05_b0_execution_release.yaml",
        V305_ROOT / "execution/v3_05_b0_evidence_freeze.sha256",
        {"v3_analytic": "main_method", "fixed_v120_shielded": "primary_baseline", "fixed_v080_shielded": "comparison_method"}, sources,
    ))
    rows.extend(v3_receipt_rows(
        "V3-09-F1", V309_ROOT,
        V309_ROOT / "offline_readiness/v3_09_f1_exact_schedule.yaml",
        V309_ROOT / "offline_readiness/v3_09_f1_execution_release.yaml",
        V309_ROOT / "execution/v3_09_f1_evidence_freeze.sha256",
        {"v3_spatial_safe": "main_method", "fixed_v120_shielded": "primary_baseline"}, sources,
    ))
    rows.extend(v3_11_rows(sources))
    rows.extend(v3_16_rows(sources))
    expected = contract["expected_rows"]
    counts = {stage: sum(1 for row in rows if row["stage"] == stage) for stage in expected if stage != "total"}
    if counts != {key: value for key, value in expected.items() if key != "total"} or len(rows) != expected["total"]:
        raise RuntimeError("source row counts differ: {}".format(counts))
    validator = module()
    for row in rows:
        validator.validate_row(row)
        row["source_path"] = "sha256-bound source; see source_and_output_manifest.sha256"
        row["row_digest"] = validator.digest({key: value for key, value in row.items() if key not in ("source_path", "row_digest")})
    rows.sort(key=lambda row: (row["stage"], int(row["sequence"])))
    jsonl = b"".join(validator.canonical_json(row) + b"\n" for row in rows)
    dictionary = yaml.safe_dump(validator.schema_document(), sort_keys=False).encode("utf-8")
    summary_value = {
        "schema_version": "1.0", "stage": "V3-18-B0",
        "status": "canonical_dataset_build_pass", "row_count": len(rows),
        "study_counts": counts,
        "scientific_rows": sum(1 for row in rows if row["scientific_inclusion"]),
        "preflight_or_support_rows": sum(1 for row in rows if not row["scientific_inclusion"]),
        "method_count": len({(row["stage"], row["method"]) for row in rows}),
        "scene_count_within_stage": len({(row["stage"], row["scene_id"]) for row in rows}),
        "cross_stage_pooling_forbidden": True,
        "source_artifacts_modified": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    summary = yaml.safe_dump(summary_value, sort_keys=False).encode("utf-8")
    generated = {JSONL: jsonl, CSV: csv_bytes(rows), DICTIONARY: dictionary, SUMMARY: summary}
    for path in sources:
        if not path.is_file() or not path.is_absolute() or W not in path.parents:
            raise RuntimeError("invalid source path: {}".format(path))
    manifest = "".join("{}  {}\n".format(file_sha(path), path.relative_to(W)) for path in sorted(sources))
    manifest += "".join("{}  {}\n".format(sha(data), path.relative_to(W)) for path, data in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated, summary_value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    generated, summary = outputs()
    for path, data in generated.items():
        atomic(path, data, args.check)
    print("status={} rows={} manifest_sha256={}".format(summary["status"], summary["row_count"], sha(generated[MANIFEST])))


if __name__ == "__main__":
    main()
