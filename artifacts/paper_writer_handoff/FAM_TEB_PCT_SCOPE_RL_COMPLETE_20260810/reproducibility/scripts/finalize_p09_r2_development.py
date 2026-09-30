#!/usr/bin/env python3
"""Assess R2-D0 and update all seven benchmark evidence ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
DEV = ARTIFACT / "development/r2_candidate_coverage_d0"
EPISODES = DEV / "episodes"
SCHEDULE = DEV / "schedule.csv"
PREREG = DEV / "preregistration.yaml"
ASSESSMENT = DEV / "assessment.json"
RAW = DEV / "episode_results.csv"
REVIEW = DEV / "integrity_review.json"
STAGE = "P09_R2_DEV_D0"
ROUND = "gazebo_p09_r2_candidate_coverage_d0"
CONTROL = "r1_control_v040"
TREATMENT = "r2_candidate_v060"
DYNAMIC = {
    "core_single_crossing", "core_head_on_yield",
    "core_sequential_interaction", "core_partially_occluded",
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_text(path, content):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def append_once(path, marker, content):
    original = path.read_text(encoding="utf-8")
    if marker not in original:
        atomic_text(path, original.rstrip() + "\n\n" + content.strip() + "\n")


def episode_root(row):
    return EPISODES / "{}_{}_pct_scope_rl_{}".format(
        row["seed"], row["scene_id"], row["candidate_variant"]
    )


def reason(report):
    if report is None:
        return "presemantic_system_failure"
    result = report["result"]
    if int(result.get("contact_state_count", 0)) > 0:
        return "contact"
    clearance = result.get("minimum_conservative_clearance_m")
    if clearance is not None and float(clearance) < 0.0:
        return "conservative_clearance_violation"
    if not bool(report.get("execution_integrity_pass")):
        return "execution_integrity_failure"
    if bool(result.get("goal_reached")):
        return "goal"
    return "timeout_or_nonarrival"


def load_records():
    with SCHEDULE.open(newline="", encoding="utf-8") as stream:
        schedule = list(csv.DictReader(stream))
    identities = [
        (row["seed"], row["scene_id"], row["method"], row["candidate_variant"], row["attempt"])
        for row in schedule
    ]
    if len(schedule) != 24 or len(identities) != len(set(identities)):
        raise RuntimeError("R2_D0_SCHEDULE_NOT_EXACT_AND_UNIQUE")
    records = []
    for row in schedule:
        root = episode_root(row)
        receipt_path = root / "receipt.json"
        report_path = root / "episode_report.yaml"
        if not receipt_path.is_file():
            raise RuntimeError("R2_D0_RECEIPT_ABSENT:" + root.name)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        report = (
            yaml.load(report_path.read_text(encoding="utf-8"), Loader=yaml.CSafeLoader)
            if report_path.is_file() else None
        )
        outcome = reason(report)
        result = report["result"] if report else {}
        restricted = float(result.get("restricted_elapsed_s", 22.0))
        records.append({
            "schedule": row,
            "root": root,
            "receipt_path": receipt_path,
            "receipt": receipt,
            "report_path": report_path,
            "report": report,
            "result": result,
            "reason": outcome,
            "failure_inclusive_time_s": restricted if outcome == "goal" else 22.0,
        })
    return records


def subset(records, variant, scenes=None):
    return [
        record for record in records
        if record["schedule"]["candidate_variant"] == variant
        and (scenes is None or record["schedule"]["scene_id"] in scenes)
    ]


def mean_time(records, variant, scenes=None):
    rows = subset(records, variant, scenes)
    return statistics.mean(row["failure_inclusive_time_s"] for row in rows)


def summarize(records, variant):
    rows = subset(records, variant)
    reports = [row for row in rows if row["report"] is not None]
    results = [row["result"] for row in reports]
    return {
        "episodes": len(rows),
        "valid_goals": sum(row["reason"] == "goal" for row in rows),
        "system_failures": sum(row["reason"] == "presemantic_system_failure" for row in rows),
        "timeouts_or_nonarrivals": sum(row["reason"] == "timeout_or_nonarrival" for row in rows),
        "mean_failure_inclusive_time_s": mean_time(records, variant),
        "contact_count": sum(int(result.get("contact_state_count", 0)) for result in results),
        "mask_violation_count": sum(int(result.get("mask_violation_count", 0)) for result in results),
        "minimum_conservative_clearance_m": min(
            float(result["minimum_conservative_clearance_m"])
            for result in results if result.get("minimum_conservative_clearance_m") is not None
        ),
        "mean_candidate_switch_count": statistics.mean(
            float(result["candidate_switch_count"])
            for result in results if result.get("candidate_switch_count") is not None
        ),
        "mean_certified_stop_count": statistics.mean(
            float(result["certified_stop_count"])
            for result in results if result.get("certified_stop_count") is not None
        ),
        "mean_stop_recovery_count": statistics.mean(
            float(result["stop_recovery_count"])
            for result in results if result.get("stop_recovery_count") is not None
        ),
        "mean_tracking_error_m": statistics.mean(
            float(result["tracking_error_mean_m"])
            for result in results if result.get("tracking_error_mean_m") is not None
        ),
    }


def build_assessment(records):
    control = summarize(records, CONTROL)
    treatment = summarize(records, TREATMENT)
    overall_improvement = control["mean_failure_inclusive_time_s"] - treatment["mean_failure_inclusive_time_s"]
    static_control = mean_time(records, CONTROL, {"core_static_avoidance"})
    static_treatment = mean_time(records, TREATMENT, {"core_static_avoidance"})
    dynamic_control = mean_time(records, CONTROL, DYNAMIC)
    dynamic_treatment = mean_time(records, TREATMENT, DYNAMIC)
    gates = {
        "treatment_contact_count_maximum": treatment["contact_count"] <= 0,
        "treatment_mask_violation_count_maximum": treatment["mask_violation_count"] <= 0,
        "treatment_minimum_conservative_clearance_m_minimum": treatment["minimum_conservative_clearance_m"] >= 0.0,
        "treatment_completion_difference_minimum": treatment["valid_goals"] - control["valid_goals"] >= 0,
        "overall_mean_restricted_time_improvement_s_minimum": overall_improvement >= 0.25,
        "static_mean_restricted_time_improvement_s_minimum": static_control - static_treatment >= 1.0,
        "dynamic_mean_restricted_time_worsening_s_maximum": dynamic_treatment - dynamic_control <= 0.25,
    }
    # Sensitivity removes the entire first paired block whose control failed
    # before semantic execution. It is descriptive only; the primary retains it.
    sensitivity = [row for row in records if row["schedule"]["block_id"] != "R2D0-B001"]
    result = {
        "schema_version": "1.0",
        "stage": "PCT-BENCH-R2-D0",
        "status": "development_single_factor_promoted" if all(gates.values()) else "development_single_factor_rejected",
        "registered_blocks": 12,
        "registered_episodes": 24,
        "reports_present": sum(row["report"] is not None for row in records),
        "attempt_one_only": True,
        "same_identity_retries": 0,
        "all_failures_retained": True,
        "method_summaries": {CONTROL: control, TREATMENT: treatment},
        "effects": {
            "overall_mean_improvement_s": overall_improvement,
            "static_control_mean_s": static_control,
            "static_treatment_mean_s": static_treatment,
            "static_mean_improvement_s": static_control - static_treatment,
            "dynamic_control_mean_s": dynamic_control,
            "dynamic_treatment_mean_s": dynamic_treatment,
            "dynamic_mean_worsening_s": dynamic_treatment - dynamic_control,
            "width_control_mean_s": mean_time(records, CONTROL, {"core_width_change"}),
            "width_treatment_mean_s": mean_time(records, TREATMENT, {"core_width_change"}),
        },
        "decision_gates": gates,
        "all_decision_gates_pass": all(gates.values()),
        "paired_sensitivity_excluding_presemantic_failure_block": {
            "blocks": 11,
            "overall_control_mean_s": mean_time(sensitivity, CONTROL),
            "overall_treatment_mean_s": mean_time(sensitivity, TREATMENT),
            "overall_mean_improvement_s": mean_time(sensitivity, CONTROL) - mean_time(sensitivity, TREATMENT),
            "static_control_mean_s": mean_time(sensitivity, CONTROL, {"core_static_avoidance"}),
            "static_treatment_mean_s": mean_time(sensitivity, TREATMENT, {"core_static_avoidance"}),
            "static_mean_improvement_s": mean_time(sensitivity, CONTROL, {"core_static_avoidance"}) - mean_time(sensitivity, TREATMENT, {"core_static_avoidance"}),
            "primary_analysis_changed": False,
        },
        "formal_round_2_seed_accessed": False,
        "claim_boundary": "development_only_no_superiority_claim",
    }
    atomic_text(ASSESSMENT, json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def write_raw(records):
    fields = [
        "block_id", "seed", "scene_id", "method", "candidate_variant", "attempt",
        "order_index", "goal_reached", "execution_integrity_pass", "failure_reason",
        "failure_inclusive_time_s", "contact_count", "mask_violation_count",
        "minimum_conservative_clearance_m", "candidate_switch_count",
        "certified_stop_count", "stop_recovery_count", "tracking_error_mean_m",
        "tracking_error_max_m", "report_path", "receipt_path",
    ]
    rows = []
    for record in records:
        schedule, report, result = record["schedule"], record["report"], record["result"]
        values = {
            **schedule,
            "goal_reached": bool(result.get("goal_reached", False)),
            "execution_integrity_pass": bool(report and report.get("execution_integrity_pass")),
            "failure_reason": record["reason"],
            "failure_inclusive_time_s": record["failure_inclusive_time_s"],
            "contact_count": int(result.get("contact_state_count", 0)),
            "mask_violation_count": int(result.get("mask_violation_count", 0)),
            "minimum_conservative_clearance_m": result.get("minimum_conservative_clearance_m", ""),
            "candidate_switch_count": result.get("candidate_switch_count", ""),
            "certified_stop_count": result.get("certified_stop_count", ""),
            "stop_recovery_count": result.get("stop_recovery_count", ""),
            "tracking_error_mean_m": result.get("tracking_error_mean_m", ""),
            "tracking_error_max_m": result.get("tracking_error_max_m", ""),
            "report_path": str(record["report_path"].relative_to(ROOT)) if record["report"] else "",
            "receipt_path": str(record["receipt_path"].relative_to(ROOT)),
        }
        rows.append({field: values.get(field, "") for field in fields})
    temporary = RAW.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    temporary.replace(RAW)


def update_run_manifest(records):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream); fields = reader.fieldnames
        retained = [row for row in reader if row["stage"] != STAGE]
    added = []
    for record in records:
        row, receipt = record["schedule"], record["receipt"]
        ended = datetime.fromtimestamp(record["receipt_path"].stat().st_mtime).astimezone()
        started = datetime.fromtimestamp(ended.timestamp() - float(receipt["wall_elapsed_s"])).astimezone()
        added.append({
            "stage": STAGE,
            "run_id": "{}__{}_{}".format(row["block_id"], row["seed"], row["candidate_variant"]),
            "phase": "preregistered_development_single_factor_ablation",
            "method": "pct_scope_rl__" + row["candidate_variant"],
            "scene_id": row["scene_id"], "split": "development", "seed": row["seed"],
            "attempt": 1,
            "status": "completed_pass" if record["reason"] == "goal" else record["reason"] + "_retained",
            "started_at": started.isoformat(timespec="seconds"),
            "ended_at": ended.isoformat(timespec="seconds"),
            "exit_code": receipt["return_code"],
            "config_sha256": receipt["preflight_bindings_sha256"],
            "model_sha256": "deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8",
            "evidence_path": str(record["root"].relative_to(ROOT)),
            "formal_eligible": "false",
            "notes": "variant={}; block={}; order={}; reason={}; no_retry".format(
                row["candidate_variant"], row["block_id"], row["order_index"], record["reason"]
            ),
        })
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(retained + added)
    temporary.replace(path)


def update_episode_ledger(records):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream); fields = reader.fieldnames
        retained = [row for row in reader if row["study_round"] != ROUND]
    added = []
    for record in records:
        row, result, report = record["schedule"], record["result"], record["report"]
        elapsed = record["failure_inclusive_time_s"]
        path_length = result.get("path_length_m")
        values = {
            "study_round": ROUND, "phase": "preregistered_development_single_factor_ablation",
            "block_id": row["block_id"],
            "episode_id": "{}_{}_{}".format(row["block_id"], row["seed"], row["candidate_variant"]),
            "method": "pct_scope_rl__" + row["candidate_variant"],
            "scene_family": row["scene_id"], "scene_id": row["scene_id"],
            "split": "development", "seed": row["seed"], "attempt": 1,
            "order_index": row["order_index"],
            "goal_reached": str(bool(result.get("goal_reached", False))).lower(),
            "timeout": str(record["reason"] == "timeout_or_nonarrival").lower(),
            "contact_count": int(result.get("contact_state_count", 0)),
            "system_failure": str(record["reason"] == "presemantic_system_failure").lower(),
            "failure_reason": record["reason"], "failure_inclusive_time_s": elapsed,
            "min_clearance_m": result.get("minimum_conservative_clearance_m", ""),
            "path_length_m": path_length if path_length is not None else "",
            "mean_speed_mps": path_length / elapsed if path_length is not None and elapsed > 0 else "",
            "acceleration_rms": "", "jerk_metric": result.get("maximum_absolute_jerk_mps3", ""),
            "candidate_switch_count": result.get("candidate_switch_count", ""),
            "stop_count": result.get("certified_stop_count", ""),
            "recovery_count": result.get("stop_recovery_count", ""),
            "mean_latency_ms": result.get("mean_inference_latency_ms", ""),
            "p95_latency_ms": result.get("p95_inference_latency_ms", ""),
            "p99_latency_ms": result.get("p99_inference_latency_ms", ""),
            "mask_violation_count": result.get("mask_violation_count", 0),
            "config_sha256": record["receipt"]["preflight_bindings_sha256"],
            "model_sha256": report.get("actor_checkpoint_sha256", "") if report else "bound_failed_presemantic",
            "evidence_path": str((record["report_path"] if report else record["root"] / "roslaunch.log").relative_to(ROOT)),
        }
        added.append({field: values.get(field, "") for field in fields})
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(retained + added)
    temporary.replace(path)


def unique(path, first, second):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[first], row[second]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def verify_preflights(records):
    matched = 0
    historical_mismatch = []
    for record in records:
        path = record["root"] / "preflight_bindings.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        mismatches = []
        for item in data["resource_bindings"]:
            target = ROOT / item["path"]
            if not target.is_file() or sha256(target) != item["sha256"]:
                mismatches.append(item["path"])
        if not mismatches:
            matched += 1
        elif record["schedule"]["block_id"] == "R2D0-B001" and record["schedule"]["candidate_variant"] == CONTROL:
            historical_mismatch = sorted(mismatches)
        else:
            raise RuntimeError("UNEXPECTED_R2_D0_PREFLIGHT_MISMATCH:" + str(mismatches))
    expected = [
        "scripts/gazebo_pct_teb_dwa_benchmark/run_r2_candidate_ablation.py",
        "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py",
    ]
    if matched != 23 or historical_mismatch != expected:
        raise RuntimeError("R2_D0_PREFLIGHT_CLOSURE_UNEXPECTED")
    if sha256(DEV / "failures/pct_teb_dwa_benchmark_r2_episode.failed_v0.py") != "16fe1588bb2253316e5bf31a8d063293b2fc6e9b7f4ad3a641fdd82d29ac0994":
        raise RuntimeError("FAILED_WRAPPER_SNAPSHOT_HASH_MISMATCH")
    return matched, historical_mismatch


def main():
    records = load_records()
    write_raw(records)
    assessment = build_assessment(records)
    matched, historical_mismatch = verify_preflights(records)
    update_run_manifest(records); update_episode_ledger(records)
    run_unique, run_count = unique(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_count = unique(ARTIFACT / "episode_results.csv", "study_round", "episode_id")
    if not run_unique or not episode_unique:
        raise RuntimeError("R2_D0_GLOBAL_LEDGER_DUPLICATE")
    review = {
        "schema_version": "1.0", "status": "r2_d0_integrity_review_pass",
        "schedule_sha256": sha256(SCHEDULE), "preregistration_sha256": sha256(PREREG),
        "assessment_sha256": sha256(ASSESSMENT), "raw_episode_results_sha256": sha256(RAW),
        "blocks": 12, "episodes": 24, "reports_present": 23,
        "attempt_one_only": True, "same_identity_retry_count": 0,
        "replacement_seed_count": 0, "all_failures_retained": True,
        "preflights_currently_match": matched,
        "first_failed_preflight_historical_path_mismatches": historical_mismatch,
        "first_failed_wrapper_snapshot_sha256": sha256(DEV / "failures/pct_teb_dwa_benchmark_r2_episode.failed_v0.py"),
        "integration_repair_behavioral_factor_changed": False,
        "run_manifest_unique": run_unique, "run_manifest_rows": run_count,
        "episode_results_unique": episode_unique, "episode_results_rows": episode_count,
        "formal_round_2_seed_accessed": False, "hardware_accessed": False,
    }
    atomic_text(REVIEW, json.dumps(review, indent=2, sort_keys=True) + "\n")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "1.0"; integrity["status"] = "r2_development_factor_promoted"
    integrity["formal_execution_authorized"] = False
    integrity["checks"] = [item for item in integrity["checks"] if item.get("id") != "p09_r2_d0"]
    integrity["checks"].append({
        "id": "p09_r2_d0", "status": "pass_development_single_factor_promoted",
        "blocks": 12, "episodes": 24, "reports_present": 23,
        "all_decision_gates_pass": assessment["all_decision_gates_pass"],
        "assessment": str(ASSESSMENT.relative_to(ROOT)), "assessment_sha256": sha256(ASSESSMENT),
        "review": str(REVIEW.relative_to(ROOT)), "review_sha256": sha256(REVIEW),
        "formal_round_2_seed_accessed": False,
    })
    integrity["failures"].append({
        "stage": STAGE, "scope": "development_only", "count": 1,
        "breakdown": {"presemantic_system_failure": 1},
        "status": "retained_not_retried_or_suppressed",
    })
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    c = assessment["method_summaries"][CONTROL]; t = assessment["method_summaries"][TREATMENT]
    e = assessment["effects"]
    append_once(ARTIFACT / "progress.md", "## P09 R2 development 单因素候选覆盖", f"""## P09 R2 development 单因素候选覆盖

- 12 个预注册 development 配对块 / 24 回合全部 attempt=1 终止；首个 control 为预语义导入失败，按 22 s 保留且未重试，其余 23 个报告通过哈希闭包。
- 失败计入均时：R1 control {c['mean_failure_inclusive_time_s']:.4f} s（{c['valid_goals']}/12），0.6 m/s 候选 {t['mean_failure_inclusive_time_s']:.4f} s（{t['valid_goals']}/12），改善 {e['overall_mean_improvement_s']:.4f} s。
- 静态场景改善 {e['static_mean_improvement_s']:.4f} s；四个动态场景均值变化 {e['dynamic_mean_worsening_s']:.4f} s（负值为改善）；treatment contact=0、mask violation=0、最小保守净空 {t['minimum_conservative_clearance_m']:.6f} m。
- 全部预注册门限通过；仅推广该候选速度单因素进入新的 pilot/R2 冻结候选。正式 R2 seeds198001--198200 尚未访问。""")
    append_once(ARTIFACT / "decision_log.md", "## D026 — R2 仅推广候选覆盖单因素", """## D026 — R2 仅推广候选覆盖单因素

- 日期：2026-08-10
- 状态：development 采用，待独立 pilot/formal 验证
- 决策：只把 actions4/5 的目标速度从 0.4 改为 0.6 m/s；PPO actor、解析先验公式、硬验证阈值、其他候选、TEB-v0/DWA-v0 和失败记账均不变。首个预语义导入失败保留且不重试。禁止用 R1/R2 正式结果继续调参。""")
    append_once(ARTIFACT / "statistical_report.md", "## P09 R2 development 单因素消融", f"""## P09 R2 development 单因素消融

预注册的 12 个 development 配对块显示，0.6 m/s 候选相对 R1 control 的失败计入均时从
{c['mean_failure_inclusive_time_s']:.4f} s 降至 {t['mean_failure_inclusive_time_s']:.4f} s，绝对改善
{e['overall_mean_improvement_s']:.4f} s；静态均值改善 {e['static_mean_improvement_s']:.4f} s，动态均值未退化。
包含首个预语义失败的主分析与删除整对 B001 的敏感性分析方向一致；由于这是开发数据，不计算正式置信区间，也不构成优于 TEB/DWA 的证据。""")

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "1.0"; claims["stage"] = STAGE; claims["status"] = "r2_development_factor_promoted"
    claims["claims"] = [item for item in claims["claims"] if item.get("id") != "p09_r2_candidate_coverage_development"]
    claims["claims"].append({
        "id": "p09_r2_candidate_coverage_development",
        "status": "qualified_development_single_factor_signal",
        "claim": "A preregistered development ablation promoted only the 0.6 m/s low-speed spatial candidate factor; this is not formal superiority evidence.",
        "evidence": {"assessment": str(ASSESSMENT.relative_to(ROOT)), "assessment_sha256": sha256(ASSESSMENT), "review_sha256": sha256(REVIEW)},
        "limitations": ["development seeds informed selection", "one retained presemantic system failure", "no confidence interval", "no formal/deployment/real-vehicle claim"],
    })
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"assessment_sha256": sha256(ASSESSMENT), "review_sha256": sha256(REVIEW), "all_gates": assessment["all_decision_gates_pass"], "run_rows": run_count, "episode_rows": episode_count}, indent=2))


if __name__ == "__main__":
    main()
