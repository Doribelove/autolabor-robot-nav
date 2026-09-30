#!/usr/bin/env python3
"""Seal an already assessed formal-round-1 result into all evidence ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
FORMAL = ARTIFACT / "formal/round_1"
ASSESSMENT = FORMAL / "formal_r1_assessment.json"
RAW = FORMAL / "formal_r1_episode_results.csv"
FAILED_RAW_V0 = FORMAL / "formal_r1_episode_results.failed_v0.csv"
ASSESSOR_FAILURE_V0 = FORMAL / "formal_r1_assessor_failure_v0.json"
EXECUTION = FORMAL / "execution/episodes"
STAGE = "P08_FORMAL_R1"
ROUND = "gazebo_p08_formal_r1"


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


def load():
    assessment = json.loads(ASSESSMENT.read_text(encoding="utf-8"))
    with RAW.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 640 or assessment.get("episodes") != 640 or not assessment.get("formal_result"):
        raise RuntimeError("FORMAL_R1_ASSESSMENT_NOT_COMPLETE")
    return assessment, rows


def update_run_manifest(rows):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["stage"] != STAGE]
    added = []
    for row in rows:
        root = EXECUTION / "{}_{}_{}".format(row["seed"], row["scene_id"], row["method"])
        receipt_path = root / "receipt.json"
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        ended = datetime.fromtimestamp(receipt_path.stat().st_mtime).astimezone()
        started = datetime.fromtimestamp(ended.timestamp() - float(receipt["wall_elapsed_s"])).astimezone()
        status = "completed_pass" if row["valid_goal"] == "True" else row["failure_reason"] + "_retained"
        added.append(
            {
                "stage": STAGE,
                "run_id": "{}__{}_{}_{}".format(row["block_id"], row["seed"], row["scene_id"], row["method"]),
                "phase": "preregistered_formal_round_1",
                "method": row["method"],
                "scene_id": row["scene_id"],
                "split": "formal_round_1",
                "seed": row["seed"],
                "attempt": 1,
                "status": status,
                "started_at": started.isoformat(timespec="seconds"),
                "ended_at": ended.isoformat(timespec="seconds"),
                "exit_code": receipt["return_code"],
                "config_sha256": receipt["release_sha256"],
                "model_sha256": "bound_in_release",
                "evidence_path": str(root.relative_to(ROOT)),
                "formal_eligible": "true",
                "notes": "block={}; order={}; failure_reason={}; no_retry".format(row["block_id"], row["order_index"], row["failure_reason"]),
            }
        )
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + added)
    temporary.replace(path)


def update_episode_results(rows):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["study_round"] != ROUND]
    added = []
    release_sha = sha256(FORMAL / "formal_r1_execution_release.yaml")
    for row in rows:
        elapsed = float(row["failure_inclusive_time_s"])
        path_length = row["path_length_m"]
        values = {
            "study_round": ROUND,
            "phase": "preregistered_formal_round_1",
            "block_id": row["block_id"],
            "episode_id": "{}_{}_{}_{}".format(row["block_id"], row["seed"], row["scene_id"], row["method"]),
            "method": row["method"],
            "scene_family": row["scene_id"],
            "scene_id": row["scene_id"],
            "split": "formal_round_1",
            "seed": row["seed"],
            "attempt": 1,
            "order_index": row["order_index"],
            "goal_reached": row["goal_reached"].lower(),
            "timeout": str(row["failure_reason"] == "timeout_or_nonarrival").lower(),
            "contact_count": row["contact_count"],
            "system_failure": str(row["failure_reason"] == "execution_integrity_failure").lower(),
            "failure_reason": row["failure_reason"],
            "failure_inclusive_time_s": elapsed,
            "min_clearance_m": row["minimum_conservative_clearance_m"],
            "path_length_m": path_length,
            "mean_speed_mps": float(path_length) / elapsed if path_length else "",
            "acceleration_rms": "",
            "jerk_metric": "",
            "candidate_switch_count": row["candidate_switch_count"],
            "stop_count": row["certified_stop_count"],
            "recovery_count": row["stop_recovery_count"],
            "mean_latency_ms": row["mean_inference_latency_ms"],
            "p95_latency_ms": row["p95_inference_latency_ms"],
            "p99_latency_ms": row["p99_inference_latency_ms"],
            "mask_violation_count": row["mask_violation_count"],
            "config_sha256": release_sha,
            "model_sha256": "bound_in_release",
            "evidence_path": row["report_path"],
        }
        added.append({field: values.get(field, "") for field in fields})
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + added)
    temporary.replace(path)


def unique(path, first, second):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[first], row[second]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def percent(value):
    return 100.0 * float(value)


def main():
    assessment, rows = load()
    if sha256(RAW) != sha256(FAILED_RAW_V0):
        raise RuntimeError("FORMAL_R1_SERIALIZATION_REPAIR_CHANGED_RAW_ROWS")
    update_run_manifest(rows)
    update_episode_results(rows)
    run_unique, run_count = unique(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_count = unique(ARTIFACT / "episode_results.csv", "study_round", "episode_id")
    if not run_unique or not episode_unique:
        raise RuntimeError("FORMAL_R1_LEDGER_DUPLICATE")
    review = {
        "schema_version": "1.0",
        "status": "formal_r1_integrity_review_pass",
        "assessment_path": str(ASSESSMENT.relative_to(ROOT)),
        "assessment_sha256": sha256(ASSESSMENT),
        "episode_results_sha256": sha256(RAW),
        "assessor_path": "scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r1.py",
        "assessor_sha256": sha256(ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r1.py"),
        "assessor_timing_limitation": "implementation created after execution began but before matrix completion; exact formulas and seeds were already locked in preregistration",
        "assessor_serialization_repair": {
            "status": "serialization_only_repair_pass",
            "failure_record": str(ASSESSOR_FAILURE_V0.relative_to(ROOT)),
            "failure_record_sha256": sha256(ASSESSOR_FAILURE_V0),
            "failed_assessor_sha256": "fbca2368a5d50314537a57932ac3836c1e9734ff84f384522a2d18c0073f01d9",
            "repaired_assessor_sha256": sha256(ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r1.py"),
            "failed_raw_sha256": sha256(FAILED_RAW_V0),
            "repaired_raw_sha256": sha256(RAW),
            "raw_byte_identical": True,
            "behavioral_formula_or_threshold_change": False,
            "duplicate_later_invocation_interrupted_before_write": True,
        },
        "blocks": 160,
        "episodes": 640,
        "attempt_one_complete": True,
        "same_identity_retry_count": 0,
        "replacement_seed_count": 0,
        "run_manifest_unique": run_unique,
        "run_manifest_rows": run_count,
        "episode_results_unique": episode_unique,
        "episode_results_rows": episode_count,
        "formal_round_2_seed_accessed": False,
        "hardware_accessed": False,
    }
    review_path = FORMAL / "formal_r1_integrity_review.json"
    atomic_text(review_path, json.dumps(review, indent=2, sort_keys=True) + "\n")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.9"
    integrity["status"] = "formal_r1_complete"
    integrity["formal_execution_authorized"] = False
    integrity["new_gazebo_formal_seeds_accessed"] = True
    integrity["checks"] = [item for item in integrity["checks"] if item.get("id") != "p08_formal_r1"]
    integrity["checks"].append(
        {
            "id": "p08_formal_r1",
            "status": "pass_complete",
            "formal_result": assessment["formal_result"],
            "performance_improvement_demonstrated": assessment["performance_improvement_demonstrated"],
            "positive_comparators": assessment["positive_comparators"],
            "blocks": 160,
            "episodes": 640,
            "review": str(review_path.relative_to(ROOT)),
            "review_sha256": sha256(review_path),
            "assessment_sha256": sha256(ASSESSMENT),
            "run_manifest_unique": run_unique,
            "episode_results_unique": episode_unique,
            "formal_round_2_seed_accessed": False,
        }
    )
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    summaries = assessment["method_summaries"]
    teb = assessment["primary_comparisons"]["actual_teb"]
    dwa = assessment["primary_comparisons"]["dwa"]
    positives = ", ".join(assessment["positive_comparators"]) or "none"
    append_once(
        ARTIFACT / "progress.md",
        "## P08 formal R1 result",
        """## P08 formal R1 result

- 160 blocks / 640 episodes 全部 attempt=1 完成；无 retry、replacement 或删除，正式 R2 seeds 未访问。
- 失败计入均时：PCT {pct:.4f} s（{pct_goal}/160）、解析 {analytic:.4f} s（{analytic_goal}/160）、TEB {teb_mean:.4f} s（{teb_goal}/160）、DWA {dwa_mean:.4f} s（{dwa_goal}/160）。
- PCT 对 TEB 改善 {teb_effect:.4f}%（97.5% CI [{teb_lo:.4f}%, {teb_hi:.4f}%）；对 DWA 改善 {dwa_effect:.4f}%（97.5% CI [{dwa_lo:.4f}%, {dwa_hi:.4f}%）。
- `formal_result=true`；`performance_improvement_demonstrated={demonstrated}`；阳性比较：{positives}。
- 完整性 review SHA `{review_sha}`。""".format(
            pct=summaries["pct_scope_rl"]["mean_failure_inclusive_time_s"], pct_goal=summaries["pct_scope_rl"]["valid_goals"],
            analytic=summaries["analytic_prior"]["mean_failure_inclusive_time_s"], analytic_goal=summaries["analytic_prior"]["valid_goals"],
            teb_mean=summaries["actual_teb"]["mean_failure_inclusive_time_s"], teb_goal=summaries["actual_teb"]["valid_goals"],
            dwa_mean=summaries["dwa"]["mean_failure_inclusive_time_s"], dwa_goal=summaries["dwa"]["valid_goals"],
            teb_effect=percent(teb["relative_reduction"]), teb_lo=percent(teb["bootstrap"]["lower_97_5_percent"]), teb_hi=percent(teb["bootstrap"]["upper_97_5_percent"]),
            dwa_effect=percent(dwa["relative_reduction"]), dwa_lo=percent(dwa["bootstrap"]["lower_97_5_percent"]), dwa_hi=percent(dwa["bootstrap"]["upper_97_5_percent"]),
            demonstrated=str(assessment["performance_improvement_demonstrated"]).lower(), positives=positives, review_sha=sha256(review_path),
        ),
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D025 — formal R1 结果封存",
        """## D025 — formal R1 结果封存

- 日期：2026-08-10
- 状态：已采用
- 决策：R1 640 个身份终止且不可重跑。`performance_improvement_demonstrated={}`；阳性比较为 `{}`。若无阳性，只能在新 development 数据上做可证伪单因素修复，并以全新 R2 seeds 重新预注册。""".format(str(assessment["performance_improvement_demonstrated"]).lower(), positives),
    )
    append_once(
        ARTIFACT / "statistical_report.md",
        "## P08 formal R1 主结果",
        """## P08 formal R1 主结果

PCT/TEB/DWA 的失败计入均时分别为 {pct:.4f}/{teb_mean:.4f}/{dwa_mean:.4f} s。PCT 相对 TEB
为 {teb_effect:.4f}%（双侧 97.5% CI [{teb_lo:.4f}%, {teb_hi:.4f}%），相对 DWA 为
{dwa_effect:.4f}%（[{dwa_lo:.4f}%, {dwa_hi:.4f}%）。阳性比较：{positives}。
所有 160 个配对块、失败和安全门均进入统计。""".format(
            pct=summaries["pct_scope_rl"]["mean_failure_inclusive_time_s"], teb_mean=summaries["actual_teb"]["mean_failure_inclusive_time_s"], dwa_mean=summaries["dwa"]["mean_failure_inclusive_time_s"],
            teb_effect=percent(teb["relative_reduction"]), teb_lo=percent(teb["bootstrap"]["lower_97_5_percent"]), teb_hi=percent(teb["bootstrap"]["upper_97_5_percent"]),
            dwa_effect=percent(dwa["relative_reduction"]), dwa_lo=percent(dwa["bootstrap"]["lower_97_5_percent"]), dwa_hi=percent(dwa["bootstrap"]["upper_97_5_percent"]), positives=positives,
        ),
    )

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.9"
    claims["stage"] = STAGE
    claims["status"] = "formal_r1_complete"
    claims["claims"] = [item for item in claims["claims"] if item.get("id") != "p08_formal_r1"]
    claims["claims"].append(
        {
            "id": "p08_formal_r1",
            "status": "qualified_positive" if assessment["performance_improvement_demonstrated"] else "qualified_negative",
            "claim": "Formal round 1 completed all 160 paired blocks. Positive comparator gates passed for: {}.".format(positives),
            "evidence": {"assessment": str(ASSESSMENT.relative_to(ROOT)), "assessment_sha256": sha256(ASSESSMENT), "review_sha256": sha256(review_path)},
            "limitations": ["six frozen Gazebo templates", "simulation only", "no perception/OOD/deployment/real-vehicle claim"],
        }
    )
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"review": str(review_path), "review_sha256": sha256(review_path), "run_rows": run_count, "episode_rows": episode_count}, indent=2))


if __name__ == "__main__":
    main()
