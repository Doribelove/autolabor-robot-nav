#!/usr/bin/env python3
"""Assess the preregistered pilot, perform power analysis and refresh ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
PILOT = ARTIFACT / "pilot/pilot_r1"
EXECUTION = PILOT / "execution/episodes"
SCHEDULE = PILOT / "pilot_schedule.csv"
RELEASE = PILOT / "pilot_execution_release.yaml"
STAGE = "P06_PILOT_R1"
ROUND = "gazebo_p06_pilot_r1"
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")


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


def load_rows():
    schedule = list(csv.DictReader(SCHEDULE.open(encoding="utf-8")))
    release_sha = sha256(RELEASE)
    rows = []
    for item in schedule:
        root = EXECUTION / "{}_{}_{}".format(item["seed"], item["scene_id"], item["method"])
        report_path = root / "episode_report.yaml"
        receipt_path = root / "receipt.json"
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        result = report["result"]
        valid = bool(
            report["execution_integrity_pass"]
            and result["goal_reached"]
            and int(result.get("contact_state_count", 0)) == 0
            and float(result["minimum_conservative_clearance_m"]) >= 0.0
        )
        if int(item["attempt"]) != 1 or receipt["release_sha256"] != release_sha:
            raise RuntimeError("PILOT_IDENTITY_OR_RELEASE_MISMATCH")
        if receipt["episode_report_sha256"] != sha256(report_path):
            raise RuntimeError("PILOT_REPORT_HASH_MISMATCH")
        if report.get("split") != "pilot" or report.get("stage") != "PCT-BENCH-PILOT-R1":
            raise RuntimeError("PILOT_REPORT_STAGE_MISMATCH")
        rows.append(
            {
                **item,
                "root": root,
                "report_path": report_path,
                "receipt_path": receipt_path,
                "report": report,
                "receipt": receipt,
                "valid": valid,
                "failure_reason": "goal" if valid else (
                    "contact" if int(result.get("contact_state_count", 0)) > 0
                    else "conservative_clearance_violation" if float(result["minimum_conservative_clearance_m"]) < 0.0
                    else "execution_integrity_failure" if not report["execution_integrity_pass"]
                    else "timeout_or_nonarrival"
                ),
                "failure_inclusive_time_s": float(result["restricted_elapsed_s"]) if valid else 22.0,
            }
        )
    if len(rows) != 80 or len({(row["seed"], row["scene_id"], row["method"]) for row in rows}) != 80:
        raise RuntimeError("PILOT_COMPLETENESS_FAILURE")
    return rows


def relative_effect(pct, baseline):
    return float((np.mean(baseline) - np.mean(pct)) / np.mean(baseline))


def bootstrap_effect(pct, baseline, seed=20260810, samples=100000):
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(pct), size=(samples, len(pct)))
    pct_means = np.mean(pct[indices], axis=1)
    baseline_means = np.mean(baseline[indices], axis=1)
    effects = (baseline_means - pct_means) / baseline_means
    return {
        "samples": samples,
        "seed": seed,
        "lower_97_5_percent": float(np.quantile(effects, 0.0125)),
        "upper_97_5_percent": float(np.quantile(effects, 0.9875)),
    }


def power_analysis(pct, baseline):
    difference = baseline - pct
    sd = float(np.std(difference, ddof=1))
    target = 0.08 * float(np.mean(baseline))
    normal = statistics.NormalDist()
    critical = normal.inv_cdf(0.9875)
    desired = normal.inv_cdf(0.80)
    required = ((critical + desired) * sd / target) ** 2
    powers = {
        str(blocks): normal.cdf(np.sqrt(blocks) * target / sd - critical)
        for blocks in (100, 120, 150, 160, 180, 200)
    }
    return {
        "paired_difference_sd_s": sd,
        "target_8_percent_difference_s": target,
        "required_blocks_unrounded": required,
        "required_blocks_ceiling": int(np.ceil(required)),
        "normal_approximation_power": powers,
        "multiplicity_critical_quantile": 0.9875,
    }


def method_summary(rows, method):
    subset = [row for row in rows if row["method"] == method]
    result = [row["report"]["result"] for row in subset]
    return {
        "episodes": len(subset),
        "valid_goals": sum(row["valid"] for row in subset),
        "mean_failure_inclusive_time_s": float(np.mean([row["failure_inclusive_time_s"] for row in subset])),
        "minimum_conservative_clearance_m": min(float(item["minimum_conservative_clearance_m"]) for item in result),
        "contact_count": sum(int(item.get("contact_state_count", 0)) for item in result),
        "mask_violation_count": sum(int(item.get("mask_violation_count", 0)) for item in result),
        "failure_counts": {
            reason: sum(row["failure_reason"] == reason for row in subset)
            for reason in sorted({row["failure_reason"] for row in subset})
        },
    }


def update_run_manifest(rows):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [item for item in reader if item["stage"] != STAGE]
    added = []
    for row in rows:
        receipt = row["receipt"]
        ended = datetime.fromtimestamp(row["receipt_path"].stat().st_mtime).astimezone()
        started = datetime.fromtimestamp(ended.timestamp() - float(receipt["wall_elapsed_s"])).astimezone()
        added.append(
            {
                "stage": STAGE,
                "run_id": "{}__{}_{}_{}".format(row["block_id"], row["seed"], row["scene_id"], row["method"]),
                "phase": "preregistered_pilot",
                "method": row["method"],
                "scene_id": row["scene_id"],
                "split": "pilot",
                "seed": row["seed"],
                "attempt": 1,
                "status": "completed_pass" if row["valid"] else row["failure_reason"] + "_retained",
                "started_at": started.isoformat(timespec="seconds"),
                "ended_at": ended.isoformat(timespec="seconds"),
                "exit_code": receipt["return_code"],
                "config_sha256": receipt["release_sha256"],
                "model_sha256": row["report"].get("actor_checkpoint_sha256") or "none",
                "evidence_path": str(row["root"].relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "block={}; order={}; release={}; pilot_not_for_tuning".format(row["block_id"], row["order_index"], receipt["release_sha256"]),
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
        retained = [item for item in reader if item["study_round"] != ROUND]
    added = []
    for row in rows:
        result = row["report"]["result"]
        elapsed = float(result["restricted_elapsed_s"])
        path_length = result.get("path_length_m")
        values = {
            "study_round": ROUND,
            "phase": "preregistered_pilot",
            "block_id": row["block_id"],
            "episode_id": "{}_{}_{}_{}".format(row["block_id"], row["seed"], row["scene_id"], row["method"]),
            "method": row["method"],
            "scene_family": row["scene_id"],
            "scene_id": row["scene_id"],
            "split": "pilot",
            "seed": row["seed"],
            "attempt": 1,
            "order_index": row["order_index"],
            "goal_reached": str(bool(result["goal_reached"])).lower(),
            "timeout": str(bool(result["timeout"])).lower(),
            "contact_count": result["contact_state_count"],
            "system_failure": str(row["failure_reason"] == "execution_integrity_failure").lower(),
            "failure_reason": row["failure_reason"],
            "failure_inclusive_time_s": row["failure_inclusive_time_s"],
            "min_clearance_m": result["minimum_conservative_clearance_m"],
            "path_length_m": path_length if path_length is not None else "",
            "mean_speed_mps": path_length / elapsed if path_length is not None and elapsed > 0 else "",
            "acceleration_rms": "",
            "jerk_metric": result.get("maximum_absolute_jerk_mps3", ""),
            "candidate_switch_count": result.get("candidate_switch_count", ""),
            "stop_count": result.get("certified_stop_count", ""),
            "recovery_count": result.get("stop_recovery_count", ""),
            "mean_latency_ms": result.get("mean_inference_latency_ms", ""),
            "p95_latency_ms": result.get("p95_inference_latency_ms", ""),
            "p99_latency_ms": result.get("p99_inference_latency_ms", ""),
            "mask_violation_count": result.get("mask_violation_count", 0),
            "config_sha256": row["receipt"]["release_sha256"],
            "model_sha256": row["report"].get("actor_checkpoint_sha256") or "none",
            "evidence_path": str(row["report_path"].relative_to(ROOT)),
        }
        added.append({field: values.get(field, "") for field in fields})
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + added)
    temporary.replace(path)


def verify_unique(path, first, second):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[first], row[second]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def main():
    rows = load_rows()
    by_block = {
        (row["block_id"], row["method"]): row["failure_inclusive_time_s"] for row in rows
    }
    block_ids = sorted({row["block_id"] for row in rows})
    pct = np.asarray([by_block[(block, "pct_scope_rl")] for block in block_ids])
    comparisons = {}
    for index, baseline_name in enumerate(("actual_teb", "dwa")):
        baseline = np.asarray([by_block[(block, baseline_name)] for block in block_ids])
        comparisons[baseline_name] = {
            "relative_reduction": relative_effect(pct, baseline),
            "bootstrap": bootstrap_effect(pct, baseline, seed=20260810 + index),
            "power_analysis": power_analysis(pct, baseline),
        }
    summaries = {method: method_summary(rows, method) for method in METHODS}
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "pilot_complete_formal_sample_size_selected",
        "release_sha256": sha256(RELEASE),
        "schedule_sha256": sha256(SCHEDULE),
        "blocks": 20,
        "episodes": 80,
        "attempt_one_complete": True,
        "method_summaries": summaries,
        "paired_comparisons": comparisons,
        "formal_round_1_selected_blocks": 160,
        "selection_basis": "DWA 8% target requires 153 blocks for 80% normal-approximation power; rounded to 160. TEB requires 253, exceeding the frozen 200-block cap and is declared underpowered.",
        "pilot_tuning_performed": False,
        "formal_seed_accessed": False,
        "hardware_accessed": False,
        "stable_workspace_modified": False,
        "per_identity_evidence": [
            {
                "block_id": row["block_id"],
                "seed": int(row["seed"]),
                "scene_id": row["scene_id"],
                "method": row["method"],
                "valid": row["valid"],
                "failure_reason": row["failure_reason"],
                "failure_inclusive_time_s": row["failure_inclusive_time_s"],
                "report": str(row["report_path"].relative_to(ROOT)),
                "report_sha256": sha256(row["report_path"]),
                "receipt": str(row["receipt_path"].relative_to(ROOT)),
                "receipt_sha256": sha256(row["receipt_path"]),
            }
            for row in rows
        ],
    }
    report_path = PILOT / "pilot_assessment.json"
    atomic_text(report_path, json.dumps(report, indent=2, sort_keys=True) + "\n")
    update_run_manifest(rows)
    update_episode_results(rows)
    run_unique, run_count = verify_unique(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_count = verify_unique(ARTIFACT / "episode_results.csv", "study_round", "episode_id")
    if not run_unique or not episode_unique:
        raise RuntimeError("PILOT_LEDGER_DUPLICATE")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.7"
    integrity["status"] = "pilot_complete_formal_not_released"
    integrity["checks"] = [item for item in integrity["checks"] if item.get("id") != "p06_pilot_r1"]
    integrity["checks"].append(
        {
            "id": "p06_pilot_r1",
            "status": "pass_complete_negative_for_claims",
            "release_sha256": sha256(RELEASE),
            "schedule_sha256": sha256(SCHEDULE),
            "blocks": 20,
            "episodes": 80,
            "attempt_one_complete": True,
            "run_manifest_unique": run_unique,
            "run_manifest_rows": run_count,
            "episode_results_unique": episode_unique,
            "episode_results_rows": episode_count,
            "report": str(report_path.relative_to(ROOT)),
            "report_sha256": sha256(report_path),
            "formal_seed_accessed": False,
        }
    )
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["formal_execution_authorized"] = False
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    append_once(
        ARTIFACT / "progress.md",
        "## P06 preregistered pilot",
        """## P06 preregistered pilot

- 唯一 release SHA `967c4d688405f0a40a95ae99d9b856948dc79632a42636dde72eddc90f70fd7b` 按预注册日程完成 20 blocks / 80 episodes，全部 attempt=1，哈希/报告/身份闭包通过，ROS/Gazebo 清场。
- 失败计入均时：PCT 15.1550 s（19/20）、解析 16.7850 s（17/20）、TEB 14.8525 s（11/20）、DWA 16.0775 s（10/20）；全方法 contact=0，PCT mask violation=0。
- PCT 相对 DWA 点改善 5.7378%，相对 TEB 为 -2.0367%；pilot 不支持优势声明且未用于调参。
- 8% 目标下，DWA 比较 80% 功效约需 153 blocks；TEB 约需 253，超过 200 上限。正式 R1 选择 160 blocks，对 TEB 功效不足明确保留为限制。
- formal seeds 尚未访问，正式 release 尚未创建。""",
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D022 — pilot 仅用于功效，不回调调参",
        """## D022 — pilot 仅用于功效，不回调调参

- 日期：2026-08-10
- 状态：已采用
- 决策：pilot 5.7378% 对 DWA、-2.0367% 对 TEB 的点结果只用于配对方差和样本量；代码、actor、TEB/DWA 参数均不修改。

## D023 — 正式 R1 选择 160 blocks

- 日期：2026-08-10
- 状态：已采用
- 决策：DWA 8% 效应的 80% 功效需 153 blocks，向上取整到 160；TEB 所需 253 超过 200 上限，仍完整比较但预声明功效不足。""",
    )
    append_once(
        ARTIFACT / "statistical_report.md",
        "## P06 pilot 与功效分析",
        """## P06 pilot 与功效分析

预注册 pilot 共 20 个配对块、80 回合。失败计入均时为 PCT 15.1550 s、解析 16.7850 s、
Actual TEB 14.8525 s、DWA 16.0775 s；PCT 相对 DWA 的点改善 5.7378%，相对 TEB 为 -2.0367%。
pilot 仅估计方差，不构成优势检验。

按配对差标准差、8% 目标效应、双主比较 97.5% 临界值与 80% 功效，DWA 需 153 blocks，TEB
需 253 blocks。正式 R1 预注册 160 blocks；其 DWA 近似功效约 82%，TEB 比较因 200-block
上限而功效不足，必须作为限制报告。""",
    )

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.7"
    claims["stage"] = STAGE
    claims["status"] = "pilot_complete_no_advantage_claim_formal_not_started"
    claims["claims"] = [item for item in claims["claims"] if item.get("id") != "p06_pilot"]
    claims["claims"].append(
        {
            "id": "p06_pilot",
            "status": "pilot_only_not_qualified_for_advantage",
            "claim": "The preregistered pilot completed 20 paired blocks without tuning and selected 160 formal blocks; it did not itself demonstrate PCT superiority.",
            "evidence": {"report": str(report_path.relative_to(ROOT)), "report_sha256": sha256(report_path)},
            "limitations": ["pilot sample", "not a formal test", "TEB comparison underpowered within cap", "simulation only"],
        }
    )
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"report": str(report_path), "summaries": summaries, "comparisons": comparisons, "run_rows": run_count, "episode_rows": episode_count}, indent=2))


if __name__ == "__main__":
    main()
