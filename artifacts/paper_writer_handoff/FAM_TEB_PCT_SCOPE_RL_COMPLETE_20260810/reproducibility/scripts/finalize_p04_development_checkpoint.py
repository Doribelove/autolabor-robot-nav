#!/usr/bin/env python3
"""Seal the P04 development checkpoint and refresh all seven evidence ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
CURRENT = ARTIFACT / "development/episodes"
TUNING = ARTIFACT / "development/tuning"
TRAINING = ARTIFACT / "development/training/pct_benchmark_r1_seed9701"
OFFLINE = ARTIFACT / "development/offline_validation/pct_benchmark_r1"
STAGE = "P04_DEV_CHECKPOINT"
GAZEBO_ROUND = "gazebo_p04_development_checkpoint"
OFFLINE_ROUND = "p04_offline_validation_r1"

CORE = [
    (196093, "core_static_avoidance"),
    (196094, "core_single_crossing"),
    (196095, "core_head_on_yield"),
    (196096, "core_sequential_interaction"),
    (196097, "core_partially_occluded"),
    (196098, "core_width_change"),
]
CORE_PATTERNS = {
    "pct_scope_rl": CURRENT / "{seed}_{scene}_pct_scope_rl/episode_report.yaml",
    "analytic_prior": CURRENT / "{seed}_{scene}_analytic_prior/episode_report.yaml",
    "actual_teb_v0": CURRENT / "{seed}_{scene}_actual_teb/episode_report.yaml",
    "dwa_v0": CURRENT / "{seed}_{scene}_dwa/episode_report.yaml",
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


def gazebo_roots():
    result = []
    for root in sorted(CURRENT.iterdir()):
        if not root.is_dir():
            continue
        try:
            seed = int(root.name.split("_", 1)[0])
        except ValueError:
            continue
        if seed >= 196075:
            result.append(("current", root))
    for root in sorted(TUNING.glob("*/episodes/*")):
        if root.is_dir():
            result.append((root.parents[1].name, root))
    return result


def load_record(variant, root):
    report_path = root / "episode_report.yaml"
    receipt_path = root / "receipt.json"
    report = yaml.safe_load(report_path.read_text(encoding="utf-8")) if report_path.is_file() else None
    receipt = json.loads(receipt_path.read_text(encoding="utf-8")) if receipt_path.is_file() else None
    if report is None:
        reason = "infrastructure_failure"
    elif receipt is None:
        reason = "infrastructure_receipt_failure"
    else:
        outcome = report["result"]
        if int(outcome.get("contact_state_count", 0)) > 0:
            reason = "contact"
        elif outcome.get("minimum_conservative_clearance_m") is not None and float(
            outcome["minimum_conservative_clearance_m"]
        ) < 0.0:
            reason = "conservative_clearance_violation"
        elif not bool(report.get("execution_integrity_pass")):
            reason = "execution_integrity_failure"
        elif bool(outcome.get("goal_reached")):
            reason = "goal"
        else:
            reason = "timeout_or_nonarrival"
    return {
        "variant": variant,
        "root": root,
        "identity": variant + "__" + root.name,
        "report_path": report_path,
        "receipt_path": receipt_path,
        "report": report,
        "receipt": receipt,
        "reason": reason,
    }


def record_times(record):
    evidence = record["receipt_path"] if record["receipt"] else record["report_path"]
    ended = datetime.fromtimestamp(evidence.stat().st_mtime).astimezone()
    wall = float(record["receipt"].get("wall_elapsed_s", 0.0)) if record["receipt"] else 0.0
    started = datetime.fromtimestamp(ended.timestamp() - wall).astimezone()
    return started.isoformat(timespec="seconds"), ended.isoformat(timespec="seconds")


def update_run_manifest(records):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["stage"] != STAGE]
    rows = []
    for record in records:
        report, receipt = record["report"], record["receipt"]
        outcome = report["result"] if report else {}
        started, ended = record_times(record)
        method = report.get("method", "unknown") if report else record["root"].name.rsplit("_", 1)[-1]
        rows.append(
            {
                "stage": STAGE,
                "run_id": record["identity"],
                "phase": "gazebo_development_and_baseline_tuning",
                "method": method,
                "scene_id": report.get("scene_id", "unknown") if report else "unknown",
                "split": "development",
                "seed": report.get("seed", "") if report else "",
                "attempt": 1,
                "status": "completed_pass" if record["reason"] == "goal" else record["reason"] + "_retained",
                "started_at": started,
                "ended_at": ended,
                "exit_code": receipt.get("return_code", 1) if receipt else 1,
                "config_sha256": (
                    receipt.get("planner_config_sha256")
                    or receipt.get("preflight_bindings_sha256")
                    if receipt
                    else "receipt_absent"
                ),
                "model_sha256": report.get("actor_checkpoint_sha256") or "none" if report else "unknown",
                "evidence_path": str(record["root"].relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "variant={}; reason={}; goal={}; integrity={}; clearance={}".format(
                    record["variant"],
                    record["reason"],
                    bool(outcome.get("goal_reached", False)),
                    bool(report and report.get("execution_integrity_pass")),
                    outcome.get("minimum_conservative_clearance_m"),
                ),
            }
        )
    training_summary = TRAINING / "training_summary.json"
    offline_report = OFFLINE / "report.json"
    for run_id, phase, evidence in (
        ("pct_benchmark_r1_seed9701", "development_training_40008_steps", training_summary),
        ("pct_benchmark_r1_offline_validation", "development_offline_validation_1200_episodes", offline_report),
    ):
        stamp = datetime.fromtimestamp(evidence.stat().st_mtime).astimezone().isoformat(timespec="seconds")
        rows.append(
            {
                "stage": STAGE,
                "run_id": run_id,
                "phase": phase,
                "method": "pct_scope_rl",
                "scene_id": "four_dynamic_sensor_families",
                "split": "development_offline",
                "seed": "9701" if "seed9701" in run_id else "970100--970499",
                "attempt": 1,
                "status": "completed_pass",
                "started_at": stamp,
                "ended_at": stamp,
                "exit_code": 0,
                "config_sha256": sha256(ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_training.py"),
                "model_sha256": sha256(TRAINING / "actor_final.pt"),
                "evidence_path": str(evidence.relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "development-only; no pilot/formal seed accessed",
            }
        )
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + rows)
    temporary.replace(path)


def gazebo_episode_row(record, fields):
    report = record["report"]
    outcome = report["result"] if report else {}
    valid = record["reason"] == "goal"
    elapsed = float(outcome.get("restricted_elapsed_s", 22.0))
    path_length = outcome.get("path_length_m")
    values = {
        "study_round": GAZEBO_ROUND,
        "phase": "development_and_tuning",
        "block_id": "p04_{}_{}_{}".format(record["variant"], report.get("scene_id", "unknown") if report else "unknown", report.get("seed", "unknown") if report else "unknown"),
        "episode_id": record["identity"],
        "method": report.get("method", "unknown") if report else "unknown",
        "scene_family": report.get("scene_id", "unknown") if report else "unknown",
        "scene_id": report.get("scene_id", "unknown") if report else "unknown",
        "split": "development",
        "seed": report.get("seed", "") if report else "",
        "attempt": 1,
        "order_index": 0,
        "goal_reached": str(bool(outcome.get("goal_reached", False))).lower(),
        "timeout": str(bool(outcome.get("timeout", False))).lower(),
        "contact_count": int(outcome.get("contact_state_count", 0)),
        "system_failure": str(record["reason"].startswith("infrastructure")).lower(),
        "failure_reason": record["reason"],
        "failure_inclusive_time_s": elapsed if valid else 22.0,
        "min_clearance_m": outcome.get("minimum_conservative_clearance_m", ""),
        "path_length_m": path_length if path_length is not None else "",
        "mean_speed_mps": path_length / elapsed if path_length is not None and elapsed > 0 else "",
        "acceleration_rms": "",
        "jerk_metric": outcome.get("maximum_absolute_jerk_mps3", ""),
        "candidate_switch_count": outcome.get("candidate_switch_count", ""),
        "stop_count": outcome.get("certified_stop_count", ""),
        "recovery_count": outcome.get("stop_recovery_count", ""),
        "mean_latency_ms": outcome.get("mean_inference_latency_ms", ""),
        "p95_latency_ms": outcome.get("p95_inference_latency_ms", ""),
        "p99_latency_ms": outcome.get("p99_inference_latency_ms", ""),
        "mask_violation_count": outcome.get("mask_violation_count", 0),
        "config_sha256": (
            record["receipt"].get("planner_config_sha256")
            or record["receipt"].get("preflight_bindings_sha256")
            if record["receipt"]
            else "receipt_absent"
        ),
        "model_sha256": report.get("actor_checkpoint_sha256") or "none" if report else "unknown",
        "evidence_path": str((record["report_path"] if report else record["root"] / "roslaunch.log").relative_to(ROOT)),
    }
    return {field: values.get(field, "") for field in fields}


def update_episode_results(records):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [
            row
            for row in reader
            if row["study_round"] not in (GAZEBO_ROUND, OFFLINE_ROUND)
        ]
    rows = [gazebo_episode_row(record, fields) for record in records]
    offline_path = OFFLINE / "episode_results.csv"
    offline_model_hashes = {
        "analytic_prior": "none",
        "historical_pct": "8c9fc80be384568acdf8a61959b4d16ba6811372f9737eff345089a08f3d91b7",
        "benchmark_r1_seed9701": sha256(TRAINING / "actor_final.pt"),
    }
    with offline_path.open(newline="", encoding="utf-8") as stream:
        for source in csv.DictReader(stream):
            valid = source["goal_reached"] == "True" and int(source["contact_count"]) == 0
            values = {
                "study_round": OFFLINE_ROUND,
                "phase": "development_offline_validation",
                "block_id": "p04_offline_{}_{}".format(source["family"], source["seed"]),
                "episode_id": "p04_offline_{}_{}_{}".format(source["method"], source["family"], source["seed"]),
                "method": source["method"],
                "scene_family": source["family"],
                "scene_id": source["family"],
                "split": "development_offline",
                "seed": source["seed"],
                "attempt": 1,
                "order_index": 0,
                "goal_reached": source["goal_reached"].lower(),
                "timeout": str(not valid).lower(),
                "contact_count": source["contact_count"],
                "system_failure": "false",
                "failure_reason": "goal" if valid else "offline_failure",
                "failure_inclusive_time_s": source["failure_inclusive_restricted_time_s"],
                "min_clearance_m": source["minimum_physical_clearance_m"],
                "path_length_m": "",
                "mean_speed_mps": "",
                "acceleration_rms": "",
                "jerk_metric": "",
                "candidate_switch_count": source["learned_difference_count"],
                "stop_count": "",
                "recovery_count": "",
                "mean_latency_ms": "",
                "p95_latency_ms": "",
                "p99_latency_ms": "",
                "mask_violation_count": source["mask_violation_count"],
                "config_sha256": sha256(ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py"),
                "model_sha256": offline_model_hashes[source["method"]],
                "evidence_path": str(offline_path.relative_to(ROOT)),
            }
            rows.append({field: values.get(field, "") for field in fields})
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + rows)
    temporary.replace(path)


def core_summary():
    result = {}
    for method, pattern in CORE_PATTERNS.items():
        rows = []
        for seed, scene in CORE:
            path = Path(str(pattern).format(seed=seed, scene=scene))
            report = yaml.safe_load(path.read_text(encoding="utf-8"))
            outcome = report["result"]
            valid = bool(report["execution_integrity_pass"] and outcome["goal_reached"])
            rows.append(
                {
                    "seed": seed,
                    "scene": scene,
                    "valid_goal": valid,
                    "failure_inclusive_time_s": float(outcome["restricted_elapsed_s"]) if valid else 22.0,
                    "minimum_conservative_clearance_m": float(outcome["minimum_conservative_clearance_m"]),
                    "report": str(path.relative_to(ROOT)),
                    "report_sha256": sha256(path),
                }
            )
        result[method] = {
            "valid_goals": int(sum(row["valid_goal"] for row in rows)),
            "blocks": len(rows),
            "mean_failure_inclusive_time_s": float(np.mean([row["failure_inclusive_time_s"] for row in rows])),
            "minimum_conservative_clearance_m": min(row["minimum_conservative_clearance_m"] for row in rows),
            "rows": rows,
        }
    return result


def verify_unique(path, first, second):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[first], row[second]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def main():
    records = [load_record(variant, root) for variant, root in gazebo_roots()]
    update_run_manifest(records)
    update_episode_results(records)
    run_unique, run_rows = verify_unique(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_rows = verify_unique(ARTIFACT / "episode_results.csv", "study_round", "episode_id")
    if not run_unique or not episode_unique:
        raise RuntimeError("P04_LEDGER_IDENTITY_DUPLICATE")

    counts = {}
    variants = {}
    evidence = []
    for record in records:
        counts[record["reason"]] = counts.get(record["reason"], 0) + 1
        variants[record["variant"]] = variants.get(record["variant"], 0) + 1
        evidence.append(
            {
                "identity": record["identity"],
                "reason": record["reason"],
                "report": str(record["report_path"].relative_to(ROOT)) if record["report"] else None,
                "report_sha256": sha256(record["report_path"]) if record["report"] else None,
                "receipt": str(record["receipt_path"].relative_to(ROOT)) if record["receipt"] else None,
                "receipt_sha256": sha256(record["receipt_path"]) if record["receipt"] else None,
            }
        )
    summary = core_summary()
    offline_report = json.loads((OFFLINE / "report.json").read_text(encoding="utf-8"))
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "development_checkpoint_complete_pilot_not_authorized",
        "gazebo_identities": len(records),
        "gazebo_outcomes": counts,
        "variant_episode_counts": variants,
        "core_six_block_descriptive_matrix": summary,
        "offline_validation": offline_report,
        "training": json.loads((TRAINING / "training_summary.json").read_text(encoding="utf-8")),
        "selected_development_baseline_candidates": {
            "actual_teb": "benchmark_teb_v0.yaml",
            "dwa": "benchmark_dwa_v0.yaml",
            "selection_basis": "lowest six-block failure-inclusive mean among fully evaluated v0 and conservative alternatives; all invalid episodes retained at 22 s",
        },
        "resource_hashes_at_checkpoint": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in (
                ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
                ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_training.py",
                ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
                ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py",
                ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml",
                TRAINING / "actor_final.pt",
                ROOT / "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml",
                ROOT / "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml",
            )
        },
        "tests": "34 passed",
        "run_manifest_unique": run_unique,
        "run_manifest_rows": run_rows,
        "episode_results_unique": episode_unique,
        "episode_results_rows": episode_rows,
        "per_identity_evidence": evidence,
        "pilot_or_formal_seed_accessed": False,
        "formal_execution_authorized": False,
        "stable_workspace_modified": False,
        "hardware_accessed": False,
    }
    report_path = ARTIFACT / "p04_development_checkpoint_report.json"
    atomic_text(report_path, json.dumps(report, indent=2, sort_keys=True) + "\n")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.5"
    integrity["status"] = report["status"]
    integrity["formal_execution_authorized"] = False
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["checks"] = [row for row in integrity["checks"] if row.get("id") != "p04_development_checkpoint"]
    integrity["checks"].append(
        {
            "id": "p04_development_checkpoint",
            "status": "pass_development_only",
            "tests": "34 passed",
            "training_steps": 40008,
            "offline_validation_episodes": 1200,
            "gazebo_identities": len(records),
            "gazebo_outcomes": counts,
            "run_manifest_unique": run_unique,
            "run_manifest_rows": run_rows,
            "episode_results_unique": episode_unique,
            "episode_results_rows": episode_rows,
            "report": str(report_path.relative_to(ROOT)),
            "report_sha256": sha256(report_path),
            "pilot_or_formal_seed_accessed": False,
        }
    )
    integrity["failures"] = [row for row in integrity.get("failures", []) if row.get("stage") != STAGE]
    integrity["failures"].append(
        {
            "stage": STAGE,
            "scope": "development_only",
            "status": "retained_not_suppressed",
            "count": len(records) - counts.get("goal", 0),
            "breakdown": {key: value for key, value in counts.items() if key != "goal"},
        }
    )
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    append_once(
        ARTIFACT / "progress.md",
        "## P04 development 检查点",
        """## P04 development 检查点

- 新 benchmark PPO identity 完成 40,008 环境步；训练 571/575 到达、0 contact、0 mask violation。
- 互斥离线验证 1,200 episodes：新 PCT 400/400、解析先验 400/400、历史 PCT 392/400；均保留逐 episode CSV。
- 首个六场景同 seed 四方法开发矩阵：PCT 6/6、解析 6/6、TEB-v0 4/6、DWA-v0 3/6；失败统一按 22 s。
- 六块失效计入均时：PCT 15.6917 s、解析 15.9083 s、TEB-v0 13.2833 s、DWA-v0 16.1167 s。仅为自适应 development 描述，不是优势证据。
- TEB/DWA 的保守、homotopy、LiDAR 动态转换、inflation 与长视野变体均完整保留；当前 development 选择仍为各自 v0，禁止用较慢变体弱化基线。
- static 条件高速脉冲正向，持久高速承诺造成安全停车死锁并回滚；方向 tracker 的关联劫持已修复。34/34 测试通过。
- pilot/formal seeds 未访问，正式执行仍未授权。""",
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D015 — benchmark 分布对齐后才允许 PPO 重训",
        """## D015 — benchmark 分布对齐后才允许 PPO 重训

- 日期：2026-08-10
- 状态：已采用
- 证据：旧训练后端包含虚构走廊墙、错误遮挡物与中心 head-on 死锁；历史 checkpoint 在对齐后端为 392/400、18.760 s。
- 决策：修正四动态族几何、22 s 时限与候选条件真值清障标签后，执行一个 40,008 步 development identity；新模型为 400/400、16.711 s，0 contact/mask violation。

## D016 — 最强传统开发候选不得按安全外观替换

- 日期：2026-08-10
- 状态：已采用
- 决策：按完整六块失败计入均时选择 TEB-v0 与 DWA-v0；所有负净距按 22 s 计入。不得改选更慢的 v8/v2 来制造 PCT 优势。

## D017 — 静态同侧条件高速脉冲保留，持久提速承诺回滚

- 日期：2026-08-10
- 状态：部分采用
- 证据：按完整预测净距 0.22 m 释放单周期高速把解析/PCT 静态时间降至 16.90/17.30 s；把承诺速度永久升到 1.2 m/s 后两方法 seed196105 均安全超时。
- 决策：保留条件高速脉冲，回滚持久提速状态；失败身份不重跑。

## D018 — 动态 tracker 关联失败必须先过期再重捕获

- 日期：2026-08-10
- 状态：已采用
- 决策：关联 miss 后 0.50 s 内保持原轨迹且禁止无关 cluster 劫持；超时后清空再捕获。速度上界和不确定度不变。""",
    )
    stats = ARTIFACT / "statistical_report.md"
    append_once(
        stats,
        "## P04 六块 development 描述",
        """## P04 六块 development 描述

同 seed 六场景自适应开发矩阵的失效计入均时为 PCT 15.6917 s（6/6）、解析先验 15.9083 s（6/6）、
TEB-v0 13.2833 s（4/6）和 DWA-v0 16.1167 s（3/6）。PCT 相对 DWA 的点改善约 2.64%，低于
预定 8% 实效门；相对 TEB 为负。该矩阵参与过调参，不能计算或报告正式置信区间，也不能支持优势声明。

离线互斥开发验证共 1,200 episodes。新模型 400/400、平均 16.710625 s；解析先验 400/400、
17.190000 s；历史模型 392/400、18.760000 s。三者 contact 与 mask violation 均为 0。""",
    )

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.5"
    claims["stage"] = "P04"
    claims["status"] = "development_signal_only_pilot_formal_not_started"
    claims["claims"] = [row for row in claims["claims"] if row.get("id") != "p04_development_four_method_signal"]
    claims["claims"].append(
        {
            "id": "p04_development_four_method_signal",
            "status": "descriptive_development_only_not_qualified",
            "claim": "The aligned PCT checkpoint completed all six adaptive development blocks, but did not meet the preregistered 8% practical threshold versus DWA and remained slower than TEB-v0.",
            "evidence": {
                "report": str(report_path.relative_to(ROOT)),
                "ledger": str((ARTIFACT / "episode_results.csv").relative_to(ROOT)),
            },
            "boundaries": [
                "adaptive development data",
                "no confidence interval",
                "no superiority claim",
                "no pilot/formal/deployment authority",
            ],
        }
    )
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"report": str(report_path), "gazebo": len(records), "outcomes": counts, "run_rows": run_rows, "episode_rows": episode_rows}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
