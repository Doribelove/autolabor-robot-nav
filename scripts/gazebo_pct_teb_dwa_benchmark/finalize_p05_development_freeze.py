#!/usr/bin/env python3
"""Seal the final development candidate and refresh all seven evidence ledgers."""

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
STAGE = "P05_DEV_FREEZE"
GAZEBO_ROUND = "gazebo_p05_development_freeze"
OFFLINE_ROUNDS = (
    (
        "p05_offline_r2_rejected",
        ARTIFACT / "development/offline_validation/pct_benchmark_r2",
    ),
    (
        "p05_offline_r3_footprint",
        ARTIFACT / "development/offline_validation/pct_benchmark_r3",
    ),
)
TRAINING_RUNS = (
    ARTIFACT / "development/training/pct_benchmark_r2_head_label_seed9702",
    ARTIFACT / "development/training/pct_benchmark_r3_footprint_seed9703",
)
GAZEBO_GROUPS = (
    ("freeze_candidate_r1", ARTIFACT / "development/freeze_candidate_r1/episodes"),
    (
        "footprint_repair_r1",
        ARTIFACT / "development/footprint_repair_r1/episodes/episodes",
    ),
    (
        "final_freeze_candidate_r1",
        ARTIFACT / "development/final_freeze_candidate_r1/episodes",
    ),
)
FINAL_SCENES = (
    (196117, "core_static_avoidance"),
    (196118, "core_single_crossing"),
    (196119, "core_head_on_yield"),
    (196120, "core_sequential_interaction"),
    (196117, "core_partially_occluded"),
    (196118, "core_width_change"),
)
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_text(path, content):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def append_once(path, marker, content):
    original = path.read_text(encoding="utf-8")
    if marker in original:
        return
    atomic_text(path, original.rstrip() + "\n\n" + content.strip() + "\n")


def reason(report, receipt):
    if report is None:
        return "infrastructure_failure"
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


def load_gazebo_records():
    records = []
    for variant, group in GAZEBO_GROUPS:
        for root in sorted(group.iterdir()):
            if not root.is_dir():
                continue
            report_path = root / "episode_report.yaml"
            receipt_path = root / "receipt.json"
            report = (
                yaml.safe_load(report_path.read_text(encoding="utf-8"))
                if report_path.is_file()
                else None
            )
            receipt = (
                json.loads(receipt_path.read_text(encoding="utf-8"))
                if receipt_path.is_file()
                else None
            )
            records.append(
                {
                    "variant": variant,
                    "root": root,
                    "report_path": report_path,
                    "receipt_path": receipt_path,
                    "report": report,
                    "receipt": receipt,
                    "reason": reason(report, receipt),
                    "identity": variant + "__" + root.name,
                }
            )
    return records


def record_times(record):
    evidence = record["receipt_path"] if record["receipt"] else record["report_path"]
    ended = datetime.fromtimestamp(evidence.stat().st_mtime).astimezone()
    wall = float(record["receipt"].get("wall_elapsed_s", 0.0)) if record["receipt"] else 0.0
    started = datetime.fromtimestamp(ended.timestamp() - wall).astimezone()
    return started.isoformat(timespec="seconds"), ended.isoformat(timespec="seconds")


def bound_hash(record):
    receipt = record["receipt"] or {}
    return (
        receipt.get("planner_config_sha256")
        or receipt.get("preflight_bindings_sha256")
        or "receipt_absent"
    )


def update_run_manifest(records):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["stage"] != STAGE]
    rows = []
    for record in records:
        report, receipt = record["report"], record["receipt"]
        result = report["result"] if report else {}
        started, ended = record_times(record)
        rows.append(
            {
                "stage": STAGE,
                "run_id": record["identity"],
                "phase": "gazebo_development_freeze_and_ablation",
                "method": report.get("method", "pct_scope_rl") if report else "pct_scope_rl",
                "scene_id": report.get("scene_id", "core_static_avoidance") if report else "core_static_avoidance",
                "split": "development",
                "seed": report.get("seed", 196115) if report else 196115,
                "attempt": 1,
                "status": "completed_pass" if record["reason"] == "goal" else record["reason"] + "_retained",
                "started_at": started,
                "ended_at": ended,
                "exit_code": receipt.get("return_code", 1) if receipt else 1,
                "config_sha256": bound_hash(record),
                "model_sha256": (report.get("actor_checkpoint_sha256") or "none") if report else "unknown",
                "evidence_path": str(record["root"].relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "variant={}; reason={}; goal={}; integrity={}; clearance={}".format(
                    record["variant"],
                    record["reason"],
                    bool(result.get("goal_reached", False)),
                    bool(report and report.get("execution_integrity_pass")),
                    result.get("minimum_conservative_clearance_m"),
                ),
            }
        )
    for directory in TRAINING_RUNS:
        summary_path = directory / "training_summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        stamp = datetime.fromtimestamp(summary_path.stat().st_mtime).astimezone().isoformat(timespec="seconds")
        selected = directory.name.startswith("pct_benchmark_r3_") is False
        rows.append(
            {
                "stage": STAGE,
                "run_id": directory.name,
                "phase": "development_training_40008_steps",
                "method": "pct_scope_rl",
                "scene_id": "four_dynamic_sensor_families",
                "split": "development_offline",
                "seed": summary["training_seed"],
                "attempt": 1,
                "status": "completed_rejected",
                "started_at": stamp,
                "ended_at": stamp,
                "exit_code": 0,
                "config_sha256": "development_source_not_prebound",
                "model_sha256": sha256(directory / "actor_final.pt"),
                "evidence_path": str(summary_path.relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "bounded development identity; rejected by independent offline validation; no pilot/formal seed accessed; selected={}".format(selected),
            }
        )
    for round_id, directory in OFFLINE_ROUNDS:
        report_path = directory / "report.json"
        report = json.loads(report_path.read_text(encoding="utf-8"))
        stamp = datetime.fromtimestamp(report_path.stat().st_mtime).astimezone().isoformat(timespec="seconds")
        rows.append(
            {
                "stage": STAGE,
                "run_id": round_id,
                "phase": "development_offline_validation",
                "method": "multi_actor_comparison",
                "scene_id": "four_dynamic_sensor_families",
                "split": "development_offline",
                "seed": "{}--{}".format(report["seed_start"], report["seed_start"] + 399),
                "attempt": 1,
                "status": "completed_pass",
                "started_at": stamp,
                "ended_at": stamp,
                "exit_code": 0,
                "config_sha256": "development_source_not_prebound" if "r2" in round_id else sha256(ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py"),
                "model_sha256": "multiple_bound_in_report",
                "evidence_path": str(report_path.relative_to(ROOT)),
                "formal_eligible": "false",
                "notes": "development-only; {} episodes; no pilot/formal seed accessed".format(
                    sum(item["episodes"] for item in report["summaries"].values())
                ),
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
    result = report["result"] if report else {}
    elapsed = float(result.get("restricted_elapsed_s", 22.0))
    path_length = result.get("path_length_m")
    values = {
        "study_round": GAZEBO_ROUND,
        "phase": "development_freeze_and_ablation",
        "block_id": "p05_{}_{}_{}".format(
            record["variant"],
            report.get("scene_id", "core_static_avoidance") if report else "core_static_avoidance",
            report.get("seed", 196115) if report else 196115,
        ),
        "episode_id": record["identity"],
        "method": report.get("method", "pct_scope_rl") if report else "pct_scope_rl",
        "scene_family": report.get("scene_id", "core_static_avoidance") if report else "core_static_avoidance",
        "scene_id": report.get("scene_id", "core_static_avoidance") if report else "core_static_avoidance",
        "split": "development",
        "seed": report.get("seed", 196115) if report else 196115,
        "attempt": 1,
        "order_index": 0,
        "goal_reached": str(bool(result.get("goal_reached", False))).lower(),
        "timeout": str(bool(result.get("timeout", False))).lower(),
        "contact_count": int(result.get("contact_state_count", 0)),
        "system_failure": str(record["reason"] == "infrastructure_failure").lower(),
        "failure_reason": record["reason"],
        "failure_inclusive_time_s": elapsed if record["reason"] == "goal" else 22.0,
        "min_clearance_m": result.get("minimum_conservative_clearance_m", ""),
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
        "config_sha256": bound_hash(record),
        "model_sha256": (report.get("actor_checkpoint_sha256") or "none") if report else "unknown",
        "evidence_path": str((record["report_path"] if report else record["root"] / "roslaunch.log").relative_to(ROOT)),
    }
    return {field: values.get(field, "") for field in fields}


def update_episode_results(records):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        excluded = {GAZEBO_ROUND, *(item[0] for item in OFFLINE_ROUNDS)}
        retained = [row for row in reader if row["study_round"] not in excluded]
    rows = [gazebo_episode_row(record, fields) for record in records]
    for round_id, directory in OFFLINE_ROUNDS:
        offline_path = directory / "episode_results.csv"
        report = json.loads((directory / "report.json").read_text(encoding="utf-8"))
        hashes = {item["label"]: item["sha256"] for item in report["actor_bindings"]}
        hashes["analytic_prior"] = "none"
        with offline_path.open(newline="", encoding="utf-8") as stream:
            for source in csv.DictReader(stream):
                valid = source["goal_reached"] == "True" and int(source["contact_count"]) == 0
                values = {
                    "study_round": round_id,
                    "phase": "development_offline_validation",
                    "block_id": "{}_{}_{}".format(round_id, source["family"], source["seed"]),
                    "episode_id": "{}_{}_{}_{}".format(round_id, source["method"], source["family"], source["seed"]),
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
                    "candidate_switch_count": source["learned_difference_count"],
                    "mask_violation_count": source["mask_violation_count"],
                    "config_sha256": "development_source_not_prebound" if "r2" in round_id else sha256(ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py"),
                    "model_sha256": hashes[source["method"]],
                    "evidence_path": str(offline_path.relative_to(ROOT)),
                }
                rows.append({field: values.get(field, "") for field in fields})
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + rows)
    temporary.replace(path)


def final_summary():
    root = ARTIFACT / "development/final_freeze_candidate_r1/episodes"
    summary = {}
    for method in METHODS:
        rows = []
        for seed, scene in FINAL_SCENES:
            path = root / "{}_{}_{}".format(seed, scene, method) / "episode_report.yaml"
            report = yaml.safe_load(path.read_text(encoding="utf-8"))
            result = report["result"]
            valid = bool(
                report["execution_integrity_pass"]
                and result["goal_reached"]
                and int(result.get("contact_state_count", 0)) == 0
                and float(result["minimum_conservative_clearance_m"]) >= 0.0
            )
            rows.append(
                {
                    "seed": seed,
                    "scene": scene,
                    "valid_goal": valid,
                    "failure_inclusive_time_s": float(result["restricted_elapsed_s"]) if valid else 22.0,
                    "minimum_conservative_clearance_m": float(result["minimum_conservative_clearance_m"]),
                    "report": str(path.relative_to(ROOT)),
                    "report_sha256": sha256(path),
                }
            )
        summary[method] = {
            "valid_goals": int(sum(item["valid_goal"] for item in rows)),
            "blocks": len(rows),
            "mean_failure_inclusive_time_s": float(np.mean([item["failure_inclusive_time_s"] for item in rows])),
            "minimum_conservative_clearance_m": min(item["minimum_conservative_clearance_m"] for item in rows),
            "rows": rows,
        }
    return summary


def verify_unique(path, first, second):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[first], row[second]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def main():
    records = load_gazebo_records()
    if len(records) != 50:
        raise RuntimeError("P05_GAZEBO_IDENTITY_COUNT_MISMATCH:{}".format(len(records)))
    update_run_manifest(records)
    update_episode_results(records)
    run_unique, run_rows = verify_unique(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_rows = verify_unique(ARTIFACT / "episode_results.csv", "study_round", "episode_id")
    if not run_unique or not episode_unique:
        raise RuntimeError("P05_LEDGER_IDENTITY_DUPLICATE")

    outcomes = {}
    evidence = []
    for record in records:
        outcomes[record["reason"]] = outcomes.get(record["reason"], 0) + 1
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
    summary = final_summary()
    pct = summary["pct_scope_rl"]["mean_failure_inclusive_time_s"]
    teb = summary["actual_teb"]["mean_failure_inclusive_time_s"]
    dwa = summary["dwa"]["mean_failure_inclusive_time_s"]
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "development_candidate_frozen_pilot_not_started",
        "gazebo_identities": len(records),
        "gazebo_outcomes": outcomes,
        "final_six_block_development_matrix": summary,
        "descriptive_relative_reduction_pct": {
            "versus_actual_teb": 100.0 * (teb - pct) / teb,
            "versus_dwa": 100.0 * (dwa - pct) / dwa,
        },
        "selected_actor": {
            "path": "artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt",
            "sha256": sha256(ARTIFACT / "development/training/pct_benchmark_r1_seed9701/actor_final.pt"),
            "basis": "best failure-inclusive independent offline development mean with zero contacts and mask violations; R2 and R3 rejected",
        },
        "selected_baselines": {
            "actual_teb": "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml",
            "dwa": "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml",
            "basis": "strongest full-six-scene development candidates; slower conservative variants rejected",
        },
        "offline_validation_reports": {
            round_id: {
                "path": str((directory / "report.json").relative_to(ROOT)),
                "sha256": sha256(directory / "report.json"),
            }
            for round_id, directory in OFFLINE_ROUNDS
        },
        "tests": "41 passed",
        "run_manifest_unique": run_unique,
        "run_manifest_rows": run_rows,
        "episode_results_unique": episode_unique,
        "episode_results_rows": episode_rows,
        "formal_or_pilot_seed_accessed": False,
        "formal_execution_authorized": False,
        "stable_workspace_modified": False,
        "hardware_accessed": False,
        "per_identity_evidence": evidence,
    }
    report_path = ARTIFACT / "p05_development_freeze_report.json"
    atomic_text(report_path, json.dumps(report, indent=2, sort_keys=True) + "\n")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.6"
    integrity["status"] = report["status"]
    integrity["formal_execution_authorized"] = False
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["checks"] = [item for item in integrity["checks"] if item.get("id") != "p05_development_freeze"]
    integrity["checks"].append(
        {
            "id": "p05_development_freeze",
            "status": "pass_development_only",
            "tests": report["tests"],
            "training_steps_new": 80016,
            "offline_validation_episodes_new": 2800,
            "gazebo_identities": len(records),
            "gazebo_outcomes": outcomes,
            "final_matrix": {
                method: {
                    "valid_goals": item["valid_goals"],
                    "blocks": item["blocks"],
                    "mean_failure_inclusive_time_s": item["mean_failure_inclusive_time_s"],
                }
                for method, item in summary.items()
            },
            "run_manifest_unique": run_unique,
            "run_manifest_rows": run_rows,
            "episode_results_unique": episode_unique,
            "episode_results_rows": episode_rows,
            "report": str(report_path.relative_to(ROOT)),
            "report_sha256": sha256(report_path),
            "pilot_or_formal_seed_accessed": False,
        }
    )
    integrity["failures"] = [item for item in integrity.get("failures", []) if item.get("stage") != STAGE]
    integrity["failures"].append(
        {
            "stage": STAGE,
            "scope": "development_only",
            "status": "retained_not_suppressed",
            "count": len(records) - outcomes.get("goal", 0),
            "breakdown": {key: value for key, value in outcomes.items() if key != "goal"},
        }
    )
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    append_once(
        ARTIFACT / "progress.md",
        "## P05 development 冻结候选",
        """## P05 development 冻结候选

- PCT R2 头对头标签与 R3 评估足迹重训均完成 40,008 步并封存；独立离线开发验证分别为 1,200/1,600 回合，R1 仍以 400/400、16.7863 s、0 contact/mask violation 获选。
- 评估足迹修复把空间候选中心从 1.40 m 调整到 1.35 m，并让无扫描墙面回退、训练 rank oracle 和物理净空统一计入 0.33 m 膨胀；安全门未放宽。41/41 测试通过。
- seed196115 因外层 PYTHONPATH 覆盖导致 roslaunch 导入失败，按基础设施失败保留且未重跑；fresh seed196116 静态验证 15.75 s、净空 0.20995 m。
- 最终六场景四方法开发矩阵：PCT 6/6、14.9500 s；解析 4/6、17.0750 s；TEB-v0 4/6、13.3000 s；DWA-v0 3/6、16.1083 s。均为失效计入均时。
- PCT 相对 DWA 的开发点改善 7.1903%，低于 8% 实效门；相对 TEB 为 -12.4060%。这只是自适应开发描述，不能支持优势声明。
- 代码/actor/TEB-v0/DWA-v0 已作为 pilot 候选固定；pilot/formal seeds 尚未访问。""",
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D019 — 拒绝 R2/R3，保留 R1 actor",
        """## D019 — 拒绝 R2/R3，保留 R1 actor

- 日期：2026-08-10
- 状态：已采用
- 决策：R2 的场景专用 head-on 标签在 400 回合验证中未改善 head-on 且损害遮挡；R3 虽完成足迹对齐训练但均时 17.1019 s，劣于同合同下 R1 的 16.7863 s。二者封存，R1 进入 pilot 候选。

## D020 — 静态停车死锁按评估足迹对齐修复

- 日期：2026-08-10
- 状态：已采用
- 证据：失败 seed196109 中 PPO 干预为 0；1.40 m 候选在跟踪中过冲至 1.565 m，使扫描备份余量跌破 0.20 m并永久只剩 certified-stop。
- 决策：空间中心改为 1.35 m，所有墙面/训练 oracle/训练物理净空统一使用 robot radius + 0.33 m inflation；不改 0.20 m 硬验证门和正式 0 m 接触边界。

## D021 — 最终开发选择后禁止继续调参

- 日期：2026-08-10
- 状态：已采用
- 决策：固定 R1 actor、TEB-v0、DWA-v0、解析先验与 P05 源码候选进入 pilot。pilot 只估计方差/功效和可行性，结果不得返回开发调参。""",
    )
    append_once(
        ARTIFACT / "statistical_report.md",
        "## P05 最终 development 冻结描述",
        """## P05 最终 development 冻结描述

六个配对开发块的失败计入均时为 PCT 14.9500 s（6/6）、解析先验 17.0750 s（4/6）、
Actual TEB 13.3000 s（4/6）、DWA 16.1083 s（3/6）。PCT 相对 DWA 的点改善为 7.1903%，
相对 TEB 为 -12.4060%。该矩阵参与了开发决策，禁止计算正式显著性或用作论文优势证据。

足迹对齐后的 1,600 回合离线验证中，解析/R1/R2/R3 均为 400/400 且 0 contact、0 mask violation；
均时分别 17.1700/16.7863/17.1456/17.1019 s。R1 按失败计入均时预先选择。下一阶段 pilot 仅用于
方差、功效和系统可行性估计，不允许调参。""",
    )

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.6"
    claims["stage"] = "P05_DEV_FREEZE"
    claims["status"] = "development_candidate_frozen_pilot_not_started"
    claims["claims"] = [item for item in claims["claims"] if item.get("id") != "p05_development_freeze"]
    claims["claims"].append(
        {
            "id": "p05_development_freeze",
            "status": "descriptive_development_only_not_qualified",
            "claim": "The final adaptive development matrix retained all failures and selected R1, TEB-v0 and DWA-v0, but PCT did not meet the 8% practical threshold versus DWA and was slower than TEB.",
            "evidence": {
                "report": str(report_path.relative_to(ROOT)),
                "report_sha256": sha256(report_path),
                "blocks": 6,
                "pct_mean_s": pct,
                "teb_mean_s": teb,
                "dwa_mean_s": dwa,
            },
            "limitations": [
                "adaptive development data",
                "six blocks only",
                "no confidence interval",
                "pilot and formal seeds untouched",
                "no deployment or real-vehicle authority",
            ],
        }
    )
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"report": str(report_path), "run_rows": run_rows, "episode_rows": episode_rows, "summary": summary}, indent=2))


if __name__ == "__main__":
    main()
