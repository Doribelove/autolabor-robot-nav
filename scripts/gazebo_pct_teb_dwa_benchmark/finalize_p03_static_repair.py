#!/usr/bin/env python3
"""Freeze the P03 static-candidate repair checkpoint without touching P02."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
EPISODES = ARTIFACT / "development/episodes"
DEV_CONFIG = ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml"
RUNTIME = ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py"
NODE = ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py"
RUNNER = ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py"
STAGE = "P03_STATIC_REPAIR"
STUDY_ROUND = "gazebo_p03_static_repair_development"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_text(path: Path, content: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def records():
    result = []
    for seed in range(196041, 196075):
        roots = sorted(EPISODES.glob("{}_core_static_avoidance_analytic_prior".format(seed)))
        if len(roots) != 1:
            raise RuntimeError("P03_IDENTITY_CARDINALITY_{}={}".format(seed, len(roots)))
        root = roots[0]
        receipt_path = root / "receipt.json"
        if not receipt_path.is_file():
            raise RuntimeError("P03_RECEIPT_ABSENT_{}".format(seed))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        report_path = root / "episode_report.yaml"
        report = yaml.safe_load(report_path.read_text(encoding="utf-8")) if report_path.is_file() else None
        result.append((seed, root, receipt_path, receipt, report_path, report))
    return result


def classify(report):
    if report is None:
        return "infrastructure_failure"
    outcome = report["result"]
    if not report["execution_integrity_pass"]:
        return "execution_integrity_failure"
    if int(outcome["contact_state_count"]) > 0:
        return "contact"
    if float(outcome["minimum_conservative_clearance_m"]) < 0.0:
        return "conservative_clearance_violation"
    if bool(outcome["goal_reached"]):
        return "goal"
    return "timeout_or_nonarrival"


def iso_times(receipt_path, wall_elapsed_s):
    ended = datetime.fromtimestamp(receipt_path.stat().st_mtime).astimezone()
    started = datetime.fromtimestamp(ended.timestamp() - float(wall_elapsed_s)).astimezone()
    return started.isoformat(timespec="seconds"), ended.isoformat(timespec="seconds")


def update_run_manifest(items):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["stage"] != STAGE]
    rows = []
    for seed, root, receipt_path, receipt, _, report in items:
        reason = classify(report)
        started, ended = iso_times(receipt_path, receipt["wall_elapsed_s"])
        outcome = report["result"] if report else {}
        rows.append({
            "stage": STAGE,
            "run_id": root.name,
            "phase": "static_candidate_coverage_and_recovery_repair",
            "method": "analytic_prior",
            "scene_id": "core_static_avoidance",
            "split": "development",
            "seed": seed,
            "attempt": 1,
            "status": "completed_pass" if reason == "goal" else "{}_retained".format(reason),
            "started_at": started,
            "ended_at": ended,
            "exit_code": receipt["return_code"],
            "config_sha256": "unfrozen_not_preflight_bound",
            "model_sha256": "none",
            "evidence_path": str(root.relative_to(ROOT)),
            "formal_eligible": "false",
            "notes": "reason={}; goal={}; integrity={}; clearance={}; timeout={}".format(
                reason,
                bool(outcome.get("goal_reached", False)),
                bool(report and report["execution_integrity_pass"]),
                outcome.get("minimum_conservative_clearance_m"),
                bool(outcome.get("timeout", False)),
            ),
        })
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + rows)
    temporary.replace(path)


def update_episode_results(items):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        retained = [row for row in reader if row["study_round"] != STUDY_ROUND]
    rows = []
    for seed, root, _, _, _, report in items:
        reason = classify(report)
        outcome = report["result"] if report else {}
        goal = bool(outcome.get("goal_reached", False))
        contact = int(outcome.get("contact_state_count", 0))
        elapsed = float(outcome.get("restricted_elapsed_s", 22.0))
        path_length = outcome.get("path_length_m")
        rows.append({
            "study_round": STUDY_ROUND,
            "phase": "static_candidate_repair",
            "block_id": "p03_static_dev_{}".format(seed),
            "episode_id": root.name,
            "method": "analytic_prior",
            "scene_family": "core_static_avoidance",
            "scene_id": "core_static_avoidance",
            "split": "development",
            "seed": seed,
            "attempt": 1,
            "order_index": 0,
            "goal_reached": str(goal).lower(),
            "timeout": str(bool(outcome.get("timeout", False))).lower(),
            "contact_count": contact,
            "system_failure": str(reason == "infrastructure_failure").lower(),
            "failure_reason": reason,
            "failure_inclusive_time_s": elapsed if reason == "goal" else 22.0,
            "min_clearance_m": outcome.get("minimum_conservative_clearance_m"),
            "path_length_m": path_length,
            "mean_speed_mps": path_length / elapsed if path_length is not None and elapsed > 0.0 else None,
            "acceleration_rms": "",
            "jerk_metric": outcome.get("maximum_absolute_jerk_mps3", ""),
            "candidate_switch_count": outcome.get("candidate_switch_count", ""),
            "stop_count": outcome.get("certified_stop_count", ""),
            "recovery_count": outcome.get("stop_recovery_count", ""),
            "mean_latency_ms": outcome.get("mean_inference_latency_ms", ""),
            "p95_latency_ms": outcome.get("p95_inference_latency_ms", ""),
            "p99_latency_ms": outcome.get("p99_inference_latency_ms", ""),
            "mask_violation_count": outcome.get("mask_violation_count", 0),
            "config_sha256": "unfrozen_not_preflight_bound",
            "model_sha256": "none",
            "evidence_path": str((root / ("episode_report.yaml" if report else "roslaunch.log")).relative_to(ROOT)),
        })
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(retained + rows)
    temporary.replace(path)


def uniqueness(path, group, identity):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    keys = [(row[group], row[identity]) for row in rows]
    return len(keys) == len(set(keys)), len(rows)


def append_once(path: Path, marker: str, content: str) -> None:
    original = path.read_text(encoding="utf-8")
    if marker not in original:
        atomic_text(path, original.rstrip() + "\n\n" + content.strip() + "\n")


def main():
    items = records()
    counts = {}
    evidence = []
    for seed, root, receipt_path, _, report_path, report in items:
        reason = classify(report)
        counts[reason] = counts.get(reason, 0) + 1
        evidence.append({
            "seed": seed,
            "reason": reason,
            "receipt_path": str(receipt_path.relative_to(ROOT)),
            "receipt_sha256": sha256(receipt_path),
            "report_path": str(report_path.relative_to(ROOT)) if report else None,
            "report_sha256": sha256(report_path) if report else None,
        })
    gate_item = items[-1]
    gate = gate_item[-1]
    outcome = gate["result"]
    gate_pass = bool(gate["execution_integrity_pass"] and outcome["goal_reached"])
    if not gate_pass:
        raise RuntimeError("P03_STATIC_GATE_FAILED")

    update_run_manifest(items)
    update_episode_results(items)
    run_unique, run_rows = uniqueness(ARTIFACT / "run_manifest.csv", "stage", "run_id")
    episode_unique, episode_rows = uniqueness(
        ARTIFACT / "episode_results.csv", "study_round", "episode_id"
    )
    if not run_unique or not episode_unique:
        raise RuntimeError("LEDGER_IDENTITY_DUPLICATE")

    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "p03_static_candidate_repair_pass_other_core_scenes_pending",
        "development_identities": len(items),
        "retained_outcomes": counts,
        "current_gate": {
            "seed": 196074,
            "goal_reached": outcome["goal_reached"],
            "execution_integrity_pass": gate["execution_integrity_pass"],
            "restricted_elapsed_s": outcome["restricted_elapsed_s"],
            "minimum_conservative_clearance_m": outcome["minimum_conservative_clearance_m"],
            "contact_count": outcome["contact_state_count"],
            "evidence": evidence[-1]["report_path"],
            "evidence_sha256": evidence[-1]["report_sha256"],
        },
        "current_resource_hashes": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in (DEV_CONFIG, RUNTIME, NODE, RUNNER)
        },
        "per_identity_evidence": evidence,
        "integrity_limit": "P03 episodes predate per-episode source preflight binding; frozen formal execution is not authorized",
        "future_runner_preflight_binding_added": True,
        "pilot_or_formal_seed_accessed": False,
        "stable_workspace_modified": False,
        "hardware_accessed": False,
        "run_manifest_unique": run_unique,
        "episode_results_unique": episode_unique,
    }
    report_path = ARTIFACT / "p03_static_candidate_repair_report.json"
    atomic_text(report_path, json.dumps(report, indent=2, sort_keys=True) + "\n")

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.4"
    integrity["status"] = report["status"]
    integrity["formal_execution_authorized"] = False
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["checks"] = [row for row in integrity["checks"] if row.get("id") != "p03_static_candidate_repair"]
    integrity["checks"].append({
        "id": "p03_static_candidate_repair",
        "status": "pass_with_declared_development_binding_limit",
        "unit_and_regression_tests": "27 passed",
        "retained_development_identities": len(items),
        "retained_outcomes": counts,
        "current_gate_seed": 196074,
        "current_gate_goal": True,
        "current_gate_integrity": True,
        "current_gate_minimum_clearance_m": outcome["minimum_conservative_clearance_m"],
        "run_manifest_unique": run_unique,
        "run_manifest_rows": run_rows,
        "episode_results_unique": episode_unique,
        "episode_results_rows": episode_rows,
        "report": str(report_path.relative_to(ROOT)),
        "report_sha256": sha256(report_path),
        "formal_authorized": False,
    })
    integrity["failures"] = [row for row in integrity.get("failures", []) if row.get("stage") != STAGE]
    integrity["failures"].append({
        "stage": STAGE,
        "scope": "development_only",
        "status": "retained_not_suppressed",
        "count": len(items) - counts.get("goal", 0),
        "breakdown": {key: value for key, value in counts.items() if key != "goal"},
    })
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    append_once(
        ARTIFACT / "progress.md",
        "## P03 静态候选修复检查点",
        """## P03 静态候选修复检查点

- development seeds196041--196074 共 34 个身份全部保留：1 goal、24 timeout/nonarrival、
  8 execution-integrity failures、1 infrastructure failure；没有相同身份 retry。
- seed196074 首次通过核心静态绕障门：goal、integrity pass、20.45 s、contact=0、最小保守净空
  0.224907 m。该结果仅证明当前解析先验在一个 development 静态身份上的链路可行。
- 修复内容包括 scan 时刻 TF、完整静态地图验证、候选覆盖/平台、停车后侧向承诺、共同 gate
  原子交接和几何可行性闭合；27/27 单元/回归测试通过。
- 本检查点之前的 development episode 未在启动前绑定逐文件源码 SHA，已声明为完整性限制；
  runner 自下一身份起生成 `preflight_bindings.json`。pilot/formal 仍未访问或授权。
""",
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D011 — 核心静态场景必须先满足冻结安全门限的几何可行性",
        """## D011 — 核心静态场景必须先满足冻结安全门限的几何可行性

- 日期：2026-08-10
- 状态：已采用
- 证据：半走廊 2.35 m、障碍半宽 0.45 m、评价半径 0.79 m 与双侧 0.20 m 门限要求
  半走廊至少 2.43 m；旧场景不存在可行横向带。
- 决策：四方法共享 `core_static_avoidance` 半走廊设为 2.55 m；障碍、安全阈值和评价器不变。

## D012 — 静态绕障候选采用平台轨迹与执行后承诺状态

- 日期：2026-08-10
- 状态：development 采用
- 决策：候选平滑外移、跨障碍保持平台并回归；停车保留已执行侧，反侧/升速只受排序惩罚，
  硬掩码始终有最终权限；越过静态安全包络后直接释放。

## D013 — 公共 warmup 按实际 gate 容差带原子交接

- 日期：2026-08-10
- 状态：已采用
- 证据：固定提前 0.02/0.04 m 在离散 odometry 下分别产生 gate 两侧失败。
- 决策：速度合格后在首个 x>=-4.03 m 采样直接交接，仍用 ±0.03 m 判定四方法共同 gate。

## D014 — development 预执行源码绑定缺口前向修复

- 日期：2026-08-10
- 状态：已采用
- 决策：不伪造 seeds196041--196074 的运行前源码哈希；声明限制。后续 runner 在 ROS 启动前
  固化 world、scene、runner、runtime、node、launch、合同及规划器 YAML 的路径+SHA。
""",
    )
    append_once(
        ARTIFACT / "statistical_report.md",
        "## P03 静态修复的 development 描述",
        """## P03 静态修复的 development 描述

静态修复序列保留 34 个互斥 development 身份：1 个 goal、24 个 timeout/nonarrival、8 个
execution-integrity failure、1 个 infrastructure failure。最终 seed196074 为 goal，限制时间
20.45 s，最小保守净空 0.224907 m，contact=0。该序列是自适应开发诊断，不能估计泛化率、
方法效应或置信区间；失败不可删除，最终门也不得进入 pilot/formal 统计。
""",
    )

    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.4"
    claims["stage"] = "P03"
    claims["status"] = "static_development_component_gate_pass_other_core_scenes_pending"
    claims["claims"] = [row for row in claims["claims"] if row.get("id") != "analytic_prior_core_static_development"]
    claims["claims"].append({
        "id": "analytic_prior_core_static_development",
        "status": "qualified_single_identity_development_component_gate",
        "claim": "The current analytic prior completed one integrity-valid core static Gazebo development episode with zero contact.",
        "evidence": {"report": str(report_path.relative_to(ROOT)), "ledger": "artifacts/gazebo_pct_teb_dwa_benchmark/episode_results.csv"},
        "boundaries": ["adaptive development sequence", "no effect estimate", "no PCT/TEB/DWA comparison", "no formal or deployment claim"],
    })
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))

    hashes_path = ARTIFACT / "method_config_hashes.yaml"
    hashes = yaml.safe_load(hashes_path.read_text(encoding="utf-8"))
    hashes["schema_version"] = "0.3"
    hashes["stage"] = "P03_development_unfrozen"
    hashes["formal_frozen"] = False
    hashes["pct_development_contract"] = {"path": str(DEV_CONFIG.relative_to(ROOT)), "sha256": sha256(DEV_CONFIG)}
    hashes["p03_static_repair_report"] = {"path": str(report_path.relative_to(ROOT)), "sha256": sha256(report_path)}
    hashes["integrity_report"] = {"path": str(integrity_path.relative_to(ROOT)), "sha256": sha256(integrity_path)}
    atomic_text(hashes_path, yaml.safe_dump(hashes, sort_keys=False, allow_unicode=True))

    print(report["status"])
    print("retained_identities={} run_rows={} episode_rows={}".format(len(items), run_rows, episode_rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
