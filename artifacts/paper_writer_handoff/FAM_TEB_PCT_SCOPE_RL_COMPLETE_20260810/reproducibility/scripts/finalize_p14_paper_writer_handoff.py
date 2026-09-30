#!/usr/bin/env python3
"""Record the pure-offline external paper-writer handoff in all project ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ART = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
PACKAGE = ROOT / "artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810"
STAGE = "P14_PAPER_WRITER_HANDOFF"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic(path: Path, text: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def append_once(path: Path, marker: str, text: str) -> None:
    original = path.read_text(encoding="utf-8")
    if marker not in original:
        atomic(path, original.rstrip() + "\n\n" + text.strip() + "\n")


def update_run_manifest() -> int:
    path = ART / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = [row for row in reader if row["stage"] != STAGE]
    stamp = datetime.now().astimezone().isoformat(timespec="seconds")
    rows.append(
        {
            "stage": STAGE,
            "run_id": "complete_external_paper_writer_handoff",
            "phase": "pure_offline_paper_handoff",
            "method": "all",
            "scene_id": "all_frozen_project_evidence",
            "split": "offline",
            "seed": "none",
            "attempt": 1,
            "status": "completed_pass",
            "started_at": stamp,
            "ended_at": stamp,
            "exit_code": 0,
            "config_sha256": sha256(
                ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/build_paper_writer_handoff.py"
            ),
            "model_sha256": "all_models_read_only_bound_by_upstream_manifests",
            "evidence_path": str(PACKAGE.relative_to(ROOT)),
            "formal_eligible": "false",
            "notes": "portable paper handoff; no simulation/training/formal seed/hardware; episode ledger unchanged",
        }
    )
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)
    return len(rows)


def validate_episode_ledger() -> int:
    path = ART / "episode_results.csv"
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    keys = [(row["study_round"], row["episode_id"]) for row in rows]
    if len(keys) != len(set(keys)):
        raise RuntimeError("P14_EPISODE_LEDGER_DUPLICATE")
    if len(rows) != 6145:
        raise RuntimeError(f"P14_EPISODE_LEDGER_UNEXPECTED_ROWS:{len(rows)}")
    return len(rows)


def main() -> None:
    package_integrity = json.loads((PACKAGE / "handoff_integrity.json").read_text())
    if package_integrity.get("status") != "paper_writer_handoff_integrity_pass":
        raise RuntimeError("P14_PACKAGE_PRECHECK_FAILED")
    run_rows = update_run_manifest()
    episode_rows = validate_episode_ledger()

    append_once(
        ART / "progress.md",
        "## P14 external paper-writer handoff",
        """## P14 external paper-writer handoff

- 已生成面向另一论文写作系统的外层权威交接目录，嵌入历史 V6-06 包并新增两轮 Gazebo formal、
  claims contract、canonical JSON、LaTeX 章节、图表、配置/代码和原始证据 SHA 索引。
- 全量索引覆盖 23,744 个 benchmark 文件（约 806 MB）；便携包不重复复制逐 episode ROS 日志。
- 本阶段为纯离线派生：无训练、仿真、formal seed、ROS/Gazebo、硬件或实车；episode 总账保持
  6,145 行且唯一。旧 V6-06 包保持字节历史，只由外层 supersession notice 更新 Gazebo 写作口径。""",
    )
    append_once(
        ART / "decision_log.md",
        "## D032 — 论文写作系统外层包优先级",
        """## D032 — 论文写作系统外层包优先级

- 日期：2026-08-10
- 状态：当前论文移交入口
- 决策：另一系统首先读取外层 `claims_contract.yaml` 与 `canonical_results.json`。旧 V6-06 包继续
  作为 V3/V5/V6 历史依据，但其“尚无 Gazebo formal”只描述 2026-08-08 当时状态；新 R1/R2
  已完成且均为 superiority 阴性。不得把时间性更新误写为 Gazebo 阳性，也不得修改历史包字节。""",
    )
    append_once(
        ART / "statistical_report.md",
        "## P14 论文系统统一结果入口",
        """## P14 论文系统统一结果入口

研究级机器总表现包含 V3-11/V3-21、V4、V5、V6 fast population、历史 V6 Gazebo preflight 和
新 Gazebo R1/R2。当前 Gazebo 终局为两轮 360 paired blocks / 1,440 episodes，无阳性比较；R2
对 DWA 7.7754% 的正区间不能覆盖冻结 8% 点门。交接包只重组已封存结果，不产生新估计量。""",
    )

    integrity_path = ART / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text())
    integrity["status"] = "paper_writer_handoff_complete"
    integrity["formal_execution_authorized"] = False
    integrity["checks"] = [
        item for item in integrity["checks"] if item.get("id") != "p14_paper_writer_handoff"
    ]
    integrity["checks"].append(
        {
            "id": "p14_paper_writer_handoff",
            "status": "pass_complete",
            "package_path": str(PACKAGE.relative_to(ROOT)),
            "full_raw_benchmark_files_indexed": 23744,
            "formal_rounds_summarized": 2,
            "formal_blocks_summarized": 360,
            "formal_episodes_summarized": 1440,
            "run_manifest_rows": run_rows,
            "episode_results_rows_unchanged": episode_rows,
            "episode_results_unique": True,
            "simulation_or_training_run": False,
            "hardware_accessed": False,
            "stable_workspace_modified": False,
        }
    )
    atomic(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    claims_path = ART / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text())
    claims["stage"] = STAGE
    claims["status"] = "paper_writer_handoff_complete"
    claims["claims"] = [
        item for item in claims["claims"] if item.get("id") != "p14_paper_writer_handoff"
    ]
    claims["claims"].append(
        {
            "id": "p14_paper_writer_handoff",
            "status": "qualified_offline_transfer",
            "claim": "All current paper-relevant historical and new Gazebo evidence is organized under one outer claims contract; no new scientific result was created.",
            "evidence": {
                "package_path": str(PACKAGE.relative_to(ROOT)),
                "required_first": "claims_contract.yaml",
                "machine_results": "canonical_results.json",
                "raw_index": "indices/full_raw_benchmark_evidence_index.csv",
            },
            "limitations": [
                "portable core omits duplicate raw ROS logs",
                "raw logs remain available only through repository paths",
                "no new execution or real-vehicle evidence",
            ],
        }
    )
    atomic(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))

    print(
        json.dumps(
            {
                "status": "p14_paper_writer_handoff_ledgers_complete",
                "run_manifest_rows": run_rows,
                "episode_results_rows": episode_rows,
                "package_path": str(PACKAGE.relative_to(ROOT)),
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
