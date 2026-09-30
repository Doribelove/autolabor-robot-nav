#!/usr/bin/env python3
"""Record the pre-outcome formal-round-1 release in all evidence ledgers."""

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
FORMAL = ARTIFACT / "formal/round_1"
RELEASE = FORMAL / "formal_r1_execution_release.yaml"
PREREG = FORMAL / "formal_r1_preregistration.yaml"
SCHEDULE = FORMAL / "formal_r1_schedule.csv"
STAGE = "P07_FORMAL_R1_RELEASE"


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


def main():
    release_sha = sha256(RELEASE)
    prereg_sha = sha256(PREREG)
    schedule_sha = sha256(SCHEDULE)
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = [row for row in reader if row["stage"] != STAGE]
    stamp = datetime.fromtimestamp(RELEASE.stat().st_mtime).astimezone().isoformat(timespec="seconds")
    rows.append(
        {
            "stage": STAGE,
            "run_id": "formal_r1_unique_release",
            "phase": "formal_preregistration_and_release",
            "method": "four_method_paired_matrix",
            "scene_id": "six_core_scenes",
            "split": "formal_round_1_unaccessed",
            "seed": "197001--197160",
            "attempt": 1,
            "status": "released_not_started",
            "started_at": stamp,
            "ended_at": stamp,
            "exit_code": 0,
            "config_sha256": release_sha,
            "model_sha256": "bound_in_release",
            "evidence_path": str(RELEASE.relative_to(ROOT)),
            "formal_eligible": "true",
            "notes": "160 blocks; 640 attempt-one episodes; outcomes unaccessed at release",
        }
    )
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.8"
    integrity["status"] = "formal_r1_released_not_started"
    integrity["formal_execution_authorized"] = True
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["checks"] = [item for item in integrity["checks"] if item.get("id") != "p07_formal_r1_release"]
    integrity["checks"].append(
        {
            "id": "p07_formal_r1_release",
            "status": "pass_preoutcome_release",
            "blocks": 160,
            "episodes": 640,
            "release": str(RELEASE.relative_to(ROOT)),
            "release_sha256": release_sha,
            "preregistration_sha256": prereg_sha,
            "schedule_sha256": schedule_sha,
            "formal_outcome_accessed": False,
        }
    )
    atomic_text(integrity_path, json.dumps(integrity, indent=2, sort_keys=True) + "\n")

    append_once(
        ARTIFACT / "progress.md",
        "## P07 formal R1 release",
        """## P07 formal R1 release

- 在任何 formal outcome 访问前冻结 160 blocks / 640 episodes，fresh seeds197001--197160；四方法在四个顺序位置各 40 次。
- 日程 SHA `89adaa3ad8c6175c2223b91f3b457c789b94858d2b606515413da6bac8c496e8`，预注册 SHA `13785745d0ff40596aacb19c8cb5880baeb9480fce8ba8e3a6714a984078156b`，唯一 release SHA `af9ce1b2e5887cc4b41ef822dd877a50eb222bcdc67107163cc2de920c1cd93f`。
- 主指标、8% 实效门、97.5% 配对 bootstrap、完成率非劣 -5pp、contact/mask 门和 22 s 失败记账已锁定；formal outcome 尚未访问。""",
    )
    append_once(
        ARTIFACT / "decision_log.md",
        "## D024 — formal R1 统计与身份冻结",
        """## D024 — formal R1 统计与身份冻结

- 日期：2026-08-10
- 状态：已采用
- 决策：160 个 fresh 配对块只允许 attempt=1；任一方法对 TEB 或 DWA 达到 8% 点改善、97.5% bootstrap 下界>0、完成率/contact/mask 门后，才可支持相应优势。禁止替换、重跑或按正式结果调参。""",
    )
    append_once(
        ARTIFACT / "statistical_report.md",
        "## P07 formal R1 预注册",
        """## P07 formal R1 预注册

正式 R1 为 160 个 paired seed-scene blocks、640 回合。两项锁定主比较分别为 PCT 对 Actual TEB
和 PCT 对 DWA；每项使用 200,000 次配对块 bootstrap 的双侧 97.5% 区间。阳性需同时满足相对
失败计入时间改善至少 8%、区间下界严格大于 0、完成率差不低于 -5pp、contact 不更高且 PCT
mask violation 为 0。所有失败统一计 22 s。""",
    )
    claims_path = ARTIFACT / "claims_evidence_matrix.yaml"
    claims = yaml.safe_load(claims_path.read_text(encoding="utf-8"))
    claims["schema_version"] = "0.8"
    claims["stage"] = STAGE
    claims["status"] = "formal_r1_released_outcomes_unaccessed"
    claims["claims"] = [item for item in claims["claims"] if item.get("id") != "p07_formal_r1_release"]
    claims["claims"].append(
        {
            "id": "p07_formal_r1_release",
            "status": "preregistered_pending_execution_no_result_claim",
            "claim": "Formal round 1 has a hash-bound 160-block preregistration and grants no result claim before execution.",
            "evidence": {"release_sha256": release_sha, "preregistration_sha256": prereg_sha, "schedule_sha256": schedule_sha},
        }
    )
    atomic_text(claims_path, yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))
    print(json.dumps({"release_sha256": release_sha, "preregistration_sha256": prereg_sha, "schedule_sha256": schedule_sha}, indent=2))


if __name__ == "__main__":
    main()
