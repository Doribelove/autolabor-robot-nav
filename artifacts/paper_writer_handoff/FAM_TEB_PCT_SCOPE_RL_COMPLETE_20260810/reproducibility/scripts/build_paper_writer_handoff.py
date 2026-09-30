#!/usr/bin/env python3
"""Build a portable, evidence-indexed handoff for an external paper-writing system."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import shutil

import yaml


ROOT = Path(__file__).resolve().parents[2]
BENCH = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
UPSTREAM = ROOT / "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package"
OUT_ROOT = ROOT / "artifacts/paper_writer_handoff"
TARGET = OUT_ROOT / "FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810"
R1_PATH = BENCH / "formal/round_1/formal_r1_assessment.json"
R2_PATH = BENCH / "formal/round_2/formal_r2_assessment.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_text(relative: str, text: str) -> Path:
    target = TARGET / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text.rstrip() + "\n", encoding="utf-8")
    return target


def copy_one(source: Path, relative: str, source_rows: list[dict]) -> None:
    if not source.is_file():
        raise RuntimeError(f"MISSING_HANDOFF_SOURCE:{source}")
    target = TARGET / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    digest = sha256(source)
    if sha256(target) != digest:
        raise RuntimeError(f"COPY_HASH_MISMATCH:{source}")
    source_rows.append(
        {
            "upstream_path": str(source.relative_to(ROOT)),
            "portable_path": relative,
            "bytes": source.stat().st_size,
            "sha256": digest,
        }
    )


def copy_tree(source: Path, relative: str, source_rows: list[dict]) -> None:
    for path in sorted(source.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
            copy_one(path, str(Path(relative) / path.relative_to(source)), source_rows)


def copy_top_level_files(source: Path, relative: str, source_rows: list[dict]) -> None:
    for path in sorted(source.iterdir()):
        if path.is_file():
            copy_one(path, str(Path(relative) / path.name), source_rows)


def assessment_summary(name: str, assessment: dict) -> dict:
    methods = {}
    for method, values in assessment["method_summaries"].items():
        methods[method] = {
            "episodes": values["episodes"],
            "valid_goals": values["valid_goals"],
            "completion_rate": values["completion_rate"],
            "mean_failure_inclusive_time_s": values["mean_failure_inclusive_time_s"],
            "minimum_conservative_clearance_m": values["minimum_conservative_clearance_m"],
            "contact_count": values["contact_count"],
            "mask_violation_count": values["mask_violation_count"],
            "failure_counts": values["failure_counts"],
        }
    comparisons = {}
    for baseline, values in assessment["primary_comparisons"].items():
        comparisons[baseline] = {
            "relative_reduction_percent": 100.0 * values["relative_reduction"],
            "two_sided_97_5_percent_interval_percent": [
                100.0 * values["bootstrap"]["lower_97_5_percent"],
                100.0 * values["bootstrap"]["upper_97_5_percent"],
            ],
            "completion_rate_difference_percentage_points": values[
                "completion_rate_difference_percentage_points"
            ],
            "positive_gate_pass": values["positive_gate_pass"],
            "gate_checks": values["gate_checks"],
        }
    return {
        "name": name,
        "blocks": assessment["blocks"],
        "episodes": assessment["episodes"],
        "all_attempt_one": assessment["attempt_one_complete"],
        "same_identity_retry_count": assessment["same_identity_retry_count"],
        "replacement_seed_count": assessment["replacement_seed_count"],
        "formal_result": assessment["formal_result"],
        "performance_improvement_demonstrated": assessment[
            "performance_improvement_demonstrated"
        ],
        "positive_comparators": assessment["positive_comparators"],
        "methods": methods,
        "comparisons": comparisons,
        "release_sha256": assessment["release_sha256"],
        "schedule_sha256": assessment["schedule_sha256"],
        "preregistration_sha256": assessment["preregistration_sha256"],
    }


def category(path: Path) -> str:
    parts = path.relative_to(BENCH).parts
    if not parts:
        return "root"
    if parts[0] == "formal":
        return f"formal_{parts[1] if len(parts) > 1 else 'unknown'}"
    if parts[0] in {"development", "pilot", "reproduction", "diagnostics", "final_transfer"}:
        return parts[0]
    return "benchmark_root"


def build_raw_evidence_index() -> tuple[int, int]:
    rows = []
    total_bytes = 0
    for path in sorted(BENCH.rglob("*")):
        if not path.is_file():
            continue
        size = path.stat().st_size
        total_bytes += size
        rows.append(
            {
                "repository_path": str(path.relative_to(ROOT)),
                "category": category(path),
                "bytes": size,
                "sha256": sha256(path),
            }
        )
    target = TARGET / "indices/full_raw_benchmark_evidence_index.csv"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["repository_path", "category", "bytes", "sha256"]
        )
        writer.writeheader()
        writer.writerows(rows)
    return len(rows), total_bytes


def first_heading(path: Path) -> str:
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            stripped = line.strip()
            if stripped.startswith("#") or stripped.startswith("\\chapter"):
                return stripped[:240]
    except OSError:
        pass
    return ""


def build_research_document_index() -> int:
    candidates = [ROOT / "AGENTS.md", ROOT / "CODEX_MASTER_PROMPT_PCT_VS_TRADITIONAL_PLANNERS.md"]
    for base in [ROOT / "docs/thesis_experiment", ROOT / "docs/future_research"]:
        if base.exists():
            candidates.extend(
                path
                for path in base.rglob("*")
                if path.is_file() and path.suffix.lower() in {".md", ".yaml", ".yml", ".tex", ".csv"}
            )
    unique = sorted(set(candidates))
    target = TARGET / "indices/repository_research_document_index.csv"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as stream:
        fields = ["repository_path", "bytes", "sha256", "first_heading"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for path in unique:
            writer.writerow(
                {
                    "repository_path": str(path.relative_to(ROOT)),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                    "first_heading": first_heading(path),
                }
            )
    return len(unique)


def write_source_index(rows: list[dict]) -> None:
    target = TARGET / "indices/portable_source_index.csv"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as stream:
        fields = ["upstream_path", "portable_path", "bytes", "sha256"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["portable_path"]))


def write_result_summary(r1: dict, r2: dict) -> None:
    historical = list(
        csv.DictReader((UPSTREAM / "CURRENT_RESULT_SUMMARY.csv").open(encoding="utf-8"))
    )
    fields = list(historical[0])
    rows = historical + [
        {
            "study": "Gazebo-PCT-R1",
            "role": "fresh preregistered confirmatory negative",
            "population": "six frozen single-robot Gazebo templates",
            "comparison": "PCT-SCOPE-RL vs Actual TEB 0.8.4",
            "episodes": "640",
            "blocks": "160",
            "primary_metric": "failure-inclusive time reduction",
            "point_percent": f"{100*r1['primary_comparisons']['actual_teb']['relative_reduction']:.6f}",
            "interval": "two-sided 97.5% CI=[-8.3569%,5.6359%]",
            "goal_counts": "153/160 vs 89/160",
            "status": "superiority not demonstrated",
        },
        {
            "study": "Gazebo-PCT-R1",
            "role": "fresh preregistered confirmatory negative",
            "population": "six frozen single-robot Gazebo templates",
            "comparison": "PCT-SCOPE-RL vs DWA",
            "episodes": "640",
            "blocks": "160",
            "primary_metric": "failure-inclusive time reduction",
            "point_percent": f"{100*r1['primary_comparisons']['dwa']['relative_reduction']:.6f}",
            "interval": "two-sided 97.5% CI=[1.0871%,11.1411%]",
            "goal_counts": "153/160 vs 80/160",
            "status": "missed preregistered 8% point gate",
        },
        {
            "study": "Gazebo-PCT-R2",
            "role": "independent fresh preregistered confirmatory negative",
            "population": "same six templates; new simulator seeds",
            "comparison": "PCT-SCOPE-RL vs Actual TEB 0.8.4",
            "episodes": "800",
            "blocks": "200",
            "primary_metric": "failure-inclusive time reduction",
            "point_percent": f"{100*r2['primary_comparisons']['actual_teb']['relative_reduction']:.6f}",
            "interval": "two-sided 97.5% CI=[-9.8083%,3.5914%]",
            "goal_counts": "180/200 vs 119/200",
            "status": "superiority not demonstrated",
        },
        {
            "study": "Gazebo-PCT-R2",
            "role": "independent fresh preregistered confirmatory negative",
            "population": "same six templates; new simulator seeds",
            "comparison": "PCT-SCOPE-RL vs DWA",
            "episodes": "800",
            "blocks": "200",
            "primary_metric": "failure-inclusive time reduction",
            "point_percent": f"{100*r2['primary_comparisons']['dwa']['relative_reduction']:.6f}",
            "interval": "two-sided 97.5% CI=[2.5521%,12.6148%]",
            "goal_counts": "180/200 vs 100/200",
            "status": "missed preregistered 8% point gate",
        },
    ]
    target = TARGET / "data/CURRENT_COMPLETE_RESULT_SUMMARY.csv"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_canonical_json(r1: dict, r2: dict) -> None:
    payload = {
        "schema_version": "1.0",
        "package_date": "2026-08-10",
        "status": "paper_writer_handoff_current",
        "source_priority": [
            "claims_contract.yaml",
            "canonical_results.json",
            "data/CURRENT_COMPLETE_RESULT_SUMMARY.csv",
            "benchmark/formal/*/assessment and episode CSV",
            "upstream_v6_06_historical for earlier V3/V5/V6 chapters",
        ],
        "historical_core_results": {
            "v3_11_actual_teb": {
                "status": "scoped_historical_positive",
                "point_percent": 12.9288,
                "one_sided_97_5_percent_lower": 12.7253,
                "scope": "three frozen single-reactive-actor Gazebo templates; shared oracle actor track",
            },
            "v3_21_actual_teb": {
                "status": "superiority_not_demonstrated",
                "point_percent": -0.1931,
                "two_sided_95_percent_interval": [-19.8668, 12.9820],
                "scope": "fresh failure-inclusive characterization",
            },
            "v4_learning": {
                "status": "meaningful_optimization_not_demonstrated",
                "point_percent": 0.0980,
                "two_sided_95_percent_interval": [0.0, 0.2941],
            },
            "v5_learning": {
                "status": "negative_practical_gate",
                "point_percent": 6.0936,
                "two_sided_95_percent_interval": [1.7867, 11.0069],
                "minimum_effect_gate_percent": 8.0,
            },
            "v6_fast_population": {
                "status": "scoped_positive",
                "point_percent": 8.402444,
                "two_sided_97_5_percent_interval": [3.484003, 13.713459],
                "pct_goals": "60/80",
                "analytic_goals": "49/80",
                "pct_mean_s": 13.115625,
                "analytic_mean_s": 14.318750,
                "scope": "four frozen sensor-level fast interaction families",
            },
            "v6_vs_v5": {
                "status": "qualified_secondary",
                "point_percent": 3.161052,
                "two_sided_95_percent_interval": [0.493580, 6.576508],
            },
            "v6_history": {"status": "not_demonstrated", "result": "exact tie"},
            "v6_gazebo_preflight": {
                "status": "historical_transfer_negative",
                "note": "three preflights failed conservative-clearance gate; later superseded only in execution availability by the new full Gazebo benchmark, not converted to a positive result",
            },
        },
        "new_gazebo_benchmark": {
            "scope": "six frozen single-robot Gazebo templates with fresh simulator seeds",
            "methods": ["pct_scope_rl", "analytic_prior", "actual_teb", "dwa"],
            "positive_gate": {
                "minimum_point_reduction_percent": 8.0,
                "two_sided_97_5_percent_interval_lower_strictly_positive": True,
                "completion_rate_difference_minimum_pp": -5.0,
                "pct_contact_not_higher": True,
                "pct_mask_violation_zero": True,
                "finite_statistics": True,
            },
            "round_1": assessment_summary("Gazebo-PCT-R1", r1),
            "round_2": assessment_summary("Gazebo-PCT-R2", r2),
            "terminal_conclusion": "Two independent formal studies did not demonstrate PCT-SCOPE-RL superiority over Actual TEB or DWA under the preregistered gates.",
            "formal_episode_total": r1["episodes"] + r2["episodes"],
            "formal_block_total": r1["blocks"] + r2["blocks"],
        },
        "real_vehicle": {
            "status": "pending_no_evidence",
            "episodes": 0,
            "claim_allowed": False,
        },
    }
    write_text("canonical_results.json", json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False))


def write_claims_contract() -> None:
    payload = {
        "schema_version": "1.0",
        "status": "current_outer_paper_claims_contract",
        "supersedes_for_gazebo_writing": "upstream_v6_06_historical/combined_claims_evidence_matrix.yaml",
        "does_not_invalidate": [
            "V3-11 scoped historical positive",
            "V3-21 fresh negative characterization",
            "V5 fast-population negative practical gate",
            "V6-02-F1 scoped fast-population positive",
        ],
        "current_gazebo_claims": [
            {
                "id": "pct_vs_actual_teb",
                "status": "not_demonstrated_after_two_independent_formal_rounds",
                "r1": "-0.9442%, two-sided 97.5% CI [-8.3569%,5.6359%]",
                "r2": "-2.7288%, two-sided 97.5% CI [-9.8083%,3.5914%]",
            },
            {
                "id": "pct_vs_dwa",
                "status": "not_demonstrated_after_two_independent_formal_rounds",
                "r1": "6.3693%, two-sided 97.5% CI [1.0871%,11.1411%]",
                "r2": "7.7754%, two-sided 97.5% CI [2.5521%,12.6148%]",
                "reason": "both point estimates were below the preregistered 8% minimum effect gate",
            },
            {
                "id": "formal_safety",
                "status": "not_demonstrated",
                "reason": "R2 PCT retained two conservative-clearance violations; no formal proof",
            },
        ],
        "allowed_claims": [
            "V6 PCT rank distillation was positive only in the frozen fast sensor-level population.",
            "The later Gazebo benchmark completed 360 fresh paired blocks and 1440 episodes over two independent formal rounds.",
            "PCT had higher valid-goal rates than TEB and DWA in both Gazebo rounds, but did not meet the preregistered time-reduction superiority gate.",
            "R2 versus DWA had a positive interval but missed the 8% point gate and is therefore a negative superiority result.",
            "Post-freeze diagnostics motivate new development-only tests of safe switching hysteresis and stop-recovery release.",
        ],
        "forbidden_claims": [
            "PCT-SCOPE-RL is superior to Actual TEB in Gazebo.",
            "PCT-SCOPE-RL is superior to DWA in Gazebo.",
            "A positive confidence interval alone overrides the frozen 8% practical-effect gate.",
            "V6 fast-simulator evidence is Gazebo or real-vehicle evidence.",
            "V3 Gazebo evidence is PCT/V6 Gazebo evidence.",
            "Formal safety, perception/OOD generalization, deployment readiness, or real-vehicle effectiveness.",
            "Temporal history benefit, because V6 and no-history were tied.",
            "Deleting failures, pooling R1/R2, or treating formal seeds as development data.",
        ],
        "real_vehicle": {"status": "pending_no_evidence", "numeric_fields_must_remain_empty": True},
    }
    write_text("claims_contract.yaml", yaml.safe_dump(payload, sort_keys=False, allow_unicode=True))


def build_readme() -> str:
    return """# 论文写作系统完整交接包：从这里开始

这是 2026-08-10 的外层权威交接包，面向另一套论文写作系统。它把历史 V3/V4/V5/V6 论文包与
最新完成的 PCT-SCOPE-RL、解析先验、Actual TEB、DWA 两轮 Gazebo 正式研究合并到同一证据入口。

## 最重要的更新

旧 V6-06 包在 2026-08-08 正确写明“V6 尚无 Gazebo 正式矩阵”。该陈述现在只具有历史意义：
2026-08-10 后续研究已经修复 Gazebo 链路、公平调优传统规划器并完成两轮全新正式矩阵。
新结果是合格阴性，不得把“后来完成 formal”误写成“后来获得优势”。

- R1：160 paired blocks / 640 episodes；未证明优于 TEB 或 DWA。
- R2：200 paired blocks / 800 episodes；仍未证明优于 TEB 或 DWA。
- R2 对 DWA 为 7.7754%，97.5% CI [2.5521%,12.6148%]，但低于冻结的 8% 点门，因此严格为阴性。
- R2 对 Actual TEB 为 -2.7288%，97.5% CI [-9.8083%,3.5914%]。

## 必读顺序

1. `00_READ_ME_FIRST.md`（本文件）
2. `claims_contract.yaml`
3. `canonical_results.json`
4. `data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`
5. `01_PROJECT_STORY_AND_PAPER_OUTLINE.md`
6. `02_METHOD_FAIRNESS_AND_EXPERIMENT_DESIGN.md`
7. `03_RESULTS_STATISTICS_AND_FAILURES.md`
8. `04_CLAIMS_LIMITATIONS_AND_NEXT_WORK.md`
9. `05_DATA_DICTIONARY.md`
10. `06_FIGURE_TABLE_CATALOG.md`
11. `07_SUPERSESSION_NOTICE.md`
12. `08_PROMPT_FOR_PAPER_WRITING_SYSTEM.md`

随后阅读 `benchmark/THESIS_RESULT_TRANSFER.md` 和 `chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex`。
历史论文主线、V3/V5/V6 数据、41 页整合稿和旧图表在 `upstream_v6_06_historical/`。

## 数据优先级

发生数字冲突时，依次以 `claims_contract.yaml`、`canonical_results.json`、新 R1/R2 assessment、
逐 episode CSV、旧历史包为准。不要用 Markdown 中的四舍五入值替代机器文件中的原始精度。

## 原始证据与体积策略

本包复制论文必需的 assessment、CSV、preregistration、release、review、配置、代码和模型摘要。
约 868 MB 的逐 episode Gazebo 日志不重复复制；`indices/full_raw_benchmark_evidence_index.csv`
给出每个原始文件的仓库路径、字节数和 SHA256，可在项目工作区逐项追溯。

本包不授权训练、仿真、调参、实车或在线 TEB 写入。另一系统只需写作、排版和声明审计。
"""


def write_narrative_files() -> None:
    write_text("00_READ_ME_FIRST.md", build_readme())
    write_text(
        "01_PROJECT_STORY_AND_PAPER_OUTLINE.md",
        """# 项目主线与推荐论文结构

## 科学主线

1. T12/V2 揭示直接学习高维 TEB 参数存在动作投影、Ackermann 耦合和执行错位。
2. V3-SCOPE 把学习动作改成有限 Ackermann 可行候选，建立候选条件预测、可达占据、硬验证、
   certified stop 和解析排序；V3-11 有限定历史阳性，V3-21 的新鲜表征未复现一般 TEB 优势。
3. V4 首次把 Masked-PPO 用于候选选择，但三轮训练及 fresh 表征仅 0.0980%，学习收益未证明。
4. V5 建立部署期纯传感器 actor、asymmetric critic、解析残差先验和硬动作掩码；正式结果
   6.0936%，虽区间为正但未达到冻结 8% 实效门。
5. V6 只加入 privileged counterfactual trajectory-rank distillation（PCT）单因素，在冻结
   四类 fast sensor-level population 上达到 8.402444% 和正 97.5% 区间；该阳性不等于 Gazebo。
6. V6 早期三次 Gazebo 迁移预检失败。2026-08-10 新研究随后修复完整闭环、公平配置 TEB/DWA，
   完成两轮独立 formal；PCT 对 TEB、DWA 的预注册优势仍未建立。

## 推荐学位论文章节

- 第1章：研究背景、Ackermann 局部导航问题、研究问题与证据边界。
- 第2章：TEB/DWA、PPO/asymmetric critic、privileged learning、shielding 与候选轨迹相关工作。
- 第3章：从 T12/V2 到 V3 的动作对齐与候选规划骨架。
- 第4章：PCT-SCOPE-RL 方法——观测、候选、解析先验、PPO、PCT 标签、硬验证和控制边界。
- 第5章：实验完整性——开发/pilot/formal 隔离、失败记账、散列冻结、公平调参和统计门。
- 第6章：V4/V5 阴性与 V6 fast-population 阳性，作为学习单因素证据。
- 第7章：最新 Gazebo 四方法公平比较，两轮 formal 均为阴性。
- 第8章：消融、失败案例、外部效度与可证伪下一步。
- 第9章：结论；fast-population 阳性和 Gazebo superiority 阴性必须同时保留。
- 实车章节：明确标记 `pending_no_evidence`，不填任何虚构数字。

## 推荐题目方向

“面向 Ackermann 移动机器人的特权候选排序强化学习与可审计安全轨迹选择研究”比“PCT 全面优于
传统规划器”更符合证据。副标题可说明包含 fast sensor-level 证据和 Gazebo 两轮公平验证。
""",
    )
    write_text(
        "02_METHOD_FAIRNESS_AND_EXPERIMENT_DESIGN.md",
        """# 方法、公平性与实验设计

## 四种方法

- PCT-SCOPE-RL：部署 actor 读取 LaserScan、里程计/局部目标、候选特征、解析先验和硬 mask，
  输出 8 个 Ackermann 可行候选之一。仿真特权状态只用于 critic/PCT 训练标签。
- 解析先验：与 PCT 共用候选生成、候选条件预测、硬验证、切换平滑、tracker 和控制边界，只用
  冻结解析得分选候选。
- Actual TEB：`teb_local_planner/TebLocalPlannerROS` 0.8.4，development 选择 v0。
- DWA：ROS DWA 本地规划器，development 选择 v0。

## 公平性

四方法共享 M2 Ackermann 模型、1.20 m/s 速度上限、0.60 m/s² 加速、0.90 m/s² 减速、15 Hz
360° LaserScan、warmup gate、目标、22 s 时限和 `/cmd_vel_navigation` 边界。传统规划器没有被
降速：TEB/DWA 分别评估多种更保守距离、膨胀、同伦和 horizon 变体，最终按六场景失效计入均时
选择完整评估中最强的 v0。全部无效到达、超时、负保守净空、接触和系统失败统一记 22 s。

PCT development 从 V6 seed9501 初始化，用 seed9701 在对齐分布训练约 40,008 步；训练 575 个
episode、571 个目标、contact=0、mask violation=0。最终 actor SHA 为
`deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8`。

## 场景与隔离

六个冻结模板为 static avoidance、single crossing、head-on yield、sequential interaction、
partially occluded 和 width change。development、pilot、R1 formal、R2 development、R2 pilot 与
R2 formal 使用互斥身份。R1 formal seeds 为 197001–197160；R2 formal seeds 为 198001–198200。

## 预注册主门

每个基线比较必须同时满足：失败计入时间点改善至少 8%；双侧 97.5% paired-block bootstrap
下界严格大于 0；完成率差不低于 -5 pp；PCT contact 不高于基线；PCT mask violation=0；统计
有限。任何一个条件失败即不得声称 superiority。

## R2 单因素

R1 阴性封存后，只在新 development 数据中把左右低速空间候选目标速度由 0.4 调到 0.6 m/s。
PPO actor、解析公式、其他候选、硬安全门、TEB、DWA 和统计定义均不变。12 个 development 配对
块显示均时改善 1.2000 s，随后用新 pilot 冻结 200-block R2；没有从 R1/R2 正式结果继续调参。
""",
    )
    write_text(
        "03_RESULTS_STATISTICS_AND_FAILURES.md",
        """# 结果、统计与失败案例

## 历史学习结果

- V4：0.0980%，95% CI [0,0.2941%]，未达到 3% 学习门。
- V5：6.0936%，95% CI [1.7867%,11.0069%]，未达到 8% 实效门。
- V6 fast population：PCT/解析 13.115625/14.318750 s、60/80 对 49/80；降低 8.402444%，
  双侧 97.5% CI [3.484003%,13.713459%]，限定阳性。
- V6/no-history 完全相同，不支持时序历史收益。

## 新 Gazebo R1

160 blocks / 640 episodes 全部 attempt=1。PCT/解析/TEB/DWA 有效目标为 153/146/89/80，均时
15.0634/16.1231/14.9225/16.0881 s。PCT 对 TEB 为 -0.9442%，97.5% CI
[-8.3569%,5.6359%]；对 DWA 为 6.3693%，CI [1.0871%,11.1411%]。无阳性比较。

## 新 Gazebo R2

200 blocks / 800 episodes 全部 attempt=1。PCT/解析/TEB/DWA 有效目标为 180/150/119/100，均时
14.8413/16.3988/14.4470/16.0925 s。PCT 对 TEB 为 -2.7288%，97.5% CI
[-9.8083%,3.5914%]；对 DWA 为 7.7754%，CI [2.5521%,12.6148%]。DWA 区间下界为正，
但点值低于冻结 8% 门，因此仍是 superiority 阴性。

## 必须保留的失败

R2 PCT 有 18 次 timeout/nonarrival 与 2 次保守净空失败；解析先验有 29 次 timeout 和 21 次
LaserScan 时间戳 TF 消息合同失败；Actual TEB 有 81 次、DWA 有 100 次负保守净空失败。所有
失败均进入 22 s 主指标。PCT R2 最小保守净空 -0.2696 m，因此不能声称形式安全。

## 分层机制证据

PCT 相对 DWA 在 partially-occluded/static-avoidance 快 8.3061/7.0338 s，但在 head-on-yield 与
single-crossing 慢 5.2666/4.6618 s。head-on 平均 24.97 次 certified stop；single-crossing
平均 14.00 次切换、15.53 次 stop；sequential 有 11 次 timeout，平均 23.24 次切换、38.67 次
stop、9.00 次恢复。这支持“保守停车/候选切换抖动”机制假设，但不是随机化因果证明。
""",
    )
    write_text(
        "04_CLAIMS_LIMITATIONS_AND_NEXT_WORK.md",
        """# 声明边界、有效性威胁与下一步

## 可以写

- PCT 蒸馏在冻结 fast sensor-level population 上相对解析先验有 8.402444% 限定阳性。
- 后续 Gazebo 研究完成两轮公平、独立、fresh-seed 正式比较，但未达到优于 TEB 或 DWA 的
  预注册综合门。
- PCT 在两轮中的有效目标率高于 TEB/DWA；这与时间 superiority 是不同 estimand。
- R2 对 DWA 的区间为正，但 7.7754% 未达到 8% 实效门，故结论仍为阴性。

## 不能写

不能声称一般 TEB/DWA 优势、形式安全、感知/OOD 或多人泛化、部署、实车、GRU history benefit，
也不能把 V3 Gazebo 阳性改写成 PCT/V6 结果。不得合并 R1/R2 重新估计，不得隐藏传统规划器或
PCT 的负保守净空失败。

## 有效性威胁

新 formal 总体仍是六个参数化模板与 simulator seeds，不是独立几何分布；actor state 只用于
事后评价，但模拟传感器与真实激光仍有 domain gap。TEB 比较在 R2 pilot 功效推算中即使 200
blocks 也略低于 80% 目标功效，因此阴性不是等效证明。保守净空是冻结评价合同，不等于物理
contact，也不等于形式化可达集证明。

## 可证伪后续

只在全新 development identities 上测试一个“安全候选切换滞回/停车恢复释放”因素，事先要求
动态均时、stop、switch 同时改善，contact=0、mask=0 且净空不低于 control；不降低任何安全门。
LaserScan 时间戳 TF-buffer/warmup 健壮性应作为独立非行为因素测试。若未来再做正式研究，必须
重新冻结、功效分析、预注册并使用全新 formal seeds；197001–197160 和 198001–198200 永久只读。
""",
    )
    write_text(
        "05_DATA_DICTIONARY.md",
        """# 数据字典

- `canonical_results.json`：全部核心历史结果和最新两轮 Gazebo 结果的机器权威总表。
- `claims_contract.yaml`：当前允许、未证明和禁止声明；写作前必须读取。
- `data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`：研究级一行一结论汇总。
- `benchmark/formal/round_*/formal_r*_assessment.json`：正式统计、门限与逐身份哈希证据。
- `benchmark/formal/round_*/formal_r*_episode_results.csv`：R1 640、R2 800 个正式 episode。
- `benchmark/formal/round_*/formal_r*_schedule.csv`：配对区块、种子、场景、方法和顺序。
- `benchmark/formal/round_*/formal_r*_preregistration.yaml`：结果访问前冻结的指标与门。
- `benchmark/formal/round_*/formal_r*_execution_release.yaml`：执行资源绑定与 SHA。
- `benchmark/final_transfer/*.csv`：方法、效应、场景和失败机制的论文表。
- `benchmark/final_transfer/*.svg`：四张可编辑新结果图。
- `benchmark/ledgers/episode_results.csv`：全研究 6,145 行 episode 总账，不等于只有正式数据。
- `benchmark/ledgers/run_manifest.csv`：运行/离线阶段总账。
- `indices/full_raw_benchmark_evidence_index.csv`：未复制的 868 MB 原始证据逐文件路径和 SHA。
- `indices/portable_source_index.csv`：包内复制品与原仓库路径的映射。
- `upstream_v6_06_historical/data/`：V3、V5、V6 fast-population 历史机器数据。

布尔 `valid_goal` 同时要求 goal、执行完整性、非负冻结保守净空和无 contact。主时间指标把所有
失败统一替换为 22 s。相对降低定义为 `1 - mean(PCT)/mean(baseline)`；正数表示 PCT 更快。
""",
    )
    write_text(
        "07_SUPERSESSION_NOTICE.md",
        """# 与旧 V6-06 交接包的关系

`upstream_v6_06_historical/` 是 2026-08-08 的完整历史论文包，内容和散列保持不变。它仍是 V3、
V5、V6 fast-population、历史 Gazebo preflight 和 41 页整合稿的权威来源。

唯一需要更新的时间性陈述是：旧包写作时“V6/PCT 没有 Gazebo 正式矩阵”；2026-08-10 后续研究
已经完成 Gazebo 正式 R1/R2。因此新论文必须新增后续研究，同时保留旧 preflight 失败。新 formal
并未建立优势，所以不得把更新写成成功迁移。`claims_contract.yaml` 对 Gazebo 相关措辞拥有最高
优先级；旧包其他冻结结果不因新研究而失效。
""",
    )
    write_text(
        "08_PROMPT_FOR_PAPER_WRITING_SYSTEM.md",
        """# 可直接发送给论文写作系统的提示词

你负责依据一个冻结证据包撰写或更新中文研究生论文。项目根目录为 `<PROJECT_ROOT>`，当前权威
交接包为 `<PROJECT_ROOT>/artifacts/paper_writer_handoff/FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810`。
学校模板初稿为 `<SCHOOL_THESIS_DRAFT>`。先复制初稿形成可回退副本，不覆盖唯一原稿。

开始前按顺序完整读取 `00_READ_ME_FIRST.md`、`claims_contract.yaml`、`canonical_results.json`、
`data/CURRENT_COMPLETE_RESULT_SUMMARY.csv`、01–07 号说明、`benchmark/THESIS_RESULT_TRANSFER.md`、
`chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex`，再读 `upstream_v6_06_historical/README_FIRST.md`
及其整合技术稿、数据字典和 claims。若旧包与外层包的 Gazebo 陈述冲突，以外层包为准。

论文必须同时呈现：V4/V5 学习阴性；V6 fast-population 限定阳性；旧 Gazebo preflight 阴性；
新 Gazebo R1/R2 均未达到 TEB/DWA superiority 门。不要为了故事连贯而删除任何反证。

必须使用以下精确结论：R1 PCT vs TEB -0.9442% [−8.3569%,5.6359%]，vs DWA 6.3693%
[1.0871%,11.1411%]；R2 vs TEB -2.7288% [−9.8083%,3.5914%]，vs DWA 7.7754%
[2.5521%,12.6148%]。所有区间为双侧 97.5%。R2/DWA 因点值低于冻结 8% 门而判阴性。

完成后交付：更新后的模板源和 PDF、逐章 changelog、claims audit、图表/表格来源清单、所有引用
未决问题、实车待填字段。实车章节保持 `pending_no_evidence`。禁止声称 PCT 优于 TEB/DWA、形式
安全、一般 OOD/感知/多人泛化、部署或实车有效，也禁止把 V3 结果写成 V6/PCT 结果。
""",
    )


def write_figure_catalog() -> None:
    rows = [
        ("G1", "benchmark/final_transfer/formal_effects_97_5_ci.svg", "两轮 Gazebo 正式效应及 97.5% CI", "主结果"),
        ("G2", "benchmark/final_transfer/formal_r2_method_outcomes.svg", "R2 四方法失败计入均时与完成率", "主结果"),
        ("G3", "benchmark/final_transfer/formal_r2_scene_strata.svg", "R2 六场景分层均时", "结果/讨论"),
        ("G4", "benchmark/final_transfer/r2_development_ablation.svg", "R2 单因素候选覆盖消融", "消融"),
    ]
    target = TARGET / "data/NEW_FIGURE_CATALOG.csv"
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["figure_id", "portable_path", "suggested_caption_zh", "recommended_section"])
        writer.writerows(rows)
    write_text(
        "06_FIGURE_TABLE_CATALOG.md",
        """# 图表目录与推荐用法

新 Gazebo 研究提供四张 SVG：效应区间图应放在主结果首位；方法均时/完成率图解释主估计量；
六场景图用于展示收益与损失抵消；development 消融只能标为开发证据。四张机器表分别是
`formal_round_summary.csv`、`formal_effect_summary.csv`、`formal_r2_scene_strata.csv` 和
`formal_r2_failure_diagnostics.csv`。

历史 V3/V5/V6 的 11 张图、booktabs 表与 PDF/SVG/PNG 版本位于
`upstream_v6_06_historical/figures/` 和 `tables/`。不要把历史 V6 Gazebo preflight 图当作新 formal
结果图；建议相邻呈现，说明前者是迁移准入失败，后者是后来修复链路后的正式阴性比较。
""",
    )


def write_latex() -> None:
    write_text(
        "chapters/CH11_PCT_TEB_DWA_GAZEBO_BENCHMARK.tex",
        r"""\chapter{PCT-SCOPE-RL 与传统局部规划器的 Gazebo 公平比较}
\label{chap:pct-gazebo-benchmark}

\section{研究问题与公平性合同}
本章检验 PCT-SCOPE-RL 在修复后的 Gazebo 闭环中能否相对 Actual TEB 0.8.4 或 DWA 达到预注册
的实际性能优势。四方法共享 Autolabor M2 Ackermann 模型、$1.20\,\mathrm{m/s}$ 速度上限、
$0.60\,\mathrm{m/s^2}$ 加速、$0.90\,\mathrm{m/s^2}$ 减速、15 Hz 360 度激光、同一 warmup gate、
目标、22 s 时限和 \texttt{/cmd\_vel\_navigation} 边界。TEB 与 DWA 均在互斥开发集上评估多种
保守配置，最终采用六场景失效计入均时最强的 v0；未通过降低传统规划器性能制造优势。

主估计量为 $1-\bar T_{\mathrm{PCT}}/\bar T_b$。阳性要求点改善不少于 8\%、双侧 97.5\%
paired-block bootstrap 下界严格大于零、完成率差不低于 $-5$ pp、PCT contact 不高于基线、
mask violation 为零且统计有限。所有 timeout、未到达、负保守净空、contact 和执行完整性失败
统一以 22 s 计入。

\section{两轮独立正式研究}
R1 使用 fresh seeds 197001--197160，形成 160 个配对块、640 个 episode。PCT、解析先验、
Actual TEB 和 DWA 的有效目标分别为 153、146、89 和 80，失效计入均时分别为
15.0634、16.1231、14.9225 和 16.0881 s。PCT 相对 TEB 的降幅为 $-0.9442\%$，双侧
97.5\% 区间为 $[-8.3569\%,5.6359\%]$；相对 DWA 为 $6.3693\%$，区间
$[1.0871\%,11.1411\%]$，未达到 8\% 门。

R1 封存后仅在新开发数据中把左右低速空间候选从 0.4 改为 0.6 m/s。配对消融显示均时改善
1.2000 s，且没有降低硬安全阈值。新 pilot 按冻结功效规则选择 200 blocks，随后 R2 使用
fresh seeds 198001--198200 完成 800 个 episode。PCT、解析、TEB、DWA 的有效目标为
180、150、119、100，均时为 14.8413、16.3988、14.4470、16.0925 s。PCT 相对 TEB 为
$-2.7288\%$，97.5\% 区间 $[-9.8083\%,3.5914\%]$；相对 DWA 为 $7.7754\%$，区间
$[2.5521\%,12.6148\%]$。尽管 DWA 比较的区间下界为正，点值仍低于冻结 8\% 门，故严格
判为阴性。

\begin{figure}[htbp]
  \centering
  \includegraphics[width=.90\textwidth]{figures/pct_gazebo/formal_effects_97_5_ci.pdf}
  \caption{两轮正式研究中 PCT 相对 Actual TEB 与 DWA 的失败计入时间降幅及双侧 97.5\% 区间。}
  \label{fig:pct-gazebo-effects}
\end{figure}

\section{失败、分层与机制诊断}
R2 PCT 保留 18 次超时和 2 次保守净空失败；解析先验保留 29 次超时和 21 次 LaserScan 时间戳
TF 消息合同失败；TEB/DWA 分别保留 81/100 次负保守净空失败。PCT 相对 DWA 在部分遮挡与
静态绕障中分别快 8.3061/7.0338 s，但在 head-on 与 single-crossing 中慢 5.2666/4.6618 s。
后两层的 certified stop 和候选切换显著较多；sequential 层有 11 次 PCT timeout。这与过度停车、
切换抖动和恢复释放不足一致，但不是独立随机化因果证明。

\section{结论与边界}
两轮完整、独立、fresh-seed Gazebo 正式研究均未按预注册综合门建立 PCT-SCOPE-RL 优于 Actual
TEB 或 DWA。该阴性不推翻 V6 在冻结 fast sensor-level population 上相对解析先验的 8.4024\%
限定阳性，但阻止将其外推为 Gazebo、一般传统规划器、形式安全、部署或实车优势。未来只能在
全新 development 数据上检验安全切换滞回/停车恢复单因素，并在重新冻结和预注册后使用全新
formal seeds。
""",
    )
    write_text(
        "tables/NEW_GAZEBO_READY_TO_PASTE_TABLES.tex",
        r"""% Requires booktabs.
\begin{table}[htbp]
\centering
\caption{Gazebo 正式 R2 四方法结果}
\begin{tabular}{lrrrrr}
\toprule
方法 & 有效目标 & 完成率/\% & 失败计入均时/s & 最小保守净空/m & contact/mask\\
\midrule
PCT-SCOPE-RL & 180/200 & 90.0 & 14.8413 & -0.2696 & 0/0\\
解析先验 & 150/200 & 75.0 & 16.3988 & 0.1645 & 0/0\\
Actual TEB & 119/200 & 59.5 & 14.4470 & -1.1259 & 0/0\\
DWA & 100/200 & 50.0 & 16.0925 & -0.3702 & 0/0\\
\bottomrule
\end{tabular}
\end{table}

\begin{table}[htbp]
\centering
\caption{两轮预注册主比较}
\begin{tabular}{llrrl}
\toprule
轮次 & 比较 & 点改善/\% & 双侧 97.5\% CI/\% & 结论\\
\midrule
R1 & PCT vs TEB & -0.9442 & [-8.3569,5.6359] & 未证明\\
R1 & PCT vs DWA & 6.3693 & [1.0871,11.1411] & 未达 8\% 门\\
R2 & PCT vs TEB & -2.7288 & [-9.8083,3.5914] & 未证明\\
R2 & PCT vs DWA & 7.7754 & [2.5521,12.6148] & 未达 8\% 门\\
\bottomrule
\end{tabular}
\end{table}
""",
    )


def copy_sources(source_rows: list[dict]) -> None:
    selected = {
        ROOT / "CODEX_MASTER_PROMPT_PCT_VS_TRADITIONAL_PLANNERS.md": "project_context/CODEX_MASTER_PROMPT_PCT_VS_TRADITIONAL_PLANNERS.md",
        ROOT / "AGENTS.md": "project_context/AGENTS.md",
        ROOT / "THESIS_RESULT_TRANSFER.md": "benchmark/THESIS_RESULT_TRANSFER.md",
        BENCH / "README_FIRST.md": "benchmark/README_FIRST.md",
        BENCH / "environment_manifest.yaml": "benchmark/environment_manifest.yaml",
        BENCH / "evidence_inventory.csv": "benchmark/evidence_inventory.csv",
        BENCH / "failure_taxonomy.csv": "benchmark/failure_taxonomy.csv",
        BENCH / "key_sha256.csv": "benchmark/key_sha256.csv",
        BENCH / "method_config_hashes.yaml": "benchmark/method_config_hashes.yaml",
        BENCH / "p00_initial_audit_report.json": "benchmark/p00_initial_audit_report.json",
        BENCH / "p02_gazebo_admission_report.json": "benchmark/p02_gazebo_admission_report.json",
        BENCH / "p03_static_candidate_repair_report.json": "benchmark/p03_static_candidate_repair_report.json",
        BENCH / "p04_development_checkpoint_report.json": "benchmark/p04_development_checkpoint_report.json",
        BENCH / "p05_development_freeze_report.json": "benchmark/p05_development_freeze_report.json",
        BENCH / "progress.md": "benchmark/ledgers/progress.md",
        BENCH / "decision_log.md": "benchmark/ledgers/decision_log.md",
        BENCH / "run_manifest.csv": "benchmark/ledgers/run_manifest.csv",
        BENCH / "episode_results.csv": "benchmark/ledgers/episode_results.csv",
        BENCH / "integrity_report.json": "benchmark/ledgers/integrity_report.json",
        BENCH / "statistical_report.md": "benchmark/ledgers/statistical_report.md",
        BENCH / "claims_evidence_matrix.yaml": "benchmark/ledgers/claims_evidence_matrix.yaml",
    }
    for source, relative in selected.items():
        copy_one(source, relative, source_rows)

    copy_tree(BENCH / "final_transfer", "benchmark/final_transfer", source_rows)
    copy_tree(BENCH / "reproduction/v6_02_f1_recomputed_20260810", "benchmark/reproduction/v6_02_f1_recomputed_20260810", source_rows)
    copy_top_level_files(BENCH / "development/r2_candidate_coverage_d0", "benchmark/development/r2_candidate_coverage_d0", source_rows)
    copy_top_level_files(BENCH / "pilot/pilot_r1", "benchmark/pilot/pilot_r1", source_rows)
    copy_top_level_files(BENCH / "pilot/pilot_r2", "benchmark/pilot/pilot_r2", source_rows)
    for round_name in ["round_1", "round_2"]:
        source = BENCH / "formal" / round_name
        for path in sorted(source.iterdir()):
            if path.is_file() and path.suffix != ".tar":
                copy_one(path, str(Path("benchmark/formal") / round_name / path.name), source_rows)

    training = BENCH / "development/training/pct_benchmark_r1_seed9701"
    for name in ["actor_final.pt", "training_summary.json", "training_episodes.jsonl", "training_updates.jsonl"]:
        copy_one(training / name, f"benchmark/model/{name}", source_rows)

    copy_tree(ROOT / "scripts/gazebo_pct_teb_dwa_benchmark", "reproducibility/scripts", source_rows)
    for path in [
        ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
        ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py",
        ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml",
        ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_r2_development.yaml",
        ROOT / "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_episode.launch",
        ROOT / "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_r2_episode.launch",
    ]:
        copy_one(path, f"reproducibility/runtime/{path.name}", source_rows)
    for path in sorted((ROOT / "src/simulation/m2_gazebo/config").glob("benchmark_*.yaml")):
        copy_one(path, f"reproducibility/configs/{path.name}", source_rows)

    context_docs = [
        "docs/thesis_experiment/DEVELOPMENT_STATUS.md",
        "docs/thesis_experiment/CURRENT_V3_11_ACTUAL_TEB_FORMAL_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V3_21_FOUR_METHOD_CHARACTERIZATION_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V3_22_THESIS_EXPORT_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V4_RL_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V5_01_SENSOR_BACKEND_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md",
        "docs/thesis_experiment/CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md",
        "docs/thesis_experiment/V3_SYSTEM_GUIDE.md",
        "docs/thesis_experiment/V4_SYSTEM_GUIDE.md",
        "docs/thesis_experiment/V5_E2E_SYSTEM_GUIDE.md",
        "docs/thesis_experiment/thesis_draft/CURRENT_THESIS_INTEGRATION_GUIDE.md",
        "docs/thesis_experiment/thesis_draft/CH09_PCT_SCOPE_RL_INTEGRATED.tex",
        "docs/thesis_experiment/thesis_draft/FAM_TEB_V6_PCT_RL_SUPPLEMENT.tex",
    ]
    for relative in context_docs:
        path = ROOT / relative
        copy_one(path, str(Path("project_context") / path.name), source_rows)

    copy_tree(UPSTREAM, "upstream_v6_06_historical", source_rows)


def write_manifest_and_integrity(raw_files: int, raw_bytes: int, doc_count: int, source_rows: list[dict]) -> None:
    manifest_path = TARGET / "content_manifest.sha256"
    entries = []
    for path in sorted(TARGET.rglob("*")):
        if path.is_file() and path.name not in {"content_manifest.sha256", "handoff_integrity.json"}:
            entries.append(f"{sha256(path)}  {path.relative_to(TARGET)}")
    manifest_path.write_text("\n".join(entries) + "\n", encoding="utf-8")
    integrity = {
        "schema_version": "1.0",
        "status": "paper_writer_handoff_integrity_pass",
        "package_path": str(TARGET.relative_to(ROOT)),
        "content_manifest_sha256": sha256(manifest_path),
        "manifest_entries": len(entries),
        "portable_copied_sources": len(source_rows),
        "full_raw_benchmark_files_indexed": raw_files,
        "full_raw_benchmark_bytes_indexed": raw_bytes,
        "research_documents_indexed": doc_count,
        "formal_rounds": 2,
        "formal_blocks": 360,
        "formal_episodes": 1440,
        "all_formal_attempt_one": True,
        "same_identity_retry_count": 0,
        "replacement_seed_count": 0,
        "current_conclusion": "superiority_over_actual_teb_or_dwa_not_demonstrated_after_two_independent_formal_rounds",
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "stable_workspace_modified": False,
    }
    write_text("handoff_integrity.json", json.dumps(integrity, indent=2, sort_keys=True))


def main() -> None:
    if TARGET.exists():
        raise RuntimeError(f"HANDOFF_TARGET_ALREADY_EXISTS:{TARGET}")
    if not UPSTREAM.is_dir():
        raise RuntimeError("UPSTREAM_V6_06_PACKAGE_MISSING")
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    TARGET.mkdir(parents=True)

    r1 = json.loads(R1_PATH.read_text(encoding="utf-8"))
    r2 = json.loads(R2_PATH.read_text(encoding="utf-8"))
    if not (
        r1["blocks"] == 160
        and r1["episodes"] == 640
        and r2["blocks"] == 200
        and r2["episodes"] == 800
        and r1["performance_improvement_demonstrated"] is False
        and r2["performance_improvement_demonstrated"] is False
        and not r1["positive_comparators"]
        and not r2["positive_comparators"]
    ):
        raise RuntimeError("CANONICAL_FORMAL_RESULT_MISMATCH")

    source_rows: list[dict] = []
    write_narrative_files()
    write_claims_contract()
    write_canonical_json(r1, r2)
    write_result_summary(r1, r2)
    write_figure_catalog()
    write_latex()
    copy_sources(source_rows)
    write_source_index(source_rows)
    raw_files, raw_bytes = build_raw_evidence_index()
    doc_count = build_research_document_index()
    write_manifest_and_integrity(raw_files, raw_bytes, doc_count, source_rows)
    print(
        json.dumps(
            {
                "target": str(TARGET.relative_to(ROOT)),
                "files": sum(path.is_file() for path in TARGET.rglob("*")),
                "bytes": sum(path.stat().st_size for path in TARGET.rglob("*") if path.is_file()),
                "raw_evidence_files_indexed": raw_files,
                "raw_evidence_bytes_indexed": raw_bytes,
                "content_manifest_sha256": sha256(TARGET / "content_manifest.sha256"),
                "integrity_sha256": sha256(TARGET / "handoff_integrity.json"),
            },
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
