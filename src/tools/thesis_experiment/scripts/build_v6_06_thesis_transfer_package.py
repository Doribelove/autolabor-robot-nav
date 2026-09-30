#!/usr/bin/env python3
"""Build a portable, evidence-bound V3+V6 thesis update package."""

import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package"
BASE_TEX = W / "docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex"
CH9 = W / "docs/thesis_experiment/thesis_draft/CH09_PCT_SCOPE_RL_INTEGRATED.tex"
CH10 = W / "docs/thesis_experiment/thesis_draft/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex"
GUIDE = W / "docs/thesis_experiment/thesis_draft/CURRENT_THESIS_INTEGRATION_GUIDE.md"
PROMPT = W / "docs/thesis_experiment/thesis_draft/CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md"
CONTRACT = W / "src/application/v5_e2e_scope_rl/config/v6_06_p1_r1_thesis_transfer_contract.yaml"
V3_MATERIALS = W / "artifacts/v3/thesis/v3_22_p1_paper_materials"
V3_PAPER = W / "artifacts/v3/paper/v3_22_p2_thesis_export"
V6_PAPER = W / "artifacts/v6/paper/v6_05_p1_thesis_export"
V5_FORMAL = W / "artifacts/v5/formal/v5_03_f1_fast_formal"
V6_FORMAL = W / "artifacts/v6/formal/v6_02_f1_fresh_formal"
V6_VALIDATION = W / "artifacts/v6/training/v6_01_r1_validation/selection_report.json"
OUT_TEX = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.tex"
OUT_PDF = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf"
OUT_LOG = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.log"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def replace_between(text, start, end, replacement):
    if text.count(start) != 1 or text.count(end) < 1:
        raise RuntimeError("non-unique transformation markers: {} / {}".format(start, end))
    left = text.index(start)
    right = text.index(end, left + len(start))
    return text[:left] + replacement.rstrip() + "\n\n" + text[right:]


EVIDENCE_SCOPE = r"""\chapter*{证据范围声明}
\addcontentsline{toc}{chapter}{证据范围声明}
本文档是对冻结 V3 技术稿的纯离线派生更新，整合了后续 V5/V6 强化学习证据；它不改写任何
历史合同、回执、种子或正式统计。V3-11 的历史阳性只覆盖三个冻结单反应式 actor Gazebo 模板、
固定 TebLocalPlannerROS 0.8.4 和共享 oracle actor track；新鲜 V3-21 失效计入研究没有复现
相对 Actual TEB 的时间优势，两项结果分别报告且不合并。

V6 的正式阳性来自四类冻结 sensor-level fast population：80 个 paired blocks、320 个 episodes
中，PCT-SCOPE-RL 相对解析先验把失效计入均时降低 8.4024\%，双侧 97.5\% 区间为
[3.4840\%,13.7135\%]。该结果不能外推为 Gazebo、Actual TEB 或部署阳性；三次 Gazebo
迁移预检均未通过冻结保守净空门，资格与正式矩阵没有运行。V6 与 no-history 消融完全相同，
不支持历史记忆收益。

本文没有实车运动、CAN/串口访问、在线 TEB 参数写入、真实人群、感知/OOD、形式化安全或部署
证据。实车章节仅为待补结构，所有数值保持为空。"""


ABSTRACT_ZH = r"""\chapter*{摘要}
\addcontentsline{toc}{chapter}{摘要}
动态环境中的 Ackermann 局部导航需要同时处理车辆运动学、他者对自车动作的条件响应、可回退
安全约束以及真实传感器信息不完备。早期 Residual SAC 研究表明，高维 TEB 参数动作在执行前
平均有 67.3\% 被可行性投影，导致 commanded action 与 executed action 存在结构性错位。
为此，本文首先提出 V3-SCOPE，把学习或规划动作改为预生成的 Ackermann 可行空间轨迹，并依次
执行候选条件交互预测、校准可达占据、硬安全验证和解析排序。V3-11 在三个冻结单 actor Gazebo
模板中取得相对 actual TebLocalPlannerROS 0.8.4 的限定历史阳性，但后续 V3-21 失效计入式
四方法研究没有复现普遍 Actual TEB 时间优势，说明解析安全候选机制有效但外部稳定性有限。

在此基础上，本文提出 PCT-SCOPE-RL。部署 actor 仅编码六帧 LaserScan、里程计、局部目标、
候选描述、解析先验和硬安全掩码，并输出 8 个 Ackermann 候选轨迹 ID；仿真特权状态只进入
Asymmetric PPO critic 和训练期 privileged counterfactual trajectory-rank distillation
（PCT）目标。解析先验与学习残差共同评分，保守优势门只允许具有足够学习优势的候选覆盖解析
动作，硬 verifier 始终掌握否决权并保留 certified-stop。

V5 仅使用特权 critic，在 60 个 fresh blocks 上得到 6.0936\% 降幅但未达到预注册 8\% 门。
V6 只增加 PCT 单因素，四个身份各训练 120,000 新环境步。在 80 个 fresh paired blocks、320 个
episodes 的正式研究中，V6/解析先验目标数为 60/80 与 49/80，失效计入均时为 13.115625 s 与
14.318750 s；主降幅为 8.402444\%，双侧 97.5\% bootstrap 区间为
[3.484003\%,13.713459\%]，全部任务、接触和掩码门通过。相对冻结 V5 的附加降幅为
3.161052\%，95\% 区间为 [0.493580\%,6.576508\%]。主要收益集中在部分遮挡族；窄走廊四方法
均失败。三次 Gazebo 迁移预检均未通过保守净空门，因此当前结论只支持冻结传感器级快速总体，
不支持 Gazebo、一般 TEB 排名、形式化安全、感知/OOD、部署或实车有效性。

\noindent\textbf{关键词：} Ackermann 移动机器人；局部导航；候选条件预测；强化学习；
仿真特权信息；Asymmetric PPO；知识蒸馏；硬安全屏蔽"""


ABSTRACT_EN = r"""\chapter*{Abstract}
\addcontentsline{toc}{chapter}{Abstract}
This thesis studies action-aligned local navigation for Ackermann mobile robots under dynamic
interaction. Earlier residual reinforcement learning over high-dimensional TEB parameters
exhibited a 67.3\% mean action-projection rate, motivating a shift from parameter actions to
pre-generated feasible trajectories. V3-SCOPE established candidate-conditioned interaction
prediction, calibrated reachable occupancy, hard verification, analytic ranking, and a
certified-stop fallback. Its historical V3-11 study showed a scoped benefit over actual
TebLocalPlannerROS 0.8.4 in three frozen single-actor Gazebo templates, whereas a fresh
failure-inclusive V3-21 characterization did not replicate general time superiority.

Building on the candidate-level safety architecture, this thesis proposes PCT-SCOPE-RL. The
deployment actor observes only six LaserScan frames, odometry, a local goal, candidate features,
analytic-prior scores, and a hard action mask. Simulation privilege is restricted to an
asymmetric PPO critic and a training-only privileged counterfactual trajectory-rank distillation
(PCT) target. The learned actor predicts residual candidate scores over an analytic prior; a
conservative advantage gate controls overrides, while the hard verifier retains veto authority.

V5, which used privileged critic information without PCT, reduced the fresh-population restricted
mean by 6.0936\% but missed the preregistered 8\% practical gate. V6 changed only the PCT factor and
trained four identities for 120,000 new environment steps each. In 80 fresh paired blocks and 320
episodes, V6 and the analytic prior reached 60/80 and 49/80 goals, with failure-inclusive means of
13.115625 and 14.318750 s. The primary reduction was 8.402444\%, with a two-sided 97.5\% bootstrap
interval of [3.484003\%,13.713459\%]; all frozen task, contact, and mask gates passed. The
secondary reduction versus frozen V5 was 3.161052\% (95\% CI [0.493580\%,6.576508\%]). The gain was
concentrated in partial occlusion, while all compared methods failed narrow-corridor yielding.
Three subsequent Gazebo transfer preflights failed the frozen conservative-clearance gate.
Consequently, the positive claim is restricted to the frozen sensor-level fast population; no
Gazebo, general TEB, formal-safety, perception/OOD, deployment, or real-vehicle claim is made."""


RESEARCH_QUESTIONS = r"""\section{研究问题}
本文围绕以下层次展开：
\begin{enumerate}
  \item 如何使规划或学习动作与 Ackermann 底盘实际执行轨迹在语义上对齐？
  \item 候选条件预测、校准可达占据和硬验证能否形成可解释、可回退的安全候选骨架？
  \item 该解析候选架构在冻结 Gazebo 总体中相对 actual TebLocalPlannerROS 有何收益和边界？
  \item 能否利用仿真特权信息训练一个部署期纯传感器的候选选择强化学习策略，并在硬安全门和
  解析先验约束下取得超过预注册实际效应门的收益？
  \item 学习收益能否迁移到 Gazebo 传感器闭环；若不能，迁移失败揭示了什么外部有效性限制？
\end{enumerate}"""


CONTRIBUTIONS = r"""\section{主要贡献}
\begin{enumerate}
  \item 提出 action-aligned 的 Ackermann 候选级局部导航架构，以有限可执行轨迹替代会被大量
  投影的高维 TEB 参数动作，并保留候选条件预测、硬安全验证和 certified-stop。
  \item 提出 PCT-SCOPE-RL：用 Asymmetric PPO 和特权反事实候选排序蒸馏训练同一传感器 actor，
  部署时删除全部特权模块，实现真实可得传感器到候选轨迹 ID 的端到端评价。
  \item 构建解析先验、学习残差、保守优势门和硬安全掩码的权限分离，使学习器优化性能、解析模块
  提供先验、verifier 保留否决权。
  \item 建立 V5 阴性到 V6 单因素阳性的 fresh-seed 证据链；V6 在 80 个 paired blocks 上相对
  解析先验降低失效计入均时 8.4024\%，双侧 97.5\% 区间为 [3.4840\%,13.7135\%]。
  \item 完整报告 V3 新鲜未复现、V6 no-history 无差异、窄走廊失败和 Gazebo 迁移失败，形成
  可复核的声明边界与后续实车实验接口。
\end{enumerate}"""


CONCLUSION = r"""\chapter{结论与展望}
本文形成了从高维参数动作诊断、解析安全候选规划到仿真特权强化学习的完整研究链。T12/V2
揭示 commanded parameter 与 executed action 的投影错位；V3-SCOPE 以 Ackermann 可行轨迹、
候选条件预测、校准占据、硬验证和解析排序建立 action-aligned 骨架。V3-11 提供了限定单 actor
Gazebo 总体中的历史 actual-TEB 阳性证据，而 V3-21 在保留 timeout 的新鲜四方法研究中没有复现
普遍优势，说明候选条件机制存在，但少量保守停止足以消除 failure-inclusive 时间收益。

在 V3 骨架基础上，PCT-SCOPE-RL 把强化学习权限限制为安全候选集合中的轨迹 ID 选择。actor
部署时只读取真实可得传感器，仿真特权只用于 asymmetric critic 和 PCT 候选排序目标；解析先验、
优势门和硬屏蔽限制学习覆盖。V5 未达到 8\% 实效门后，V6 只增加 PCT 单因素，并在 80 个 fresh
paired blocks 上得到 8.4024\% 主效应和严格为正的双侧 97.5\% 区间，且相对 V5 仍有 3.1611\%
附加收益。这构成本文的核心强化学习阳性结论。

结论必须保持范围限定：收益集中在部分遮挡总体，窄走廊未解决，no-history 消融完全相同；三次
Gazebo 传感器迁移预检又均未通过保守净空门。因此当前工作证明的是冻结传感器级快速总体内的
特权候选排序蒸馏收益，而不是一般动态导航、形式化安全、Actual TEB 排名、部署或实车有效性。
后续应首先建立闭环交互一致的传感器占据与验证链，再通过 shadow、低速资格和预注册配对矩阵补充
实车外部有效性；在真实数据产生之前，本文实车章节保持为空。"""


BIBLIOGRAPHY = r"""\begin{thebibliography}{9}
\bibitem{schulman2017ppo} J. Schulman et al., ``Proximal Policy Optimization Algorithms,'' arXiv:1707.06347, 2017.
\bibitem{pinto2018asymmetric} L. Pinto et al., ``Asymmetric Actor Critic for Image-Based Robot Learning,'' RSS, 2018.
\bibitem{chen2020cheating} D. Chen et al., ``Learning by Cheating,'' CoRL, PMLR 100, 2020.
\bibitem{alshiekh2018shielding} M. Alshiekh et al., ``Safe Reinforcement Learning via Shielding,'' AAAI, 2018.
\bibitem{carr2022partialshield} S. Carr et al., ``Safe Reinforcement Learning via Shielding under Partial Observability,'' 2022.
\bibitem{rosmann2017teb} C. Rösmann, F. Hoffmann, and T. Bertram, ``Integrated Online Trajectory Planning and Optimization in Distinctive Topologies,'' Robotics and Autonomous Systems, 2017.
\bibitem{long2017sensorrl} P. Long et al., ``Towards Optimally Decentralized Multi-Robot Collision Avoidance via Deep Reinforcement Learning,'' 2017.
\end{thebibliography}"""


def build_tex():
    text = BASE_TEX.read_text(encoding="utf-8")
    text = text.replace(
        "pdftitle={面向Ackermann移动机器人的候选条件交互预测与安全空间轨迹选择}",
        "pdftitle={面向Ackermann移动机器人的候选条件安全规划与仿真特权强化学习研究}",
    )
    text = text.replace("FAM-TEB / V3-SCOPE 论文技术稿", "FAM-TEB / V3-SCOPE / PCT-SCOPE-RL 技术稿")
    text = replace_between(
        text, "\\title{", "\\author{",
        r"""\title{\Huge 面向 Ackermann 移动机器人的\\候选条件安全规划与\\仿真特权强化学习研究\\[0.8em]\Large V3-SCOPE 与 PCT-SCOPE-RL 可复核仿真研究}""",
    )
    text = text.replace(r"\fancyhead[R]{\leftmark}", r"\fancyhead[R]{论文技术稿}")
    text = text.replace("\\date{2026 年 8 月 6 日}", "\\date{2026 年 8 月 8 日}")
    text = text.replace(
        r"\special{pdf:trailerid [<56333232503154484553495350444631><56333232503154484553495350444631>]}",
        r"\special{pdf:trailerid [<56363036544845534953445241465431><56363036544845534953445241465431>]}",
    )
    text = replace_between(text, r"\chapter*{证据范围声明}", r"\chapter*{摘要}", EVIDENCE_SCOPE)
    text = replace_between(text, r"\chapter*{摘要}", r"\chapter*{Abstract}", ABSTRACT_ZH)
    text = replace_between(text, r"\chapter*{Abstract}", r"\tableofcontents", ABSTRACT_EN)
    text = replace_between(text, r"\section{研究问题}", r"\section{主要贡献}", RESEARCH_QUESTIONS)
    text = replace_between(text, r"\section{主要贡献}", r"\chapter{系统目标与总体架构}", CONTRIBUTIONS)
    text = text.replace(
        "../../../artifacts/v3/thesis/v3_22_p1_paper_materials/tables/", "tables/v3/"
    ).replace(
        "../../../artifacts/v3/thesis/v3_22_p1_paper_materials/figures/", "figures/v3/"
    )
    text = text.replace(
        r"\chapter{讨论}",
        r"\input{chapters/CH09_PCT_SCOPE_RL_INTEGRATED.tex}" + "\n\n" +
        r"\input{chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex}" + "\n\n" +
        r"\chapter{讨论}" + "\n" + r"\section{V3 安全骨架与 V6 学习层的关系}" + "\n"
        "V3 与 V6 共享候选级动作、硬否决权和解析回退的研究范式，但它们使用独立实现、种子与证据。"
        "V3 解决候选条件预测和可行空间选择，V6 解决仿真特权如何通过 PCT 改善部署期纯传感器候选评价。"
        "两者构成递进架构，不允许把 V3 的 Gazebo 阳性移植为 V6 的学习或感知证据。\n\n"
        r"\section{内部阳性与迁移阴性为何可以并存}" + "\n"
        "V6-02-F1 回答冻结快速交互总体中的学习收益，V6-03/04 则检查扫描、跟踪、占据和跟踪器组成的"
        "Gazebo 闭环迁移链。前者通过说明 PCT 在该总体内有效；后者失败说明当前感知--verifier 接口"
        "不能承载同样的外部结论。二者不是统计矛盾，而是内部有效性与外部有效性的不同层级。\n",
    )
    text = text.replace(
        "  \\item 训练步数为 0，本文不评价学习方法的最终潜力，只说明当前日志不支持可识别训练。",
        "  \\item V3 的训练步数为 0，不能声称 V3 学习收益；V6 另建在线交互训练并取得限定快速总体阳性，但 Gazebo 迁移失败。",
    )
    text = replace_between(text, r"\chapter{结论与展望}", r"\appendix", CONCLUSION)
    evidence_anchor = (
        r"V3-22 paper materials review & \hashbreak{88547275a6627027}{517aa55d2128469b}"
        r"{b30de1721fc162e9}{2ab9d6d787be0959}\\"
    )
    evidence_extra = evidence_anchor + "\n" + (
        r"V5 formal assessment & \hashbreak{683d2823db46179c}{44633c70210e47ec}"
        r"{7cf0df3a6ed3e8e5}{ec8bd70a0da59e25}\\" "\n"
        r"V6 formal assessment & \hashbreak{08a0ab06343cad51}{ee206ba502d2a115}"
        r"{c6fd707d58620f9b}{b4f519a71ec95c1b}\\" "\n"
        r"V6 paper review & \hashbreak{ff4b6423d8fb610f}{e29486ab059c86a1}"
        r"{e86f284055c8b520}{cf9e38cf4e470b41}\\"
    )
    if evidence_anchor not in text:
        raise RuntimeError("V3 evidence table anchor missing")
    text = text.replace(evidence_anchor, evidence_extra)
    claim_anchor = "感知/OOD 泛化 & forbidden & 未运行感知退化或 OOD 正式研究。\\\\"
    claim_extra = (
        "V5 特权 critic 学习收益 & not demonstrated & 6.0936\\% 未达到预注册 8\\% 实效门。\\\\\n"
        "V6 PCT 快速总体收益 & qualified & 8.4024\\%，双侧 97.5\\% CI [3.4840\\%,13.7135\\%]；仅限四类冻结 sensor-level fast population。\\\\\n"
        "V6 Gazebo/Actual TEB 收益 & not demonstrated & 三次预检净空门失败，资格和正式矩阵均未运行。\\\\\n"
        "时序历史收益 & not demonstrated & V6 与 no-history 在 80 个正式块上完全相同。\\\\\n"
        + claim_anchor
    )
    if claim_anchor not in text:
        raise RuntimeError("claims table anchor missing")
    text = text.replace(claim_anchor, claim_extra)
    text = text.replace(
        "学习收益 & forbidden & 训练步数为 0，learning gate 已关闭。\\\\",
        "V3 学习收益 & forbidden & V3 训练步数为 0；V6 是独立训练和证据链。\\\\",
    )
    text = text.replace(
        "实车与部署 & forbidden & 无硬件执行，runtime-ready 与 real-vehicle-ready 均为 false。\\\\",
        "实车与部署 & pending/not demonstrated & 当前无硬件执行；实车章节只保留待补合同和空表。\\\\",
    )
    if r"\end{document}" not in text:
        raise RuntimeError("document end missing")
    text = text.replace(r"\end{document}", BIBLIOGRAPHY + "\n\n" + r"\end{document}")
    return text


def copy_tree_files(source, destination, suffixes=None):
    for path in sorted(source.iterdir()):
        if not path.is_file() or (suffixes and path.suffix not in suffixes):
            continue
        destination.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, destination / path.name)


def copy_data():
    copy_tree_files(V3_MATERIALS / "figures", ROOT / "figures/v3", {".pdf", ".svg", ".png"})
    copy_tree_files(V3_MATERIALS / "tables", ROOT / "tables/v3", {".tex"})
    (ROOT / "figures/v6").mkdir(parents=True, exist_ok=True)
    for path in sorted(V6_PAPER.glob("figure_v6_*")):
        if path.suffix in {".pdf", ".svg"}:
            shutil.copy2(path, ROOT / "figures/v6" / path.name)

    v3_files = [
        W / "artifacts/v3/thesis/v3_18_a0_project_audit/system_architecture.yaml",
        W / "artifacts/v3/thesis/v3_18_a0_project_audit/method_evidence_matrix.csv",
        W / "artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_formal_summary.csv",
        W / "artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_stratum_results.csv",
        W / "artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_block_results.csv",
        W / "artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_claims.yaml",
        W / "artifacts/v3/thesis/v3_21_f1_execution/method_summary.csv",
        W / "artifacts/v3/thesis/v3_21_f1_execution/comparison_summary.csv",
        W / "artifacts/v3/thesis/v3_21_f1_execution/stratum_results.csv",
        W / "artifacts/v3/thesis/v3_21_f1_execution/block_results.csv",
        W / "artifacts/v3/thesis/v3_21_f1_execution/canonical_episodes.csv",
        W / "artifacts/v3/thesis/v3_21_f1_execution/formal_characterization_assessment.yaml",
        W / "artifacts/v3/thesis/v3_21_f1_execution/independent_result_review.yaml",
        V3_PAPER / "v3_22_p2_result_summary.csv",
        V3_PAPER / "v3_22_p2_claims.yaml",
    ]
    v5_files = [V5_FORMAL / name for name in (
        "formal_assessment.json", "formal_episodes.csv", "paired_blocks.csv", "bootstrap_distribution.csv"
    )]
    v6_files = [V6_PAPER / name for name in (
        "v6_02_f1_method_summary.csv", "v6_effect_summary.csv", "v6_02_f1_family_summary.csv",
        "v6_02_f1_formal_episodes.csv", "v6_gazebo_preflight_summary.csv", "v6_05_p1_claims.yaml",
        "literature_primary_sources.yaml",
    )] + [V6_FORMAL / "formal_assessment.json", V6_VALIDATION]
    copied = []
    for group, files in (("v3", v3_files), ("v5", v5_files), ("v6", v6_files)):
        destination = ROOT / "data" / group
        destination.mkdir(parents=True, exist_ok=True)
        seen = set()
        for path in files:
            name = path.name
            if name in seen:
                name = path.parent.name + "_" + name
            seen.add(name)
            shutil.copy2(path, destination / name)
            copied.append((path, destination / name))
    return copied


def build_summaries(copied):
    v5 = json.loads((V5_FORMAL / "formal_assessment.json").read_text())
    v6 = json.loads((V6_FORMAL / "formal_assessment.json").read_text())
    rows = [
        ["V3-11-F1", "historical confirmatory", "three frozen single-actor Gazebo templates",
         "V3 vs Actual TEB 0.8.4", 60, 30, "paired restricted-time reduction", "12.9288",
         "one-sided 97.5% lower=12.7253%", "30/30 vs 30/30", "scoped positive"],
        ["V3-21-F1", "fresh characterization", "three templates with failure inclusion",
         "V3 vs Actual TEB 0.8.4", 120, 30, "paired restricted-time reduction", "-0.1931",
         "two-sided 95% CI=[-19.8668%,12.9820%]", "28/30 vs 30/30", "superiority not demonstrated"],
        ["V5-03-F1", "fresh confirmatory negative", "four-family sensor-level fast population",
         "V5 vs analytic prior", 180, 60, "restricted-time reduction", "6.0936",
         "two-sided 95% CI=[1.7867%,11.0069%]", "see data/v5", "missed 8% gate"],
        ["V6-02-F1", "fresh confirmatory positive", "four-family sensor-level fast population",
         "PCT-SCOPE-RL vs analytic prior", 320, 80, "restricted-time reduction", "8.402444",
         "two-sided 97.5% CI=[3.484003%,13.713459%]", "60/80 vs 49/80", "scoped positive"],
        ["V6-02-F1 secondary", "fresh secondary", "same fast population",
         "PCT-SCOPE-RL vs frozen V5", 160, 80, "restricted-time reduction", "3.161052",
         "two-sided 95% CI=[0.493580%,6.576508%]", "60/80 vs 56/80", "positive secondary"],
        ["V6-03/04", "terminal preflight", "three fresh single-block Gazebo preflights",
         "V6/analytic/Actual TEB", 9, 3, "conservative clearance", "NA",
         "all V6 and analytic clearances negative", "V6 final preflight timeout", "transfer negative"],
        ["Real vehicle", "pending", "not executed", "not available", 0, 0, "not available", "NA",
         "NA", "NA", "no evidence; placeholder only"],
    ]
    with (ROOT / "CURRENT_RESULT_SUMMARY.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["study", "role", "population", "comparison", "episodes", "blocks",
                         "primary_metric", "point_percent", "interval", "goal_counts", "status"])
        writer.writerows(rows)

    claims = {
        "schema_version": "1.0", "stage": "V6-06-P1", "status": "current_thesis_claims_freeze",
        "thesis_core_claim": {
            "status": "qualified",
            "text_zh": "PCT 特权反事实候选排序蒸馏在冻结四类传感器级快速总体中相对解析先验降低失效计入均时 8.4024%。",
            "point_percent": 8.402444,
            "two_sided_97_5_percent_interval": [3.484003, 13.713459],
            "scope": "four_frozen_sensor_level_fast_interaction_families",
        },
        "claims": [
            {"id": "v3_historical_teb", "status": "qualified_historical", "value": "12.9288%, lower 12.7253%", "boundary": "V3-11 only; three frozen single-actor Gazebo templates; shared oracle track"},
            {"id": "v3_fresh_teb", "status": "not_demonstrated", "value": "-0.1931%, 95% CI [-19.8668%,12.9820%]", "boundary": "V3-21 failure-inclusive characterization"},
            {"id": "v5_learning", "status": "negative_practical_gate", "value": "6.0936%", "boundary": "positive interval but below frozen 8% gate"},
            {"id": "v6_learning", "status": "qualified", "value": "8.402444%, 97.5% CI [3.484003%,13.713459%]", "boundary": "fast sensor-level population only"},
            {"id": "v6_vs_v5", "status": "qualified_secondary", "value": "3.161052%, 95% CI [0.493580%,6.576508%]", "boundary": "same formal population"},
            {"id": "v6_history", "status": "not_demonstrated", "value": "exact tie", "boundary": "V6 versus V6 no-history"},
            {"id": "v6_gazebo", "status": "not_demonstrated", "value": "three failed preflights", "boundary": "no qualification or formal matrix"},
            {"id": "real_vehicle", "status": "pending_no_evidence", "value": None, "boundary": "placeholder only"},
        ],
        "forbidden_claims": [
            "general superiority over TEB", "V6 Gazebo effectiveness", "formal safety",
            "temporal-history benefit", "multi-actor or OOD generalization", "deployment readiness",
            "real-vehicle effectiveness", "fabricated real-vehicle values",
        ],
        "upstream_checks": {
            "v5_formal_status": v5["status"], "v6_formal_status": v6["status"],
            "v6_method_effectiveness_demonstrated": v6["method_effectiveness_demonstrated"],
            "v6_real_vehicle_authorized": v6["real_vehicle_authorized"],
        },
    }
    write(ROOT / "combined_claims_evidence_matrix.yaml", yaml.safe_dump(claims, sort_keys=False, allow_unicode=True))

    dictionary = "# 数据字典\n\n"
    dictionary += "本包只包含论文写作需要的派生数据，不用于重新选择模型或修改冻结结论。\n\n"
    dictionary += "## 顶层文件\n\n- `CURRENT_RESULT_SUMMARY.csv`：V3/V5/V6/实车占位的统一研究级摘要。\n- `combined_claims_evidence_matrix.yaml`：允许、未证明和禁止声明的权威边界。\n\n"
    dictionary += "## data/v3\n\n- `v3_12_*`：V3-11 历史 actual-TEB 正式结果的总表、分层、区块与 claims。\n- `method_summary.csv`、`comparison_summary.csv`、`stratum_results.csv`、`block_results.csv`：V3-21 四方法新鲜表征。\n- `canonical_episodes.csv`：V3-21 的 120 个 failure-inclusive episode。\n- `formal_characterization_assessment.yaml`：V3-21 终态结论。\n\n"
    dictionary += "## data/v5\n\n- `formal_assessment.json`：V5 6.0936% 阴性实效门结论。\n- `formal_episodes.csv`、`paired_blocks.csv`、`bootstrap_distribution.csv`：V5 逐回合、区块和 bootstrap 数据。\n\n"
    dictionary += "## data/v6\n\n- `formal_assessment.json`：V6 正式主结论。\n- `v6_02_f1_formal_episodes.csv`：320 个正式 episode。\n- `v6_02_f1_method_summary.csv`、`v6_02_f1_family_summary.csv`、`v6_effect_summary.csv`：方法、场景和效应表。\n- `v6_gazebo_preflight_summary.csv`：三次迁移负结果。\n- `selection_report.json`：四训练身份验证选择。\n\n"
    dictionary += "## figures 与 tables\n\n- `figures/v3`：5 张 V3 图，PDF/SVG/PNG。\n- `figures/v6`：6 张 V6 图，PDF/SVG。\n- `tables/v3`：5 个 V3 booktabs 三线表。V6 的主表已写入独立章节，原始数值以 CSV 为准。\n"
    write(ROOT / "DATA_DICTIONARY.md", dictionary)

    with (ROOT / "SOURCE_FILE_INDEX.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["upstream_path", "portable_path", "sha256"])
        for source, target in copied:
            writer.writerow([str(source.relative_to(W)), str(target.relative_to(ROOT)), sha(source)])


def write_handoff_files():
    (ROOT / "chapters").mkdir(parents=True, exist_ok=True)
    chapter9 = CH9.read_text(encoding="utf-8").replace(
        "V6 相对解析先验的主效应为 8.4024\\%，双侧 97.5\\% block-bootstrap 区间为\n"
        "[3.4840\\%,13.7135\\%]。",
        "V6 相对解析先验的主效应为 8.4024\\%，双侧 97.5\\% block-bootstrap 区间为\\\\\n"
        "$[3.4840\\%,13.7135\\%]$。",
    )
    write(ROOT / "chapters" / CH9.name, chapter9)
    shutil.copy2(CH10, ROOT / "chapters" / CH10.name)
    shutil.copy2(GUIDE, ROOT / GUIDE.name)
    prompt = PROMPT.read_text(encoding="utf-8").replace(
        "v6_06_p1_thesis_transfer_package", "v6_06_p1_r1_thesis_transfer_package"
    )
    write(ROOT / PROMPT.name, prompt)
    readme = """# README FIRST：当前 V3+V6 论文更新移交包

这是面向另一台电脑学校模板论文写作的自包含更新包。它不替代学校模板，但提供当前实验数据、
可编辑图、三线表、整合技术稿、声明边界、实车占位和 Codex 提示词。

## 最先打开

1. `FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf`：当前内容整合稿；
2. `CURRENT_THESIS_INTEGRATION_GUIDE.md`：如何修改旧 V3 初稿；
3. `CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md`：直接交给另一台电脑 Codex；
4. `combined_claims_evidence_matrix.yaml`：任何结果冲突时的权威声明边界；
5. `DATA_DICTIONARY.md`：数据和图表说明；
6. `chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex`：保持为空的实车章节。

## 当前核心结论

V6 PCT-SCOPE-RL 在四类冻结 sensor-level fast population 的 80 个 fresh paired blocks 中，
相对解析先验降低 failure-inclusive restricted mean time 8.402444%，双侧 97.5% CI 为
[3.484003%,13.713459%]。该结果不是 Gazebo、Actual TEB 或实车结论；三次 Gazebo 预检失败，
实车实验尚未执行。

## 校验

在本目录运行 `sha256sum -c terminal_content_manifest.sha256`。整合稿由冻结 V3 源派生，未修改
V3/V5/V6 上游证据。
"""
    write(ROOT / "README_FIRST.md", readme)
    context = """# Codex 机器交接上下文

- 原项目根目录：`/home/robot/robot_ws_base_rl`；复制到其他机器后优先使用本包相对路径。
- 稳定实车工作区 `/home/robot/robot_ws` 不属于论文写作输入，且本阶段未修改。
- V3 原始终态 TeX/PDF、V5/V6 正式评估均为冻结上游；学校模板迁移只能派生编辑，不能回写历史证据。
- `FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.tex` 是可独立编译的内容参考，不是学校模板。
- V5-00 历史架构合同有已记录的 YAML 缩进缺陷且被发布哈希绑定；论文写作不应修补它。
- 另一台电脑不需要重跑 ROS/Gazebo/训练即可完成论文更新；所有论文数值已导出到 `data/`。
- 实车实验状态为 `pending_no_evidence`。未提供新日志前，不得让 Codex补全任何结果数字。
- 如初稿中的数字与本包冲突，先查 `combined_claims_evidence_matrix.yaml`，再查对应 CSV/assessment。
"""
    write(ROOT / "CODEX_HANDOFF_MACHINE_CONTEXT.md", context)
    environment = {
        "schema_version": "1.0", "stage": "V6-06-P1-R1", "build_mode": "pure_offline_xelatex",
        "source_date_epoch": 1786147200, "timezone": "UTC_during_build", "paper_size": "A4",
        "engine": "XeLaTeX_via_latexmk", "hardware_accessed": False, "real_vehicle_used": False,
        "simulation_run": False, "training_run": False,
    }
    write(ROOT / "build_environment.yaml", yaml.safe_dump(environment, sort_keys=False))


def compile_pdf():
    environment = dict(os.environ)
    environment.update({"SOURCE_DATE_EPOCH": "1786147200", "FORCE_SOURCE_DATE": "1", "TZ": "UTC"})
    with tempfile.TemporaryDirectory(prefix="v6_06_thesis_build.", dir="/tmp") as temp:
        command = [
            "latexmk", "-xelatex", "-interaction=nonstopmode", "-halt-on-error",
            "-outdir={}".format(temp), OUT_TEX.name,
        ]
        subprocess.run(command, cwd=str(ROOT), env=environment, check=True)
        built_pdf = Path(temp) / OUT_PDF.name
        built_log = Path(temp) / OUT_LOG.name
        if not built_pdf.is_file() or not built_log.is_file():
            raise RuntimeError("integrated PDF build outputs absent")
        shutil.copy2(built_pdf, OUT_PDF)
        shutil.copy2(built_log, OUT_LOG)


def write_upstream_manifest(copied):
    sources = [BASE_TEX, CH9, CH10, GUIDE, PROMPT, CONTRACT,
               V3_PAPER / "FAM_TEB_V3_THESIS_DRAFT.pdf",
               V3_PAPER / "independent_paper_review.yaml",
               V5_FORMAL / "formal_assessment.json",
               V6_FORMAL / "formal_assessment.json",
               V6_PAPER / "v6_05_p1_paper_review_r2.yaml"]
    sources.extend(source for source, _ in copied)
    unique = sorted(set(sources), key=lambda p: str(p))
    with (ROOT / "upstream_evidence_manifest.sha256").open("w", encoding="utf-8", newline="\n") as stream:
        for path in unique:
            stream.write("{}  {}\n".format(sha(path), path.relative_to(W)))


def main():
    if ROOT.exists():
        raise RuntimeError("refuse overwrite existing transfer package")
    ROOT.mkdir(parents=True)
    copy_data_result = copy_data()
    write_handoff_files()
    write(OUT_TEX, build_tex())
    build_summaries(copy_data_result)
    write_upstream_manifest(copy_data_result)
    compile_pdf()
    print("status=v6_06_p1_r1_thesis_transfer_package_built")
    print("pdf_sha256={}".format(sha(OUT_PDF)))
    print("output_root={}".format(ROOT))


if __name__ == "__main__":
    main()
