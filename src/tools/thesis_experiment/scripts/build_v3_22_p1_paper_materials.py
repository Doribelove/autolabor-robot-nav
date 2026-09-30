#!/usr/bin/env python3
"""Build deterministic thesis figures, three-line tables and narratives.

This is a pure-offline adapter over the frozen V3-21 CSV/YAML evidence.  It
does not launch ROS, access hardware, refit a model, or alter experiment data.
"""

import argparse
import csv
import hashlib
import io
import os
from pathlib import Path
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


W = Path("/home/robot/robot_ws_base_rl")
OUT = Path("artifacts/v3/thesis/v3_22_p1_paper_materials")
CONTRACT = Path("config/thesis_experiments/v3/v3_22_p1_paper_materials_contract.yaml")
V321 = Path("artifacts/v3/thesis/v3_21_f1_execution")
ASSESS = V321 / "formal_characterization_assessment.yaml"
EPISODES = V321 / "canonical_episodes.csv"
METHODS = V321 / "method_summary.csv"
COMPARISONS = V321 / "comparison_summary.csv"
BLOCKS = V321 / "block_results.csv"
STRATA = V321 / "stratum_results.csv"
REVIEW = V321 / "independent_result_review.yaml"
V311 = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml")
BUILDER = Path("src/tools/thesis_experiment/scripts/build_v3_22_p1_paper_materials.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_22_p1_paper_materials.py")

METHOD_ORDER = [
    "v3_spatial_safe", "actual_teb", "fixed_v120_shielded",
    "v3_no_candidate_conditioning",
]
LABELS = {
    "v3_spatial_safe": "V3 spatial-safe",
    "actual_teb": "Actual TEB 0.8.4",
    "fixed_v120_shielded": "Fixed-v1.20 shielded",
    "v3_no_candidate_conditioning": "No candidate conditioning",
}
SHORT = {
    "v3_spatial_safe": "V3",
    "actual_teb": "Actual TEB",
    "fixed_v120_shielded": "Fixed-v1.20",
    "v3_no_candidate_conditioning": "No conditioning",
}
COLORS = ["#1769AA", "#667085", "#D48806", "#C74440"]
INPUTS = [CONTRACT, ASSESS, EPISODES, METHODS, COMPARISONS, BLOCKS, STRATA, REVIEW, V311, BUILDER, REVIEWER]


def sha_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def read(path):
    return (W / path).read_bytes()


def rows(path):
    with (W / path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def atomic_write(path, payload):
    target = W / path
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def configure_plotting():
    matplotlib.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
        "figure.dpi": 140,
        "savefig.dpi": 300,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "svg.hashsalt": "v3-22-p1-thesis-materials",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })


def save_figure(fig, root, name):
    payloads = {}
    for suffix in ("pdf", "svg", "png"):
        target = root / (name + "." + suffix)
        kwargs = {"bbox_inches": "tight"}
        if suffix == "pdf":
            kwargs["metadata"] = {
                "Title": name, "Author": "V3-22-P1 deterministic export",
                "Creator": "matplotlib", "CreationDate": None, "ModDate": None,
            }
        elif suffix == "svg":
            kwargs["metadata"] = {"Title": name, "Creator": "matplotlib"}
        fig.savefig(str(target), format=suffix, **kwargs)
        payloads[OUT / "figures" / target.name] = target.read_bytes()
    plt.close(fig)
    return payloads


def figure_payloads(tmp):
    configure_plotting()
    payloads = {}
    method_rows = {row["method"]: row for row in rows(METHODS)}
    comparison_rows = rows(COMPARISONS)
    block_rows = rows(BLOCKS)
    stratum_rows = rows(STRATA)
    episode_rows = rows(EPISODES)

    # Method outcomes: task completion and failure-inclusive restricted time.
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.05), gridspec_kw={"wspace": 0.32})
    success = [100.0 * float(method_rows[m]["goal_success_rate"]) for m in METHOD_ORDER]
    times = [float(method_rows[m]["restricted_time_mean_s"]) for m in METHOD_ORDER]
    x = np.arange(4)
    axes[0].bar(x, success, color=COLORS, width=0.68)
    axes[0].set_ylim(0, 108); axes[0].set_ylabel("Goal success (%)")
    axes[0].set_title("(a) Task completion")
    axes[1].bar(x, times, color=COLORS, width=0.68)
    axes[1].axhline(22.0, color="#98A2B3", lw=0.8, ls="--")
    axes[1].set_ylim(0, 24); axes[1].set_ylabel("Restricted time (s, lower is better)")
    axes[1].set_title("(b) Failure-inclusive outcome")
    for ax, values, fmt in ((axes[0], success, "{:.1f}"), (axes[1], times, "{:.2f}")):
        ax.set_xticks(x); ax.set_xticklabels([SHORT[m] for m in METHOD_ORDER], rotation=22, ha="right")
        ax.grid(axis="y", color="#E4E7EC", lw=0.6, zorder=0)
        for i, value in enumerate(values): ax.text(i, value + (1.5 if ax is axes[0] else 0.35), fmt.format(value), ha="center", va="bottom", fontsize=8)
    payloads.update(save_figure(fig, tmp, "fig01_method_outcomes"))

    # Pairwise block bootstrap intervals.
    fig, ax = plt.subplots(figsize=(6.8, 2.9))
    y = np.arange(len(comparison_rows))[::-1]
    names = ["vs Actual TEB 0.8.4", "vs Fixed-v1.20 shielded", "vs No candidate conditioning"]
    for i, row in enumerate(comparison_rows):
        point = 100.0 * float(row["point_relative_reduction"])
        low = 100.0 * float(row["ci95_lower"]); high = 100.0 * float(row["ci95_upper"])
        ax.errorbar(point, y[i], xerr=np.array([[point-low], [high-point]]), fmt="o", color=COLORS[i+1], capsize=3, lw=1.5)
        ax.text(high + 1.0, y[i], "{:.2f}% [{:.2f}, {:.2f}]".format(point, low, high), va="center", fontsize=8)
    ax.axvline(0, color="#344054", lw=0.9, ls="--")
    ax.set_yticks(y); ax.set_yticklabels(names); ax.set_xlabel("V3 relative restricted-time reduction (%)")
    ax.set_title("Paired block-bootstrap effects (two-sided 95% CI)")
    ax.set_xlim(-25, 80); ax.grid(axis="x", color="#E4E7EC", lw=0.6)
    payloads.update(save_figure(fig, tmp, "fig02_pairwise_effects"))

    # Paired block plot against actual TEB; failure is retained at 22 s.
    fig, ax = plt.subplots(figsize=(4.9, 4.15))
    x_teb = np.array([float(r["actual_teb_time_s"]) for r in block_rows])
    y_v3 = np.array([float(r["v3_spatial_safe_time_s"]) for r in block_rows])
    success_v3 = np.array([int(r["v3_spatial_safe_success"]) for r in block_rows], dtype=bool)
    ax.scatter(x_teb[success_v3], y_v3[success_v3], s=35, color=COLORS[0], alpha=0.78, label="V3 goal reached")
    ax.scatter(x_teb[~success_v3], y_v3[~success_v3], s=70, marker="X", color=COLORS[3], label="V3 retained timeout")
    low, high = 6.4, 22.6
    ax.plot([low, high], [low, high], color="#98A2B3", ls="--", lw=1, label="Equal time")
    ax.set_xlim(low, 8.2); ax.set_ylim(6.3, high)
    ax.set_xlabel("Actual TEB restricted time (s)"); ax.set_ylabel("V3 restricted time (s)")
    ax.set_title("Thirty paired scene-seed blocks")
    ax.grid(color="#E4E7EC", lw=0.6); ax.legend(loc="upper left")
    payloads.update(save_figure(fig, tmp, "fig03_v3_vs_actual_teb_blocks"))

    # Six scene-policy strata, preserving both success and restricted-time views.
    scene_names = {"bounded_corridor_crossing_110": "Bounded 110", "open_crossing_90": "Open 90", "wide_corridor_crossing_70": "Wide 70"}
    row_labels = [scene_names[r["scene"]] + " / " + r["actor_policy"].title() for r in stratum_rows]
    success_matrix = np.array([[100*float(r[m+"_success_rate"]) for m in METHOD_ORDER] for r in stratum_rows])
    time_matrix = np.array([[float(r[m+"_mean_time_s"]) for m in METHOD_ORDER] for r in stratum_rows])
    fig, axes = plt.subplots(1, 2, figsize=(8.25, 4.25), gridspec_kw={"wspace": 0.08})
    images = [axes[0].imshow(success_matrix, cmap="Blues", vmin=0, vmax=100, aspect="auto"), axes[1].imshow(time_matrix, cmap="YlOrRd", vmin=6, vmax=22, aspect="auto")]
    for j, (ax, matrix, title, suffix) in enumerate(zip(axes, (success_matrix, time_matrix), ("(a) Goal success", "(b) Restricted time"), ("%", " s"))):
        ax.set_xticks(np.arange(4)); ax.set_xticklabels([SHORT[m] for m in METHOD_ORDER], rotation=25, ha="right")
        ax.set_yticks(np.arange(6)); ax.set_yticklabels(row_labels if j == 0 else [])
        ax.set_title(title)
        for rr in range(matrix.shape[0]):
            for cc in range(matrix.shape[1]):
                value = matrix[rr, cc]
                color = "white" if (j == 0 and value > 55) or (j == 1 and value > 14) else "#101828"
                ax.text(cc, rr, ("{:.0f}" if j == 0 else "{:.2f}").format(value) + suffix, ha="center", va="center", fontsize=7, color=color)
    payloads.update(save_figure(fig, tmp, "fig04_scene_policy_strata"))

    # Empirical minimum-clearance distribution (descriptive, not a safety proof).
    fig, ax = plt.subplots(figsize=(7.0, 3.35))
    data = [[float(r["minimum_clearance_m"]) for r in episode_rows if r["method"] == m] for m in METHOD_ORDER]
    bp = ax.boxplot(data, patch_artist=True, widths=0.58, showfliers=False, medianprops={"color": "#101828", "linewidth": 1.2})
    for patch, color in zip(bp["boxes"], COLORS): patch.set_facecolor(color); patch.set_alpha(0.55)
    rng = np.random.RandomState(3221)
    for i, (values, color) in enumerate(zip(data, COLORS), start=1):
        ax.scatter(rng.normal(i, 0.045, len(values)), values, s=13, color=color, alpha=0.62, edgecolor="none")
    ax.set_xticks(np.arange(1,5)); ax.set_xticklabels([SHORT[m] for m in METHOD_ORDER], rotation=18, ha="right")
    ax.set_ylabel("Empirical minimum clearance (m)")
    ax.set_title("Finite-simulation clearance distribution (descriptive only)")
    ax.grid(axis="y", color="#E4E7EC", lw=0.6)
    payloads.update(save_figure(fig, tmp, "fig05_minimum_clearance"))
    return payloads


def latex_escape(value):
    return str(value).replace("_", r"\_").replace("%", r"\%")


def table_payloads():
    methods = {r["method"]: r for r in rows(METHODS)}
    comparisons = rows(COMPARISONS)
    strata = rows(STRATA)
    payloads = {}
    method_lines = [
        r"\begin{table}[htbp]", r"\centering", r"\caption{V3-21 四方法失效计入式总体结果。}",
        r"\label{tab:v321-methods}", r"\small", r"\begin{tabular}{lrrrrrr}", r"\toprule",
        r"方法 & 成功率 & 受限时间/s & 路径/m & 最小净空/m & 回退率 & contact\\", r"\midrule",
    ]
    for m in METHOD_ORDER:
        r = methods[m]
        method_lines.append("{} & {:.1f}\\% & {:.3f} & {:.3f} & {:.3f} & {:.1f}\\% & {}\\\\".format(
            LABELS[m], 100*float(r["goal_success_rate"]), float(r["restricted_time_mean_s"]),
            float(r["path_length_mean_m"]), float(r["minimum_clearance_min_m"]),
            100*float(r["fallback_rate"]), r["contact_count"]))
    method_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    payloads[OUT / "tables" / "tab01_method_results.tex"] = ("\n".join(method_lines)+"\n").encode()

    comp_labels = ["Actual TEB 0.8.4", "Fixed-v1.20 shielded", "去候选条件化消融"]
    comp_lines = [r"\begin{table}[htbp]", r"\centering", r"\caption{V3-21 以配对区块为单位的比较结果。}", r"\label{tab:v321-comparisons}", r"\small", r"\begin{tabular}{lrrrr}", r"\toprule", r"比较对象 & 相对降幅/\% & 双侧95\% CI/\% & Holm $p$ & 成功率（V3/对方）\\", r"\midrule"]
    for label, r in zip(comp_labels, comparisons):
        comp_lines.append("{} & {:.3f} & [{:.3f}, {:.3f}] & {:.6f} & {:.1f}\\%/{:.1f}\\%\\\\".format(
            label, 100*float(r["point_relative_reduction"]), 100*float(r["ci95_lower"]),
            100*float(r["ci95_upper"]), float(r["holm_p"]), 100*float(r["main_success_rate"]),
            100*float(r["comparator_success_rate"])))
    comp_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    payloads[OUT / "tables" / "tab02_pairwise_results.tex"] = ("\n".join(comp_lines)+"\n").encode()

    scene_short = {"bounded_corridor_crossing_110": "bounded-110", "open_crossing_90": "open-90", "wide_corridor_crossing_70": "wide-70"}
    stratum_lines = [r"\begin{table}[htbp]", r"\centering", r"\caption{V3-21 六个场景--行为分层结果（成功率/受限均时）。}", r"\label{tab:v321-strata}", r"\scriptsize", r"\begin{tabular}{llrrrr}", r"\toprule", r"场景 & actor & V3 & Actual TEB & Fixed-v1.20 & 去条件化\\", r"\midrule"]
    for r in strata:
        cells = []
        for m in METHOD_ORDER:
            cells.append("{:.0f}\\%/{:.2f}s".format(100*float(r[m+"_success_rate"]), float(r[m+"_mean_time_s"])))
        stratum_lines.append("{} & {} & {} & {} & {} & {}\\\\".format(scene_short[r["scene"]], r["actor_policy"], *cells))
    stratum_lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    payloads[OUT / "tables" / "tab03_stratum_results.tex"] = ("\n".join(stratum_lines)+"\n").encode()

    config = r"""\begin{table}[htbp]
\centering
\caption{V3-21 可复现实验配置。}
\label{tab:v321-config}
\small
\begin{tabular}{p{0.30\textwidth}p{0.61\textwidth}}
\toprule
配置项 & 冻结值\\
\midrule
实验单位 & 30 个 paired scene--seed blocks，120 episodes\\
随机种子 & 8201--8230；每种方法共享区块 seed\\
场景 & open 90$^\circ$、wide corridor 70$^\circ$、bounded corridor 110$^\circ$\\
actor 行为 & cautious / hesitant，共六个 scene$\times$policy strata\\
方法 & V3 spatial-safe、Actual TEB 0.8.4、Fixed-v1.20 shielded、去候选条件化消融\\
失败处理 & 所有已消费失败保留，并以 22.0 s 受限时间进入 intention-to-treat 统计\\
推断 & paired block bootstrap 20000 次，双侧 95\% CI；三比较 Holm 校正\\
执行隔离 & ROS master 端口 24002--24240，低于 Linux ephemeral range\\
训练与实物 & 0 training steps；未访问硬件、CAN/串口或实车\\
\bottomrule
\end{tabular}
\end{table}
"""
    payloads[OUT / "tables" / "tab04_experiment_configuration.tex"] = config.encode()

    integrity = r"""\begin{table}[htbp]
\centering
\caption{四方法研究链中的失效关闭与证据保留。}
\label{tab:v319-v321-integrity}
\small
\begin{tabular}{llll}
\toprule
阶段 & 预定/完成 & 终止或偏差 & 处置\\
\midrule
V3-19-Q1 & 24/24 & 主方法成功 5/6，资格门失败 & 正式 seeds 退役，未启动正式矩阵\\
V3-20-F1 & 120/77 & ROS master 端口碰撞（seq.78） & 不重试；剩余 42 单元 forfeited\\
V3-21-F1 & 120/120 & seq.107 gate 误差超限 0.002286 m & 保留 ITT；排除敏感性结论不变\\
\bottomrule
\end{tabular}
\end{table}
"""
    payloads[OUT / "tables" / "tab05_integrity_history.tex"] = integrity.encode()
    return payloads


def narrative_payloads():
    zh = """# V3-21 论文结果分析材料

## 实验定位

V3-21 是一项新鲜随机种子、四方法、失效计入式表征研究。30 个 scene--seed 配对区块覆盖三个冻结单反应式 actor Gazebo 模板和 cautious/hesitant 两类行为，共 120 个一次性回合。所有已经消费的任务失败均以 22 s 受限时间保留。该研究没有训练、感知退化、多人/OOD 或实车环节。

## 主要结果

主方法 V3 spatial-safe 完成 28/30 个目标（93.33%），受限时间均值为 7.785 s；Actual TEB 0.8.4 完成 30/30 个目标，均值为 7.770 s。V3 相对 Actual TEB 的配对相对降幅为 -0.193%，双侧 95% block-bootstrap 区间为 [-19.867%, 12.982%]，Holm 校正 p=0.755。因而，这批数据不支持 V3 相对 Actual TEB 的失效计入式时间优势。去掉唯一协议偏差区块后的点估计为 -0.643%，结论不变。

相对 Fixed-v1.20 shielded，V3 的受限时间缩短 64.614%（95% CI [57.682%, 69.250%]）；相对去候选条件化消融缩短 49.454%（95% CI [37.001%, 58.800%]）。后者成功率仅为 43.33%，而 V3 为 93.33%，说明候选条件预测在该冻结总体内具有明确的探索性机制价值。Fixed-v1.20 全部触发 certified-stop timeout，这一结果说明纯固定前向策略过于保守，不代表所有固定规划器。

四种方法 contact 均为 0。V3 的经验最小净空为 0.476 m，Actual TEB 为 0.0395 m；这些只是有限仿真的描述量，不能写成形式化安全或真实环境安全证明。V3 的规划 P99 延迟为 10.756 ms。

## 与 V3-11 的关系

历史 V3-11 正式研究在另一组 fresh seeds 上得到 V3 相对 Actual TEB 12.9288% 的阳性时间降幅，双方目标成功均为 100%。V3-21 则出现两个 V3 certified-stop timeout，使失效计入式均值不再优于 Actual TEB。两个阶段必须分别报告，不能事后合并、挑选或用 V3-11 覆盖 V3-21。论文应将其解释为：限定总体中的历史阳性机制证据存在，但新的更严格复现实验未证明 Actual TEB 性能优势，性能结论对少量保守回退敏感。

## 合法结论

可以陈述：系统、采集框架和四方法评测已实现；V3-21 120/120 一次性执行完成；相对去候选条件化消融存在大幅任务/时间差；相对 Actual TEB 未证明失效计入式时间提升。不得陈述：V3 普遍优于 TEB、具有形式化安全、感知/OOD 泛化、部署或实车性能优势。
"""
    en = """# V3-21 paper-ready results narrative

V3-21 is a fresh-seed, four-method, failure-inclusive characterization over 30 paired scene-seed blocks and 120 one-shot episodes. V3 spatial-safe reached 28/30 goals (93.33%) with a mean restricted time of 7.785 s; actual TebLocalPlannerROS 0.8.4 reached 30/30 goals with 7.770 s. The paired relative reduction was -0.193% (two-sided 95% block-bootstrap CI [-19.867%, 12.982%], Holm-adjusted p=0.755), so this study does not demonstrate a failure-inclusive time advantage over actual TEB.

Against fixed-v1.20 shielded, V3 reduced restricted time by 64.614% (95% CI [57.682%, 69.250%]); against the no-candidate-conditioning ablation it reduced time by 49.454% (95% CI [37.001%, 58.800%]). The ablation reached only 13/30 goals, compared with 28/30 for V3, providing exploratory evidence for candidate-conditioned prediction within the frozen population. All methods recorded zero contact states. Empirical clearance is descriptive finite-simulation evidence, not a formal safety guarantee.

The historical V3-11 study remains a valid, separately preregistered positive result (12.9288% reduction with 100% success for both methods), but V3-21 did not reproduce the actual-TEB efficiency advantage because two conservative V3 fallbacks were retained at 22 s. The studies must not be pooled or selectively filtered. The defensible conclusion is therefore bounded mechanism evidence plus a negative fresh replication for failure-inclusive superiority over actual TEB.
"""
    claims = {
        "schema_version": "1.0", "stage": "V3-22-P1",
        "status": "paper_materials_ready_for_independent_review",
        "historical_formal_result": {"stage": "V3-11-F1", "valid": True, "point_reduction_percent": 12.9288, "scope": "three_frozen_single_reactive_actor_Gazebo_templates_shared_oracle_track"},
        "fresh_characterization": {"stage": "V3-21-F1", "episodes": 120, "paired_blocks": 30, "formal_result": False, "characterization_result_available": True, "actual_teb_improvement_demonstrated": False, "point_reduction_percent": -0.19305019305020038, "two_sided_95_interval_percent": [-19.8668384879725, 12.982005141388184]},
        "exploratory_ablation_evidence": {"comparison": "v3_spatial_safe_vs_v3_no_candidate_conditioning", "point_reduction_percent": 49.45352234606644, "two_sided_95_interval_percent": [37.00102816638785, 58.80024360535931], "main_success_percent": 93.33333333333333, "ablation_success_percent": 43.333333333333336},
        "forbidden_claims": ["general_V3_superiority_over_TEB", "perception_robustness", "multi_actor_or_OOD_generalization", "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance"],
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    return {
        OUT / "RESULTS_NARRATIVE_ZH.md": zh.encode("utf-8"),
        OUT / "RESULTS_NARRATIVE_EN.md": en.encode("utf-8"),
        OUT / "paper_claims.yaml": yaml.safe_dump(claims, sort_keys=False, allow_unicode=True).encode("utf-8"),
    }


def make_outputs():
    assessment = yaml.safe_load(read(ASSESS))
    if assessment["episode_count"] != 120 or assessment["performance_improvement_vs_actual_teb_demonstrated"]:
        raise RuntimeError("V3-21 input claim state differs")
    with tempfile.TemporaryDirectory(prefix="v3_22_figures_") as directory:
        output = figure_payloads(Path(directory))
    output.update(table_payloads())
    output.update(narrative_payloads())
    readme = """# V3-22-P1 thesis paper materials

Deterministic pure-offline export from the frozen V3-21 evidence.

- `figures/`: five publication figures in PDF, SVG and 300-dpi PNG.
- `tables/`: five LaTeX booktabs three-line tables.
- `RESULTS_NARRATIVE_ZH.md` / `RESULTS_NARRATIVE_EN.md`: paper-ready result interpretation.
- `paper_claims.yaml`: qualified, not-demonstrated and forbidden claims.

V3-21 does not demonstrate failure-inclusive superiority over actual TEB.  The historical V3-11 positive result is retained as separate evidence and is not pooled.  No ROS/Gazebo execution, training, hardware or real vehicle was used to create these materials.
"""
    output[OUT / "README.md"] = readme.encode("utf-8")
    manifest_lines = []
    for path in INPUTS:
        manifest_lines.append("{}  {}\n".format(sha_bytes(read(path)), path))
    for path in sorted(output, key=str):
        manifest_lines.append("{}  {}\n".format(sha_bytes(output[path]), path))
    output[OUT / "paper_materials_manifest.sha256"] = "".join(manifest_lines).encode("utf-8")
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    output = make_outputs()
    for path, payload in output.items():
        target = W / path
        if args.check:
            if not target.is_file() or target.read_bytes() != payload:
                raise RuntimeError("generated material differs: {}".format(path))
        else:
            if target.exists():
                raise RuntimeError("refuse overwrite: {}".format(path))
            atomic_write(path, payload)
    print("status=paper_materials_built files={} manifest_sha256={}".format(len(output), sha_bytes(output[OUT / "paper_materials_manifest.sha256"])))


if __name__ == "__main__":
    main()

