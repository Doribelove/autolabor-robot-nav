#!/usr/bin/env python3
"""Deterministically export V4 tables, figures, claims and evidence manifest."""

import argparse
import csv
import hashlib
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from v4_scope_rl.io_utils import atomic_yaml, sha256_file


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def write_csv(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def bootstrap_interval(values, resamples=10000, seed=4401):
    values = np.asarray(values, dtype=np.float64)
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(values), size=(resamples, len(values)))
    samples = values[indices].mean(axis=1)
    return np.quantile(samples, [0.025, 0.975]).tolist()


def save_figure(figure, stem):
    figure.savefig(str(stem) + ".svg", bbox_inches="tight", metadata={"Date": None})
    figure.savefig(str(stem) + ".pdf", bbox_inches="tight", metadata={"CreationDate": None, "ModDate": None})
    plt.close(figure)


def architecture_figure(output_dir):
    figure, axis = plt.subplots(figsize=(12.2, 2.4))
    axis.axis("off")
    labels = [
        "Scene\nsnapshot",
        "Ackermann\ncandidates",
        "Reachable\noccupancy",
        "Hard\nverifier",
        "Boolean\nsafe mask",
        "Masked-PPO\ncandidate ID",
        "Activation\nrecheck",
        "Tracker /\ncmd_vel_navigation",
    ]
    colors = ["#d9eaf7", "#d9eaf7", "#d9eaf7", "#f9d5d3", "#f9d5d3", "#dcefd8", "#f9d5d3", "#d9eaf7"]
    for index, (label, color) in enumerate(zip(labels, colors)):
        x = index * 1.45
        axis.add_patch(
            plt.Rectangle((x, 0.38), 1.15, 0.86, facecolor=color, edgecolor="#303030", linewidth=1.0)
        )
        axis.text(x + 0.575, 0.81, label, ha="center", va="center", fontsize=8.5)
        if index < len(labels) - 1:
            axis.annotate("", xy=(x + 1.42, 0.81), xytext=(x + 1.17, 0.81), arrowprops={"arrowstyle": "->", "lw": 1.1})
    axis.text(7.25, 1.58, "Learning authority", color="#2f7d32", ha="center", fontsize=9, weight="bold")
    axis.annotate("", xy=(7.98, 1.32), xytext=(7.98, 1.52), arrowprops={"arrowstyle": "->", "color": "#2f7d32"})
    axis.text(5.1, 0.08, "Safety/feasibility remain outside the learned policy", color="#a32828", ha="center", fontsize=9)
    axis.set_xlim(-0.15, 11.45)
    axis.set_ylim(0.0, 1.95)
    save_figure(figure, output_dir / "figure_v4_architecture")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--final-pdf")
    args = parser.parse_args()
    workspace = Path(args.workspace).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    architecture_figure(output_dir)

    fresh_path = workspace / "artifacts/v4/v4_04_n1_fresh_seed_characterization.yaml"
    fresh = load_yaml(fresh_path)
    episodes = fresh["episodes"]
    analytic = {item["seed"]: item for item in episodes if item["policy"] == "analytic"}
    learned = {item["seed"]: item for item in episodes if item["policy"] == "masked_ppo"}
    seeds = sorted(analytic)
    paired_rows = []
    reductions = []
    differences = []
    for seed in seeds:
        baseline = analytic[seed]
        policy = learned[seed]
        difference = baseline["restricted_elapsed_s"] - policy["restricted_elapsed_s"]
        reduction = difference / baseline["restricted_elapsed_s"]
        differences.append(difference)
        reductions.append(reduction)
        paired_rows.append(
            {
                "seed": seed,
                "actor_policy": baseline["actor_policy"],
                "actor_angle_deg": baseline["actor_angle_deg"],
                "analytic_time_s": baseline["restricted_elapsed_s"],
                "masked_ppo_time_s": policy["restricted_elapsed_s"],
                "paired_difference_s": difference,
                "paired_relative_reduction": reduction,
                "analytic_goal": int(baseline["goal_reached"]),
                "masked_ppo_goal": int(policy["goal_reached"]),
                "minimum_clearance_m": min(baseline["minimum_clearance_m"], policy["minimum_clearance_m"]),
            }
        )
    write_csv(output_dir / "v4_04_n1_paired_blocks.csv", list(paired_rows[0]), paired_rows)

    interval = bootstrap_interval(reductions)
    summary = fresh["summary"]
    result_rows = [
        {
            "method": "same_set_receding_analytic",
            "blocks": 30,
            "goals": 30,
            "mean_restricted_time_s": summary["analytic_mean_restricted_elapsed_s"],
            "contacts": 0,
            "mask_violations": 0,
            "relative_reduction_vs_analytic": 0.0,
        },
        {
            "method": "masked_ppo_seed4104",
            "blocks": 30,
            "goals": 30,
            "mean_restricted_time_s": summary["learned_mean_restricted_elapsed_s"],
            "contacts": 0,
            "mask_violations": 0,
            "relative_reduction_vs_analytic": summary["paired_relative_time_reduction_mean"],
        },
    ]
    write_csv(output_dir / "v4_04_n1_method_summary.csv", list(result_rows[0]), result_rows)

    factor_rows = [
        {"stage": "V4-02-R0", "factor": "baseline", "training_steps": 100000, "validation_reduction_percent": 0.0, "gate_pass": 0},
        {"stage": "V4-03-F1", "factor": "response persistence", "training_steps": 150000, "validation_reduction_percent": 0.0, "gate_pass": 0},
        {"stage": "V4-03-F2-s4103", "factor": "time reward alignment", "training_steps": 150000, "validation_reduction_percent": 0.10393907563025209, "gate_pass": 0},
        {"stage": "V4-03-F2-s4104", "factor": "time reward alignment", "training_steps": 150000, "validation_reduction_percent": 0.07268907563025209, "gate_pass": 0},
        {"stage": "V4-03-F3", "factor": "actor response rate", "training_steps": 50000, "validation_reduction_percent": 0.0, "gate_pass": 0},
        {"stage": "V4-04-N1", "factor": "fresh terminal characterization", "training_steps": 0, "validation_reduction_percent": 100.0 * summary["paired_relative_time_reduction_mean"], "gate_pass": 0},
    ]
    write_csv(output_dir / "v4_factor_history.csv", list(factor_rows[0]), factor_rows)

    figure, axis = plt.subplots(figsize=(8.2, 4.0))
    labels = [row["stage"] for row in factor_rows]
    values = [row["validation_reduction_percent"] for row in factor_rows]
    axis.bar(np.arange(len(labels)), values, color="#4c78a8")
    axis.axhline(3.0, color="#d62728", linestyle="--", linewidth=1.4, label="Frozen 3% gate")
    axis.set_ylabel("Paired time reduction (%)")
    axis.set_xticks(np.arange(len(labels)), labels, rotation=25, ha="right")
    axis.set_ylim(min(-0.1, min(values) - 0.1), 3.35)
    axis.legend(frameon=False)
    axis.grid(axis="y", alpha=0.25)
    save_figure(figure, output_dir / "figure_v4_factor_history")

    figure, axis = plt.subplots(figsize=(8.2, 3.8))
    axis.axhline(0.0, color="#202020", linewidth=0.9)
    colors = ["#2ca02c" if value > 0.0 else "#9aa0a6" for value in differences]
    axis.bar(np.arange(1, len(seeds) + 1), differences, color=colors)
    axis.set_xlabel("Fresh paired block")
    axis.set_ylabel("Analytic - Masked-PPO time (s)")
    axis.set_title("V4-04-N1: one of 30 blocks improved")
    axis.grid(axis="y", alpha=0.25)
    save_figure(figure, output_dir / "figure_v4_fresh_paired_differences")

    action_counts = {"analytic": np.zeros(6, dtype=int), "masked_ppo": np.zeros(6, dtype=int)}
    for item in episodes:
        for decision in item.get("trace", []):
            action_counts[item["policy"]][int(decision["action_id"])] += 1
    figure, axis = plt.subplots(figsize=(9.2, 4.2))
    x = np.arange(6)
    axis.bar(x - 0.19, action_counts["analytic"], 0.38, label="Receding analytic", color="#7f7f7f")
    axis.bar(x + 0.19, action_counts["masked_ppo"], 0.38, label="Masked-PPO", color="#4c78a8")
    axis.set_xticks(x, ["stop", "v0.4", "v0.8", "v1.2", "left", "right"])
    axis.set_ylabel("Decision count")
    axis.legend(frameon=False)
    axis.grid(axis="y", alpha=0.25)
    save_figure(figure, output_dir / "figure_v4_action_distribution")

    claims = {
        "schema_version": "1.0",
        "architecture_generation": "v4",
        "stage": "V4-06-P1",
        "formal_result": False,
        "reinforcement_learning_module_implemented": True,
        "hard_action_mask_invariant_passed": True,
        "global_training_environment_steps": 600000,
        "fresh_characterization": {
            "paired_blocks": 30,
            "episodes": 60,
            "analytic_mean_restricted_elapsed_s": summary["analytic_mean_restricted_elapsed_s"],
            "masked_ppo_mean_restricted_elapsed_s": summary["learned_mean_restricted_elapsed_s"],
            "paired_relative_time_reduction_mean": summary["paired_relative_time_reduction_mean"],
            "bootstrap_two_sided_95_percent_interval": interval,
            "positive_block_fraction": summary["positive_block_fraction"],
            "goal_rate_both": 1.0,
            "contact_count": 0,
            "mask_violation_count": 0,
        },
        "frozen_efficiency_gate_minimum": 0.03,
        "meaningful_optimization_demonstrated": False,
        "actual_teb_comparison_run_in_v4": False,
        "gazebo_run_in_v4": False,
        "real_vehicle_used": False,
        "permitted_claim_zh": "实现了带硬安全动作掩码的强化学习候选选择模块，但未证明其相对解析排序器具有有意义的效率提升。",
        "forbidden_claims": [
            "V4优于Actual TEB",
            "V4已通过Gazebo正式评测",
            "强化学习带来显著性能提升",
            "形式化安全保证",
            "实车或部署就绪",
        ],
    }
    atomic_yaml(output_dir / "v4_06_p1_claims.yaml", claims)

    evidence_relpaths = [
        "src/application/v4_scope_rl/config/v4_00_architecture_contract.yaml",
        "src/application/v4_scope_rl/config/v4_02_f1_response_persistence_preregistration.yaml",
        "src/application/v4_scope_rl/config/v4_03_f2_time_alignment_preregistration.yaml",
        "src/application/v4_scope_rl/config/v4_03_f3_response_rate_preregistration.yaml",
        "src/application/v4_scope_rl/config/v4_04_n1_terminal_characterization_preregistration.yaml",
        "src/application/v4_scope_rl/src/v4_scope_rl/fast_env.py",
        "src/application/v4_scope_rl/src/v4_scope_rl/masked_ppo.py",
        "src/application/v4_scope_rl/src/v4_scope_rl/evaluation.py",
        "artifacts/v4/v4_02_r0_engineering_probe.yaml",
        "artifacts/v4/v4_02_f1_learning_value_audit.yaml",
        "artifacts/v4/v4_03_f1_seed4102/training_report.yaml",
        "artifacts/v4/v4_03_f2_seed4103/training_report.yaml",
        "artifacts/v4/v4_03_f2_seed4104/training_report.yaml",
        "artifacts/v4/v4_03_f3_seed4105/training_report.yaml",
        "artifacts/v4/v4_04_n1_fresh_seed_characterization.yaml",
        "artifacts/v4/v4_component_tests.xml",
    ]
    if args.final_pdf:
        final_pdf = Path(args.final_pdf).resolve()
        evidence_relpaths.append(str(final_pdf.relative_to(workspace)))
    output_relpaths = [
        str(path.relative_to(workspace))
        for path in sorted(output_dir.glob("*"))
        if path.name != "v4_06_p1_evidence_manifest.sha256"
    ]
    roster = []
    for relative in evidence_relpaths + output_relpaths:
        path = workspace / relative
        if path.exists():
            roster.append((relative, sha256_file(path)))
    roster = sorted(set(roster))
    manifest_path = output_dir / "v4_06_p1_evidence_manifest.sha256"
    manifest_path.write_text("".join("{}  {}\n".format(digest, path) for path, digest in roster), encoding="utf-8")
    print(
        yaml.safe_dump(
            {
                "output_dir": str(output_dir),
                "paired_relative_reduction": float(np.mean(reductions)),
                "bootstrap_95_interval": interval,
                "manifest_sha256": sha256_file(manifest_path),
            },
            sort_keys=False,
        ),
        end="",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
