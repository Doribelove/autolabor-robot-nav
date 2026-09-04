#!/usr/bin/env python3
"""Plot isolated Hybrid A* benchmark path CSV artifacts."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def load_path(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return [(float(row["x"]), float(row["y"]), float(row["yaw"]))
            for row in rows]


def draw_obstacles(axis, case_name):
    if case_name == "obstacle_inside_turn":
        axis.add_patch(Rectangle((0.35, 0.55), 1.85, 0.60,
                                 color="#333333", alpha=0.8))
    elif case_name == "obstacle_wall_gap":
        axis.add_patch(Rectangle((-0.15, -9.9), 0.30, 12.20,
                                 color="#333333", alpha=0.8))
    elif case_name == "obstacle_reverse_corridor":
        axis.axhspan(0.75, 10.0, color="#333333", alpha=0.8)
        axis.axhspan(-10.0, -0.75, color="#333333", alpha=0.8)


def draw_path(axis, samples, label, color):
    x = [sample[0] for sample in samples]
    y = [sample[1] for sample in samples]
    axis.plot(x, y, "o-", color=color, linewidth=2.0,
              markersize=2.5, label=label)
    arrow_step = max(1, len(samples) // 8)
    for px, py, yaw in samples[::arrow_step]:
        axis.arrow(px, py, 0.20 * math.cos(yaw), 0.20 * math.sin(yaw),
                   color=color, width=0.008, head_width=0.08,
                   length_includes_head=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", required=True)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--candidate-label", default="optimized")
    parser.add_argument("--baseline-label", default="c872762 baseline")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    figure, axis = plt.subplots(figsize=(7.2, 6.0), constrained_layout=True)
    draw_obstacles(axis, args.case)
    if args.baseline and args.baseline.exists():
        draw_path(axis, load_path(args.baseline), args.baseline_label,
                  "#777777")
    candidate = load_path(args.candidate)
    draw_path(axis, candidate, args.candidate_label, "#d62728")
    axis.scatter([candidate[0][0]], [candidate[0][1]], marker="s",
                 s=70, color="#2ca02c", label="start", zorder=5)
    axis.scatter([candidate[-1][0]], [candidate[-1][1]], marker="*",
                 s=110, color="#1f77b4", label="goal", zorder=5)
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_title(args.case)
    axis.legend(loc="best")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)


if __name__ == "__main__":
    main()
