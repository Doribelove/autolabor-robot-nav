#!/usr/bin/env python3
"""Validate and summarize the complete frozen T11 matrix."""

from pathlib import Path
import sys

import yaml

from thesis_experiment.t11_evaluator import evaluate_t11


ROOT = Path("/home/robot/robot_ws_base_rl")


def main():
    amendment = ROOT / "experiments/manifests/t11/budget_amendment.yaml"
    report = evaluate_t11(
        ROOT,
        training_seeds=(101, 102, 103, 104),
        amendment_path=amendment,
    )
    output = ROOT / "artifacts/t11/evaluation/t11_evaluation_report.yaml"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({
        "passed": report["passed"], "run_count": report["run_count"],
        "evaluation_episode_count": report["evaluation_episode_count"],
        "report": str(output),
    }, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
