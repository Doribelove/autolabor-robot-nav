#!/usr/bin/env python3
"""Generate the machine-readable T09/T10 fairness acceptance report."""

import argparse
from pathlib import Path
import sys

import yaml

from thesis_experiment.sac_pair_evaluator import evaluate_sac_pair


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", default=str(WORKSPACE / "artifacts/t10/paired_sac_acceptance.yaml")
    )
    args = parser.parse_args()
    report = evaluate_sac_pair(
        WORKSPACE / "config/thesis_experiments/t09_sac.yaml",
        WORKSPACE / "config/thesis_experiments/t10_direct_theta_sac.yaml",
        WORKSPACE / "artifacts/t09/gazebo_sac_smoke/t09_gazebo_sac_smoke.yaml",
        WORKSPACE / "artifacts/t10/gazebo_sac_smoke/t10_gazebo_sac_smoke.yaml",
    )
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump({"passed": report["passed"], "report": str(destination)},
                         sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
