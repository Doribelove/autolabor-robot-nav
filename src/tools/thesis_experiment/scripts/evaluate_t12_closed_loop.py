#!/usr/bin/env python3
"""Evaluate the paired T12 no-training Gazebo study."""

from pathlib import Path
import sys

import yaml

from thesis_experiment.run_artifacts import write_checksums
from thesis_experiment.t12_closed_loop import evaluate_closed_loop


ROOT = Path("/home/robot/robot_ws_base_rl/artifacts/t12/closed_loop")


def main() -> int:
    report = evaluate_closed_loop(ROOT)
    output = ROOT / "t12_closed_loop_report.yaml"
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    files = [output]
    for path in sorted((ROOT / "runs").glob("t12_*_seed*/checksums.sha256")):
        files.append(path)
    write_checksums(ROOT / "checksums.sha256", files, ROOT)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

