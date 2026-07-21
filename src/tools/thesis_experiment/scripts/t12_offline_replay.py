#!/usr/bin/env python3
"""Run T12 read-only telemetry replay and write an auditable bundle."""

import csv
from pathlib import Path
import sys

import yaml

from thesis_experiment.run_artifacts import write_checksums
from thesis_experiment.t12_replay import evaluate_replay


ROOT = Path("/home/robot/robot_ws_base_rl")


def main() -> int:
    config = ROOT / "config/thesis_experiments/t12_shadow.yaml"
    output = ROOT / "artifacts/t12/offline_replay"
    output.mkdir(parents=True, exist_ok=True)
    report, decisions = evaluate_replay(config, ROOT)
    decision_path = output / "shadow_decisions.csv"
    with decision_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(decisions[0].keys()))
        writer.writeheader()
        writer.writerows(decisions)
    report_path = output / "t12_replay_report.yaml"
    report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    write_checksums(
        output / "checksums.sha256", (report_path, decision_path), base_dir=output)
    print(yaml.safe_dump({
        "passed": report["passed"], "episode_count": report["episode_count"],
        "step_row_count": report["step_row_count"],
        "counterfactual_false_stop_fraction": report["counterfactual_false_stop_fraction"],
        "report": str(report_path),
    }, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())

