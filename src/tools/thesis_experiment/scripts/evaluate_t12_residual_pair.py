#!/usr/bin/env python3
"""Write the frozen T12 three-method residual paired evaluation report."""

from pathlib import Path
import sys

import yaml

from thesis_experiment.run_artifacts import write_checksums
from thesis_experiment.t12_residual_pair import evaluate


ROOT = Path("/home/robot/robot_ws_base_rl/artifacts/t12/residual_paired_eval")


def main() -> int:
    report = evaluate(ROOT)
    output = ROOT / "t12_residual_paired_eval_report.yaml"
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    files = [output]
    files.extend(sorted((ROOT / "runs").glob("t12e_*_seed*/checksums.sha256")))
    write_checksums(ROOT / "checksums.sha256", files, ROOT)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["integrity_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
