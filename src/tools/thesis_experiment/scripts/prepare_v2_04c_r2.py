#!/usr/bin/env python3
"""Validate the timeout-only V2-04C qualification retry and write its plan."""

import argparse
from pathlib import Path

from thesis_experiment.v2_04c import build_v2_04c_r2_plan, write_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/robot/robot_ws_base_rl"))
    args = parser.parse_args()
    root = args.workspace.resolve()
    amendment = (
        root / "config/thesis_experiments/v2/v2_04c_ttc_qualification_r2_amendment.yaml"
    )
    output = root / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification_r2_plan.yaml"
    write_plan(build_v2_04c_r2_plan(amendment, root), output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
