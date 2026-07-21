#!/usr/bin/env python3
"""Validate V2-04C and materialize its immutable qualification/refinement plans."""

import argparse
from pathlib import Path

from thesis_experiment.v2_04c import build_v2_04c_plans, write_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/robot/robot_ws_base_rl"))
    args = parser.parse_args()
    root = args.workspace.resolve()
    plans = build_v2_04c_plans(
        root / "config/thesis_experiments/v2/v2_04c_refinement_contract.yaml", root
    )
    output = root / "artifacts/v2/calibration/v2_04c"
    q_path = output / "v2_04c_ttc_qualification_plan.yaml"
    r_path = output / "v2_04c_refinement_plan.yaml"
    write_plan(plans["qualification"], q_path)
    write_plan(plans["refinement"], r_path)
    print(q_path)
    print(r_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
