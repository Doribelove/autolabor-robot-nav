#!/usr/bin/env python3
"""Generate the evaluator-only V2-03 synthetic acceptance artifact."""

import argparse
import os
from pathlib import Path
import sys

import yaml

from thesis_experiment import load_v2_yaml, validate_world_model_contract
from thesis_experiment.v2_03_acceptance import evaluate_v2_03_synthetic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contract")
    parser.add_argument("world_config")
    parser.add_argument("supervisor_config")
    parser.add_argument("output")
    parser.add_argument("--workspace", default="/home/robot/robot_ws_base_rl")
    args = parser.parse_args()
    contract = load_v2_yaml(args.contract)
    validate_world_model_contract(contract, workspace=args.workspace, verify_profiles=True)
    world = load_v2_yaml(args.world_config)
    supervisor = load_v2_yaml(args.supervisor_config)
    report = evaluate_v2_03_synthetic(world, supervisor, contract)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    os.replace(str(temporary), str(output))
    print(yaml.safe_dump({"output": str(output.resolve()), "passed": report["passed"]}, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
