#!/usr/bin/env python3
"""Run a single no-ROS V3-SCOPE planner probe from YAML inputs."""

import argparse
from pathlib import Path

import yaml

from v3_scope_planner.core import plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--contract", required=True)
    parser.add_argument("--model", required=True)
    args = parser.parse_args()
    snapshot = yaml.safe_load(Path(args.snapshot).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(args.contract).read_text(encoding="utf-8"))
    model = yaml.safe_load(Path(args.model).read_text(encoding="utf-8"))
    print(yaml.safe_dump(plan(snapshot, model, contract), sort_keys=False))


if __name__ == "__main__":
    main()
