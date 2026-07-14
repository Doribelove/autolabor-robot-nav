#!/usr/bin/env python3
"""Evaluate one V2 episode trace against a compiled scene instance."""

import argparse
import json
import os
from pathlib import Path

import yaml

from thesis_experiment.v2_contract import load_yaml, validate_evaluation_contract
from thesis_experiment.v2_evaluator import (
    V2EvaluationError,
    evaluate_v2_episode,
    load_v2_trace,
    trace_sha256,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instance")
    parser.add_argument("trace")
    parser.add_argument("output")
    parser.add_argument(
        "--contract",
        default="config/thesis_experiments/v2/evaluation_contract.yaml",
    )
    parser.add_argument("--workspace", default="/home/robot/robot_ws_base_rl")
    args = parser.parse_args()
    root = Path(args.workspace).resolve()
    output = Path(args.output).resolve()
    try:
        output.relative_to((root / "artifacts" / "v2").resolve())
    except ValueError:
        parser.error("output must remain under artifacts/v2")
    try:
        validate_evaluation_contract(load_yaml(root / args.contract))
        instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
        rows = load_v2_trace(args.trace)
        report = evaluate_v2_episode(instance, rows, trace_sha256(args.trace))
    except (OSError, yaml.YAMLError, V2EvaluationError, ValueError) as exc:
        print(json.dumps({"status": "invalid", "error": str(exc)}, ensure_ascii=False))
        return 2
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    os.replace(str(temporary), str(output))
    print(json.dumps({"status": "valid", "termination": report["termination"],
                      "output": str(output)}, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
