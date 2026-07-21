#!/usr/bin/env python3
"""Validate thesis YAML/CSV configuration files without starting ROS."""

import argparse
import json
import sys

from teb_rl_tuner import (
    ConfigValidationError,
    load_yaml_mapping,
    validate_a_teb,
    validate_experiment_contract,
    validate_runtime_config,
)
from thesis_experiment import (
    SchemaValidationError,
    V2ContractError,
    load_metric_schema,
    validate_action_pipeline_contract,
    validate_architecture_contract,
    validate_evaluation_contract,
    validate_mode_thresholds,
    validate_parameter_registry,
    validate_simulation_contract,
    validate_run_manifest,
    validate_state_contract,
    validate_v1_baseline_snapshot,
    validate_world_model_contract,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "kind",
        choices=(
            "contract",
            "runtime-config",
            "a-teb",
            "run-manifest",
            "metric-schema",
            "v2-architecture",
            "v2-parameter-registry",
            "v2-mode-thresholds",
            "v2-state-contract",
            "v2-simulation-contract",
            "v2-evaluation-contract",
            "v2-world-model-contract",
            "v2-action-pipeline-contract",
            "v1-baseline-snapshot",
        ),
    )
    parser.add_argument("path")
    parser.add_argument("--require-frozen", action="store_true")
    parser.add_argument("--runtime-ready", action="store_true")
    parser.add_argument("--allow-placeholders", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        if args.kind == "metric-schema":
            fields = load_metric_schema(args.path)
            result = {"kind": args.kind, "field_count": len(fields), "status": "valid"}
        else:
            data = load_yaml_mapping(args.path)
            if args.kind == "contract":
                validate_experiment_contract(data)
            elif args.kind == "runtime-config":
                validate_runtime_config(data)
            elif args.kind == "a-teb":
                validate_a_teb(data, require_frozen=args.require_frozen)
            elif args.kind == "run-manifest":
                validate_run_manifest(data, allow_placeholders=args.allow_placeholders)
            elif args.kind == "v2-architecture":
                validate_architecture_contract(data)
            elif args.kind == "v2-parameter-registry":
                validate_parameter_registry(data, require_runtime_ready=args.runtime_ready)
            elif args.kind == "v2-mode-thresholds":
                validate_mode_thresholds(data, require_runtime_ready=args.runtime_ready)
            elif args.kind == "v2-state-contract":
                validate_state_contract(data, require_runtime_ready=args.runtime_ready)
            elif args.kind == "v2-simulation-contract":
                validate_simulation_contract(data)
            elif args.kind == "v2-evaluation-contract":
                validate_evaluation_contract(data)
            elif args.kind == "v2-world-model-contract":
                contract_path = __import__("pathlib").Path(args.path).resolve()
                validate_world_model_contract(
                    data, workspace=contract_path.parents[3], verify_profiles=True
                )
            elif args.kind == "v2-action-pipeline-contract":
                contract_path = __import__("pathlib").Path(args.path).resolve()
                validate_action_pipeline_contract(
                    data, workspace=contract_path.parents[3], verify_profiles=True
                )
            elif args.kind == "v1-baseline-snapshot":
                validate_v1_baseline_snapshot(
                    data, "/home/robot/robot_ws_base_rl", verify_evidence=True
                )
            result = {"kind": args.kind, "status": "valid"}
    except (ConfigValidationError, SchemaValidationError, V2ContractError) as exc:
        print(json.dumps({"kind": args.kind, "status": "invalid", "error": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
