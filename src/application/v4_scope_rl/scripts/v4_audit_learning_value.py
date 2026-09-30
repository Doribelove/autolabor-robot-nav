#!/usr/bin/env python3
"""Execute the preregistered V4-02-F1 no-training learning-value audit."""

import argparse
import sys

from v4_scope_rl.io_utils import atomic_yaml, sha256_file
from v4_scope_rl.learning_audit import audit_response_value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--factor", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    import yaml

    with open(args.factor, "r", encoding="utf-8") as stream:
        factor = yaml.safe_load(stream)
    if factor.get("stage") != "V4-02-F1":
        raise RuntimeError("FACTOR_STAGE_MISMATCH")
    first, last = factor["identity_separation"]["f1_learning_audit_seed_range_inclusive"]
    candidate = factor["single_factor"]["candidate"]
    audit = audit_response_value(
        range(int(first), int(last) + 1),
        {
            "cautious_yield_memory_s": float(candidate["cautious_yield_memory_s"]),
            "hesitant_yield_memory_s": float(candidate["hesitant_yield_memory_s"]),
        },
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v4",
        "stage": "V4-02-F1",
        "status": "learning_value_audit_complete",
        "training_steps": 0,
        "simulation_only": True,
        "hardware_accessed": False,
        "factor": {"path": args.factor, "sha256": sha256_file(args.factor)},
        "seed_range_inclusive": [int(first), int(last)],
        **audit,
    }
    report_sha256 = atomic_yaml(args.output, report)
    print("{} {} gate={}".format(args.output, report_sha256, audit["summary"]["audit_gate_pass"]))
    return 0 if audit["summary"]["audit_gate_pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
