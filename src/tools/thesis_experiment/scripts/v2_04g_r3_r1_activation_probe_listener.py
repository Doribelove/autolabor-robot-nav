#!/usr/bin/env python3
"""R3-R1 atomic-input readiness listener over the frozen fault taxonomy."""

import importlib.util
from pathlib import Path
import sys

import yaml


STAGE = "V2-04G-R3-R1"
SOURCE = Path(__file__).with_name("v2_04g_r2_r1_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r2_r1_frozen_taxonomy_for_r3_r1", SOURCE
)
_FROZEN = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_FROZEN)
_FROZEN.STAGE = STAGE


def _argument_path(name):
    try:
        return Path(sys.argv[sys.argv.index(name) + 1])
    except (ValueError, IndexError):
        raise ValueError("{} is required".format(name))


def add_atomic_input_gate(report):
    mismatch_count = 0
    input_join_fault_count = 0
    for sample in report.get("fault_samples", []):
        reason = str((sample.get("latest_context") or {}).get("reason", ""))
        mismatch_count += int(reason == "world_model_sequence_mismatch")
        input_join_fault_count += int(reason.startswith("world_model_input_join_"))
    report["world_model_sequence_mismatch_count"] = mismatch_count
    report["world_model_input_join_fault_count"] = input_join_fault_count
    report["hard_gates"]["atomic_world_model_input_alignment"] = (
        mismatch_count == 0 and input_join_fault_count == 0
    )
    report["all_hard_gates_pass"] = all(report["hard_gates"].values())
    report["status"] = "pass" if report["all_hard_gates_pass"] else "fail"
    return report


def main():
    frozen_return = _FROZEN.main()
    output = _argument_path("--output")
    report = yaml.safe_load(output.read_text(encoding="utf-8"))
    report = add_atomic_input_gate(report)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(output)
    return 0 if frozen_return == 0 and report["all_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
