#!/usr/bin/env python3
"""Run R8 through the frozen, minimal pre-execution arity repair."""

import importlib.util
from pathlib import Path
import sys


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/tools/thesis_experiment/scripts")
)
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_recovery_qualification.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_runner", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
ORIGINAL_RUN_EPISODE = BASE.run_episode


def repaired_run_episode(row, duplicate_row, scene, root, ros_port):
    """Normalize the erroneous duplicate row argument, changing no semantics."""
    if row != duplicate_row:
        raise RuntimeError("duplicated row argument differs")
    return ORIGINAL_RUN_EPISODE(row, scene, root, ros_port)


BASE.run_episode = repaired_run_episode
BASE.CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r1_runner_repair_contract.yaml"
)
BASE.READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r8_r1_readiness"
)
BASE.OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r1_recovery_qualification"
)


if __name__ == "__main__":
    raise SystemExit(BASE.main())
