#!/usr/bin/env python3
"""Run R8 semantics on fresh R8-R2 identities using the original runner."""

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
R6 = BASE.R6
BASE.CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r2_fresh_runner_recovery_contract.yaml"
)
BASE.READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r8_r2_readiness"
)
BASE.OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r2_recovery_qualification"
)


if __name__ == "__main__":
    raise SystemExit(BASE.main())
