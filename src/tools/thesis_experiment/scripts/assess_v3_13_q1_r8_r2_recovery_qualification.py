#!/usr/bin/env python3
"""Assess fresh R8-R2 expanded-predictor qualification evidence."""

import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r8_recovery_qualification.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_assessor", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
BASE.RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_r2_recovery_qualification.py"
)
BASE.CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r2_fresh_runner_recovery_contract.yaml"
)
BASE.READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r8_r2_readiness"
)
BASE.EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r2_recovery_qualification"
)
BASE.ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r8_r2_assessment"
)
BASE.ASSESSMENT = BASE.ROOT / "predictor_qualification.yaml"
BASE.FREEZE = BASE.ROOT / "evidence_freeze.yaml"
BASE.ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r8_r2_recovery_qualification.py"
)
BASE.REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r8_r2_qualification_result.py"
)


if __name__ == "__main__":
    BASE.main()
