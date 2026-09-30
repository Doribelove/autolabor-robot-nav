#!/usr/bin/env python3
"""Run the fresh R14-R1 stage-identity repair qualification."""

import copy
import importlib.util
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/tools/thesis_experiment/scripts")
)
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r14_progress_qualification.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r14_frozen_runner", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
COMMON = BASE.COMMON
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_r1_progress_predictor_qualification_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_r1_progress_qualification"
)
STAGE = "V3-13-Q1-R14-R1"


def _rewrite_stage(path, extra=None):
    value = yaml.safe_load(COMMON.read_bytes(path))
    value["stage"] = STAGE
    if extra:
        value.update(extra)
    BASE.ORIGINAL_ATOMIC_YAML(path, value)
    return value


def run_episode(row, scene, root, ros_port):
    receipt = BASE.run_episode(row, scene, root, ros_port)
    _rewrite_stage(root / "actor_specs.yaml")
    report = _rewrite_stage(
        root / "episode_report.yaml",
        {
            "method": (
                "fresh_stage_identity_repair_five_action_"
                "progress_predictor_qualification_clone"
            ),
        },
    )
    receipt = copy.deepcopy(receipt)
    receipt["stage"] = STAGE
    receipt["episode_report"] = report
    receipt["episode_report_sha256"] = COMMON.sha(
        COMMON.read_bytes(root / "episode_report.yaml")
    )
    receipt["actor_specs_sha256"] = COMMON.sha(
        COMMON.read_bytes(root / "actor_specs.yaml")
    )
    BASE.ORIGINAL_ATOMIC_YAML(root / "matrix_receipt.yaml", receipt)
    return receipt


def summarize(receipts, contract_sha, contract):
    value = BASE.summarize(receipts, contract_sha, contract)
    value["stage"] = STAGE
    value["status"] = (
        "fresh_stage_repair_progress_qualification_collection_pass"
        if value["collection_gate_pass"]
        else "fresh_stage_repair_progress_qualification_collection_fail"
    )
    return value


class _AssessorCompatibility:
    summarize = staticmethod(summarize)


R6 = _AssessorCompatibility
BASE.CONTRACT = CONTRACT
BASE.READINESS = READINESS
BASE.OUTPUT = OUTPUT
BASE.R12.CONTRACT = CONTRACT
BASE.R12.READINESS = READINESS
BASE.R12.OUTPUT = OUTPUT
BASE.R12.run_episode = run_episode
BASE.R12.summarize = summarize


def stage_correcting_atomic_yaml(path, value):
    if (
        Path(path).name == "exact_schedule.yaml"
        and Path(path).parent.resolve() == OUTPUT.resolve()
    ):
        value = copy.deepcopy(value)
        value["stage"] = STAGE
    return BASE.ORIGINAL_ATOMIC_YAML(path, value)


COMMON.atomic_yaml = stage_correcting_atomic_yaml


if __name__ == "__main__":
    raise SystemExit(BASE.R12.main())
