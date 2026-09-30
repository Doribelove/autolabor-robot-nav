#!/usr/bin/env python3
"""Run V3-13-Q1-R1 with a closed thesis-workspace ROS overlay."""

import importlib.util
import os
from pathlib import Path
import sys


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_actual_teb_matrix.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_terminal_runner", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)

CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r1_overlay_repair_contract.yaml"
)
BASE.CONTRACT = CONTRACT
BASE.PHASES = {
    "preflight": {
        "readiness": W
        / "artifacts/v3/multiactor/v3_13_q1_r1_preflight_readiness",
        "output": W
        / "artifacts/v3/multiactor/v3_13_q1_r1_preflight",
        "units": 2,
    },
    "qualification": {
        "readiness": W
        / "artifacts/v3/multiactor/v3_13_q1_r1_readiness",
        "output": W
        / "artifacts/v3/multiactor/v3_13_q1_r1_execution",
        "units": 32,
    },
}
ORIGINAL_CLEAN_ENVIRONMENT = BASE.common.clean_environment


def closed_overlay_environment(runtime_home, ros_port):
    environment = ORIGINAL_CLEAN_ENVIRONMENT(runtime_home, ros_port)
    environment.update(
        {
            "ROS_PACKAGE_PATH": (
                "/home/robot/robot_ws_base_rl/src:"
                "/opt/ros/noetic/share"
            ),
            "CMAKE_PREFIX_PATH": (
                "/home/robot/robot_ws_base_rl/devel:"
                "/opt/ros/noetic"
            ),
            "PYTHONPATH": (
                "/home/robot/robot_ws_base_rl/devel/"
                "lib/python3/dist-packages:"
                "/opt/ros/noetic/lib/python3/dist-packages"
            ),
            "LD_LIBRARY_PATH": (
                "/home/robot/robot_ws_base_rl/devel/lib:"
                "/opt/ros/noetic/lib:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu"
            ),
        }
    )
    return environment


BASE.common.clean_environment = closed_overlay_environment
ORIGINAL_RUN_EPISODE = BASE.run_episode


def run_episode(row, scene, root, ros_port, stage):
    receipt = ORIGINAL_RUN_EPISODE(
        row, scene, root, ros_port, stage
    )
    receipt["workspace_overlay_explicit"] = True
    receipt["workspace_overlay_root"] = str(W)
    receipt["foreign_workspace_precedence_used"] = False
    BASE.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


BASE.run_episode = run_episode


if __name__ == "__main__":
    sys.exit(BASE.main())
