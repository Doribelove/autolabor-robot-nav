#!/usr/bin/env python3
"""R2 single-factor candidate-coverage benchmark episode.

The R1 implementation is imported without modification.  The only behavioral
factor is the target speed of actions 4 and 5; all verifier and execution
thresholds remain inherited byte-for-byte from R1.
"""

from __future__ import annotations

import os
import sys
import importlib.util
from pathlib import Path

import numpy as np
import rospy
import yaml


def load_r1_source():
    workspace = Path(os.environ["THESIS_WS"]).resolve()
    source = workspace / (
        "src/application/v5_e2e_scope_rl/scripts/"
        "pct_teb_dwa_benchmark_episode.py"
    )
    specification = importlib.util.spec_from_file_location(
        "pct_teb_dwa_benchmark_episode_r1_source", str(source)
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("R1_EPISODE_SOURCE_IMPORT_SPEC_INVALID")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


r1 = load_r1_source()


VARIANTS = {
    "r1_control_v040": (0.4, "spatial_left_v040", "spatial_right_v040"),
    "r2_candidate_v060": (0.6, "spatial_left_v060", "spatial_right_v060"),
}


def apply_candidate_variant(runtime, variant):
    if variant not in VARIANTS:
        raise RuntimeError("UNKNOWN_R2_CANDIDATE_VARIANT:" + str(variant))
    speed, left_name, right_name = VARIANTS[variant]
    target_speeds = np.asarray(runtime.target_speeds, dtype=np.float64).copy()
    target_speeds[4:6] = speed
    action_names = list(runtime.action_names)
    action_names[4:6] = [left_name, right_name]
    runtime.target_speeds = target_speeds
    runtime.action_names = tuple(action_names)
    return target_speeds.tolist()


class R2BenchmarkEpisode(r1.BenchmarkEpisode):
    def __init__(self):
        super().__init__()
        self.candidate_variant = str(
            rospy.get_param("~candidate_variant", "r2_candidate_v060")
        )
        self.candidate_target_speeds_mps = apply_candidate_variant(
            self.runtime, self.candidate_variant
        )

    def run(self):
        report = super().run()
        report["candidate_contract_variant"] = self.candidate_variant
        report["candidate_target_speeds_mps"] = self.candidate_target_speeds_mps
        report["r2_single_behavioral_factor"] = (
            "actions_4_5_target_speed_0.4_to_0.6_mps"
            if self.candidate_variant == "r2_candidate_v060"
            else "none_r1_control"
        )
        temporary = self.report_path + ".r2.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("pct_teb_dwa_benchmark_r2_episode")
    try:
        R2BenchmarkEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr("R2 benchmark episode failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
