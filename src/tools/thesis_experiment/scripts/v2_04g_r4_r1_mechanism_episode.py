#!/usr/bin/env python3
"""Run one R4-R1 episode with byte-frozen R4 evaluator semantics."""

import argparse
import importlib.util
from pathlib import Path

import rospy
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R4_RUNNER = Path(__file__).with_name("v2_04g_r4_mechanism_episode.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_frozen_episode_r4_r1", R4_RUNNER)
_R4 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R4)


class R4R1MechanismEpisode(_R4.R4MechanismEpisode):
    def run(self):
        evaluation = super().run()
        evaluation["stage"] = "V2-04G-R4-R1"
        audit = evaluation.get("clearance_audit")
        if isinstance(audit, dict):
            audit["stage"] = "V2-04G-R4-R1"
            (self.output_dir / "clearance_audit.yaml").write_text(
                yaml.safe_dump(audit, sort_keys=False), encoding="utf-8")
        (self.output_dir / "evaluation.yaml").write_text(
            yaml.safe_dump(evaluation, sort_keys=False), encoding="utf-8")
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--method", choices=_R4._R3._R2._R1._LEGACY.METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=("V2-04G-R4-R1",), required=True)
    parser.add_argument("--split", choices=("calibration",), required=True)
    parser.add_argument("--profile-id", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to((WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1").resolve())
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("v2_04g_r4_r1_mechanism_episode")
    report = R4R1MechanismEpisode(
        instance, args.method, output, args.split, args.profile_id).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
