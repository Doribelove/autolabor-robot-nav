#!/usr/bin/env python3
"""Run one V2-04G-R5 episode with the frozen R4-R1 evaluator semantics."""

import argparse
import importlib.util
from pathlib import Path

import rospy
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
R4_R1_RUNNER = Path(__file__).with_name("v2_04g_r4_r1_mechanism_episode.py")
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r4_r1_frozen_episode_for_r5", R4_R1_RUNNER
)
_R4_R1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R4_R1)


def _atomic_yaml(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


class R5MechanismEpisode(_R4_R1.R4R1MechanismEpisode):
    """Change only evidence identity after the frozen evaluator has run."""

    def run(self):
        evaluation = super().run()
        evaluation["stage"] = STAGE
        audit = evaluation.get("clearance_audit")
        if isinstance(audit, dict):
            audit["stage"] = STAGE
            _atomic_yaml(self.output_dir / "clearance_audit.yaml", audit)
        _atomic_yaml(self.output_dir / "evaluation.yaml", evaluation)
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument(
        "--method",
        choices=_R4_R1._R4._R3._R2._R1._LEGACY.METHODS,
        required=True,
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--split", choices=("calibration",), required=True)
    parser.add_argument("--profile-id", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to(
        (WORKSPACE / "artifacts/v2/calibration/v2_04g_r5").resolve()
    )
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("v2_04g_r5_mechanism_episode")
    report = R5MechanismEpisode(
        instance, args.method, output, args.split, args.profile_id
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
