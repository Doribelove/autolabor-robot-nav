#!/usr/bin/env python3
"""Run one R6-I1 episode with the byte-frozen R4-R1 evaluator."""

import argparse
import importlib.util
import os
from pathlib import Path
import tempfile

import rospy
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I1"
SOURCE = Path(__file__).with_name(
    "v2_04g_r4_r1_mechanism_episode.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r4_r1_episode_for_r6_i1", SOURCE
)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)


def _atomic_yaml(path, value):
    target = Path(path)
    payload = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


class R6I1MechanismEpisode(_BASE.R4R1MechanismEpisode):
    """Change only evidence identity after the frozen evaluator runs."""

    def __init__(self, *args, attempt, **kwargs):
        self.r6_attempt = attempt
        super().__init__(*args, **kwargs)

    def run(self):
        evaluation = super().run()
        identity = {
            "stage": STAGE,
            "profile_id": self.profile_id,
            "scene_id": evaluation["scene_id"],
            "seed": int(evaluation["seed"]),
            "attempt": self.r6_attempt,
        }
        evaluation.update(identity)
        audit = evaluation.get("clearance_audit")
        if not isinstance(audit, dict):
            raise RuntimeError("R6-I1 clearance audit is missing")
        audit.update(identity)
        _atomic_yaml(self.output_dir / "clearance_audit.yaml", audit)
        _atomic_yaml(self.output_dir / "evaluation.yaml", evaluation)
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument(
        "--method",
        choices=_BASE._R4._R3._R2._R1._LEGACY.METHODS,
        required=True,
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--split", choices=("calibration",), required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to(
        (WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1").resolve()
    )
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("v2_04g_r6_i1_mechanism_episode")
    report = R6I1MechanismEpisode(
        instance,
        args.method,
        output,
        args.split,
        args.profile_id,
        attempt=args.attempt,
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
