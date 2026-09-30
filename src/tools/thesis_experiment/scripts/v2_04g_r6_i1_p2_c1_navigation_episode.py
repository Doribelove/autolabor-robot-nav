#!/usr/bin/env python3
"""Bind the frozen Q1 navigation trace adapter to P2-C1 evidence identity."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import rospy
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_p2_q1_navigation_episode.py")
_SPEC = importlib.util.spec_from_file_location("p2_c1_frozen_q1_navigation", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--instance-sha256", required=True)
    parser.add_argument("--method-id", choices=_BASE.METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--sequence", type=int, choices=range(13, 343), required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to(
        (
            WORKSPACE
            / "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
        ).resolve()
    )
    instance_path = Path(args.instance).resolve()
    payload = instance_path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != args.instance_sha256:
        raise RuntimeError("P2-C1 source instance SHA256 differs")
    instance = yaml.safe_load(payload)
    if (
        instance.get("scene", {}).get("split") != "calibration"
        or instance["scene"].get("seed") not in range(5301, 5331)
    ):
        raise RuntimeError("P2-C1 navigation instance identity differs")
    semantic = args.method_id in _BASE.CIRCLE_METHODS
    p1_alias = (
        "r6_semantics_circle_contact" if semantic else "fixed_teb_reference"
    )
    method = "rule_multi_anchor" if semantic else "fixed_teb"
    source_binding = {
        "path": str(instance_path.relative_to(WORKSPACE)),
        "sha256": digest,
    }
    rospy.init_node("v2_04g_p2_c1_navigation_episode")
    report = _BASE.Q1NavigationEpisode(
        instance,
        method,
        output,
        "calibration",
        p1_alias,
        attempt=args.attempt,
        source_binding=source_binding,
        method_id=p1_alias,
        actual_method_id=args.method_id,
        sequence=args.sequence,
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
