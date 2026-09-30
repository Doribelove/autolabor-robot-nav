#!/usr/bin/env python3
"""Capture and verify a future P2-Q1 fixed-TEB readback boundary."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

from dynamic_reconfigure.client import Client
import rospy
import yaml


STAGE = "V2-04G-P2-Q1"
NAMESPACE = "/move_base/TebLocalPlannerROS"
METHODS = (
    "fixed_teb_v120__TEB-Default_alias",
    "fixed_teb_v125",
    "fixed_teb_v130",
    "fixed_teb_v135",
    "fixed_teb_v140",
    "TEB-Tuned-V1",
)


def _atomic_yaml(path, value):
    path = Path(path)
    payload = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


def _normalized(raw):
    values = {}
    for key, value in raw.items():
        if key == "groups":
            continue
        if type(value) not in (bool, int, float, str):
            raise RuntimeError(
                "unsupported dynamic-reconfigure scalar: {}".format(key)
            )
        if isinstance(value, float) and not math.isfinite(value):
            raise RuntimeError(
                "non-finite dynamic-reconfigure scalar: {}".format(key)
            )
        values[str(key)] = value
    return values


def _snapshot(timeout_s):
    client = Client(NAMESPACE, timeout=timeout_s)
    try:
        raw = client.get_configuration(timeout=timeout_s)
    finally:
        client.close()
    if not raw:
        raise RuntimeError("fixed-TEB readback is empty")
    values = _normalized(raw)
    payload = json.dumps(
        values, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return values, hashlib.sha256(payload).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("capture", "verify"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--profile-id", choices=METHODS, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--seed", type=int, choices=range(5301, 5331), required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04g_p2_q1_fixed_snapshot", anonymous=True)
    values, digest = _snapshot(args.timeout_s)
    baseline_digest = None
    unchanged = None
    if args.mode == "verify":
        if args.baseline is None:
            parser.error("--baseline is required for verify")
        baseline = yaml.safe_load(args.baseline.read_text(encoding="utf-8"))
        baseline_digest = baseline.get("configuration_sha256")
        unchanged = digest == baseline_digest
    document = {
        "schema_version": "1.0",
        "stage": STAGE,
        "record_type": "fixed_teb_{}_readback".format(args.mode),
        "profile_id": args.profile_id,
        "scene_id": args.scene_id,
        "seed": args.seed,
        "attempt": args.attempt,
        "teb_namespace": NAMESPACE,
        "configuration": values,
        "configuration_sha256": digest,
        "baseline_configuration_sha256": baseline_digest,
        "no_write_boundary_unchanged": unchanged,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    _atomic_yaml(args.output, document)
    if unchanged is False:
        raise RuntimeError("fixed-TEB configuration changed during episode")
    print(yaml.safe_dump(document, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
