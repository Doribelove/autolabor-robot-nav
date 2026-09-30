#!/usr/bin/env python3
"""Capture and verify the no-write fixed-TEB dynamic-reconfigure boundary."""

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


STAGE = "V2-04G-P1-E1"
NAMESPACE = "/move_base/TebLocalPlannerROS"


def _atomic_yaml(path, value):
    payload = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _normalized(raw):
    values = {}
    for key, value in raw.items():
        if key == "groups":
            continue
        if type(value) not in (bool, int, float, str):
            raise RuntimeError("unsupported dynamic-reconfigure scalar: {}".format(key))
        if isinstance(value, float) and not math.isfinite(value):
            raise RuntimeError("non-finite dynamic-reconfigure scalar: {}".format(key))
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
    parser.add_argument("--profile-id", choices=("fixed_teb_reference",), required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04g_p1_e1_fixed_snapshot", anonymous=True)
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
