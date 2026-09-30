#!/usr/bin/env python3
"""Build or materialize the deterministic pure-offline V3-00 bundle.

There is deliberately no execute, ROS, Gazebo, seed-allocation or training
mode.  Materialization refuses to overwrite an existing artifact root.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _load_foundation():
    source = (
        WORKSPACE
        / "src/tools/thesis_experiment/src/thesis_experiment/v3_foundation.py"
    )
    spec = importlib.util.spec_from_file_location("_v3_foundation_builder", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load {}".format(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


foundation = _load_foundation()


def _materialize(bundle):
    output = WORKSPACE / foundation.OUTPUT_ROOT
    foundation.require(not output.exists(), "V3-00 output root already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=".v3_00_build_", dir=str(output.parent))
    )
    old_umask = os.umask(0o077)
    try:
        for relative, payload in bundle.items():
            local = relative.relative_to(foundation.OUTPUT_ROOT)
            target = temporary / local
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
        os.replace(str(temporary), str(output))
    except Exception:
        shutil.rmtree(str(temporary), ignore_errors=True)
        raise
    finally:
        os.umask(old_umask)
    foundation.validate_persisted_bundle(WORKSPACE, bundle)


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--materialize", action="store_true")
    args = parser.parse_args()

    first = foundation.build_bundle(WORKSPACE)
    second = foundation.build_bundle(WORKSPACE)
    foundation.require(first == second, "V3-00 builder is not byte deterministic")
    if args.materialize:
        _materialize(first)
    print(
        json.dumps(
            {
                "status": "v3_00_pure_offline_bundle_valid",
                "materialized": bool(args.materialize),
                "output_count": len(first),
                "outputs": {
                    relative.as_posix(): foundation.sha256_bytes(payload)
                    for relative, payload in sorted(
                        first.items(), key=lambda item: item[0].as_posix()
                    )
                },
                "execution_authorized": False,
                "training_authorized": False,
                "evidence_budget_authorized": 0,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
