#!/usr/bin/env python3
"""Persist the mechanically discovered R6-I1 execution dependency closure."""

import argparse
import os
from pathlib import Path
import tempfile

import yaml

from thesis_experiment.v2_04g_r6_i1_dependency import (
    build_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml"
)


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    root = args.workspace.resolve()
    output = args.output.resolve()
    if output != (root / OUTPUT.relative_to(WORKSPACE)).resolve():
        parser.error("output path must be the canonical R6-I1 closure")
    output.parent.mkdir(parents=True, exist_ok=True)
    document = build_dependency_closure(root)
    _atomic_yaml(output, document)
    print(yaml.safe_dump(document, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
