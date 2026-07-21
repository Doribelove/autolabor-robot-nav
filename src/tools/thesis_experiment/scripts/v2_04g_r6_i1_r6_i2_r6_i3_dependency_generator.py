#!/usr/bin/env python3
"""Persist the offline-only R6-I3 execution-readiness dependency closure."""

import argparse
import os
from pathlib import Path
import sys
import tempfile

import yaml


sys.dont_write_bytecode = True

from thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_dependency import (
    EXECUTION_CLOSURE,
    build_dependency_closure,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT = WORKSPACE / EXECUTION_CLOSURE


def _atomic_yaml(path: Path, value: object) -> None:
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
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
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    root = args.workspace.resolve()
    output = args.output.resolve()
    canonical_output = (root / EXECUTION_CLOSURE).resolve()
    if output != canonical_output:
        parser.error("output must be the canonical R6-I3 closure path")
    if not output.parent.is_dir():
        parser.error("static R6-I3 readiness artifact root is missing")
    document = build_dependency_closure(root)
    _atomic_yaml(output, document)
    print(yaml.safe_dump(document, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
