#!/usr/bin/env python3
"""Build or verify the pure-offline V2-04G-P2-D2 design closure."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import tempfile


WORKSPACE_DEFAULT = Path("/home/robot/robot_ws_base_rl")
MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d2.py"
)
SOURCE_FREEZE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design/"
    "p2_d2_source_evidence_freeze.sha256"
)


def load_module(workspace):
    path = workspace / MODULE_RELATIVE
    spec = importlib.util.spec_from_file_location("v2_04g_r6_i1_p2_d2_runtime", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load P2-D2 design module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".{}-".format(path.name), dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def source_freeze_bytes(module, workspace, source_entries):
    contract_payload = module.stable_read(workspace, module.CONTRACT_RELATIVE)
    entries = [
        {
            "path": module.CONTRACT_RELATIVE.as_posix(),
            "sha256": module.sha256_bytes(contract_payload),
        }
    ]
    entries.extend(source_entries)
    by_path = {}
    for entry in entries:
        previous = by_path.get(entry["path"])
        if previous is not None and previous != entry["sha256"]:
            raise module.DesignError(
                "source freeze SHA conflict: {}".format(entry["path"])
            )
        by_path[entry["path"]] = entry["sha256"]
    return "".join(
        "{}  {}\n".format(by_path[path], path) for path in sorted(by_path)
    ).encode("utf-8")


def require_equal(module, actual, expected, label):
    if actual != expected:
        raise module.DesignError(
            "{} is not byte-equal to deterministic rebuild".format(label)
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE_DEFAULT)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    arguments = parser.parse_args()

    workspace = arguments.workspace.resolve()
    module = load_module(workspace)
    report, amendment, source_entries = module.build_outputs(workspace)
    amendment_payload = module.amendment_bytes(amendment)
    report_payload = module.report_bytes(report)
    source_freeze_payload = source_freeze_bytes(module, workspace, source_entries)
    outputs = (
        (module.AMENDMENT_RELATIVE, amendment_payload, "candidate-bank amendment"),
        (module.REPORT_RELATIVE, report_payload, "design report"),
        (SOURCE_FREEZE_RELATIVE, source_freeze_payload, "source evidence freeze"),
    )
    if arguments.write:
        for relative, payload, _ in outputs:
            atomic_write(workspace / relative, payload)
    else:
        for relative, payload, label in outputs:
            require_equal(
                module, module.stable_read(workspace, relative), payload, label
            )

    print(report["status"])
    print("source_paths={}".format(len(source_entries) + 1))
    print("amendment_sha256={}".format(hashlib.sha256(amendment_payload).hexdigest()))
    print("report_sha256={}".format(hashlib.sha256(report_payload).hexdigest()))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print("p2_d2_offline_designer_failed: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
