#!/usr/bin/env python3
"""Build or verify the pure-offline V2-04G-P2-D1 attribution report."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import tempfile


WORKSPACE_DEFAULT = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE_DEFAULT
    / "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d1.py"
)
SPEC = importlib.util.spec_from_file_location("v2_04g_r6_i1_p2_d1_runtime", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("cannot load P2-D1 attribution module")
P2_D1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(P2_D1)

BLOCKS_RELATIVE = P2_D1.BLOCKS_RELATIVE
CONTRACT_RELATIVE = P2_D1.CONTRACT_RELATIVE
REPORT_RELATIVE = P2_D1.REPORT_RELATIVE
AttributionError = P2_D1.AttributionError
build_outputs = P2_D1.build_outputs
canonical_blocks_bytes = P2_D1.canonical_blocks_bytes
report_bytes = P2_D1.report_bytes
sha256_bytes = P2_D1.sha256_bytes
stable_read = P2_D1.stable_read


SOURCE_FREEZE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "p2_d1_source_evidence_freeze.sha256"
)


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".{}-".format(path.name), dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def source_freeze_bytes(workspace: Path, closure):
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    entries = [{
        "path": CONTRACT_RELATIVE.as_posix(),
        "sha256": sha256_bytes(contract_payload),
    }]
    entries.extend({"path": item["path"], "sha256": item["sha256"]} for item in closure)
    by_path = {}
    for item in entries:
        previous = by_path.get(item["path"])
        if previous is not None and previous != item["sha256"]:
            raise AttributionError("source freeze SHA conflict: {}".format(item["path"]))
        by_path[item["path"]] = item["sha256"]
    return "".join(
        "{}  {}\n".format(by_path[path], path) for path in sorted(by_path)
    ).encode("utf-8")


def require_equal(actual: bytes, expected: bytes, label: str) -> None:
    if actual != expected:
        raise AttributionError("{} is not byte-equal to deterministic rebuild".format(label))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE_DEFAULT)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true", help="write deterministic offline artifacts")
    mode.add_argument("--check-only", action="store_true", help="verify existing artifacts without writing")
    arguments = parser.parse_args()

    workspace = arguments.workspace.resolve()
    report, blocks, source_closure = build_outputs(workspace)
    block_payload = canonical_blocks_bytes(blocks)
    report_payload = report_bytes(report)
    freeze_payload = source_freeze_bytes(workspace, source_closure)

    outputs = (
        (BLOCKS_RELATIVE, block_payload, "block table"),
        (REPORT_RELATIVE, report_payload, "attribution report"),
        (SOURCE_FREEZE_RELATIVE, freeze_payload, "source evidence freeze"),
    )
    if arguments.write:
        for relative, payload, _ in outputs:
            atomic_write(workspace / relative, payload)
    else:
        for relative, payload, label in outputs:
            require_equal(stable_read(workspace, relative), payload, label)

    print(report["status"])
    print("blocks={}".format(len(blocks)))
    print("source_paths={}".format(len(source_closure) + 1))
    print("report_sha256={}".format(hashlib.sha256(report_payload).hexdigest()))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AttributionError, OSError, ValueError) as exc:
        print("p2_d1_trace_attribution_failed: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
