#!/usr/bin/env python3
"""Independent pure-offline review of V3-10-P1 paper exports and hash closure."""
import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER = W / "src/tools/thesis_experiment/scripts/build_v3_10_p1_paper_closure.py"
ROOT = Path("artifacts/v3/formal/v3_10_p1_paper_closure")
REVIEW = ROOT / "v3_10_p1_independent_closure_review.yaml"
TERMINAL = ROOT / "v3_10_p1_terminal_freeze.sha256"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read(relative):
    return (W / relative).read_bytes()


def load_builder():
    spec = importlib.util.spec_from_file_location("v3_10_p1_builder", BUILDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def manifest_entries(payload):
    entries = []
    require(b"\\n" not in payload, "manifest contains literal backslash-n separator")
    require(payload.endswith(b"\n"), "manifest lacks terminal newline")
    for line in payload.decode("utf-8").splitlines():
        digest, relative = line.split("  ", 1)
        require(len(digest) == 64, "invalid manifest digest")
        entries.append((digest, Path(relative)))
    return entries


def review_value(builder):
    outputs = builder.compute_outputs()
    for relative, payload in outputs.items():
        require(read(relative) == payload, "paper export differs: {}".format(relative))
    manifest_payload = read(builder.MANIFEST)
    entries = manifest_entries(manifest_payload)
    require(len(entries) == 9 + len(outputs), "manifest resource count differs")
    for digest, relative in entries:
        require(sha(read(relative)) == digest, "manifest hash drift: {}".format(relative))
    require(not (W / ROOT / "execution").exists(), "unexpected V3-10 execution root")
    require(not (W / "experiments/manifests/v3/v3_10_p1_execution_release.yaml").exists(), "unexpected V3-10 release")
    require(not (W / "experiments/manifests/v3/v3_10_p1_simulation_authorization.yaml").exists(), "unexpected V3-10 authorization")
    assessment = yaml.safe_load(read(builder.ASSESSMENT))
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-10-P1",
        "status": "v3_10_p1_offline_paper_evidence_closure_pass",
        "formal_source_stage": "V3-09-F1",
        "formal_result": True,
        "performance_improvement_demonstrated": True,
        "assessment_sha256": sha(read(builder.ASSESSMENT)),
        "independent_formal_review_sha256": sha(read(builder.REVIEW)),
        "paper_evidence_manifest_sha256": sha(manifest_payload),
        "paper_export_count": len(outputs),
        "manifest_resource_count": len(entries),
        "point_relative_reduction": assessment["primary_statistics"]["point_relative_reduction"],
        "one_sided_lower_relative_reduction": assessment["primary_statistics"]["one_sided_lower_relative_reduction"],
        "goal_success_rate_by_method": {
            name: value["goal_success_rate"]
            for name, value in assessment["pooled_metrics"]["method_metrics"].items()
        },
        "historical_manifest_bytes_preserved": True,
        "new_manifests_use_real_newlines": True,
        "execution_authorized": False,
        "training_authorized": False,
        "runtime_ready": False,
        "real_vehicle_ready": False,
        "actual_teb_comparison": False,
    }


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file() and path.read_bytes() == payload, "closure output differs: {}".format(relative))
    elif path.exists():
        require(path.read_bytes() == payload, "refusing overwrite: {}".format(relative))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    builder = load_builder()
    value = review_value(builder)
    review_payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
    write_or_check(REVIEW, review_payload, args.check)
    resources = [
        builder.MANIFEST,
        builder.SUMMARY,
        builder.POLICY,
        builder.BLOCK,
        builder.FIGURE,
        builder.CLAIMS,
        REVIEW,
    ]
    terminal_payload = "".join("{}  {}\n".format(sha(read(path)), path) for path in resources).encode("utf-8")
    require(b"\\n" not in terminal_payload, "terminal manifest serialization differs")
    write_or_check(TERMINAL, terminal_payload, args.check)
    print("status={} review_sha256={} terminal_sha256={}".format(
        value["status"], sha(review_payload), sha(terminal_payload)
    ))


if __name__ == "__main__":
    main()
