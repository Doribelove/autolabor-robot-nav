#!/usr/bin/env python3
"""Pure-offline attribution of the V3-14 common-prefix actor mismatch."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path("config/thesis_experiments/v3/v3_16_d1_actor_prefix_attribution_contract.yaml")
SOURCE = Path("src/tools/thesis_experiment/scripts/diagnose_v3_16_d1_actor_prefix.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_16_d1_actor_prefix.py")
PLUGIN = Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp")
RENDERER = Path("src/tools/thesis_experiment/scripts/v3_13_multiactor_design.py")
EVIDENCE_ROOT = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support")
SUMMARY = EVIDENCE_ROOT / "evidence_summary.yaml"
ASSESSMENT = EVIDENCE_ROOT / "assessment.yaml"
REVIEW = EVIDENCE_ROOT / "independent_result_review.yaml"
ROOT = Path("artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution")
REPORT = ROOT / "attribution_report.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def read(path): return (W / path).read_bytes()
def sha(payload): return hashlib.sha256(payload).hexdigest()
def ybytes(value): return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != payload: raise RuntimeError("{} differs".format(path))
        return
    if target.exists(): raise RuntimeError("{} exists".format(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def ranges(receipts, section, field):
    rows = []
    for block in range(1, 9):
        group = [r for r in receipts if int(r["row"]["block_index"]) == block]
        tracks = [r["episode_report"][section][field] for r in group]
        coordinates = []; speeds = []
        for track_id in (1, 2):
            selected = [next(x for x in value if int(x["track_id"]) == track_id) for value in tracks]
            coordinates.append(max(float(x["coordinate_m"]) for x in selected) - min(float(x["coordinate_m"]) for x in selected))
            speeds.append(max(float(x["coordinate_speed_mps"]) for x in selected) - min(float(x["coordinate_speed_mps"]) for x in selected))
        rows.append({"block_index": block, "maximum_coordinate_range_m": max(coordinates), "maximum_speed_range_mps": max(speeds)})
    return rows


def build():
    contract = yaml.safe_load(read(CONTRACT)); summary = yaml.safe_load(read(SUMMARY))
    expected = contract["upstream"]
    for path, key in ((SUMMARY, "v3_14_s1_evidence_summary_sha256"), (ASSESSMENT, "v3_14_s1_assessment_sha256"), (REVIEW, "v3_14_s1_review_sha256")):
        if sha(read(path)) != expected[key]: raise RuntimeError("upstream drift: {}".format(path))
    receipts = []
    for path in sorted((W / EVIDENCE_ROOT / "episodes").glob("*/matrix_receipt.yaml")):
        receipt = yaml.safe_load(path.read_bytes())
        if receipt["row"]["phase"] == "evidence": receipts.append(receipt)
    if len(receipts) != 40: raise RuntimeError("evidence receipt count")
    gate = ranges(receipts, "warmup", "gate_actor_tracks")
    branch = ranges(receipts, "common_stop_prefix", "decision_actor_tracks")
    blocks = []
    for left, right in zip(gate, branch):
        blocks.append({"block_index": left["block_index"], "gate_coordinate_range_m": left["maximum_coordinate_range_m"], "gate_speed_range_mps": left["maximum_speed_range_mps"], "branch_coordinate_range_m": right["maximum_coordinate_range_m"], "branch_speed_range_mps": right["maximum_speed_range_mps"], "branch_coordinate_gate_pass": right["maximum_coordinate_range_m"] <= 0.05, "branch_speed_gate_pass": right["maximum_speed_range_mps"] <= 0.05})
    plugin_text = read(PLUGIN).decode("utf-8"); renderer_text = read(RENDERER).decode("utf-8")
    max_gate_coordinate = max(x["gate_coordinate_range_m"] for x in blocks)
    max_gate_speed = max(x["gate_speed_range_mps"] for x in blocks)
    max_branch_coordinate = max(x["branch_coordinate_range_m"] for x in blocks)
    max_branch_speed = max(x["branch_speed_range_mps"] for x in blocks)
    checks = {
        "all_40_receipts_have_action_binding": all(r["scientific_inclusion_binding_pass"] for r in receipts),
        "ego_pairing_gate_passed": summary["gate_checks"]["paired_ego_x"],
        "gate_actor_coordinate_below_0_01_m": max_gate_coordinate < 0.01,
        "branch_actor_coordinate_gate_failed": not summary["gate_checks"]["paired_actor_coordinate"],
        "branch_actor_speed_gate_failed": not summary["gate_checks"]["paired_actor_speed"],
        "renderer_enables_actor_before_branch": '"start_delay": "5.0"' in renderer_text and '"activation_ego_x": "-3.0"' in renderer_text,
        "plugin_response_has_no_branch_gate": "response_enable_time" not in plugin_text,
        "execution_authority_zero": not contract["authority"]["simulation_execution_authorized"],
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-16-D1",
        "status": "v3_16_d1_prebranch_response_amplification_attribution_pass" if all(checks.values()) else "v3_16_d1_attribution_fail",
        "receipt_count": len(receipts), "max_gate_actor_coordinate_range_m": max_gate_coordinate,
        "max_gate_actor_speed_range_mps": max_gate_speed, "max_branch_actor_coordinate_range_m": max_branch_coordinate,
        "max_branch_actor_speed_range_mps": max_branch_speed, "coordinate_amplification_ratio": max_branch_coordinate / max_gate_coordinate,
        "blocks": blocks, "failed_coordinate_blocks": [x["block_index"] for x in blocks if not x["branch_coordinate_gate_pass"]],
        "failed_speed_blocks": [x["block_index"] for x in blocks if not x["branch_speed_gate_pass"]],
        "supported_hypothesis": "H3_prebranch_reactive_response_amplified_small_gate_differences",
        "rejected_hypotheses": ["H1_action_or_ego_mismatch", "H2_material_actor_mismatch_already_at_gate"],
        "preregistered_next_single_factor": {"name": "postbranch_response_enable_gate", "response_enable_sim_s": 12.55, "branch_target_sim_s": 12.5},
        "checks": checks, "execution_authorized": False, "training_authorized": False, "hardware_accessed": False, "real_vehicle_used": False,
    }
    resources = [CONTRACT, SOURCE, REVIEWER, PLUGIN, RENDERER, SUMMARY, ASSESSMENT, REVIEW]
    dependencies = {"schema_version": "1.0", "stage": "V3-16-D1", "resource_count": len(resources), "resources": [{"path": str(p), "sha256": sha(read(p))} for p in resources]}
    values = {REPORT: ybytes(report), DEPENDENCIES: ybytes(dependencies)}
    manifest = "".join("{}  {}\n".format(sha(read(p)), p) for p in resources) + "".join("{}  {}\n".format(sha(v), p) for p, v in values.items())
    values[MANIFEST] = manifest.encode("utf-8")
    return values


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    values = build()
    for path, payload in values.items(): write_or_check(path, payload, args.check)
    report = yaml.safe_load(values[REPORT]); print("status={} report_sha256={} manifest_sha256={}".format(report["status"], sha(values[REPORT]), sha(values[MANIFEST])))


if __name__ == "__main__": main()

