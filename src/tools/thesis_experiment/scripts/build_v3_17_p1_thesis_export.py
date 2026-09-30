#!/usr/bin/env python3
"""Build and seal the pure-offline V3-17-P1 thesis export metadata."""
import argparse
import csv
import hashlib
import io
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/paper/v3_17_p1_thesis_export")
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
TEX = Path("docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex")
SUMMARY = ROOT / "v3_17_p1_result_summary.csv"
CLAIMS = ROOT / "v3_17_p1_claims.yaml"
README = ROOT / "README.md"
MANIFEST = ROOT / "v3_17_p1_evidence_manifest.sha256"

V311 = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml")
V311_REVIEW = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/independent_result_review.yaml")
V314_SUMMARY = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/evidence_summary.yaml")
V314_REVIEW = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/independent_result_review.yaml")
V316_D1 = Path("artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution/attribution_report.yaml")
V316_SUMMARY = Path("artifacts/v3/multiactor/v3_16_s1_response_gated_support/evidence_summary.yaml")
V316_TERMINAL = Path("artifacts/v3/multiactor/v3_16_s1_response_gated_support/terminal_result.yaml")
V316_ASSESSMENT = Path("artifacts/v3/multiactor/v3_16_s1_response_gated_support/assessment_r1.yaml")
V316_REVIEW = Path("artifacts/v3/multiactor/v3_16_s1_response_gated_support/independent_result_review_r1.yaml")
BUILDER = Path("src/tools/thesis_experiment/scripts/build_v3_17_p1_thesis_export.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_17_p1_thesis_export.py")

EXPECTED = {
    V311: "9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9",
    V311_REVIEW: "2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39",
    V314_SUMMARY: "d6ffde4c658170ccf6ed992bd4131ee28f72a81882d64f73b41c38038f53e8d4",
    V314_REVIEW: "4773d76437a01148984770ca502dce96cfa6d3bc2743029e0bb7a1f10f94c60a",
    V316_D1: "1f84987b14a9a8721eea5823e965d7c0283f0a4250f7efb7137cf060b08e7396",
    V316_SUMMARY: "088483a89952f157b51f5411a96e270f3d639c214a244d1d6fbdbc4b9e116590",
    V316_TERMINAL: "2a855255d4f38051bca43f3a3d0ae3c2cbfb43970718395f2d8d82ae25d7dbce",
    V316_ASSESSMENT: "6a30f54abea88949722cf10e58e4da59aaa11a631f9eece2ee373a9f766ebaf5",
    V316_REVIEW: "c4469571a1a8f8e999e733138d42466fa0c0a6bd472bcebe5d044fe512a4765c",
}


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def write_or_check(path, payload, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != payload:
            raise RuntimeError("{} differs".format(path))
        return
    if target.exists():
        raise RuntimeError("{} exists".format(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def csv_bytes(rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(("study", "metric", "unit", "value", "gate", "status"))
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def outputs():
    if not (W / PDF).is_file() or not read(PDF).startswith(b"%PDF"):
        raise RuntimeError("compiled thesis PDF absent")
    for path, expected in EXPECTED.items():
        if sha(read(path)) != expected:
            raise RuntimeError("frozen evidence drift: {}".format(path))

    v311 = yaml.safe_load(read(V311))
    v314 = yaml.safe_load(read(V314_SUMMARY))
    v316 = yaml.safe_load(read(V316_SUMMARY))
    terminal = yaml.safe_load(read(V316_TERMINAL))
    assessment = yaml.safe_load(read(V316_ASSESSMENT))
    if (
        not v311["performance_improvement_demonstrated"]
        or terminal["evidence_collection_gate_pass"]
        or assessment["predictor_fit_authorized"]
    ):
        raise RuntimeError("claim state differs")

    metrics = v311["metrics"]
    old_speed = float(v314["maximum_decision_actor_speed_range_mps"])
    new_speed = float(v316["maximum_decision_actor_speed_range_mps"])
    speed_reduction = 100.0 * (1.0 - new_speed / old_speed)
    coordinate_pass_blocks = sum(
        max(actor["coordinate_range_m"] for actor in block["actor_ranges"]) <= 0.05
        for block in v316["blocks"]
    )
    rows = [
        ("V3-11-F1", "completed_episodes", "count", 60, 60, "pass"),
        ("V3-11-F1", "paired_mean_reduction", "percent", "{:.4f}".format(metrics["point_reduction_percent"]), ">=10", "pass"),
        ("V3-11-F1", "one_sided_97_5pct_lower", "percent", "{:.4f}".format(metrics["bootstrap"]["one_sided_lower_reduction_percent"]), ">=5", "pass"),
        ("V3-14-S1", "max_actor_coordinate_range", "m", "{:.9f}".format(v314["maximum_decision_actor_coordinate_range_m"]), "<=0.05", "fail"),
        ("V3-14-S1", "max_actor_speed_range", "m/s", "{:.9f}".format(old_speed), "<=0.05", "fail"),
        ("V3-16-S1", "completed_units", "count", 45, 45, "pass"),
        ("V3-16-S1", "scientific_episodes", "count", 40, 40, "pass"),
        ("V3-16-S1", "max_actor_coordinate_range", "m", "{:.9f}".format(v316["maximum_decision_actor_coordinate_range_m"]), "<=0.05", "fail"),
        ("V3-16-S1", "max_actor_speed_range", "m/s", "{:.9f}".format(new_speed), "<=0.05", "pass"),
        ("V3-16-S1", "speed_mismatch_reduction_vs_v3_14", "percent", "{:.4f}".format(speed_reduction), "mechanism_only", "qualified"),
        ("V3-16-S1", "coordinate_gate_pass_blocks", "count", coordinate_pass_blocks, 8, "fail"),
        ("V3-16-S1", "evidence_collection_gate", "boolean", "false", "true", "fail"),
    ]
    summary = csv_bytes(rows)
    claims = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-17-P1",
        "status": "terminal_pure_offline_thesis_export",
        "qualified_primary_claim": {
            "study": "V3-11-F1",
            "comparison": "v3_spatial_safe_vs_actual_TebLocalPlannerROS_0_8_4",
            "population": "three_frozen_single_reactive_actor_Gazebo_templates_cautious_hesitant_shared_oracle_track",
            "paired_blocks": 30,
            "episodes": 60,
            "point_reduction_percent": 12.9288,
            "one_sided_97_5pct_lower_percent": 12.7253,
            "goal_success_both_percent": 100.0,
            "contact_state_count": 0,
        },
        "qualified_mechanism_claim": {
            "study": "V3-16-S1",
            "factor": "reactive_actor_response_enable_sim_s_0_to_12_55",
            "maximum_speed_mismatch_before_mps": old_speed,
            "maximum_speed_mismatch_after_mps": new_speed,
            "speed_mismatch_reduction_percent": speed_reduction,
            "scope": "common_prefix_speed_mechanism_only",
        },
        "terminal_extension_claim": {
            "study": "V3-16-S1",
            "status": "multi_actor_support_not_demonstrated",
            "completed_units": 45,
            "scientific_episodes": 40,
            "maximum_actor_coordinate_range_m": float(v316["maximum_decision_actor_coordinate_range_m"]),
            "maximum_actor_speed_range_mps": new_speed,
            "coordinate_gate_pass_blocks": coordinate_pass_blocks,
            "coordinate_gate_total_blocks": 8,
            "coordinate_gate_m": 0.05,
            "speed_gate_mps": 0.05,
            "predictor_fit_authorized": False,
        },
        "forbidden_claims": [
            "perception_robustness",
            "multi_actor_performance",
            "OOD_generalization",
            "formal_safety",
            "learning_benefit",
            "deployment_readiness",
            "real_vehicle_performance",
            "all_TEB_versions_superiority",
        ],
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    claims_payload = yaml.safe_dump(claims, sort_keys=False, allow_unicode=True).encode("utf-8")
    readme = (
        "# V3-17-P1 thesis export\n\n"
        "Terminal pure-offline 25-page A4 export with the V3-16 single-factor study.\n\n"
        "- Primary claim remains the scoped V3-11 actual-TEB result.\n"
        "- V3-16 qualifies only an 88.07% speed-mismatch mechanism reduction.\n"
        "- Multi-actor support remains not demonstrated because coordinate matching failed.\n"
        "- No training, hardware, CAN/serial, online TEB writes or real vehicle were used.\n"
    ).encode("utf-8")
    generated = {SUMMARY: summary, CLAIMS: claims_payload, README: readme}
    resources = [
        TEX, PDF, V311, V311_REVIEW, V314_SUMMARY, V314_REVIEW, V316_D1, V316_SUMMARY,
        V316_TERMINAL, V316_ASSESSMENT, V316_REVIEW, BUILDER, REVIEWER,
        Path("docs/thesis_experiment/thesis_draft/CH07_V3_SCOPE_FORMAL_RESULTS.md"),
        Path("docs/thesis_experiment/thesis_draft/CLAIMS_EVIDENCE_MATRIX.md"),
        Path("docs/thesis_experiment/CURRENT_V3_16_RESPONSE_GATED_TERMINAL_HANDOFF.md"),
    ]
    manifest = "".join("{}  {}\n".format(sha(read(path)), path) for path in resources)
    manifest += "".join("{}  {}\n".format(sha(payload), path) for path, payload in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    values = outputs()
    for path, payload in values.items():
        write_or_check(path, payload, args.check)
    print(
        "status=terminal_pure_offline_thesis_export pdf_sha256={} manifest_sha256={}".format(
            sha(read(PDF)), sha(values[MANIFEST])
        )
    )


if __name__ == "__main__":
    main()

