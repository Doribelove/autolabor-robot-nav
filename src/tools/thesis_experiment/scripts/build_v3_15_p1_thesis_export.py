#!/usr/bin/env python3
"""Build and seal pure-offline V3-15-P1 thesis export metadata."""

import argparse
import csv
import hashlib
import io
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/paper/v3_15_p1_thesis_export")
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
TEX = Path("docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex")
SUMMARY = ROOT / "v3_15_p1_result_summary.csv"
CLAIMS = ROOT / "v3_15_p1_claims.yaml"
README = ROOT / "README.md"
MANIFEST = ROOT / "v3_15_p1_evidence_manifest.sha256"

V311 = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml")
V311_REVIEW = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/independent_result_review.yaml")
V312_MANIFEST = Path("artifacts/v3/actual_teb/v3_12_p1_paper_closure/v3_12_p1_actual_teb_evidence_manifest.sha256")
V314_SUMMARY = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/evidence_summary.yaml")
V314_TERMINAL = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/terminal_result.yaml")
V314_ASSESSMENT = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/assessment.yaml")
V314_REVIEW = Path("artifacts/v3/multiactor/v3_14_s1_multiaction_support/independent_result_review.yaml")
BUILDER = Path("src/tools/thesis_experiment/scripts/build_v3_15_p1_thesis_export.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_15_p1_thesis_export.py")

EXPECTED = {
    V311: "9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9",
    V311_REVIEW: "2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39",
    V312_MANIFEST: "319bce3401859e25fa1e5da68d97669a6aebf593688c475003a8844b91798c0b",
    V314_SUMMARY: "d6ffde4c658170ccf6ed992bd4131ee28f72a81882d64f73b41c38038f53e8d4",
    V314_ASSESSMENT: "58c472560699e10276f76c063dd243ab5666f8e534346c51e88aa9f2b49958b9",
    V314_REVIEW: "4773d76437a01148984770ca502dce96cfa6d3bc2743029e0bb7a1f10f94c60a",
}


def read(path): return (W / path).read_bytes()
def sha(payload): return hashlib.sha256(payload).hexdigest()
def ybytes(value): return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def csv_bytes(rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(("study", "metric", "unit", "value", "gate", "status"))
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


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
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def outputs():
    if not (W / PDF).is_file() or not read(PDF).startswith(b"%PDF"):
        raise RuntimeError("compiled thesis PDF absent")
    for path, expected in EXPECTED.items():
        if sha(read(path)) != expected:
            raise RuntimeError("frozen evidence drift: {}".format(path))
    v311 = yaml.safe_load(read(V311)); metrics = v311["metrics"]
    v314 = yaml.safe_load(read(V314_SUMMARY)); terminal = yaml.safe_load(read(V314_TERMINAL))
    if not v311["performance_improvement_demonstrated"] or terminal["evidence_collection_gate_pass"]:
        raise RuntimeError("terminal claim state differs")
    rows = [
        ("V3-11-F1", "completed_episodes", "count", 60, 60, "pass"),
        ("V3-11-F1", "paired_blocks", "count", 30, 30, "pass"),
        ("V3-11-F1", "v3_mean_restricted_gate_to_goal", "s", "{:.4f}".format(metrics["mean_restricted_elapsed_s_by_method"]["v3_spatial_safe"]), "lower_than_baseline", "pass"),
        ("V3-11-F1", "actual_teb_mean_restricted_gate_to_goal", "s", "{:.4f}".format(metrics["mean_restricted_elapsed_s_by_method"]["actual_teb"]), "baseline", "context"),
        ("V3-11-F1", "paired_mean_reduction", "percent", "{:.4f}".format(metrics["point_reduction_percent"]), ">=10", "pass"),
        ("V3-11-F1", "one_sided_97_5pct_lower", "percent", "{:.4f}".format(metrics["bootstrap"]["one_sided_lower_reduction_percent"]), ">=5", "pass"),
        ("V3-11-F1", "goal_success_both_methods", "percent", "100.0000", "100", "pass"),
        ("V3-11-F1", "contact_state_count", "count", 0, 0, "pass"),
        ("V3-14-S1", "completed_units", "count", 45, 45, "pass"),
        ("V3-14-S1", "scientific_episodes", "count", 40, 40, "pass"),
        ("V3-14-S1", "action_binding_rate", "proportion", "1.000000", "1.0", "pass"),
        ("V3-14-S1", "max_actor_coordinate_range", "m", "{:.9f}".format(v314["maximum_decision_actor_coordinate_range_m"]), "<=0.05", "fail"),
        ("V3-14-S1", "max_actor_speed_range", "m/s", "{:.9f}".format(v314["maximum_decision_actor_speed_range_mps"]), "<=0.05", "fail"),
        ("V3-14-S1", "evidence_collection_gate", "boolean", "false", "true", "fail"),
    ]
    summary = csv_bytes(rows)
    claims = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-15-P1",
        "status": "terminal_pure_offline_thesis_export",
        "paper_title_zh": "面向 Ackermann 移动机器人的候选条件交互预测与安全空间轨迹选择",
        "qualified_primary_claim": {
            "study": "V3-11-F1", "comparison": "v3_spatial_safe_vs_actual_TebLocalPlannerROS_0_8_4",
            "population": "three_frozen_single_reactive_actor_Gazebo_templates_cautious_hesitant_shared_oracle_track",
            "paired_blocks": 30, "episodes": 60, "point_reduction_percent": 12.9288,
            "one_sided_97_5pct_lower_percent": 12.7253, "goal_success_both_percent": 100.0,
            "contact_state_count": 0,
        },
        "terminal_extension_claim": {
            "study": "V3-14-S1", "status": "multi_actor_support_not_demonstrated",
            "completed_units": 45, "scientific_episodes": 40, "action_binding_rate": 1.0,
            "maximum_actor_coordinate_range_m": float(v314["maximum_decision_actor_coordinate_range_m"]),
            "maximum_actor_speed_range_mps": float(v314["maximum_decision_actor_speed_range_mps"]),
            "coordinate_gate_m": 0.05, "speed_gate_mps": 0.05, "predictor_fit_authorized": False,
        },
        "forbidden_claims": ["perception_robustness", "multi_actor_performance", "OOD_generalization", "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance", "all_TEB_versions_superiority"],
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    claims_payload = ybytes(claims)
    readme = """# V3-15-P1 thesis export\n\nTerminal pure-offline paper export dated 2026-08-06.\n\n- `FAM_TEB_V3_THESIS_DRAFT.pdf`: 23-page A4 thesis technical draft.\n- `v3_15_p1_result_summary.csv`: machine-readable primary positive and extension-negative results.\n- `v3_15_p1_claims.yaml`: qualified and forbidden claims.\n- `v3_15_p1_evidence_manifest.sha256`: conventional evidence closure.\n- `independent_paper_review.yaml`: independent PDF/claim/evidence review.\n\nNo hardware, real vehicle, CAN/serial, training, or online TEB writes were used.\n""".encode("utf-8")
    generated = {SUMMARY: summary, CLAIMS: claims_payload, README: readme}
    resources = [TEX, PDF, V311, V311_REVIEW, V312_MANIFEST, V314_SUMMARY, V314_TERMINAL, V314_ASSESSMENT, V314_REVIEW, BUILDER, REVIEWER,
                 Path("docs/thesis_experiment/thesis_draft/CH07_V3_SCOPE_FORMAL_RESULTS.md"), Path("docs/thesis_experiment/thesis_draft/CLAIMS_EVIDENCE_MATRIX.md")]
    manifest = "".join("{}  {}\n".format(sha(read(path)), path) for path in resources)
    manifest += "".join("{}  {}\n".format(sha(payload), path) for path, payload in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    values = outputs()
    for path, payload in values.items(): write_or_check(path, payload, args.check)
    print("status=terminal_pure_offline_thesis_export pdf_sha256={} manifest_sha256={}".format(sha(read(PDF)), sha(values[MANIFEST])))


if __name__ == "__main__": main()

