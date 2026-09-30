#!/usr/bin/env python3
"""Compile and seal the terminal V3-22-P2 thesis PDF export."""

import argparse
import csv
import hashlib
import io
import os
from pathlib import Path
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/paper/v3_22_p2_thesis_export")
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
LOG = ROOT / "FAM_TEB_V3_THESIS_DRAFT.log"
TEX = Path("docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex")
SUMMARY = ROOT / "v3_22_p2_result_summary.csv"
CLAIMS = ROOT / "v3_22_p2_claims.yaml"
ENVIRONMENT = ROOT / "build_environment.yaml"
README = ROOT / "README.md"
MANIFEST = ROOT / "v3_22_p2_evidence_manifest.sha256"
BUILDER = Path("src/tools/thesis_experiment/scripts/build_v3_22_p2_thesis_export.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_22_p2_thesis_export.py")

V311 = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml")
V311_REVIEW = Path("artifacts/v3/actual_teb/v3_11_f1_assessment/independent_result_review.yaml")
V321 = Path("artifacts/v3/thesis/v3_21_f1_execution/formal_characterization_assessment.yaml")
V321_REVIEW = Path("artifacts/v3/thesis/v3_21_f1_execution/independent_result_review.yaml")
MATERIALS_MANIFEST = Path("artifacts/v3/thesis/v3_22_p1_paper_materials/paper_materials_manifest.sha256")
MATERIALS_REVIEW = Path("artifacts/v3/thesis/v3_22_p1_paper_materials/independent_paper_materials_review.yaml")


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def atomic(path, payload, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != payload:
            raise RuntimeError("{} differs".format(path))
        return
    if target.exists():
        raise RuntimeError("refuse overwrite: {}".format(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def compile_pdf():
    output = W / ROOT
    output.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    environment.update({"SOURCE_DATE_EPOCH": "1785974400", "FORCE_SOURCE_DATE": "1", "TZ": "UTC"})
    command = [
        "latexmk", "-xelatex", "-interaction=nonstopmode", "-halt-on-error",
        "-outdir={}".format(output), TEX.name,
    ]
    subprocess.run(command, cwd=str(W / TEX.parent), env=environment, check=True)
    if not (W / PDF).is_file() or not read(PDF).startswith(b"%PDF"):
        raise RuntimeError("compiled PDF absent")


def csv_payload(rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(("study", "metric", "unit", "value", "status"))
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def derived_outputs():
    if not (W / PDF).is_file() or not (W / LOG).is_file():
        raise RuntimeError("compile outputs absent")
    v311 = yaml.safe_load(read(V311)); v321 = yaml.safe_load(read(V321))
    if not v311["performance_improvement_demonstrated"]:
        raise RuntimeError("historical V3-11 claim differs")
    if v321["formal_result"] or v321["performance_improvement_vs_actual_teb_demonstrated"]:
        raise RuntimeError("fresh V3-21 claim differs")
    rows = [
        ("V3-18-B0", "historical_canonical_episodes", "count", 825, "qualified_data_adapter"),
        ("V3-21-F1", "episodes", "count", 120, "complete"),
        ("V3-21-F1", "paired_blocks", "count", 30, "complete"),
        ("V3-21-F1", "v3_goal_success", "percent", "93.3333", "descriptive"),
        ("V3-21-F1", "actual_teb_goal_success", "percent", "100.0000", "descriptive"),
        ("V3-21-F1", "v3_vs_actual_teb_reduction", "percent", "-0.1931", "not_demonstrated"),
        ("V3-21-F1", "v3_vs_actual_teb_two_sided_ci", "percent", "[-19.8668,12.9820]", "crosses_zero"),
        ("V3-21-F1", "v3_vs_fixed_v120_reduction", "percent", "64.6136", "exploratory"),
        ("V3-21-F1", "v3_vs_no_conditioning_reduction", "percent", "49.4535", "exploratory_mechanism"),
        ("V3-21-F1", "protocol_deviations", "count", 1, "retained_ITT_formal_result_false"),
        ("V3-22-P1", "publication_figures", "count", 5, "reviewed"),
        ("V3-22-P1", "three_line_tables", "count", 5, "reviewed"),
    ]
    summary = csv_payload(rows)
    claims = {
        "schema_version": "1.0", "stage": "V3-22-P2",
        "status": "terminal_thesis_pdf_export",
        "paper_title_zh": "面向 Ackermann 移动机器人的候选条件交互预测与安全空间轨迹选择",
        "historical_qualified_result": {
            "stage": "V3-11-F1", "comparison": "v3_spatial_safe_vs_actual_TebLocalPlannerROS_0_8_4",
            "point_reduction_percent": 12.9288, "one_sided_97_5_lower_percent": 12.7253,
            "goal_success_both_percent": 100.0,
            "scope": "three_frozen_single_reactive_actor_Gazebo_templates_shared_oracle_track",
        },
        "fresh_characterization": {
            "stage": "V3-21-F1", "episodes": 120, "paired_blocks": 30,
            "formal_result": False, "characterization_result_available": True,
            "actual_teb_improvement_demonstrated": False,
            "point_reduction_percent": -0.19305019305020038,
            "two_sided_95_interval_percent": [-19.8668384879725, 12.982005141388184],
            "protocol_deviation_count": 1,
        },
        "exploratory_ablation_result": {
            "comparison": "v3_spatial_safe_vs_v3_no_candidate_conditioning",
            "point_reduction_percent": 49.45352234606644,
            "two_sided_95_interval_percent": [37.00102816638785, 58.80024360535931],
        },
        "interpretation": "historical_scoped_positive_not_replicated_as_failure_inclusive_actual_teb_superiority",
        "forbidden_claims": [
            "general_V3_superiority_over_TEB", "perception_robustness", "multi_actor_or_OOD_generalization",
            "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance",
        ],
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    claims_payload = yaml.safe_dump(claims, sort_keys=False, allow_unicode=True).encode("utf-8")
    build_environment = {
        "schema_version": "1.0", "stage": "V3-22-P2", "build_mode": "pure_offline_xelatex",
        "source_date_epoch": 1785974400, "timezone": "UTC_during_build",
        "ros_distribution": "noetic", "roslaunch_version": "1.17.4", "gazebo_version": "11.15.1",
        "teb_local_planner_version": "0.8.4", "python_version": "3.8.10",
        "paper_size": "A4", "engine": "XeTeX_plus_xdvipdfmx", "hardware_accessed": False,
    }
    environment_payload = yaml.safe_dump(build_environment, sort_keys=False).encode("utf-8")
    readme = """# V3-22-P2 terminal thesis export

This is the terminal 32-page A4 XeLaTeX thesis technical draft.

- The historical V3-11 actual-TEB positive result remains separately qualified.
- The fresh V3-21 failure-inclusive study does not demonstrate superiority over actual TEB.
- V3-21 provides exploratory candidate-conditioning ablation evidence.
- Five publication figures and five LaTeX three-line tables are included.
- No training, hardware, CAN/serial, online parameter writes or real vehicle were used.

Use `independent_paper_review.yaml` and `terminal_freeze.sha256` for final verification.
""".encode("utf-8")
    output = {SUMMARY: summary, CLAIMS: claims_payload, ENVIRONMENT: environment_payload, README: readme}
    resources = [
        TEX, PDF, LOG, V311, V311_REVIEW, V321, V321_REVIEW, MATERIALS_MANIFEST, MATERIALS_REVIEW,
        BUILDER, REVIEWER, Path("docs/thesis_experiment/THESIS_EXPERIMENT_REPRODUCTION.md"),
        Path("docs/thesis_experiment/thesis_draft/CH07_V3_SCOPE_FORMAL_RESULTS.md"),
        Path("docs/thesis_experiment/thesis_draft/CH08_V3_FOUR_METHOD_REPLICATION_AND_ABLATION.md"),
        Path("docs/thesis_experiment/thesis_draft/CLAIMS_EVIDENCE_MATRIX.md"),
    ]
    manifest = "".join("{}  {}\n".format(sha(read(path)), path) for path in resources)
    manifest += "".join("{}  {}\n".format(sha(payload), path) for path, payload in sorted(output.items(), key=lambda item: str(item[0])))
    output[MANIFEST] = manifest.encode("utf-8")
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    if args.build == args.check:
        parser.error("choose exactly one of --build or --check")
    if args.build:
        if (W / PDF).exists():
            raise RuntimeError("terminal PDF already exists")
        compile_pdf()
    output = derived_outputs()
    for path, payload in output.items():
        atomic(path, payload, args.check)
    print("status=terminal_thesis_pdf_export pdf_sha256={} manifest_sha256={}".format(sha(read(PDF)), sha(output[MANIFEST])))


if __name__ == "__main__":
    main()

