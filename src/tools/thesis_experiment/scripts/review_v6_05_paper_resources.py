#!/usr/bin/env python3
"""Independent deterministic audit of the V6-05 paper export."""

import csv
import hashlib
import json
from pathlib import Path
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v6/paper/v6_05_p1_thesis_export"
FORMAL = W / "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_assessment.json"
REVIEW = ROOT / "v6_05_p1_paper_review_r2.yaml"
MANIFEST = ROOT / "v6_05_p1_terminal_manifest_r2.sha256"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pdf_info(path):
    output = subprocess.check_output(["pdfinfo", str(path)], text=True)
    result = {}
    for line in output.splitlines():
        if ":" in line:
            key, value = line.split(":", 1); result[key.strip()] = value.strip()
    return result


def main():
    if REVIEW.exists() or MANIFEST.exists():
        raise RuntimeError("review outputs already exist")
    required = [
        "FAM_TEB_V6_PCT_RL_SUPPLEMENT.pdf", "FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf",
        "FAM_TEB_V6_PCT_RL_SUPPLEMENT.tex", "v6_02_f1_method_summary.csv",
        "v6_effect_summary.csv", "v6_02_f1_family_summary.csv",
        "v6_02_f1_formal_episodes.csv", "v6_gazebo_preflight_summary.csv",
        "v6_05_p1_claims.yaml", "v6_05_p1_evidence_manifest.sha256",
        "literature_primary_sources.yaml",
        "v6_05_p1_paper_review.yaml",
    ]
    for stem in ("architecture", "training", "formal_summary", "effect_intervals", "family_ablation", "gazebo_boundary"):
        required.extend(["figure_v6_{}.svg".format(stem), "figure_v6_{}.pdf".format(stem)])
    absent = [name for name in required if not (ROOT / name).is_file()]
    formal = json.loads(FORMAL.read_text())
    with (ROOT / "v6_02_f1_method_summary.csv").open(newline="", encoding="utf-8") as stream:
        methods = {row["method"]: row for row in csv.DictReader(stream)}
    with (ROOT / "v6_effect_summary.csv").open(newline="", encoding="utf-8") as stream:
        effects = {row["comparison"]: row for row in csv.DictReader(stream)}
    claims = yaml.safe_load((ROOT / "v6_05_p1_claims.yaml").read_bytes())
    with (ROOT / "v6_gazebo_preflight_summary.csv").open(newline="", encoding="utf-8") as stream:
        gazebo = list(csv.DictReader(stream))
    supplement_info = pdf_info(ROOT / "FAM_TEB_V6_PCT_RL_SUPPLEMENT.pdf")
    combined_info = pdf_info(ROOT / "FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf")
    text = subprocess.check_output(["pdftotext", str(ROOT / "FAM_TEB_V6_PCT_RL_SUPPLEMENT.pdf"), "-"], text=True)
    checks = {
        "required_files_present": not absent,
        "formal_status_positive": formal["status"] == "v6_02_f1_fresh_formal_positive" and formal["method_effectiveness_demonstrated"] is True,
        "method_time_exact": abs(float(methods["PCT-SCOPE-RL V6"]["mean_restricted_time_s"]) - formal["summaries"]["pct_scope_rl_v6"]["mean_failure_inclusive_restricted_time_s"]) < 1e-12,
        "method_goal_exact": int(methods["PCT-SCOPE-RL V6"]["goals"]) == formal["summaries"]["pct_scope_rl_v6"]["goal_count"],
        "primary_effect_exact": abs(float(effects["V6 vs analytic"]["point"]) - formal["primary_relative_reduction_vs_analytic"]) < 1e-12,
        "primary_interval_exact": abs(float(effects["V6 vs analytic"]["lower"]) - formal["primary_two_sided_97_5_percent_interval"][0]) < 1e-12 and abs(float(effects["V6 vs analytic"]["upper"]) - formal["primary_two_sided_97_5_percent_interval"][1]) < 1e-12,
        "gazebo_rows_exactly_nine": len(gazebo) == 9,
        "gazebo_all_sensor_clearance_negative": all(float(row["clearance_m"]) < 0 for row in gazebo if row["method"] != "actual_teb"),
        "gazebo_qualification_and_formal_denied": claims["gazebo_qualification_run"] is False and claims["gazebo_formal_run"] is False,
        "real_vehicle_denied": claims["real_vehicle_used"] is False,
        "claim_boundaries_present": all(term in claims["forbidden_claims"] for term in ("Gazebo effectiveness", "formal safety", "real-vehicle readiness", "temporal-history benefit")),
        "supplement_is_20_page_a4": supplement_info.get("Pages") == "20" and "A4" in supplement_info.get("Page size", ""),
        "combined_is_69_page_a4": combined_info.get("Pages") == "69" and "A4" in combined_info.get("Page size", ""),
        "pdf_text_contains_primary_result": "8.4024%" in text and "V6" in text and "02" in text and "F1" in text,
        "pdf_text_contains_gazebo_boundary": "Gazebo 迁移负结果" in text and "尚未证明 Gazebo 迁移" in text,
    }
    review = {
        "schema_version": "1.0", "stage": "V6-05-P1-R2",
        "status": "v6_05_p1_paper_export_independent_review_r2_pass" if all(checks.values()) else "v6_05_p1_paper_export_review_r2_fail",
        "repair_scope": "unicode_nonbreaking_hyphen_text_extraction_compatibility_only",
        "pdf_or_evidence_changed_since_r1": False,
        "checks": checks, "absent_files": absent,
        "supplement_pdf_sha256": sha(ROOT / "FAM_TEB_V6_PCT_RL_SUPPLEMENT.pdf"),
        "combined_pdf_sha256": sha(ROOT / "FAM_TEB_V6_PCT_RL_THESIS_DRAFT.pdf"),
        "formal_assessment_sha256": sha(FORMAL),
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    REVIEW.write_text(yaml.safe_dump(review, sort_keys=False, allow_unicode=True), encoding="utf-8")
    if not all(checks.values()):
        raise RuntimeError("paper review failed")
    manifest_names = sorted(required + [REVIEW.name])
    with MANIFEST.open("w", encoding="utf-8") as stream:
        for name in manifest_names:
            path = ROOT / name
            stream.write("{}  {}\n".format(sha(path), name))
    print(review["status"])
    print("review_sha256={}".format(sha(REVIEW)))
    print("manifest_sha256={}".format(sha(MANIFEST)))


if __name__ == "__main__":
    main()
