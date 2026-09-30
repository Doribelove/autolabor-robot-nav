#!/usr/bin/env python3
"""Independent offline review for the portable V3+V6 thesis transfer package."""

import csv
import hashlib
import json
from pathlib import Path
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package"
PDF = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.pdf"
TEX = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.tex"
LOG = ROOT / "FAM_TEB_CURRENT_INTEGRATED_TECHNICAL_DRAFT.log"
REVIEW = ROOT / "independent_transfer_package_review.yaml"
MANIFEST = ROOT / "terminal_content_manifest.sha256"
V3_PDF = W / "artifacts/v3/paper/v3_22_p2_thesis_export/FAM_TEB_V3_THESIS_DRAFT.pdf"
V3_REVIEW = W / "artifacts/v3/paper/v3_22_p2_thesis_export/independent_paper_review.yaml"
V6_FORMAL = W / "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_assessment.json"
V6_REVIEW = W / "artifacts/v6/paper/v6_05_p1_thesis_export/v6_05_p1_paper_review_r2.yaml"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pdf_info(path):
    output = subprocess.check_output(["pdfinfo", str(path)], text=True)
    info = {}
    for line in output.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            info[key.strip()] = value.strip()
    return info


def fonts_embedded(path):
    output = subprocess.check_output(["pdffonts", str(path)], text=True)
    rows = [line.split() for line in output.splitlines()[2:] if line.strip()]
    return bool(rows) and all(len(row) >= 5 and row[-5] == "yes" for row in rows), len(rows)


def main():
    if REVIEW.exists() or MANIFEST.exists():
        raise RuntimeError("refuse overwrite existing independent review")
    required = [
        "README_FIRST.md", "CURRENT_THESIS_INTEGRATION_GUIDE.md",
        "CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md", "CODEX_HANDOFF_MACHINE_CONTEXT.md",
        "DATA_DICTIONARY.md", "CURRENT_RESULT_SUMMARY.csv", "SOURCE_FILE_INDEX.csv",
        "combined_claims_evidence_matrix.yaml", "upstream_evidence_manifest.sha256",
        "build_environment.yaml", TEX.name, PDF.name, LOG.name,
        "chapters/CH09_PCT_SCOPE_RL_INTEGRATED.tex",
        "chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex",
        "data/v3/canonical_episodes.csv", "data/v5/formal_assessment.json",
        "data/v6/formal_assessment.json", "data/v6/v6_02_f1_formal_episodes.csv",
        "data/v6/v6_gazebo_preflight_summary.csv",
    ]
    absent = [name for name in required if not (ROOT / name).is_file()]
    formal = json.loads(V6_FORMAL.read_text())
    claims = yaml.safe_load((ROOT / "combined_claims_evidence_matrix.yaml").read_bytes())
    with (ROOT / "CURRENT_RESULT_SUMMARY.csv").open(newline="", encoding="utf-8") as stream:
        rows = {row["study"]: row for row in csv.DictReader(stream)}
    with (ROOT / "data/v6/v6_02_f1_method_summary.csv").open(newline="", encoding="utf-8") as stream:
        methods = {row["method"]: row for row in csv.DictReader(stream)}
    with (ROOT / "data/v6/v6_gazebo_preflight_summary.csv").open(newline="", encoding="utf-8") as stream:
        gazebo = list(csv.DictReader(stream))
    info = pdf_info(PDF)
    embedded, font_rows = fonts_embedded(PDF)
    pdf_text = subprocess.check_output(["pdftotext", str(PDF), "-"], text=True)
    tex = TEX.read_text(encoding="utf-8")
    log = LOG.read_text(encoding="utf-8", errors="replace")
    prompt = (ROOT / "CODEX_PROMPT_FOR_SCHOOL_TEMPLATE.md").read_text(encoding="utf-8")
    robot = (ROOT / "chapters/CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER.tex").read_text(encoding="utf-8")
    upstream_check = subprocess.run(
        ["sha256sum", "-c", str(ROOT / "upstream_evidence_manifest.sha256")],
        cwd=str(W), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    checks = {
        "required_files_present": not absent,
        "upstream_evidence_manifest_pass": upstream_check.returncode == 0,
        "v3_terminal_pdf_immutable": sha(V3_PDF) == "22f4e423a4211590c2d4936c8be4a7f5d6963da83083e0dd8f86b0197af58b7d",
        "v3_review_immutable": sha(V3_REVIEW) == "adb4317773484601dd3a1b2006098051521c611fdaa7073138991112a7a3d7a5",
        "v6_formal_immutable": sha(V6_FORMAL) == "08a0ab06343cad51ee206ba502d2a115c6fd707d58620f9bb4f519a71ec95c1b",
        "v6_review_immutable": sha(V6_REVIEW) == "ff4b6423d8fb610fe29486ab059c86a1e86f284055c8b520cf9e38cf4e470b41",
        "formal_result_exact": formal["method_effectiveness_demonstrated"] is True and abs(formal["primary_relative_reduction_vs_analytic"] - 0.08402444347446525) < 1e-15,
        "portable_method_summary_exact": abs(float(methods["PCT-SCOPE-RL V6"]["mean_restricted_time_s"]) - 13.115625) < 1e-12 and int(methods["PCT-SCOPE-RL V6"]["goals"]) == 60,
        "current_summary_exact": rows["V6-02-F1"]["point_percent"] == "8.402444" and rows["Real vehicle"]["status"] == "no evidence; placeholder only",
        "claims_scope_exact": claims["claims"][-1]["status"] == "pending_no_evidence" and "V6 Gazebo effectiveness" in claims["forbidden_claims"],
        "gazebo_negative_rows_exact": len(gazebo) == 9 and all(float(row["clearance_m"]) < 0 for row in gazebo if row["method"] != "actual_teb"),
        "v3_figure_inventory": len(list((ROOT / "figures/v3").glob("*.pdf"))) == 5 and len(list((ROOT / "figures/v3").glob("*.svg"))) == 5,
        "v6_figure_inventory": len(list((ROOT / "figures/v6").glob("*.pdf"))) == 6 and len(list((ROOT / "figures/v6").glob("*.svg"))) == 6,
        "v3_table_inventory": len(list((ROOT / "tables/v3").glob("*.tex"))) == 5,
        "integrated_source_self_contained": "../../../artifacts" not in tex and "CH09_PCT_SCOPE_RL_INTEGRATED" in tex and "CH10_REAL_ROBOT_EXPERIMENT_PLACEHOLDER" in tex,
        "latex_log_clean": "LaTeX Error" not in log and "Undefined control sequence" not in log and "There were undefined references" not in log,
        "pdf_is_a4_searchable": "A4" in info.get("Page size", "") and int(info.get("Pages", "0")) >= 40 and info.get("Encrypted") == "no",
        "pdf_fonts_embedded": embedded,
        "pdf_contains_v3_v6_results": all(term in pdf_text for term in ("12.9288", "8.4024", "PCT-SCOPE-RL", "Gazebo 迁移负结果")),
        "pdf_contains_real_placeholder": "实车实验设计与待补结果" in pdf_text and "待实车实验" in pdf_text,
        "real_vehicle_values_absent": "--" in robot and "尚未执行实车运动" in robot,
        "codex_prompt_has_mandatory_boundaries": all(term in prompt for term in ("V6 已在 Gazebo 中证明有效", "实车实验表明", "8.402444", "PENDING_REAL_ROBOT_FIELDS.md")),
    }
    status = "v6_06_p1_r1_thesis_transfer_independent_review_pass" if all(checks.values()) else "v6_06_p1_r1_thesis_transfer_review_fail"
    review = {
        "schema_version": "1.0", "stage": "V6-06-P1-R1", "status": status,
        "checks": checks, "absent_files": absent,
        "pdf_pages": int(info.get("Pages", "0")), "pdf_page_size": info.get("Page size"),
        "embedded_font_rows": font_rows, "integrated_pdf_sha256": sha(PDF),
        "upstream_check_output": upstream_check.stdout.splitlines(),
        "hardware_accessed": False, "real_vehicle_used": False, "simulation_run": False,
    }
    REVIEW.write_text(yaml.safe_dump(review, sort_keys=False, allow_unicode=True), encoding="utf-8")
    if not all(checks.values()):
        print(yaml.safe_dump(review, sort_keys=False, allow_unicode=True))
        raise RuntimeError("independent thesis transfer review failed")
    content = sorted(
        path for path in ROOT.rglob("*")
        if path.is_file() and path.name != MANIFEST.name
    )
    with MANIFEST.open("w", encoding="utf-8", newline="\n") as stream:
        for path in content:
            stream.write("{}  {}\n".format(sha(path), path.relative_to(ROOT)))
    print(status)
    print("pdf_sha256={}".format(sha(PDF)))
    print("review_sha256={}".format(sha(REVIEW)))
    print("manifest_sha256={}".format(sha(MANIFEST)))


if __name__ == "__main__":
    main()
