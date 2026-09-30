#!/usr/bin/env python3
"""Independent PDF, claim and evidence review for V3-22-P2."""

import hashlib
from pathlib import Path
import re
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/paper/v3_22_p2_thesis_export")
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
LOG = ROOT / "FAM_TEB_V3_THESIS_DRAFT.log"
MANIFEST = ROOT / "v3_22_p2_evidence_manifest.sha256"
CLAIMS = ROOT / "v3_22_p2_claims.yaml"
MATERIALS_MANIFEST = Path("artifacts/v3/thesis/v3_22_p1_paper_materials/paper_materials_manifest.sha256")
REVIEW = ROOT / "independent_paper_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(path):
    return hashlib.sha256((W / path).read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise RuntimeError(message)


def output(command):
    return subprocess.check_output(command, cwd=str(W), text=True, errors="replace")


def main():
    require((W / PDF).is_file() and (W / PDF).read_bytes().startswith(b"%PDF"), "PDF absent")
    entries = []
    for line in (W / MANIFEST).read_text().splitlines():
        digest, name = line.split("  ", 1); path = Path(name)
        require((W / path).is_file(), "manifest path absent: {}".format(path))
        require(sha(path) == digest, "manifest hash differs: {}".format(path))
        entries.append(path)
    require(len(entries) == len(set(entries)), "duplicate manifest path")

    info = output(["pdfinfo", str(W / PDF)])
    require(re.search(r"Pages:\s+32\b", info), "page count differs")
    require("A4" in info, "paper size differs")
    require("Encrypted:      no" in info, "PDF encrypted")
    text = output(["pdftotext", str(W / PDF), "-"])
    for needle in ("四方法新鲜种子复现与消融表征", "12.9288", "49.454", "未复现", "8201–8230"):
        require(needle in text, "PDF text missing: {}".format(needle))
    fonts = output(["pdffonts", str(W / PDF)])
    font_rows = [line for line in fonts.splitlines()[2:] if line.strip()]
    require(font_rows and all(" yes " in line for line in font_rows), "font embedding differs")
    log = (W / LOG).read_text(errors="replace")
    require("Output written on" in log, "XeLaTeX completion absent")
    require("undefined references" not in log.lower(), "undefined references remain")
    require("LaTeX Error" not in log and "Fatal error" not in log, "LaTeX error remains")
    material_lines = (W / MATERIALS_MANIFEST).read_text().splitlines()
    require(sum("/figures/" in line for line in material_lines) == 15, "five figures in three formats not bound")
    require(sum("/tables/" in line for line in material_lines) == 5, "five tables not bound")

    claims = yaml.safe_load((W / CLAIMS).read_text())
    require(claims["historical_qualified_result"]["stage"] == "V3-11-F1", "historical result absent")
    fresh = claims["fresh_characterization"]
    require(fresh["formal_result"] is False and fresh["actual_teb_improvement_demonstrated"] is False, "fresh result inflated")
    require(claims["interpretation"] == "historical_scoped_positive_not_replicated_as_failure_inclusive_actual_teb_superiority", "interpretation drift")
    require(claims["training_steps"] == 0 and not claims["hardware_accessed"] and not claims["real_vehicle_used"], "authority drift")

    review = {
        "schema_version": "1.0", "stage": "V3-22-P2",
        "status": "terminal_thesis_pdf_independent_review_pass",
        "checks": {
            "manifest_hash_closure": True, "pdf_signature": True, "a4_32_pages": True,
            "fonts_embedded": True, "latex_no_error_or_undefined_reference": True,
            "five_figures_and_five_tables_bound_by_materials_manifest": True,
            "historical_positive_and_fresh_nonreplication_both_disclosed": True,
            "fresh_actual_teb_superiority_not_claimed": True, "authority_boundary_closed": True,
        },
        "pdf_sha256": sha(PDF), "manifest_sha256": sha(MANIFEST),
        "page_count": 32, "paper_size": "A4", "embedded_font_rows": len(font_rows),
        "formal_result_for_v3_21": False, "characterization_result_available": True,
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(review, sort_keys=False, allow_unicode=True).encode("utf-8")
    target = W / REVIEW
    if target.exists(): require(target.read_bytes() == payload, "review differs")
    else: target.write_bytes(payload)
    freeze = "{}  {}\n{}  {}\n{}  {}\n".format(sha(PDF), PDF, sha(MANIFEST), MANIFEST, hashlib.sha256(payload).hexdigest(), REVIEW).encode("utf-8")
    target = W / FREEZE
    if target.exists(): require(target.read_bytes() == freeze, "freeze differs")
    else: target.write_bytes(freeze)
    print("status=terminal_thesis_pdf_independent_review_pass pdf_sha256={} review_sha256={}".format(sha(PDF), hashlib.sha256(payload).hexdigest()))


if __name__ == "__main__":
    main()
