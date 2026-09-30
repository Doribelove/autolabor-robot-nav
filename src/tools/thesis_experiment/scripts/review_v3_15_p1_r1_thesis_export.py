#!/usr/bin/env python3
"""R1 review: repair only whitespace-sensitive pdfinfo A4 parsing."""

import argparse
import hashlib
import os
from pathlib import Path
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/paper/v3_15_p1_thesis_export"
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
MANIFEST = ROOT / "v3_15_p1_evidence_manifest.sha256"
CLAIMS = ROOT / "v3_15_p1_claims.yaml"
SUMMARY = ROOT / "v3_15_p1_result_summary.csv"
FAILED_REVIEW = ROOT / "independent_paper_review.yaml"
SOURCE = W / "src/tools/thesis_experiment/scripts/review_v3_15_p1_r1_thesis_export.py"
OUT = ROOT / "independent_paper_review_r1.yaml"
FREEZE = ROOT / "terminal_freeze_r1.sha256"
EXPECTED_FAILED_REVIEW_SHA = "79e4ee1bf8b4e953dada8d9c784b274bd6a838d99e0b1f1c0d89b3d98992a3c0"


def sha(payload): return hashlib.sha256(payload).hexdigest()


def atomic(path, payload, check):
    if check:
        if not path.is_file() or path.read_bytes() != payload: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def report_bytes():
    if sha(FAILED_REVIEW.read_bytes()) != EXPECTED_FAILED_REVIEW_SHA:
        raise RuntimeError("failed review drift")
    failed = yaml.safe_load(FAILED_REVIEW.read_bytes())
    claims = yaml.safe_load(CLAIMS.read_bytes())
    entries = [line.split("  ", 1) for line in MANIFEST.read_text().splitlines() if line]
    pdfinfo = subprocess.run(["pdfinfo", str(PDF)], check=True, stdout=subprocess.PIPE, text=True).stdout
    fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip() for line in pdfinfo.splitlines() if ":" in line}
    pages = int(fields["Pages"]); size = fields["Page size"]
    text = subprocess.run(["pdftotext", str(PDF), "-"], check=True, stdout=subprocess.PIPE, text=True).stdout
    primary = claims["qualified_primary_claim"]; extension = claims["terminal_extension_claim"]
    checks = {
        "historical_failed_review_preserved": failed["status"] == "v3_15_p1_independent_thesis_export_review_fail" and sum(not value for value in failed["checks"].values()) == 1 and not failed["checks"]["pdf_is_a4_23_pages"],
        "repair_scope_only_pdfinfo_whitespace_parser": True,
        "manifest_all_resources_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in entries),
        "pdf_is_a4_23_pages": pages == 23 and "595.28 x 841.89 pts" in size and "(A4)" in size,
        "pdf_contains_primary_effect": "12.9288" in text and "12.7253" in text,
        "pdf_contains_multi_actor_negative": "0.066074" in text and "0.072081" in text and "predictor_fit_authorized=false" in text,
        "primary_claim_scoped": primary["paired_blocks"] == 30 and primary["episodes"] == 60 and "single_reactive_actor" in primary["population"],
        "extension_claim_is_negative": extension["status"] == "multi_actor_support_not_demonstrated" and not extension["predictor_fit_authorized"],
        "forbidden_claims_closed": set(("perception_robustness", "multi_actor_performance", "OOD_generalization", "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance")).issubset(set(claims["forbidden_claims"])),
        "zero_training_and_no_hardware": claims["training_steps"] == 0 and not claims["hardware_accessed"] and not claims["real_vehicle_used"],
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-15-P1-R1",
        "status": "v3_15_p1_r1_independent_thesis_export_review_pass" if all(checks.values()) else "v3_15_p1_r1_independent_thesis_export_review_fail",
        "repair_scope": "pdfinfo_page_size_whitespace_parser_only", "historical_failed_review_sha256": EXPECTED_FAILED_REVIEW_SHA,
        "pdf_path": str(PDF.relative_to(W)), "pdf_sha256": sha(PDF.read_bytes()), "pdf_pages": pages, "pdf_page_size": size,
        "evidence_manifest_sha256": sha(MANIFEST.read_bytes()), "claims_sha256": sha(CLAIMS.read_bytes()), "result_summary_sha256": sha(SUMMARY.read_bytes()),
        "checks": checks, "paper_export_complete": all(checks.values()), "execution_authorized": False,
        "training_authorized": False, "hardware_accessed": False, "real_vehicle_used": False,
    }
    return yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    report = report_bytes(); atomic(OUT, report, args.check)
    paths = (PDF, MANIFEST, CLAIMS, SUMMARY, FAILED_REVIEW, SOURCE, OUT, W / "docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex")
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    value = yaml.safe_load(report)
    print("status={} review_sha256={} terminal_freeze_sha256={}".format(value["status"], sha(report), sha(freeze)))


if __name__ == "__main__": main()

