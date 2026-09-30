#!/usr/bin/env python3
"""Independently review and terminally freeze the V3-15-P1 thesis export."""

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
OUT = ROOT / "independent_paper_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(payload): return hashlib.sha256(payload).hexdigest()


def atomic(path, payload, check):
    if check:
        if not path.is_file() or path.read_bytes() != payload: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def build_report():
    claims = yaml.safe_load(CLAIMS.read_bytes())
    entries = [line.split("  ", 1) for line in MANIFEST.read_text().splitlines() if line]
    pdfinfo = subprocess.run(["pdfinfo", str(PDF)], check=True, stdout=subprocess.PIPE, text=True).stdout
    pages = int(next(line.split(":", 1)[1].strip() for line in pdfinfo.splitlines() if line.startswith("Pages:")))
    text = subprocess.run(["pdftotext", str(PDF), "-"], check=True, stdout=subprocess.PIPE, text=True).stdout
    primary = claims["qualified_primary_claim"]; extension = claims["terminal_extension_claim"]
    checks = {
        "manifest_all_resources_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in entries),
        "pdf_is_a4_23_pages": pages == 23 and "Page size:       595.28 x 841.89 pts (A4)" in pdfinfo,
        "pdf_contains_primary_effect": "12.9288" in text and "12.7253" in text,
        "pdf_contains_multi_actor_negative": "0.066074" in text and "0.072081" in text and "predictor_fit_authorized=false" in text,
        "primary_claim_scoped": primary["paired_blocks"] == 30 and primary["episodes"] == 60 and "single_reactive_actor" in primary["population"],
        "extension_claim_is_negative": extension["status"] == "multi_actor_support_not_demonstrated" and not extension["predictor_fit_authorized"],
        "forbidden_claims_closed": set(("perception_robustness", "multi_actor_performance", "OOD_generalization", "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance")).issubset(set(claims["forbidden_claims"])),
        "zero_training_and_no_hardware": claims["training_steps"] == 0 and not claims["hardware_accessed"] and not claims["real_vehicle_used"],
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-15-P1",
        "status": "v3_15_p1_independent_thesis_export_review_pass" if all(checks.values()) else "v3_15_p1_independent_thesis_export_review_fail",
        "pdf_path": str(PDF.relative_to(W)), "pdf_sha256": sha(PDF.read_bytes()), "pdf_pages": pages,
        "evidence_manifest_sha256": sha(MANIFEST.read_bytes()), "claims_sha256": sha(CLAIMS.read_bytes()),
        "result_summary_sha256": sha(SUMMARY.read_bytes()), "checks": checks,
        "paper_export_complete": all(checks.values()), "execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    return yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    report = build_report()
    atomic(OUT, report, args.check)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in (PDF, MANIFEST, CLAIMS, SUMMARY, OUT, W / "docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex")).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    status = yaml.safe_load(report)["status"]
    print("status={} review_sha256={} terminal_freeze_sha256={}".format(status, sha(report), sha(freeze)))


if __name__ == "__main__": main()

