#!/usr/bin/env python3
"""Independently review and freeze the V3-17-P1 thesis export."""
import argparse
import csv
import hashlib
import os
from pathlib import Path
import subprocess

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/paper/v3_17_p1_thesis_export"
PDF = ROOT / "FAM_TEB_V3_THESIS_DRAFT.pdf"
MANIFEST = ROOT / "v3_17_p1_evidence_manifest.sha256"
CLAIMS = ROOT / "v3_17_p1_claims.yaml"
SUMMARY = ROOT / "v3_17_p1_result_summary.csv"
OUT = ROOT / "independent_paper_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def atomic(path, payload, check):
    if check:
        if not path.is_file() or path.read_bytes() != payload:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def report_bytes():
    claims = yaml.safe_load(CLAIMS.read_bytes())
    with SUMMARY.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    entries = [line.split("  ", 1) for line in MANIFEST.read_text().splitlines() if line]
    pdfinfo = subprocess.run(["pdfinfo", str(PDF)], check=True, stdout=subprocess.PIPE, text=True).stdout
    fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip() for line in pdfinfo.splitlines() if ":" in line}
    pages = int(fields["Pages"])
    text = subprocess.run(["pdftotext", str(PDF), "-"], check=True, stdout=subprocess.PIPE, text=True).stdout
    primary = claims["qualified_primary_claim"]
    mechanism = claims["qualified_mechanism_claim"]
    extension = claims["terminal_extension_claim"]
    checks = {
        "manifest_all_resources_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in entries),
        "pdf_is_a4_25_pages": pages == 25 and "595.28 x 841.89 pts" in fields["Page size"] and "(A4)" in fields["Page size"],
        "pdf_contains_primary_effect": "12.9288" in text and "12.7253" in text,
        "pdf_contains_v3_16_mechanism_and_negative": all(token in text for token in ("88.07", "0.008597", "0.067472", "predictor_fit_authorized=false")),
        "primary_claim_scoped": primary["paired_blocks"] == 30 and primary["episodes"] == 60 and "single_reactive_actor" in primary["population"],
        "mechanism_claim_scoped": 88.0 < mechanism["speed_mismatch_reduction_percent"] < 88.1 and mechanism["scope"] == "common_prefix_speed_mechanism_only",
        "extension_claim_is_negative": extension["status"] == "multi_actor_support_not_demonstrated" and extension["coordinate_gate_pass_blocks"] == 5 and not extension["predictor_fit_authorized"],
        "summary_contains_both_speed_pass_and_coordinate_fail": any(row["study"] == "V3-16-S1" and row["metric"] == "max_actor_speed_range" and row["status"] == "pass" for row in rows) and any(row["study"] == "V3-16-S1" and row["metric"] == "max_actor_coordinate_range" and row["status"] == "fail" for row in rows),
        "forbidden_claims_closed": set(("perception_robustness", "multi_actor_performance", "OOD_generalization", "formal_safety", "learning_benefit", "deployment_readiness", "real_vehicle_performance")).issubset(set(claims["forbidden_claims"])),
        "zero_training_and_no_hardware": claims["training_steps"] == 0 and not claims["hardware_accessed"] and not claims["real_vehicle_used"],
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-17-P1",
        "status": "v3_17_p1_independent_thesis_export_review_pass" if all(checks.values()) else "v3_17_p1_independent_thesis_export_review_fail",
        "pdf_path": str(PDF.relative_to(W)),
        "pdf_sha256": sha(PDF.read_bytes()),
        "pdf_pages": pages,
        "pdf_page_size": fields["Page size"],
        "evidence_manifest_sha256": sha(MANIFEST.read_bytes()),
        "claims_sha256": sha(CLAIMS.read_bytes()),
        "result_summary_sha256": sha(SUMMARY.read_bytes()),
        "checks": checks,
        "paper_export_complete": all(checks.values()),
        "execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = report_bytes()
    atomic(OUT, report, args.check)
    freeze_paths = (
        PDF, MANIFEST, CLAIMS, SUMMARY, OUT,
        W / "docs/thesis_experiment/thesis_draft/FAM_TEB_V3_THESIS_DRAFT.tex",
        W / "src/tools/thesis_experiment/scripts/review_v3_17_p1_thesis_export.py",
    )
    freeze = "".join(
        "{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in freeze_paths
    ).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    value = yaml.safe_load(report)
    print(
        "status={} review_sha256={} terminal_freeze_sha256={}".format(
            value["status"], sha(report), sha(freeze)
        )
    )


if __name__ == "__main__":
    main()

