#!/usr/bin/env python3
"""Recheck the frozen claims and portable thesis-package files without running experiments."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PDF = ROOT / "FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.pdf"


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check(condition: bool, message: str, checks: list[str]) -> None:
    if not condition:
        raise AssertionError(message)
    checks.append(message)


def main() -> None:
    checks: list[str] = []
    evidence = ROOT / "evidence"
    canonical = json.loads((evidence / "canonical_results.json").read_text())
    historical = canonical["historical_core_results"]
    benchmark = canonical["new_gazebo_benchmark"]
    check(benchmark["formal_block_total"] == 360 and benchmark["formal_episode_total"] == 1440,
          "Gazebo totals are 360 blocks and 1440 episodes", checks)
    check(abs(historical["v6_fast_population"]["point_percent"] - 8.402444) < 1e-6,
          "V6 fast-population point estimate matches canonical evidence", checks)
    check(historical["v6_fast_population"]["status"] == "scoped_positive",
          "V6 fast-population scope is qualified", checks)
    check(historical["v3_21_actual_teb"]["status"] == "superiority_not_demonstrated",
          "V3-21 fresh result remains negative", checks)
    check(sha(evidence / "FAM_TEB_PCT_SCOPE_RL_COMPLETE_20260810.tar.gz") ==
          "c22a369561f1e9041d02fa76a564f3640cd7528a871293be9b163a374bdc4b29",
          "historical outer handoff archive matches its original SHA256", checks)

    for round_no, blocks, episodes in [(1, 160, 640), (2, 200, 800)]:
        assessment = json.loads((evidence / f"formal_r{round_no}_assessment.json").read_text())
        check(assessment["blocks"] == blocks and assessment["episodes"] == episodes and
              assessment["formal_result"] is True and
              assessment["performance_improvement_demonstrated"] is False,
              f"R{round_no} formal volume and negative superiority match assessment", checks)
        check(assessment["all_failures_retained"] is True and
              assessment["attempt_one_complete"] is True and
              assessment["replacement_seed_count"] == 0,
              f"R{round_no} failures and attempt-one identities are retained", checks)
        for baseline in ("actual_teb", "dwa"):
            comparison = assessment["primary_comparisons"][baseline]
            canonical_comparison = benchmark[f"round_{round_no}"]["comparisons"][baseline]
            check(abs(comparison["relative_reduction"] * 100 -
                      canonical_comparison["relative_reduction_percent"]) < 1e-9 and
                  comparison["positive_gate_pass"] is False,
                  f"R{round_no}/{baseline} estimate and gate match canonical results", checks)

    n05 = json.loads((evidence / "N05_independent_review.json").read_text())
    check(n05["consumed_attempt1"] == 24 and n05["complete_pairs"] == 12 and
          n05["formal_result"] is False and
          n05["performance_improvement_demonstrated"] is False and
          n05["all_safety_scoped_pass"] is False,
          "N05 remains a 24-episode negative development diagnostic", checks)
    check(all(row["completed"] == 2 and row["timeout"] == 10
              for row in n05["method_outcomes"].values()),
          "N05 both arms retain 2 goals and 10 timeouts", checks)

    figures = json.loads((ROOT / "appendix_figures/FIGURE_PACKAGE_AUDIT_R01.json").read_text())
    check(figures["figure_count"] == 30 and
          figures["selected_screenshot_group_count"] == 8 and
          figures["selected_unique_native_frames"] == 13,
          "figure audit contains 30 groups, 8 native-screen groups and 13 unique frames", checks)
    check(sha(ROOT / "appendix_figures/FIGURE_ATLAS.pdf") ==
          "63a656f6d59cce29c54cde51ec8a7c8ffd11bdd777db60e929b36d6fd61eaa96",
          "30-page figure atlas matches frozen SHA256", checks)

    info = subprocess.check_output(["pdfinfo", str(PDF)], text=True)
    check(re.search(r"^Pages:\s+45$", info, re.MULTILINE) is not None and
          re.search(r"^Page size:\s+595\.28 x 841\.89 pts \(A4\)$", info, re.MULTILINE)
          is not None, "technical draft is 45 A4 pages", checks)
    paper_text = subprocess.check_output(["pdftotext", "-layout", str(PDF), "-"], text=True)
    check(all(marker in paper_text for marker in ("N05", "12.9288", "8.402444", "7.7754")),
          "PDF contains the scoped positive and negative study markers", checks)
    log = (ROOT / "FAM_TEB_EXISTING_EVIDENCE_TECHNICAL_DRAFT.log").read_text()
    check("Overfull \\hbox" not in log and "undefined references" not in log.lower(),
          "final LaTeX pass has no overfull boxes or undefined references", checks)

    output = {
        "schema": "existing_evidence_thesis_validation_v1",
        "status": "pass",
        "checks": checks,
        "checks_passed": len(checks),
        "pdf_sha256": sha(PDF),
        "figure_atlas_sha256": sha(ROOT / "appendix_figures/FIGURE_ATLAS.pdf"),
        "limits": "Evidence/format validation only; no new experiment, school-template or real-vehicle qualification.",
    }
    (ROOT / "VALIDATION.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": output["status"], "checks_passed": len(checks),
                      "pdf_sha256": output["pdf_sha256"]}))


if __name__ == "__main__":
    main()
