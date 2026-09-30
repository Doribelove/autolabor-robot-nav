#!/usr/bin/env python3
"""Build the deterministic inherited-evidence inventory for the new benchmark."""

from __future__ import annotations

import csv
import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/evidence_inventory.csv"

# This is intentionally explicit. Adding or removing inherited evidence is a reviewed
# decision rather than an implicit recursive directory scan.
EVIDENCE = [
    ("instruction", "required", "CODEX_MASTER_PROMPT_PCT_VS_TRADITIONAL_PLANNERS.md", "new-study authority"),
    ("instruction", "declared_missing", "CODEX_HANDOFF_PCT_TEB_DWA_GAZEBO_BENCHMARK.md", "requested handoff; input gap"),
    ("handoff", "required", "docs/thesis_experiment/CURRENT_V6_PCT_SCOPE_RL_HANDOFF.md", "terminal V6 scope and prohibitions"),
    ("handoff", "required", "docs/thesis_experiment/CURRENT_V6_06_THESIS_TRANSFER_HANDOFF.md", "terminal transfer scope"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/README_FIRST.md", "package entry point"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/CURRENT_THESIS_INTEGRATION_GUIDE.md", "thesis integration guide"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/combined_claims_evidence_matrix.yaml", "historical claim boundaries"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/DATA_DICTIONARY.md", "field definitions"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/CURRENT_RESULT_SUMMARY.csv", "historical summary"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/CODEX_HANDOFF_MACHINE_CONTEXT.md", "machine context"),
    ("paper_package", "required", "artifacts/v6/paper/v6_06_p1_r1_thesis_transfer_package/terminal_content_manifest.sha256", "77-file package seal"),
    ("contract", "required", "src/application/v5_e2e_scope_rl/config/v6_00_pct_scope_rl_contract.yaml", "V6 method contract"),
    ("contract", "required", "src/application/v5_e2e_scope_rl/config/v6_03_gazebo_contract.yaml", "historical Gazebo contract"),
    ("contract", "required", "src/application/v5_e2e_scope_rl/config/v6_03_r1_environment_repair.yaml", "historical environment repair"),
    ("contract", "required", "src/application/v5_e2e_scope_rl/config/v6_03_r2_safety_alignment.yaml", "historical shield-alignment factor"),
    ("contract", "required", "src/application/v5_e2e_scope_rl/config/v6_04_directional_shield_contract.yaml", "historical directional-shield factor"),
    ("model", "required", "artifacts/v6/training/v6_01_r1_training_release.json", "selected checkpoint release"),
    ("model", "required", "artifacts/v6/training/v6_01_r1_validation/selection_report.json", "selection evidence"),
    ("model", "required", "artifacts/v6/training/v6_01_r1/pct_seed9503/actor_step_0080640.pt", "frozen selected PCT actor"),
    ("formal_evidence", "required", "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_episodes.csv", "original 320 rows"),
    ("formal_evidence", "required", "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_assessment.json", "original statistics"),
    ("formal_evidence", "required", "artifacts/v6/formal/v6_02_f1_fresh_formal/bootstrap_distributions.csv", "original paired bootstrap"),
    ("formal_evidence", "required", "artifacts/v6/formal/v6_02_f1_fresh_formal/sha256_manifest.json", "original formal seal"),
    ("reproduction", "required", "artifacts/gazebo_pct_teb_dwa_benchmark/reproduction/v6_02_f1_recomputed_20260810/formal_episodes.csv", "independent recomputation"),
    ("reproduction", "required", "artifacts/gazebo_pct_teb_dwa_benchmark/reproduction/v6_02_f1_recomputed_20260810/reproduction_comparison.json", "scientific equality report"),
    ("gazebo_negative", "required", "artifacts/v6/gazebo/v6_03_p0_preflight/episodes/001_954099_open_crossing_90_cautious_pct_scope_rl_v6/roslaunch.log", "original migration failure log"),
    ("gazebo_negative", "required", "artifacts/v6/gazebo/v6_03_r1_p0_preflight/matrix_summary_recovered_offline.yaml", "environment-repair negative"),
    ("gazebo_negative", "required", "artifacts/v6/gazebo/v6_03_r2_p0_preflight/matrix_summary.yaml", "isotropic-shield negative"),
    ("gazebo_negative", "required", "artifacts/v6/gazebo/v6_04_p0_preflight/matrix_summary.yaml", "directional-shield negative"),
    ("runtime", "required", "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py", "frozen runtime candidate and observation semantics"),
    ("runtime", "required", "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/directional_safety.py", "frozen historical tracker semantics"),
    ("runtime", "required", "src/application/v5_e2e_scope_rl/scripts/v6_sensor_gazebo_episode_v604.py", "historical Gazebo controller"),
    ("baseline", "required", "src/simulation/m2_gazebo/config/fixed_teb.yaml", "Actual TEB starting configuration"),
    ("baseline", "required", "src/simulation/m2_gazebo/config/fixed_dwa.yaml", "DWA starting configuration"),
    ("baseline", "required", "src/simulation/m2_gazebo/launch/m2_fixed_teb.launch", "TEB launch"),
    ("baseline", "required", "src/simulation/m2_gazebo/launch/m2_fixed_dwa.launch", "DWA launch"),
    ("robot", "required", "src/simulation/m2_gazebo/urdf/m2_sensors.xacro", "laser geometry and plugin"),
    ("robot", "required", "src/simulation/m2_gazebo/urdf/m2_base.xacro", "wheelbase and footprint geometry"),
    ("world", "required", "src/tools/thesis_experiment/scripts/run_v3_11_actual_teb_matrix.py", "inherited world renderer"),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    invalid = []
    for category, requirement, relative, role in EVIDENCE:
        path = ROOT / relative
        exists = path.is_file()
        expected_missing = requirement == "declared_missing"
        status = "present" if exists else ("declared_missing" if expected_missing else "missing")
        if (not exists) and (not expected_missing):
            invalid.append(relative)
        rows.append(
            {
                "category": category,
                "requirement": requirement,
                "status": status,
                "path": relative,
                "sha256": sha256(path) if exists else "",
                "bytes": path.stat().st_size if exists else "",
                "role": role,
                "new_study_mutability": "read_only_inherited",
            }
        )
    with OUTPUT.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if invalid:
        raise SystemExit("required evidence missing: " + ", ".join(invalid))
    print(f"wrote {len(rows)} evidence bindings to {OUTPUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
