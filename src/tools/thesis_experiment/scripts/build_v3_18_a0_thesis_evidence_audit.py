#!/usr/bin/env python3
"""Build the V3-18-A0 architecture, evidence, schema, and gap audit."""
import argparse
import csv
import hashlib
import importlib.util
import io
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_a0_project_audit"
CONTRACT = W / "config/thesis_experiments/v3/v3_18_a0_thesis_evidence_audit_contract.yaml"
SCHEMA_SOURCE = W / "src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py"
ARCHITECTURE = ROOT / "system_architecture.yaml"
INVENTORY = ROOT / "experiment_inventory.csv"
MATRIX = ROOT / "method_evidence_matrix.csv"
SCHEMA = ROOT / "standard_collection_schema.yaml"
GAPS = ROOT / "gap_analysis.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"

SOURCES = {
    "T11": W / "artifacts/t11/evaluation/t11_evaluation_report.yaml",
    "T12": W / "artifacts/t12/residual_training/t12_residual_learning_diagnosis.yaml",
    "V2P1": W / "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/p1_e1_four_gate_assessment.yaml",
    "V305": W / "artifacts/v3/formal/v3_05_b0/execution/v3_05_b0_formal_assessment.yaml",
    "V309": W / "artifacts/v3/formal/v3_09_f1/execution/v3_09_f1_formal_assessment.yaml",
    "V311": W / "artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml",
    "V314": W / "artifacts/v3/multiactor/v3_14_s1_multiaction_support/evidence_summary.yaml",
    "V316": W / "artifacts/v3/multiactor/v3_16_s1_response_gated_support/assessment_r1.yaml",
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def ybytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def csv_bytes(header, rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(header)
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_schema_module():
    spec = importlib.util.spec_from_file_location("v3_18_thesis_dataset", SCHEMA_SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def outputs():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    for name, path in SOURCES.items():
        if not path.is_file():
            raise RuntimeError("missing {} evidence: {}".format(name, path))
    v2 = yaml.safe_load(SOURCES["V2P1"].read_bytes())
    v305 = yaml.safe_load(SOURCES["V305"].read_bytes())
    v309 = yaml.safe_load(SOURCES["V309"].read_bytes())
    v311 = yaml.safe_load(SOURCES["V311"].read_bytes())
    v314 = yaml.safe_load(SOURCES["V314"].read_bytes())
    v316 = yaml.safe_load(SOURCES["V316"].read_bytes())

    architecture = {
        "schema_version": "1.0",
        "stage": "V3-18-A0",
        "system_goal": "action-aligned, explainable, fail-closed local navigation for Ackermann robots in dynamic interaction",
        "workspace_boundaries": {
            "thesis_workspace": str(W),
            "stable_real_robot_workspace": "/home/robot/robot_ws",
            "stable_workspace_modified": False,
        },
        "physical_platform": {
            "base": "Autolabor M2 Ackermann",
            "sensors": ["Livox lidar", "GPS", "odometry", "camera"],
            "hardware_calibration_status": "not qualified by current simulation evidence",
        },
        "command_safety_chain": [
            "/cmd_vel_navigation", "GPS speed limiter", "/cmd_vel_gps",
            "FOD mode arbiter", "/cmd_vel", "M2 driver sanitizer",
        ],
        "algorithm_generations": {
            "V1_T11_T12": "TEB parameter RL/residual control; exposed action projection and cross-seed instability",
            "V2_FAM_TEB": "typed scene profiles, feasible decoder, semantic TTC overlays and transaction audit",
            "V3_SCOPE": "Ackermann-feasible trajectory candidates, candidate-conditioned prediction, calibrated occupancy, hard verifier and analytic ranker",
        },
        "v3_runtime_dataflow": [
            "scan/odom/GPS/TF/global path/costmap/health",
            "immutable scene snapshot",
            "Ackermann candidate generator with certified stop",
            "candidate-conditioned actor predictor",
            "calibrated reachable occupancy tubes",
            "hard verifier",
            "analytic ranker",
            "pure-pursuit tracker",
            "/cmd_vel_navigation boundary",
        ],
        "experiment_integrity_layers": [
            "preregistration", "exact schedule", "fresh seeds", "attempt-1 identity",
            "consumption journal", "raw report SHA", "deterministic assessment",
            "independent review", "claim gate", "terminal freeze",
        ],
        "current_claim_boundary": {
            "qualified": "V3-11 single-reactive-actor oracle-track improvement versus actual TebLocalPlannerROS 0.8.4",
            "not_demonstrated": ["multi-actor support", "learning benefit", "V2 obvious performance improvement"],
            "forbidden": ["perception robustness", "OOD generalization", "formal safety", "deployment", "real-vehicle performance"],
        },
    }

    inventory_rows = [
        ("T11/T12", "V1_T12_learning", "RL/residual TEB parameter control", "TEB and zero-residual references", "safety/projection ablations", 2, "multiple training/test scenes", "historical_negative", "not pooled with V2/V3"),
        ("V2-04G-P1", "v2_dynamic_semantics", "circle-contact semantic treatment", "legacy semantic control", "fixed TEB contextual", 90, 3, "formal_valid_negative", "270 episodes; 3.7624% < 5%"),
        ("V3-05-B0", "v3_longitudinal", "v3 analytic longitudinal", "fixed-v1.20 shielded", "fixed-v0.80 shielded", 30, 1, "formal_valid_negative", "270 episodes; early longitudinal design"),
        ("V3-09-F1", "v3_spatial_idealized", "v3 spatial-safe", "fixed-v1.20 shielded", "none", 30, 1, "formal_positive", "180 episodes; comparator is not actual TEB"),
        ("V3-11-F1", "v3_spatial_actual_teb", "v3 spatial-safe", "actual TebLocalPlannerROS 0.8.4", "none", 30, 3, "formal_positive", "60 episodes; cautious/hesitant; oracle track"),
        ("V3-14-S1", "v3_multiactor_support", "five candidate cloned interventions", "common-prefix gate", "action-binding integrity", 8, 2, "terminal_negative", "40 scientific + 5 preflight"),
        ("V3-16-S1", "v3_multiactor_support_response_gate", "response gate 12.55 s", "V3-14 response at 0 s", "speed/coordinate decomposition", 8, 2, "terminal_negative_mechanism_positive", "40 scientific + 5 preflight"),
    ]
    inventory = csv_bytes(
        ("stage", "comparability_group", "main_method", "primary_baseline", "comparison_or_ablation", "statistical_blocks_or_seeds", "scene_templates", "result_status", "scope_note"),
        inventory_rows,
    )

    matrix_rows = [
        ("V2-04G-P1", "circle_contact", "main/treatment", 90, 270, "three frozen dynamic templates", "3.7624%", "2.7904%", "pass", "negative"),
        ("V3-05-B0", "v3_analytic", "main", 30, 90, "parameterized crossing", "1.2007%", "0.9647%", "93.33% success", "negative"),
        ("V3-05-B0", "fixed_v120_shielded", "primary baseline", 30, 90, "parameterized crossing", "reference", "reference", "91.11% success", "context"),
        ("V3-05-B0", "fixed_v080_shielded", "comparison", 30, 90, "parameterized crossing", "9.9992% descriptive", "not inferential", "18.89% success", "context"),
        ("V3-09-F1", "v3_spatial_safe", "main", 30, 90, "parameterized crossing", "22.3844%", "21.8235%", "100% success", "positive"),
        ("V3-09-F1", "fixed_v120_shielded", "idealized comparator", 30, 90, "parameterized crossing", "reference", "reference", "100% success", "context"),
        ("V3-11-F1", "v3_spatial_safe", "main", 30, 30, "three frozen crossing templates", "12.9288%", "12.7253%", "100% success", "positive"),
        ("V3-11-F1", "actual_teb_0_8_4", "primary baseline", 30, 30, "three frozen crossing templates", "reference", "reference", "100% success", "context"),
        ("V3-16-S1", "response_gate_12_55", "mechanism treatment", 8, 40, "two multi-actor templates", "88.0730% speed mismatch reduction", "not a performance CI", "coordinate gate failed", "mechanism_only"),
    ]
    matrix = csv_bytes(
        ("stage", "method", "role", "blocks", "episodes", "population", "point_effect", "one_sided_lower", "task_or_safety", "claim_state"),
        matrix_rows,
    )

    schema_module = load_schema_module()
    schema = ybytes(schema_module.schema_document())
    gaps = {
        "schema_version": "1.0",
        "stage": "V3-18-A0",
        "status": "audit_complete_new_same_population_benchmark_required",
        "evidence_strengths": [
            "actual TEB formal paired baseline exists",
            "fresh multi-scene/multi-seed execution exists",
            "idealized comparator and longitudinal ablations exist",
            "negative learning and multi-actor evidence is preserved",
            "deterministic assessments and paper assets exist",
        ],
        "material_gaps": [
            {"id": "G1", "gap": "no canonical episode-level dataset spanning frozen studies", "action": "implement adapters and manifest-backed collector"},
            {"id": "G2", "gap": "main/actual-TEB/fixed comparator/ablation are not evaluated in one fresh population", "action": "new four-method simulation-only benchmark"},
            {"id": "G3", "gap": "figures and three-line tables are stage-specific", "action": "single deterministic analysis/export pipeline"},
            {"id": "G4", "gap": "reproduction commands are distributed across handoffs", "action": "generate top-level reproduction book and one-command offline rebuild"},
            {"id": "G5", "gap": "perception/OOD and independent complex geometry remain untested", "action": "future separately authorized study; not silently added to current claim"},
            {"id": "G6", "gap": "multi-actor common-prefix coordinate gate remains negative", "action": "do not fit predictor; future snapshot/reset design only"},
        ],
        "next_preregistered_study": {
            "methods": contract["future_experiment_gap"]["required_same_population_methods"],
            "purpose": "thesis benchmark and component ablation, not retuning V3-11",
            "fresh_seeds_required": True,
            "qualification_required": True,
            "training_steps": 0,
            "hardware_access": False,
        },
        "cross_stage_pooling_forbidden": True,
        "execution_authorized": False,
    }
    generated = {
        ARCHITECTURE: ybytes(architecture),
        INVENTORY: inventory,
        MATRIX: matrix,
        SCHEMA: schema,
        GAPS: ybytes(gaps),
    }
    resources = [CONTRACT, SCHEMA_SOURCE, Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_18_a0_thesis_evidence_audit.py"] + list(SOURCES.values())
    if not all(path.is_absolute() and W in path.parents for path in resources):
        raise RuntimeError("audit resources must resolve inside the thesis workspace")
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in resources)
    manifest += "".join("{}  {}\n".format(sha(data), path.relative_to(W)) for path, data in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    generated = outputs()
    for path, data in generated.items():
        atomic(path, data, args.check)
    print("status=v3_18_a0_project_audit_pass manifest_sha256={}".format(sha(generated[MANIFEST])))


if __name__ == "__main__":
    main()
