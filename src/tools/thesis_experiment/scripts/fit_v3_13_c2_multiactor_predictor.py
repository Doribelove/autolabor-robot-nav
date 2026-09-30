#!/usr/bin/env python3
"""Fit and internally cross-validate the V3-13-C2 predictor."""

import argparse
import hashlib
import importlib.util
import math
import os
from pathlib import Path

import numpy as np
import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c2_predictor_fit_contract.yaml"
)
CORE = Path(
    "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_c2_multiactor_predictor.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_c2_multiactor_predictor.py"
)
C1 = Path("artifacts/v3/multiactor/v3_13_c1_execution/episodes")
ROOT = Path("artifacts/v3/multiactor/v3_13_c2_predictor")
MODEL = ROOT / "candidate_conditioned_predictor.yaml"
CV = ROOT / "internal_cross_validation.yaml"
RECEIPT = ROOT / "fit_receipt.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def feature(row, time_s):
    time_s = float(time_s)
    return np.asarray(
        [
            time_s,
            time_s * time_s,
            time_s * time_s * time_s,
            row["initial_speed_mps"] * time_s,
            -row["initial_coordinate_m"] * time_s,
            row["path_origin_x_m"] * time_s,
            row["path_origin_x_m"] * time_s * time_s,
        ],
        dtype=np.float64,
    )


def load_rows():
    rows = []
    report_paths = sorted(
        path.relative_to(W)
        for path in (W / C1).glob("*/episode_report.yaml")
    )
    require(len(report_paths) == 64, "C1 report count")
    for report_path in report_paths:
        report = load_yaml(report_path)
        for gate_track in report["warmup"]["gate_actor_tracks"]:
            track_id = int(gate_track["track_id"])
            samples = []
            for sample in report["result"]["trace"]:
                actor = next(
                    item
                    for item in sample["actors"]
                    if int(item["track_id"]) == track_id
                )
                samples.append(
                    {
                        "time_s": float(sample["elapsed_s"]),
                        "coordinate_m": float(actor["coordinate_m"]),
                    }
                )
            rows.append(
                {
                    "report_path": str(report_path),
                    "block_index": int(report["block_index"]),
                    "scene_id": report["scene_id"],
                    "candidate_id": report["forced_candidate_id"],
                    "track_id": track_id,
                    "observable_mode": report[
                        "observable_modes_at_gate"
                    ][track_id],
                    "initial_coordinate_m": float(
                        gate_track["coordinate_m"]
                    ),
                    "initial_speed_mps": float(
                        gate_track["coordinate_speed_mps"]
                    ),
                    "path_origin_x_m": float(
                        gate_track["path_origin_x_m"]
                    ),
                    "samples": samples,
                }
            )
    require(len(rows) == 128, "actor trajectory count")
    return rows, report_paths


def fit_heads(rows, contract):
    heads = {}
    ridge_lambda = float(
        contract["model"]["ridge_lambda_after_rms_feature_scaling"]
    )
    for mode in contract["model"]["observable_modes"]:
        heads[mode] = {}
        for candidate_id in contract["model"]["candidate_ids"]:
            selected = [
                row
                for row in rows
                if row["observable_mode"] == mode
                and row["candidate_id"] == candidate_id
            ]
            require(len(selected) >= 8, "insufficient head trajectories")
            x_rows = []
            targets = []
            for row in selected:
                for sample in row["samples"]:
                    x_rows.append(feature(row, sample["time_s"]))
                    targets.append(
                        sample["coordinate_m"]
                        - row["initial_coordinate_m"]
                    )
            matrix = np.vstack(x_rows)
            target = np.asarray(targets, dtype=np.float64)
            scale = np.sqrt(np.mean(matrix * matrix, axis=0))
            require(np.all(scale > 1.0e-12), "degenerate feature")
            standardized = matrix / scale
            coefficients = np.linalg.solve(
                standardized.T @ standardized
                + ridge_lambda * np.eye(standardized.shape[1]),
                standardized.T @ target,
            ) / scale
            heads[mode][candidate_id] = {
                "trajectory_count": len(selected),
                "sample_count": len(targets),
                "coefficients": [
                    float(value) for value in coefficients
                ],
            }
    return heads


def predict(row, head, time_s):
    return row["initial_coordinate_m"] + float(
        feature(row, time_s)
        @ np.asarray(head["coefficients"], dtype=np.float64)
    )


def evaluate_rows(rows, heads, calibration):
    radius = float(calibration["coordinate_radius_m"])
    growth = float(calibration["coordinate_growth_mps"])
    values = []
    for row in rows:
        head = heads[row["observable_mode"]][row["candidate_id"]]
        errors = [
            abs(
                sample["coordinate_m"]
                - predict(row, head, sample["time_s"])
            )
            for sample in row["samples"]
        ]
        simultaneous = all(
            error
            <= radius + growth * float(sample["time_s"]) + 1.0e-12
            for error, sample in zip(errors, row["samples"])
        )
        values.append(
            {
                "report_path": row["report_path"],
                "block_index": row["block_index"],
                "scene_id": row["scene_id"],
                "candidate_id": row["candidate_id"],
                "track_id": row["track_id"],
                "observable_mode": row["observable_mode"],
                "trajectory_rmse_m": math.sqrt(
                    sum(error * error for error in errors) / len(errors)
                ),
                "maximum_absolute_error_m": max(errors),
                "simultaneous_trajectory_covered": simultaneous,
            }
        )
    return values


def build_outputs():
    contract = load_yaml(CONTRACT)
    assessment_path = Path(
        "artifacts/v3/multiactor/v3_13_c1_assessment/"
        "clone_assessment.yaml"
    )
    require(
        sha(read(assessment_path))
        == contract["upstream"]["c1_assessment_sha256"],
        "C1 assessment differs",
    )
    assessment = load_yaml(assessment_path)
    require(assessment["predictor_fit_authorized"], "fit not authorized")
    require(
        not assessment["policy_or_ranker_training_authorized"],
        "policy training boundary differs",
    )
    rows, report_paths = load_rows()
    scenes = sorted({row["scene_id"] for row in rows})
    fold_rows = []
    all_heldout = []
    for heldout_scene in scenes:
        training = [
            row for row in rows if row["scene_id"] != heldout_scene
        ]
        heldout = [
            row for row in rows if row["scene_id"] == heldout_scene
        ]
        heads = fit_heads(training, contract)
        values = evaluate_rows(
            heldout, heads, contract["calibration"]
        )
        all_heldout.extend(values)
        fold_rows.append(
            {
                "heldout_scene_family": heldout_scene,
                "training_trajectories": len(training),
                "heldout_trajectories": len(heldout),
                "simultaneous_trajectory_coverage": sum(
                    row["simultaneous_trajectory_covered"]
                    for row in values
                )
                / len(values),
                "mean_trajectory_rmse_m": sum(
                    row["trajectory_rmse_m"] for row in values
                )
                / len(values),
                "maximum_trajectory_rmse_m": max(
                    row["trajectory_rmse_m"] for row in values
                ),
            }
        )
    gates = contract["internal_cross_validation"]
    global_coverage = sum(
        row["simultaneous_trajectory_covered"]
        for row in all_heldout
    ) / len(all_heldout)
    mean_rmse = sum(
        row["trajectory_rmse_m"] for row in all_heldout
    ) / len(all_heldout)
    maximum_rmse = max(
        row["trajectory_rmse_m"] for row in all_heldout
    )
    terminal_width = 2.0 * (
        float(contract["calibration"]["coordinate_radius_m"])
        + 7.0
        * float(contract["calibration"]["coordinate_growth_mps"])
    )
    gate_checks = {
        "global_simultaneous_trajectory_coverage": global_coverage
        >= float(gates["global_simultaneous_trajectory_coverage_min"]),
        "every_heldout_family_simultaneous_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_heldout_family_simultaneous_coverage_min"
                ]
            )
            for row in fold_rows
        ),
        "mean_trajectory_rmse": mean_rmse
        <= float(gates["mean_trajectory_rmse_m_max"]),
        "maximum_trajectory_rmse": maximum_rmse
        <= float(gates["maximum_trajectory_rmse_m_max"]),
        "terminal_full_width": terminal_width
        <= float(gates["terminal_full_width_m_max"]),
        "hidden_label_feature_count": 0
        <= int(gates["hidden_label_feature_count_max"]),
    }
    cv = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C2",
        "status": (
            "internal_family_heldout_cross_validation_pass"
            if all(gate_checks.values())
            else "internal_family_heldout_cross_validation_fail"
        ),
        "split": "leave_one_scene_family_out",
        "fold_count": len(fold_rows),
        "trajectory_count": len(all_heldout),
        "global_simultaneous_trajectory_coverage": global_coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": maximum_rmse,
        "terminal_full_width_at_7s_m": terminal_width,
        "folds": fold_rows,
        "gate_checks": gate_checks,
        "cross_validation_pass": all(gate_checks.values()),
        "fresh_qualification_claim": False,
    }
    require(cv["cross_validation_pass"], "internal CV gate failed")
    heads = fit_heads(rows, contract)
    model = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C2",
        "model_id": "v3_13_c2_candidate_conditioned_contextual_ridge_1",
        "model_type": contract["model"]["model_type"],
        "feature_order": contract["model"]["feature_order"],
        "ridge_lambda_after_rms_feature_scaling": contract["model"][
            "ridge_lambda_after_rms_feature_scaling"
        ],
        "hidden_policy_runtime_input": False,
        "heads": heads,
        "calibration": {
            "coordinate_radius_m": float(
                contract["calibration"]["coordinate_radius_m"]
            ),
            "coordinate_growth_mps": float(
                contract["calibration"]["coordinate_growth_mps"]
            ),
        },
        "training_actor_trajectories": len(rows),
        "training_trace_samples": sum(
            len(row["samples"]) for row in rows
        ),
        "internal_cross_validation_pass": True,
        "fresh_qualification_pass": False,
        "runtime_ready": False,
    }
    model_payload = yaml_bytes(model)
    cv_payload = yaml_bytes(cv)
    receipt = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C2",
        "status": "deterministic_predictor_fit_pass",
        "contract_sha256": sha(read(CONTRACT)),
        "core_sha256": sha(read(CORE)),
        "model_sha256": sha(model_payload),
        "internal_cross_validation_sha256": sha(cv_payload),
        "source_report_count": len(report_paths),
        "actor_trajectory_count": len(rows),
        "trace_sample_count": sum(
            len(row["samples"]) for row in rows
        ),
        "supervised_system_identification_fit": True,
        "policy_or_ranker_training_steps": 0,
        "simulation_environment_steps": 0,
        "fresh_qualification_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {
        MODEL: model_payload,
        CV: cv_payload,
        RECEIPT: yaml_bytes(receipt),
    }


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    resources = [
        CONTRACT,
        CORE,
        FITTER,
        REVIEWER,
        Path(
            "artifacts/v3/multiactor/v3_13_c1_assessment/"
            "clone_assessment.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_c1_assessment/"
            "evidence_freeze.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_c1_assessment/"
            "independent_result_review.yaml"
        ),
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path) for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_c2_predictor_{} model_sha256={} cv_sha256={} manifest_sha256={}".format(
            "reproducible" if arguments.check else "fitted",
            sha(read(MODEL)),
            sha(read(CV)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()

