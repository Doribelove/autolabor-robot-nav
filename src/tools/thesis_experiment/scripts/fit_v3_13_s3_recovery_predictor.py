#!/usr/bin/env python3
"""Fit the V3-13-S3 assertive recovery heads and replay internal CV."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s3_recovery_predictor_fit_contract.yaml"
)
C2_FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_c2_multiactor_predictor.py"
)
CORE = Path(
    "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_s3_recovery_predictor.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s3_recovery_predictor.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_s3_predictor.py"
)
S2 = Path("artifacts/v3/multiactor/v3_13_s2_execution/episodes")
ROOT = Path("artifacts/v3/multiactor/v3_13_s3_predictor")
MODEL = ROOT / "candidate_conditioned_recovery_predictor.yaml"
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


def load_c2_fitter():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s3_c2_fitter", W / C2_FITTER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_rows():
    rows = []
    report_paths = sorted(
        path.relative_to(W)
        for path in (W / S2).glob("*/episode_report.yaml")
    )
    require(len(report_paths) == 32, "S2 report count")
    for report_path in report_paths:
        report = load_yaml(report_path)
        require(
            set(report["observable_modes_at_branch"].values())
            == {"assertive"},
            "non-assertive S2 report",
        )
        for initial in report["common_stop_prefix"][
            "decision_actor_tracks"
        ]:
            track_id = int(initial["track_id"])
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
                    "observable_mode": "assertive",
                    "initial_coordinate_m": float(
                        initial["coordinate_m"]
                    ),
                    "initial_speed_mps": float(
                        initial["coordinate_speed_mps"]
                    ),
                    "path_origin_x_m": float(
                        initial["path_origin_x_m"]
                    ),
                    "samples": samples,
                }
            )
    require(len(rows) == 64, "S2 actor trajectory count")
    return rows, report_paths


def build_outputs():
    contract = load_yaml(CONTRACT)
    assessment_path = Path(contract["upstream"]["s2_assessment_path"])
    require(
        sha(read(assessment_path))
        == contract["upstream"]["s2_assessment_sha256"],
        "S2 assessment differs",
    )
    assessment = load_yaml(assessment_path)
    require(
        assessment["assertive_recovery_predictor_fit_authorized"],
        "assertive recovery fit not authorized",
    )
    require(
        not assessment["policy_or_ranker_training_authorized"],
        "policy training boundary differs",
    )
    c2_path = Path(contract["upstream"]["c2_predictor_path"])
    require(
        sha(read(c2_path))
        == contract["upstream"]["c2_predictor_sha256"],
        "C2 predictor differs",
    )
    c2_model = load_yaml(c2_path)
    c2 = load_c2_fitter()
    rows, report_paths = load_rows()
    fit_contract = {
        "model": {
            "ridge_lambda_after_rms_feature_scaling": contract[
                "model"
            ]["ridge_lambda_after_rms_feature_scaling"],
            "observable_modes": ["assertive"],
            "candidate_ids": contract["model"]["candidate_ids"],
        }
    }
    calibration = contract["calibration"]
    all_heldout = []
    folds = []
    for heldout_block in range(1, 9):
        training = [
            row
            for row in rows
            if row["block_index"] != heldout_block
        ]
        heldout = [
            row
            for row in rows
            if row["block_index"] == heldout_block
        ]
        heads = c2.fit_heads(training, fit_contract)
        values = c2.evaluate_rows(heldout, heads, calibration)
        all_heldout.extend(values)
        folds.append(
            {
                "heldout_block_index": heldout_block,
                "training_trajectories": len(training),
                "heldout_trajectories": len(heldout),
                "simultaneous_trajectory_coverage": sum(
                    value["simultaneous_trajectory_covered"]
                    for value in values
                )
                / len(values),
                "mean_trajectory_rmse_m": sum(
                    value["trajectory_rmse_m"] for value in values
                )
                / len(values),
                "maximum_trajectory_rmse_m": max(
                    value["trajectory_rmse_m"] for value in values
                ),
            }
        )
    global_coverage = sum(
        value["simultaneous_trajectory_covered"]
        for value in all_heldout
    ) / len(all_heldout)
    mean_rmse = sum(
        value["trajectory_rmse_m"] for value in all_heldout
    ) / len(all_heldout)
    maximum_rmse = max(
        value["trajectory_rmse_m"] for value in all_heldout
    )
    terminal_width = 2.0 * (
        float(calibration["coordinate_radius_m"])
        + 7.0 * float(calibration["coordinate_growth_mps"])
    )
    gates = contract["internal_cross_validation"]
    checks = {
        "global_simultaneous_trajectory_coverage": global_coverage
        >= float(gates["global_simultaneous_trajectory_coverage_min"]),
        "every_block_simultaneous_trajectory_coverage": all(
            fold["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_block_simultaneous_trajectory_coverage_min"
                ]
            )
            for fold in folds
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
        "stage": "V3-13-S3",
        "status": (
            "internal_leave_one_block_out_validation_pass"
            if all(checks.values())
            else "internal_leave_one_block_out_validation_fail"
        ),
        "split": "leave_one_matched_context_block_out",
        "fold_count": len(folds),
        "trajectory_count": len(all_heldout),
        "global_simultaneous_trajectory_coverage": global_coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": maximum_rmse,
        "terminal_full_width_at_7s_m": terminal_width,
        "folds": folds,
        "gate_checks": checks,
        "cross_validation_pass": all(checks.values()),
        "fresh_qualification_claim": False,
    }
    require(cv["cross_validation_pass"], "internal CV failed")
    new_heads = c2.fit_heads(rows, fit_contract)["assertive"]
    model = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S3",
        "model_id": (
            "v3_13_s3_candidate_conditioned_contextual_recovery_1"
        ),
        "model_type": c2_model["model_type"],
        "feature_order": list(c2_model["feature_order"]),
        "ridge_lambda_after_rms_feature_scaling": contract["model"][
            "ridge_lambda_after_rms_feature_scaling"
        ],
        "hidden_policy_runtime_input": False,
        "heads": {
            "cautious": c2_model["heads"]["cautious"],
            "hesitant": c2_model["heads"]["hesitant"],
            "assertive": new_heads,
        },
        "calibration": {
            "coordinate_radius_m": float(
                calibration["coordinate_radius_m"]
            ),
            "coordinate_growth_mps": float(
                calibration["coordinate_growth_mps"]
            ),
        },
        "retained_C2_predictor_sha256": sha(read(c2_path)),
        "assertive_training_actor_trajectories": len(rows),
        "assertive_training_trace_samples": sum(
            len(row["samples"]) for row in rows
        ),
        "internal_cross_validation_pass": True,
        "fresh_recovery_qualification_pass": False,
        "runtime_ready": False,
    }
    model_payload = yaml_bytes(model)
    cv_payload = yaml_bytes(cv)
    receipt = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S3",
        "status": "deterministic_assertive_recovery_fit_pass",
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
        C2_FITTER,
        CORE,
        FITTER,
        REVIEWER,
        TEST,
        Path(
            "artifacts/v3/multiactor/v3_13_s2_assessment/"
            "sequential_clone_assessment.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_s2_assessment/"
            "evidence_freeze.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_s2_assessment/"
            "independent_result_review.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_c2_predictor/"
            "candidate_conditioned_predictor.yaml"
        ),
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_s3_predictor_{} model_sha256={} "
        "cv_sha256={} manifest_sha256={}".format(
            "reproducible" if arguments.check else "fit",
            sha(read(MODEL)),
            sha(read(CV)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
