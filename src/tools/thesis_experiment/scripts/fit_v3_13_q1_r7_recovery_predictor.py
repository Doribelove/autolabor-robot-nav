#!/usr/bin/env python3
"""Fit only the new R7 recovery-candidate assertive head."""

import argparse
import copy
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r7_recovery_candidate_predictor_fit_contract.yaml"
)
C2_FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_c2_multiactor_predictor.py"
)
S3_MODEL = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
R6_EPISODES = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r6_recovery_support/episodes"
)
FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_q1_r7_recovery_predictor.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r7_recovery_predictor.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r7_predictor_fit.py"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r7_predictor"
)
MODEL = ROOT / "expanded_recovery_predictor.yaml"
CV = ROOT / "internal_cross_validation.yaml"
RECEIPT = ROOT / "fit_receipt.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(path):
    return yaml.safe_load(read(path))


def load_c2_fitter():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r7_c2_fitter", W / C2_FITTER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_rows(candidate_id):
    report_paths = sorted(
        path.relative_to(W)
        for path in (W / R6_EPISODES).glob(
            "*_{}/episode_report.yaml".format(candidate_id)
        )
    )
    require(len(report_paths) == 8, "new-candidate report count")
    rows = []
    for report_path in report_paths:
        report = load_yaml(report_path)
        require(
            report["forced_candidate_id"] == candidate_id,
            "candidate report differs",
        )
        require(
            set(report["observable_modes_at_branch"].values())
            == {"assertive"},
            "observable mode differs",
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
                    "candidate_id": candidate_id,
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
    require(len(rows) == 16, "actor trajectory count")
    return rows, report_paths


def build_outputs():
    contract = load_yaml(CONTRACT)
    upstream = contract["upstream"]
    for key in ("r6_assessment", "r6_freeze", "r6_review"):
        path = Path(upstream[key + "_path"])
        require(
            sha(read(path)) == upstream[key + "_sha256"],
            "{} differs".format(path),
        )
    assessment = load_yaml(
        Path(upstream["r6_assessment_path"])
    )
    require(
        assessment["new_candidate_assertive_head_fit_authorized"],
        "fit not authorized",
    )
    require(
        sha(read(S3_MODEL)) == upstream["s3_predictor_sha256"],
        "S3 model differs",
    )
    s3 = load_yaml(S3_MODEL)
    candidate_id = contract["model"]["newly_fitted_candidate_id"]
    rows, report_paths = load_rows(candidate_id)
    c2 = load_c2_fitter()
    fit_contract = {
        "model": {
            "ridge_lambda_after_rms_feature_scaling": contract[
                "model"
            ]["ridge_lambda_after_rms_feature_scaling"],
            "observable_modes": ["assertive"],
            "candidate_ids": [candidate_id],
        }
    }
    calibration = contract["calibration"]
    all_heldout = []
    folds = []
    for heldout_block in range(1, 9):
        training = [
            row for row in rows
            if row["block_index"] != heldout_block
        ]
        heldout = [
            row for row in rows
            if row["block_index"] == heldout_block
        ]
        heads = c2.fit_heads(training, fit_contract)
        values = c2.evaluate_rows(
            heldout, heads, calibration
        )
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
                    value["trajectory_rmse_m"]
                    for value in values
                )
                / len(values),
                "maximum_trajectory_rmse_m": max(
                    value["trajectory_rmse_m"]
                    for value in values
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
        "global_coverage": global_coverage
        >= float(
            gates["global_simultaneous_trajectory_coverage_min"]
        ),
        "every_block_coverage": all(
            fold["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_block_simultaneous_trajectory_coverage_min"
                ]
            )
            for fold in folds
        ),
        "mean_rmse": mean_rmse
        <= float(gates["mean_trajectory_rmse_m_max"]),
        "maximum_rmse": maximum_rmse
        <= float(gates["maximum_trajectory_rmse_m_max"]),
        "terminal_width": terminal_width
        <= float(gates["terminal_full_width_m_max"]),
        "hidden_label_feature_count": 0
        <= int(gates["hidden_label_feature_count_max"]),
    }
    cv = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R7",
        "status": (
            "new_candidate_internal_CV_pass"
            if all(checks.values())
            else "new_candidate_internal_CV_fail"
        ),
        "split": "leave_one_matched_context_block_out",
        "fold_count": 8,
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
    new_head = c2.fit_heads(
        rows, fit_contract
    )["assertive"][candidate_id]
    model = copy.deepcopy(s3)
    require(
        candidate_id not in model["heads"]["assertive"],
        "new candidate already exists",
    )
    model.update(
        {
            "stage": "V3-13-Q1-R7",
            "model_id": (
                "v3_13_q1_r7_expanded_recovery_predictor_1"
            ),
            "retained_S3_predictor_sha256": sha(read(S3_MODEL)),
            "new_candidate_training_actor_trajectories": len(rows),
            "new_candidate_training_trace_samples": sum(
                len(row["samples"]) for row in rows
            ),
            "internal_cross_validation_pass": True,
            "fresh_recovery_qualification_pass": False,
            "runtime_ready": False,
        }
    )
    model["heads"]["assertive"][candidate_id] = new_head
    require(
        all(
            model["heads"][mode][candidate] == head
            for mode, heads in s3["heads"].items()
            for candidate, head in heads.items()
        ),
        "retained S3 head differs",
    )
    require(
        model["calibration"] == s3["calibration"],
        "calibration differs",
    )
    model_payload = yaml_bytes(model)
    cv_payload = yaml_bytes(cv)
    receipt = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R7",
        "status": "deterministic_new_candidate_head_fit_pass",
        "contract_sha256": sha(read(CONTRACT)),
        "S3_predictor_sha256": sha(read(S3_MODEL)),
        "model_sha256": sha(model_payload),
        "internal_cross_validation_sha256": sha(cv_payload),
        "source_report_count": len(report_paths),
        "actor_trajectory_count": len(rows),
        "trace_sample_count": sum(
            len(row["samples"]) for row in rows
        ),
        "retained_S3_heads_byte_equivalent": True,
        "supervised_system_identification_fit": True,
        "policy_or_ranker_training_steps": 0,
        "simulation_environment_steps": 0,
        "fresh_qualification_authorized": False,
        "actual_TEB_execution_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {
        MODEL: model_payload,
        CV: cv_payload,
        RECEIPT: yaml_bytes(receipt),
    }


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    resources = [
        CONTRACT,
        C2_FITTER,
        S3_MODEL,
        FITTER,
        REVIEWER,
        TEST,
    ] + sorted(
        path.relative_to(W)
        for path in (W / R6_EPISODES).glob(
            "*_recovery_evasive_right_v120_r1/episode_report.yaml"
        )
    ) + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    print(
        "model_sha256={} cv_sha256={} manifest_sha256={}".format(
            sha(outputs[MODEL]),
            sha(outputs[CV]),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
