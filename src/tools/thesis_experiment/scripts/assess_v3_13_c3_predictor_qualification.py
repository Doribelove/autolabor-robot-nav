#!/usr/bin/env python3
"""Assess fresh V3-13-C3 predictor qualification evidence."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT_DIR = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPT_DIR))
import run_v3_13_c1_multiactor_clones as c1_runner


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c3_predictor_qualification_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_c2_predictor/"
    "candidate_conditioned_predictor.yaml"
)
READINESS = Path("artifacts/v3/multiactor/v3_13_c3_readiness")
EXECUTION = Path("artifacts/v3/multiactor/v3_13_c3_execution")
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_c3_predictor_qualification.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_c3_predictor_result.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_c3_assessment")
ASSESSMENT = ROOT / "predictor_qualification.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"


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


def features(track, time_s):
    time_s = float(time_s)
    return [
        time_s,
        time_s * time_s,
        time_s * time_s * time_s,
        float(track["coordinate_speed_mps"]) * time_s,
        -float(track["coordinate_m"]) * time_s,
        float(track["path_origin_x_m"]) * time_s,
        float(track["path_origin_x_m"]) * time_s * time_s,
    ]


def build_outputs():
    contract = load_yaml(CONTRACT)
    model = load_yaml(MODEL)
    require(
        sha(read(MODEL)) == contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    schedule = load_yaml(EXECUTION / "exact_schedule.yaml")
    released = load_yaml(READINESS / "exact_schedule.yaml")
    require(schedule == released, "execution schedule differs")
    rows = schedule["rows"]
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 64, "episode count")
    reports = []
    resources = [
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "raw_execution_summary.yaml",
        CONTRACT,
        MODEL,
        ASSESSOR,
        REVIEWER,
    ]
    for row, root in zip(rows, roots):
        receipt = load_yaml(root / "matrix_receipt.yaml")
        report = load_yaml(root / "episode_report.yaml")
        require(receipt["row"] == row, "receipt row differs")
        require(
            receipt["episode_report_sha256"]
            == sha(read(root / "episode_report.yaml")),
            "report hash differs",
        )
        require(report["episode_collection_pass"], "collection failed")
        reports.append(report)
        resources.extend(
            root / name
            for name in (
                "world.sdf",
                "scene_spec.yaml",
                "actor_specs.yaml",
                "episode_report.yaml",
                "matrix_receipt.yaml",
                "roslaunch.log",
            )
        )
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(len(events) == 130, "journal event count")
    require(events[0]["event"] == "execution_opened", "journal opening")
    require(events[-1]["event"] == "execution_complete", "journal terminal")
    require(
        not any(event["event"] == "terminal_failure" for event in events),
        "terminal failure observed",
    )
    blocks = {}
    trajectory_rows = []
    radius = float(model["calibration"]["coordinate_radius_m"])
    growth = float(model["calibration"]["coordinate_growth_mps"])
    for report in reports:
        blocks.setdefault(int(report["block_index"]), []).append(report)
        for gate_track in report["warmup"]["gate_actor_tracks"]:
            track_id = int(gate_track["track_id"])
            mode = report["observable_modes_at_gate"][track_id]
            head = model["heads"][mode][report["forced_candidate_id"]]
            coefficients = [
                float(value) for value in head["coefficients"]
            ]
            errors = []
            covered = True
            for sample in report["result"]["trace"]:
                actor = next(
                    item
                    for item in sample["actors"]
                    if int(item["track_id"]) == track_id
                )
                time_s = float(sample["elapsed_s"])
                center = float(gate_track["coordinate_m"]) + sum(
                    coefficient * feature
                    for coefficient, feature in zip(
                        coefficients, features(gate_track, time_s)
                    )
                )
                error = abs(float(actor["coordinate_m"]) - center)
                errors.append(error)
                if error > radius + growth * time_s + 1.0e-12:
                    covered = False
            trajectory_rows.append(
                {
                    "scene_id": report["scene_id"],
                    "block_index": int(report["block_index"]),
                    "candidate_id": report["forced_candidate_id"],
                    "track_id": track_id,
                    "observable_mode": mode,
                    "trajectory_rmse_m": math.sqrt(
                        sum(value * value for value in errors)
                        / len(errors)
                    ),
                    "maximum_absolute_error_m": max(errors),
                    "simultaneous_trajectory_covered": covered,
                }
            )
    block_checks = []
    for block_index, block in sorted(blocks.items()):
        actions = {
            report["forced_candidate_id"] for report in block
        }
        ego_x = [
            float(report["warmup"]["gate_pose"]["x_m"])
            for report in block
        ]
        actor_ranges = []
        for track_id in (1, 2):
            tracks = [
                next(
                    track
                    for track in report["warmup"]["gate_actor_tracks"]
                    if int(track["track_id"]) == track_id
                )
                for report in block
            ]
            actor_ranges.append(
                {
                    "track_id": track_id,
                    "coordinate_range_m": max(
                        float(track["coordinate_m"]) for track in tracks
                    )
                    - min(
                        float(track["coordinate_m"]) for track in tracks
                    ),
                    "speed_range_mps": max(
                        float(track["coordinate_speed_mps"])
                        for track in tracks
                    )
                    - min(
                        float(track["coordinate_speed_mps"])
                        for track in tracks
                    ),
                }
            )
        block_checks.append(
            {
                "block_index": block_index,
                "full_action_support": actions
                == set(contract["qualification_population"]["actions"]),
                "minimum_propensity": min(
                    float(report["behavior_propensity"])
                    for report in block
                ),
                "ego_x_range_m": max(ego_x) - min(ego_x),
                "actor_ranges": actor_ranges,
            }
        )
    scene_rows = []
    for scene_id in sorted(
        {row["scene_id"] for row in trajectory_rows}
    ):
        selected = [
            row for row in trajectory_rows if row["scene_id"] == scene_id
        ]
        scene_rows.append(
            {
                "scene_id": scene_id,
                "trajectory_count": len(selected),
                "simultaneous_trajectory_coverage": sum(
                    row["simultaneous_trajectory_covered"]
                    for row in selected
                )
                / len(selected),
                "mean_trajectory_rmse_m": sum(
                    row["trajectory_rmse_m"] for row in selected
                )
                / len(selected),
                "maximum_trajectory_rmse_m": max(
                    row["trajectory_rmse_m"] for row in selected
                ),
            }
        )
    gates = contract["fresh_qualification_gates"]
    coverage = sum(
        row["simultaneous_trajectory_covered"]
        for row in trajectory_rows
    ) / len(trajectory_rows)
    mean_rmse = sum(
        row["trajectory_rmse_m"] for row in trajectory_rows
    ) / len(trajectory_rows)
    max_rmse = max(
        row["trajectory_rmse_m"] for row in trajectory_rows
    )
    max_ego = max(row["ego_x_range_m"] for row in block_checks)
    max_coordinate = max(
        actor["coordinate_range_m"]
        for row in block_checks
        for actor in row["actor_ranges"]
    )
    max_speed = max(
        actor["speed_range_mps"]
        for row in block_checks
        for actor in row["actor_ranges"]
    )
    gate_checks = {
        "completed_units_exact": len(reports)
        == int(gates["completed_units_exact"]),
        "complete_action_support_rate": all(
            row["full_action_support"] for row in block_checks
        ),
        "minimum_behavior_propensity": min(
            row["minimum_propensity"] for row in block_checks
        )
        >= float(gates["minimum_behavior_propensity"]),
        "global_simultaneous_trajectory_coverage": coverage
        >= float(gates["global_simultaneous_trajectory_coverage_min"]),
        "every_scene_family_simultaneous_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_scene_family_simultaneous_coverage_min"
                ]
            )
            for row in scene_rows
        ),
        "mean_trajectory_rmse": mean_rmse
        <= float(gates["mean_trajectory_rmse_m_max"]),
        "maximum_trajectory_rmse": max_rmse
        <= float(gates["maximum_trajectory_rmse_m_max"]),
        "all_finite_tracks": all(
            c1_runner.finite_trace(report) for report in reports
        ),
        "common_gate_valid_rate": all(
            report["gate_physical_valid"] for report in reports
        ),
        "paired_gate_ego_position": max_ego
        <= float(gates["paired_gate_ego_position_difference_max_m"]),
        "paired_gate_actor_coordinate": max_coordinate
        <= float(
            gates["paired_gate_actor_coordinate_difference_max_m"]
        ),
        "paired_gate_actor_speed": max_speed
        <= float(
            gates["paired_gate_actor_speed_difference_max_mps"]
        ),
    }
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C3",
        "status": (
            "fresh_predictor_qualification_pass"
            if all(gate_checks.values())
            else "fresh_predictor_qualification_fail"
        ),
        "predictor_sha256": sha(read(MODEL)),
        "completed_units": len(reports),
        "trajectory_count": len(trajectory_rows),
        "global_simultaneous_trajectory_coverage": coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": max_rmse,
        "scene_results": scene_rows,
        "maximum_gate_ego_x_range_m": max_ego,
        "maximum_gate_actor_coordinate_range_m": max_coordinate,
        "maximum_gate_actor_speed_range_mps": max_speed,
        "gate_checks": gate_checks,
        "fresh_predictor_qualification_pass": all(
            gate_checks.values()
        ),
        "actual_teb_multiactor_qualification_release_authorized": all(
            gate_checks.values()
        ),
        "policy_or_ranker_training_authorized": False,
        "perception_claim_authorized": False,
        "performance_claim_authorized": False,
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    freeze = {
        "schema_version": "1.0",
        "stage": "V3-13-C3",
        "status": "closed_fresh_predictor_qualification_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
    }
    return {
        ASSESSMENT: yaml_bytes(assessment),
        FREEZE: yaml_bytes(freeze),
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
    assessment = load_yaml(ASSESSMENT)
    print(
        "status={} assessment_sha256={} freeze_sha256={}".format(
            assessment["status"],
            sha(read(ASSESSMENT)),
            sha(read(FREEZE)),
        )
    )
    return 0 if assessment["fresh_predictor_qualification_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
