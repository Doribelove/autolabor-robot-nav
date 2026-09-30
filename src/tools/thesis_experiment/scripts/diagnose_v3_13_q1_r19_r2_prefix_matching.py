#!/usr/bin/env python3
"""Pure-offline attribution of the R19-R2 clone-prefix mismatch."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
COLLECTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r2_boundary_atomic_support"
)
ASSESSMENT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r2_assessment"
)
ROOT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r2_d1_prefix_matching_diagnosis"
)
REPORT = ROOT / "prefix_matching_diagnosis.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
INPUTS = [
    COLLECTION / "collection_summary.yaml",
    ASSESSMENT / "support_assessment.yaml",
    ASSESSMENT / "evidence_freeze.yaml",
    ASSESSMENT / "independent_result_review.yaml",
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r19_r2_boundary_atomic_support_contract.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_clone_episode.py"
    ),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "diagnose_v3_13_q1_r19_r2_prefix_matching.py"
    ),
]


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


def episode_reports():
    paths = sorted((W / COLLECTION / "episodes").glob(
        "*/episode_report.yaml"
    ))
    require(len(paths) == 40, "R19-R2 report count differs")
    return [
        (path, yaml.safe_load(path.read_bytes()))
        for path in paths
    ]


def range_of(values):
    return max(values) - min(values)


def build_outputs():
    contract = yaml.safe_load(read(INPUTS[4]))
    summary = yaml.safe_load(read(INPUTS[0]))
    assessment = yaml.safe_load(read(INPUTS[1]))
    review = yaml.safe_load(read(INPUTS[3]))
    require(summary["completed_units"] == 40, "collection incomplete")
    require(not summary["collection_gate_pass"], "unexpected pass")
    require(
        assessment["status"]
        == "fresh_boundary_atomic_five_action_support_assessment_fail",
        "assessment status differs",
    )
    require(
        review["status"]
        == "independent_boundary_atomic_support_assessment_fail",
        "review status differs",
    )
    require(
        summary["r3_terminal_guard_execution_rate"] == 1.0
        and summary["r3_boundary_atomic_execution_rate"] == 1.0,
        "R3 execution gates differ",
    )
    grouped = {}
    for path, report in episode_reports():
        grouped.setdefault(int(report["block_index"]), []).append(
            (path, report)
        )
    block_metrics = []
    for block_index in sorted(grouped):
        rows = grouped[block_index]
        ego_x = [
            float(row["common_stop_prefix"]["decision_pose"]["x_m"])
            for _, row in rows
        ]
        gate_x = [
            float(row["warmup"]["gate_pose"]["x_m"])
            for _, row in rows
        ]
        warmup_duration = [
            float(row["warmup"]["duration_s"])
            for _, row in rows
        ]
        actor_ranges = []
        for track_id in (1, 2):
            coordinates = []
            for _, row in rows:
                track = next(
                    item
                    for item in row["common_stop_prefix"][
                        "decision_actor_tracks"
                    ]
                    if int(item["track_id"]) == track_id
                )
                coordinates.append(float(track["coordinate_m"]))
            actor_ranges.append(
                {
                    "track_id": track_id,
                    "coordinate_range_m": range_of(coordinates),
                }
            )
        block_metrics.append(
            {
                "block_index": block_index,
                "seed": int(rows[0][1]["seed"]),
                "warmup_duration_range_s": range_of(warmup_duration),
                "gate_ego_x_range_m": range_of(gate_x),
                "decision_ego_x_range_m": range_of(ego_x),
                "actor_coordinate_ranges_m": actor_ranges,
            }
        )
    gates = contract["matching_gates"]
    failed_blocks = [
        block
        for block in block_metrics
        if (
            block["decision_ego_x_range_m"]
            > float(gates["paired_decision_ego_x_difference_max_m"])
            or max(
                item["coordinate_range_m"]
                for item in block["actor_coordinate_ranges_m"]
            )
            > float(
                gates[
                    "paired_decision_actor_coordinate_difference_max_m"
                ]
            )
        )
    ]
    require(
        [block["block_index"] for block in failed_blocks] == [1, 8],
        "failed-block attribution differs",
    )
    runtime_text = read(INPUTS[5]).decode("utf-8")
    actor_text = read(INPUTS[6]).decode("utf-8")
    require(
        "rate = rospy.Rate(20)" in runtime_text
        and "pose[\"x_m\"] >= self.gate_world_x - 0.018"
        in runtime_text,
        "warmup source signature differs",
    )
    require(
        "activation_ego_x_m_" in actor_text
        and "ego_position.X() < activation_ego_x_m_" in actor_text,
        "actor activation source signature differs",
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R2-D1",
        "status": "offline_prefix_matching_diagnosis_pass",
        "upstream": {
            "collection_summary_sha256": sha(read(INPUTS[0])),
            "support_assessment_sha256": sha(read(INPUTS[1])),
            "evidence_freeze_sha256": sha(read(INPUTS[2])),
            "independent_result_review_sha256": sha(read(INPUTS[3])),
        },
        "replayed_episode_count": 40,
        "failed_matching_blocks": failed_blocks,
        "failure_scope": {
            "paired_ego_x_gate": False,
            "paired_actor_coordinate_gate": False,
            "all_other_collection_gates": True,
            "r3_terminal_guard_execution_rate": 1.0,
            "r3_boundary_atomic_execution_rate": 1.0,
            "r3_minimum_dynamic_clearance_m": float(
                assessment["action_statistics"][
                    "recovery_evasive_right_v120_r3"
                ]["minimum_clearance_m"]
            ),
            "r3_contact_state_count": int(
                assessment["action_statistics"][
                    "recovery_evasive_right_v120_r3"
                ]["contact_state_count"]
            ),
        },
        "causal_attribution": {
            "warmup_command_loop_hz": 20,
            "warmup_sample_period_s": 0.05,
            "failed_block_warmup_duration_range_s": 0.05,
            "gate_crossing_is_sampled_not_state_locked": True,
            "actor_activation_is_conditioned_on_ego_x": True,
            "mechanism": (
                "one 20 Hz warmup sample changes the stopped ego pose; "
                "the ego-x actor activation and reactive prefix then "
                "amplify the cloned branch-state difference"
            ),
            "post_branch_boundary_atomic_execution_is_not_causal": True,
        },
        "single_factor_repair_design": {
            "factor": "prebranch_warmup_command_sampling_rate_hz",
            "control": 20,
            "treatment": 100,
            "gate_position_and_speed_thresholds_unchanged": True,
            "actor_plugin_unchanged": True,
            "candidate_geometry_and_postbranch_execution_unchanged": True,
            "next_stage": (
                "fresh_one_block_five_action_matching_preflight"
            ),
        },
        "R19_R2_remains_failed": True,
        "R19_R2_seed_reuse_authorized": False,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    report_payload = yaml_bytes(report)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in INPUTS
    )
    manifest += "{}  {}\n".format(sha(report_payload), REPORT)
    return report_payload, manifest.encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent: {}".format(path))
        require(target.read_bytes() == payload, "output drift: {}".format(path))
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite: {}".format(path))
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report, manifest = build_outputs()
    write_or_check(REPORT, report, args.check)
    write_or_check(MANIFEST, manifest, args.check)
    print(
        "status=offline_prefix_matching_diagnosis_pass "
        "diagnosis_sha256={} manifest_sha256={}".format(
            sha(report), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
