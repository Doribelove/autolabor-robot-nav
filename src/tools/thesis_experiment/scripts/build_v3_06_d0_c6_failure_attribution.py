#!/usr/bin/env python3
"""Build the pure-offline V3-06-D0 C6 failure attribution."""

import copy
import hashlib
import math
from collections import Counter
from pathlib import Path
import sys

import yaml

W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
from v3_scope_planner import core

CONTRACT = Path("config/thesis_experiments/v3/v3_06_d0_c6_failure_attribution_contract.yaml")
PLAN = Path("experiments/manifests/v3/v3_06_d0_c6_failure_attribution_plan.yaml")
ROOT = Path("artifacts/v3/analysis/v3_06_d0")
REPORT = ROOT / "v3_06_d0_c6_failure_attribution.yaml"
FREEZE = ROOT / "v3_06_d0_evidence_freeze.sha256"
REVIEW = ROOT / "v3_06_d0_independent_review.yaml"
TERMINAL = ROOT / "v3_06_d0_terminal_freeze.sha256"
PLANNER = Path("artifacts/v3/development/v3_06_c0/v3_06_c0_effective_planner_contract.yaml")
MODEL = Path("artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml")
C6_EPISODES = Path("artifacts/v3/qualification/v3_06_c6/execution/episodes")

SOURCES = (
    CONTRACT,
    PLAN,
    Path("artifacts/v3/qualification/v3_06_c6/v3_06_c6_terminal_freeze.sha256"),
    Path("artifacts/v3/qualification/v3_06_c6/execution/v3_06_c6_qualification_assessment.yaml"),
    Path("artifacts/v3/qualification/v3_06_c6/execution/v3_06_c6_independent_result_review.yaml"),
    Path("artifacts/v3/development/v3_06_c1_i1/v3_06_c1_i1_terminal_freeze.sha256"),
    Path("artifacts/v3/qualification/v3_06_c3/v3_06_c3_terminal_freeze.sha256"),
    PLANNER,
    MODEL,
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_06_d0_c6_failure_attribution.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_06_d0_c6_failure_attribution.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_06_d0_c6_failure_attribution.py"),
)


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load(path):
    value = yaml.safe_load(read(path))
    core.require(isinstance(value, dict), "YAML root differs")
    return value


def dump(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode()


def write_once(path, payload):
    target = W / path
    if target.exists():
        core.require(
            target.read_bytes() == payload,
            "refusing overwrite: {}".format(path),
        )
    else:
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)


def receipt(path):
    payload = read(path)
    return {
        "path": path.as_posix(),
        "sha256": sha(payload),
        "size_bytes": len(payload),
    }


def nearest_rank(values, probability):
    ordered = sorted(values)
    return ordered[max(0, int(math.ceil(probability * len(ordered))) - 1)]


def episode_paths(stage):
    roots = {
        "V3-06-C1": Path("artifacts/v3/development/v3_06_c1/execution/episodes"),
        "V3-06-C3": Path("artifacts/v3/qualification/v3_06_c3/execution/episodes"),
        "V3-06-C6": Path("artifacts/v3/qualification/v3_06_c6/execution/episodes"),
    }
    return sorted((W / roots[stage]).glob("*/episode_receipt.yaml"))


def latency_summary(stage):
    groups = {}
    for absolute in episode_paths(stage):
        episode = yaml.safe_load(absolute.read_bytes())
        method = episode["row"]["method"]
        groups.setdefault(method, []).extend(
            float(row["planner_latency_ms"]) for row in episode["decisions"]
        )
    return {
        method: {
            "decision_count": len(values),
            "p50_ms": nearest_rank(values, 0.50),
            "p99_ms": nearest_rank(values, 0.99),
            "maximum_ms": max(values),
        }
        for method, values in sorted(groups.items())
    }


def endpoint(candidate):
    point = next(
        row
        for row in candidate["points"]
        if abs(float(row["time_from_start_s"]) - 0.25) <= 1e-12
    )
    return float(point["x_m"]), float(point["longitudinal_velocity_mps"])


def build():
    contract = load(CONTRACT)
    plan = load(PLAN)
    core.require(
        contract["stage"] == "V3-06-D0"
        and contract["status"]
        == "posthoc_analysis_frozen_before_report_generation"
        and plan["status"] == "frozen_before_report_generation",
        "D0 contract differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "D0 authority must remain zero",
    )
    assessment = load(Path(contract["upstream"]["c6_assessment_path"]))
    core.require(
        assessment["status"] == "v3_06_c6_dense_cached_qualification_fail"
        and assessment["qualified_for_fresh_formal_evaluation"] is False,
        "C6 terminal interpretation differs",
    )
    planner = load(PLANNER)
    model = load(MODEL)
    baseline_targets = [
        float(x) for x in contract["analysis"]["baseline_target_speeds_mps"]
    ]
    snapshots = []
    episode_resources = []
    for absolute in sorted(
        (W / C6_EPISODES).glob("*sparse_control/episode_receipt.yaml")
    ):
        relative = absolute.relative_to(W)
        episode = yaml.safe_load(absolute.read_bytes())
        snapshots.extend(
            row["snapshot_runtime_input_without_policy"]
            for row in episode["decisions"]
        )
        episode_resources.append(relative)
    core.require(len(snapshots) == 614, "C6 sparse decision count differs")

    baseline_contract = copy.deepcopy(planner)
    baseline_contract["candidate_generator"][
        "target_speeds_mps"
    ] = baseline_targets
    baseline_results = [
        core.plan(snapshot, model, baseline_contract)
        for snapshot in snapshots
    ]
    grid = []
    for raw_speed in contract["analysis"]["candidate_speed_grid_mps"]:
        speed = float(raw_speed)
        amended = copy.deepcopy(planner)
        amended["candidate_generator"]["target_speeds_mps"] = sorted(
            baseline_targets + [speed]
        )
        candidate_id = "feasible_v{:03d}".format(
            int(round(speed * 100.0))
        )
        accepted = selected = stop_rescues = 0
        progress_gain = speed_gain = 0.0
        rejection_codes = Counter()
        for snapshot, baseline in zip(snapshots, baseline_results):
            result = core.plan(snapshot, model, amended)
            evaluation = next(
                row
                for row in result["evaluations"]
                if row["candidate_id"] == candidate_id
            )
            if evaluation["accepted"]:
                accepted += 1
            else:
                rejection_codes.update(evaluation["rejection_codes"])
            baseline_id = baseline["decision"]["selected_candidate_id"]
            selected_id = result["decision"]["selected_candidate_id"]
            baseline_candidate = next(
                row
                for row in baseline["candidate_set"]["candidates"]
                if row["candidate_id"] == baseline_id
            )
            selected_candidate = next(
                row
                for row in result["candidate_set"]["candidates"]
                if row["candidate_id"] == selected_id
            )
            bx, bv = endpoint(baseline_candidate)
            ax, av = endpoint(selected_candidate)
            progress_gain += ax - bx
            speed_gain += av - bv
            if selected_id == candidate_id:
                selected += 1
                stop_rescues += baseline_id == "certified_stop_v000"
        grid.append(
            {
                "speed_mps": speed,
                "accepted_decision_count": accepted,
                "selected_decision_count": selected,
                "certified_stop_rescue_count": stop_rescues,
                "aggregate_commitment_x_gain_m": progress_gain,
                "aggregate_commitment_terminal_speed_gain_mps": speed_gain,
                "leading_rejection_codes": [
                    {"code": code, "count": count}
                    for code, count in rejection_codes.most_common(6)
                ],
            }
        )
    best = max(
        grid, key=lambda row: row["aggregate_commitment_x_gain_m"]
    )
    v020 = next(row for row in grid if row["speed_mps"] == 0.20)
    adjacent_gain = (
        best["aggregate_commitment_x_gain_m"]
        - v020["aggregate_commitment_x_gain_m"]
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-D0",
        "status": "offline_c6_failure_attribution_complete",
        "formal_result": False,
        "c6_terminal_result": {
            "qualified": False,
            "point_effect": assessment["paired_effect"][
                "point_relative_reduction_vs_control"
            ],
            "positive_pair_fraction": assessment["paired_effect"][
                "positive_pair_fraction"
            ],
            "planner_latency_p99_ms": assessment[
                "pooled_integration_metrics"
            ]["planner_latency_p99_ms"],
            "failed_gates": sorted(
                key
                for key, value in assessment[
                    "qualification_gates"
                ].items()
                if value is False
            ),
        },
        "source_population": {
            "episode_count": len(episode_resources),
            "decision_count": len(snapshots),
            "method": "sparse_control",
        },
        "straight_creep_grid": grid,
        "grid_summary": {
            "best_speed_mps": best["speed_mps"],
            "best_aggregate_commitment_x_gain_m": best[
                "aggregate_commitment_x_gain_m"
            ],
            "v020_aggregate_commitment_x_gain_m": v020[
                "aggregate_commitment_x_gain_m"
            ],
            "best_minus_v020_gain_m": adjacent_gain,
            "best_minus_v020_gain_per_decision_m": adjacent_gain
            / len(snapshots),
            "nearby_speed_search_has_material_room": adjacent_gain >= 0.10,
        },
        "recorded_latency_by_stage_and_method": {
            stage: latency_summary(stage)
            for stage in contract["analysis"]["latency_stages"]
        },
        "diagnosis": {
            "straight_creep_family_saturated": adjacent_gain < 0.10,
            "python_monkeypatch_cache_realtime_qualified": False,
            "next_factor_class": (
                "spatial_or_temporal_ackermann_trajectory_diversity"
            ),
            "existing_speed_only_predictor_valid_for_spatial_interventions": False,
            "new_spatial_intervention_data_and_calibration_required": True,
        },
        "authority": contract["authority"],
        "claim_boundary": contract["claim_boundary"],
        "source_receipts": [
            receipt(path)
            for path in SOURCES + tuple(episode_resources)
        ],
    }
    return report


def main():
    report = build()
    payload = dump(report)
    write_once(REPORT, payload)
    resources = [
        Path(row["path"]) for row in report["source_receipts"]
    ] + [REPORT]
    freeze = (
        "\n".join(
            "{}  {}".format(sha(read(path)), path) for path in resources
        )
        + "\n"
    ).encode()
    write_once(FREEZE, freeze)
    print(
        "status={} report_sha256={}".format(report["status"], sha(payload))
    )


if __name__ == "__main__":
    main()
