#!/usr/bin/env python3
"""Build deterministic, pure-offline paper exports from frozen V3-09-F1 evidence."""
import argparse
import csv
import hashlib
import io
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/formal/v3_10_p1_paper_closure")
CONTRACT = Path("config/thesis_experiments/v3/v3_10_p1_paper_evidence_closure_contract.yaml")
PLAN = Path("experiments/manifests/v3/v3_10_p1_offline_paper_closure_plan.yaml")
ASSESSMENT = Path("artifacts/v3/formal/v3_09_f1/execution/v3_09_f1_formal_assessment.yaml")
REVIEW = Path("artifacts/v3/formal/v3_09_f1/execution/v3_09_f1_independent_result_review.yaml")
HISTORICAL_EVIDENCE_FREEZE = Path("artifacts/v3/formal/v3_09_f1/execution/v3_09_f1_evidence_freeze.sha256")
HISTORICAL_TERMINAL_FREEZE = Path("artifacts/v3/formal/v3_09_f1/v3_09_f1_terminal_freeze.sha256")
BUILDER = Path("src/tools/thesis_experiment/scripts/build_v3_10_p1_paper_closure.py")
REVIEWER = Path("src/tools/thesis_experiment/scripts/review_v3_10_p1_paper_closure.py")
TEST = Path("src/tools/thesis_experiment/tests/test_v3_10_p1.py")

SUMMARY = ROOT / "v3_10_p1_formal_summary.csv"
POLICY = ROOT / "v3_10_p1_policy_results.csv"
BLOCK = ROOT / "v3_10_p1_block_results.csv"
FIGURE = ROOT / "v3_10_p1_formal_result.svg"
CLAIMS = ROOT / "v3_10_p1_claims.yaml"
MANIFEST = ROOT / "v3_10_p1_evidence_manifest.sha256"

EXPECTED_ASSESSMENT_SHA = "d950256f4da310f65d604031ec72009e014facfb7cb0c14be8cf19ce1b9c346b"
EXPECTED_REVIEW_SHA = "206732c86b536e7f9a520c58dac6d386ff6752595bdc767d4957e4804e5f8d5a"
EXPECTED_EVIDENCE_FREEZE_SHA = "5a86ee866ab1b990eb430b6692364ec84dc69314af56ce55c7ede2042cb3851d"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    return (W / relative).read_bytes()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def csv_bytes(header, rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(header)
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "missing export: {}".format(relative))
        require(path.read_bytes() == payload, "export drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        require(path.read_bytes() == payload, "refusing overwrite: {}".format(relative))
    else:
        path.write_bytes(payload)


def render_svg(spatial_mean, baseline_mean, point, lower):
    scale = 22.0
    spatial_width = spatial_mean * scale
    baseline_width = baseline_mean * scale
    point_x = 520 + point * 900
    lower_x = 520 + lower * 900
    gate5_x = 520 + 0.05 * 900
    gate10_x = 520 + 0.10 * 900
    return """<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="420" viewBox="0 0 1000 420">
<rect width="1000" height="420" fill="#ffffff"/>
<style>
text {{ font-family: DejaVu Sans, Arial, sans-serif; fill: #172033; }}
.title {{ font-size: 24px; font-weight: 700; }}
.label {{ font-size: 16px; }}
.small {{ font-size: 13px; fill: #4a5568; }}
</style>
<text x="40" y="42" class="title">V3-09-F1 formal paired simulation result</text>
<text x="40" y="72" class="small">30 fresh blocks · 180 episodes · both methods 100% goal success</text>
<text x="40" y="116" class="label">Restricted goal time (lower is better)</text>
<text x="40" y="158" class="small">V3 spatial-safe</text>
<rect x="180" y="138" width="{spatial_width:.3f}" height="28" rx="4" fill="#1677ff"/>
<text x="{spatial_label_x:.3f}" y="158" class="label">{spatial_mean:.3f} s</text>
<text x="40" y="208" class="small">Fixed-v1.20 shielded</text>
<rect x="180" y="188" width="{baseline_width:.3f}" height="28" rx="4" fill="#94a3b8"/>
<text x="{baseline_label_x:.3f}" y="208" class="label">{baseline_mean:.3f} s</text>
<line x1="520" y1="128" x2="520" y2="256" stroke="#cbd5e1"/>
<text x="520" y="278" class="small">0%</text>
<line x1="{gate5_x:.3f}" y1="128" x2="{gate5_x:.3f}" y2="256" stroke="#eab308" stroke-dasharray="5 4"/>
<text x="{gate5_label_x:.3f}" y="278" class="small">5% lower gate</text>
<line x1="{gate10_x:.3f}" y1="128" x2="{gate10_x:.3f}" y2="256" stroke="#f97316" stroke-dasharray="5 4"/>
<text x="{gate10_label_x:.3f}" y="300" class="small">10% point gate</text>
<line x1="{lower_x:.3f}" y1="142" x2="{point_x:.3f}" y2="142" stroke="#0f766e" stroke-width="8" stroke-linecap="round"/>
<circle cx="{lower_x:.3f}" cy="142" r="7" fill="#0f766e"/>
<circle cx="{point_x:.3f}" cy="142" r="10" fill="#134e4a"/>
<text x="520" y="116" class="label">Paired relative reduction</text>
<text x="520" y="190" class="label">Point: {point_pct:.3f}%</text>
<text x="520" y="220" class="label">One-sided 95% lower: {lower_pct:.3f}%</text>
<text x="40" y="350" class="small">Scope: parameterized reactive single-actor crossing simulation.</text>
<text x="40" y="376" class="small">Comparator is a frozen fixed-v1.20 shielded straight policy, not actual TEB.</text>
<text x="40" y="402" class="small">No claim of general safety, deployment readiness, or real-vehicle performance.</text>
</svg>
""".format(
        spatial_width=spatial_width,
        baseline_width=baseline_width,
        spatial_label_x=190 + spatial_width,
        baseline_label_x=190 + baseline_width,
        spatial_mean=spatial_mean,
        baseline_mean=baseline_mean,
        point_x=point_x,
        lower_x=lower_x,
        gate5_x=gate5_x,
        gate10_x=gate10_x,
        gate5_label_x=gate5_x - 40,
        gate10_label_x=gate10_x - 42,
        point_pct=point * 100.0,
        lower_pct=lower * 100.0,
    ).encode("utf-8")


def compute_outputs():
    require(sha(read(ASSESSMENT)) == EXPECTED_ASSESSMENT_SHA, "assessment hash drift")
    require(sha(read(REVIEW)) == EXPECTED_REVIEW_SHA, "independent review hash drift")
    require(sha(read(HISTORICAL_EVIDENCE_FREEZE)) == EXPECTED_EVIDENCE_FREEZE_SHA, "evidence freeze hash drift")
    contract = load_yaml(CONTRACT)
    plan = load_yaml(PLAN)
    assessment = load_yaml(ASSESSMENT)
    review = load_yaml(REVIEW)
    require(contract["authority"]["evidence_budget_units"] == 0, "V3-10 authority differs")
    require(contract["authority"]["execution_authorized"] is False, "V3-10 execution authority differs")
    require(contract["authority"]["training_authorized"] is False, "V3-10 training authority differs")
    require(plan["status"] == "frozen_pure_offline_plan", "V3-10 plan differs")
    require(assessment["formal_result"] is True, "formal result missing")
    require(assessment["performance_improvement_demonstrated"] is True, "positive result missing")
    require(all(assessment["formal_gates"].values()), "formal gate failure")
    require(review["status"] == "v3_09_f1_independent_formal_positive_review", "review status differs")
    require(review["actual_teb_comparison"] is False, "actual TEB boundary differs")

    pooled = assessment["pooled_metrics"]
    primary = assessment["primary_statistics"]
    spatial = pooled["method_metrics"]["v3_spatial_safe"]
    baseline = pooled["method_metrics"]["fixed_v120_shielded"]
    summary_rows = [
        ("completed_episodes", "count", "180"),
        ("statistical_blocks", "count", "30"),
        ("v3_spatial_safe_goal_success", "proportion", "{:.6f}".format(spatial["goal_success_rate"])),
        ("fixed_v120_shielded_goal_success", "proportion", "{:.6f}".format(baseline["goal_success_rate"])),
        ("v3_spatial_safe_mean_restricted_goal_time", "s", "{:.6f}".format(spatial["restricted_goal_time_mean_s"])),
        ("fixed_v120_shielded_mean_restricted_goal_time", "s", "{:.6f}".format(baseline["restricted_goal_time_mean_s"])),
        ("paired_relative_reduction", "proportion", "{:.9f}".format(primary["point_relative_reduction"])),
        ("one_sided_95pct_lower_relative_reduction", "proportion", "{:.9f}".format(primary["one_sided_lower_relative_reduction"])),
        ("positive_block_fraction", "proportion", "{:.6f}".format(primary["positive_block_fraction"])),
        ("minimum_conservative_clearance", "m", "{:.9f}".format(pooled["minimum_conservative_clearance_m"])),
        ("conservative_clearance_violations", "count", str(pooled["conservative_clearance_violation_count"])),
        ("exact_core_replay_rate", "proportion", "{:.6f}".format(pooled["exact_core_replay_rate"])),
        ("planner_latency_p99", "ms", "{:.6f}".format(pooled["planner_latency_p99_ms"])),
        ("decision_count", "count", str(pooled["decision_count"])),
    ]
    outputs = {
        SUMMARY: csv_bytes(("metric", "unit", "value"), summary_rows),
    }

    policy_rows = []
    for key, metrics in pooled["method_policy_metrics"].items():
        method, actor_policy = key.split("__", 1)
        policy_rows.append((
            method,
            actor_policy,
            metrics["episode_count"],
            "{:.6f}".format(metrics["goal_success_rate"]),
            "{:.6f}".format(metrics["restricted_goal_time_mean_s"]),
            "{:.6f}".format(metrics["restricted_goal_time_median_s"]),
        ))
    outputs[POLICY] = csv_bytes(
        ("method", "actor_policy", "episode_count", "goal_success_rate", "mean_restricted_goal_time_s", "median_restricted_goal_time_s"),
        policy_rows,
    )

    block_rows = []
    for block_id, values in assessment["block_means_restricted_goal_time_s"].items():
        v3_time = float(values["v3_spatial_safe"])
        baseline_time = float(values["fixed_v120_shielded"])
        reduction = (baseline_time - v3_time) / baseline_time
        block_rows.append((
            block_id,
            block_id.rsplit("_", 1)[-1],
            "{:.6f}".format(v3_time),
            "{:.6f}".format(baseline_time),
            "{:.9f}".format(reduction),
            str(v3_time < baseline_time).lower(),
        ))
    outputs[BLOCK] = csv_bytes(
        ("block_id", "seed", "v3_spatial_safe_mean_s", "fixed_v120_shielded_mean_s", "relative_reduction", "positive_block"),
        block_rows,
    )

    claims = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-10-P1",
        "status": "paper_claim_boundary_frozen",
        "formal_source_stage": "V3-09-F1",
        "formal_result": True,
        "performance_improvement_demonstrated": True,
        "qualified_claim": (
            "Within 30 fresh parameterized reactive single-actor crossing blocks, "
            "V3 spatial-safe reduced restricted goal time by 22.3844% relative to "
            "the frozen fixed-v1.20 shielded straight-policy comparator; the "
            "one-sided 95% block-bootstrap lower bound was 21.8235%, with 100% "
            "goal success for both methods and all audited integrity/safety gates passing."
        ),
        "qualified_scope": {
            "fresh_blocks": 30,
            "episodes": 180,
            "actor_policies": ["cautious", "assertive", "hesitant"],
            "scene_family": "parameterized_reactive_single_actor_crossing",
            "comparison": "v3_spatial_safe_vs_fixed_v120_shielded",
        },
        "forbidden_extrapolations": {
            "actual_teb_superiority": False,
            "general_navigation_safety": False,
            "cross_scene_generalization": False,
            "learned_policy_benefit": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "historical_manifest_serialization": {
            "preserved": True,
            "issue": "literal_backslash_n_separators",
            "formal_result_impact": "none",
            "replacement_behavior": "new_closure_binds_exact_historical_bytes_with_conventional_multiline_manifest",
        },
    }
    outputs[CLAIMS] = yaml_bytes(claims)
    outputs[FIGURE] = render_svg(
        float(spatial["restricted_goal_time_mean_s"]),
        float(baseline["restricted_goal_time_mean_s"]),
        float(primary["point_relative_reduction"]),
        float(primary["one_sided_lower_relative_reduction"]),
    )
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = compute_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, args.check)
    resources = [
        CONTRACT,
        PLAN,
        ASSESSMENT,
        REVIEW,
        HISTORICAL_EVIDENCE_FREEZE,
        HISTORICAL_TERMINAL_FREEZE,
        BUILDER,
        REVIEWER,
        TEST,
    ] + list(outputs)
    manifest = "".join("{}  {}\n".format(sha(read(path)), path) for path in resources).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    print("status=v3_10_p1_paper_exports_{} manifest_sha256={}".format(
        "reproducible" if args.check else "built", sha(manifest)
    ))


if __name__ == "__main__":
    main()
