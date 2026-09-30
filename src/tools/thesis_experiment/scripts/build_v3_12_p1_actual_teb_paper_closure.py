#!/usr/bin/env python3
"""Build deterministic paper exports from frozen V3-11-F1 evidence."""

import argparse
import csv
import hashlib
import io
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/actual_teb/v3_12_p1_paper_closure")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_12_p1_actual_teb_paper_closure_contract.yaml"
)
ASSESSMENT = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/formal_assessment.yaml"
)
REVIEW = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "independent_result_review.yaml"
)
EVIDENCE_FREEZE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/"
    "formal_evidence_freeze.yaml"
)
TERMINAL_FREEZE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_assessment/terminal_freeze.yaml"
)
EXECUTION_RELEASE = Path(
    "experiments/manifests/v3/v3_11_f1_actual_teb_formal_release.yaml"
)
ANALYSIS_REPAIR_RELEASE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/"
    "formal_analysis_r1_release.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_12_p1_actual_teb_paper_closure.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_12_p1_actual_teb_paper_closure.py"
)

SUMMARY = ROOT / "v3_12_p1_actual_teb_formal_summary.csv"
STRATUM = ROOT / "v3_12_p1_actual_teb_stratum_results.csv"
BLOCK = ROOT / "v3_12_p1_actual_teb_block_results.csv"
FIGURE = ROOT / "v3_12_p1_actual_teb_formal_result.svg"
CLAIMS = ROOT / "v3_12_p1_actual_teb_claims.yaml"
MANIFEST = ROOT / "v3_12_p1_actual_teb_evidence_manifest.sha256"

EXPECTED_ASSESSMENT_SHA = (
    "9b19f8bd29f1d6c06197e932b03359a2f78081da65e636f52f7f610529c3f2f9"
)
EXPECTED_REVIEW_SHA = (
    "2d5e8c21ea70f29df97e646f703159297ff0d2da6ef10edb1341da057c133b39"
)
EXPECTED_EVIDENCE_FREEZE_SHA = (
    "a90b180c08de489706a7be7ddace09280570240b13449e275d172545e815470f"
)
EXPECTED_TERMINAL_FREEZE_SHA = (
    "f4396e25716dfb13bdc3e8d33a02387c117ab5a8c066e0d6a3e94b2fe3ac7465"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required resource absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def csv_bytes(header, rows):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(header)
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


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


def render_svg(metrics):
    v3_time = metrics["mean_restricted_elapsed_s_by_method"]["v3_spatial_safe"]
    teb_time = metrics["mean_restricted_elapsed_s_by_method"]["actual_teb"]
    point = metrics["point_reduction_percent"]
    lower = metrics["bootstrap"]["one_sided_lower_reduction_percent"]
    time_scale = 54.0
    v3_width = v3_time * time_scale
    teb_width = teb_time * time_scale
    effect_x0 = 610.0
    effect_scale = 22.0
    point_x = effect_x0 + point * effect_scale
    lower_x = effect_x0 + lower * effect_scale
    gate5_x = effect_x0 + 5.0 * effect_scale
    gate10_x = effect_x0 + 10.0 * effect_scale
    strata = list(metrics["stratum_metrics"].items())
    rows = []
    for index, (name, values) in enumerate(strata):
        y = 350 + 34 * index
        short_name = (
            name.replace("bounded_corridor_crossing_110", "bounded-110")
            .replace("wide_corridor_crossing_70", "wide-70")
            .replace("open_crossing_90", "open-90")
            .replace("__", " / ")
        )
        reduction = values["mean_reduction_percent"]
        width = reduction * 20.0
        rows.append(
            '<text x="60" y="{y}" class="small">{name}</text>'
            '<rect x="350" y="{bar_y}" width="{width:.3f}" height="18" '
            'rx="3" fill="#2a9d8f"/>'
            '<text x="{label_x:.3f}" y="{y}" class="small">{value:.3f}%</text>'.format(
                y=y,
                name=short_name,
                bar_y=y - 15,
                width=width,
                label_x=360 + width,
                value=reduction,
            )
        )
    return """<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="590" viewBox="0 0 1100 590">
<rect width="1100" height="590" fill="#ffffff"/>
<style>
text {{ font-family: DejaVu Sans, Arial, sans-serif; fill: #172033; }}
.title {{ font-size: 25px; font-weight: 700; }}
.label {{ font-size: 16px; }}
.small {{ font-size: 13px; fill: #4a5568; }}
</style>
<text x="45" y="42" class="title">V3-11-F1: fair paired comparison with actual TEB</text>
<text x="45" y="70" class="small">30 fresh paired blocks · 60 episodes · both methods 100% goal success · zero contact states</text>
<text x="45" y="112" class="label">Restricted gate-to-goal time (lower is better)</text>
<text x="45" y="158" class="small">V3 spatial-safe</text>
<rect x="180" y="138" width="{v3_width:.3f}" height="28" rx="4" fill="#1677ff"/>
<text x="{v3_label_x:.3f}" y="158" class="label">{v3_time:.3f} s</text>
<text x="45" y="208" class="small">Actual TebLocalPlannerROS</text>
<rect x="180" y="188" width="{teb_width:.3f}" height="28" rx="4" fill="#94a3b8"/>
<text x="{teb_label_x:.3f}" y="208" class="label">{teb_time:.3f} s</text>
<text x="610" y="112" class="label">Paired mean reduction</text>
<line x1="{gate5_x:.3f}" y1="130" x2="{gate5_x:.3f}" y2="240" stroke="#eab308" stroke-dasharray="5 4"/>
<line x1="{gate10_x:.3f}" y1="130" x2="{gate10_x:.3f}" y2="240" stroke="#f97316" stroke-dasharray="5 4"/>
<line x1="{lower_x:.3f}" y1="155" x2="{point_x:.3f}" y2="155" stroke="#0f766e" stroke-width="9" stroke-linecap="round"/>
<circle cx="{lower_x:.3f}" cy="155" r="7" fill="#0f766e"/>
<circle cx="{point_x:.3f}" cy="155" r="10" fill="#134e4a"/>
<text x="610" y="195" class="label">Point: {point:.3f}%</text>
<text x="610" y="225" class="label">One-sided 97.5% lower: {lower:.3f}%</text>
<text x="{gate5_label_x:.3f}" y="260" class="small">5% lower gate</text>
<text x="{gate10_label_x:.3f}" y="282" class="small">10% point gate</text>
<text x="45" y="320" class="label">Scene × actor-policy stratum mean reductions</text>
{stratum_rows}
<text x="45" y="568" class="small">Scope: three frozen single-reactive-actor Gazebo templates; shared oracle actor track; no training or real vehicle.</text>
</svg>
""".format(
        v3_width=v3_width,
        teb_width=teb_width,
        v3_label_x=190 + v3_width,
        teb_label_x=190 + teb_width,
        v3_time=v3_time,
        teb_time=teb_time,
        gate5_x=gate5_x,
        gate10_x=gate10_x,
        lower_x=lower_x,
        point_x=point_x,
        point=point,
        lower=lower,
        gate5_label_x=gate5_x - 38,
        gate10_label_x=gate10_x - 38,
        stratum_rows="\n".join(rows),
    ).encode("utf-8")


def compute_outputs():
    require(sha(read(ASSESSMENT)) == EXPECTED_ASSESSMENT_SHA, "assessment drift")
    require(sha(read(REVIEW)) == EXPECTED_REVIEW_SHA, "review drift")
    require(
        sha(read(EVIDENCE_FREEZE)) == EXPECTED_EVIDENCE_FREEZE_SHA,
        "evidence freeze drift",
    )
    require(
        sha(read(TERMINAL_FREEZE)) == EXPECTED_TERMINAL_FREEZE_SHA,
        "terminal freeze drift",
    )
    contract = load_yaml(CONTRACT)
    assessment = load_yaml(ASSESSMENT)
    review = load_yaml(REVIEW)
    require(contract["authority"]["evidence_budget_units"] == 0, "evidence budget differs")
    require(contract["authority"]["execution_authorized"] is False, "execution authority differs")
    require(contract["authority"]["training_authorized"] is False, "training authority differs")
    require(assessment["formal_result"] is True, "formal result missing")
    require(
        assessment["performance_improvement_demonstrated"] is True,
        "formal positive result missing",
    )
    require(all(assessment["gates"].values()), "formal gate failure")
    require(
        review["status"]
        == "independent_actual_teb_formal_meaningful_improvement_review_pass",
        "independent formal review differs",
    )
    metrics = assessment["metrics"]
    summary_rows = [
        ("completed_episodes", "count", str(metrics["episode_count"])),
        ("paired_blocks", "count", str(metrics["paired_block_count"])),
        ("v3_goal_success_rate", "proportion", "{:.9f}".format(metrics["goal_success_rate_by_method"]["v3_spatial_safe"])),
        ("actual_teb_goal_success_rate", "proportion", "{:.9f}".format(metrics["goal_success_rate_by_method"]["actual_teb"])),
        ("v3_mean_restricted_gate_to_goal_time", "s", "{:.9f}".format(metrics["mean_restricted_elapsed_s_by_method"]["v3_spatial_safe"])),
        ("actual_teb_mean_restricted_gate_to_goal_time", "s", "{:.9f}".format(metrics["mean_restricted_elapsed_s_by_method"]["actual_teb"])),
        ("paired_mean_reduction", "proportion", "{:.12f}".format(metrics["point_reduction_percent"] / 100.0)),
        ("one_sided_97_5pct_lower_reduction", "proportion", "{:.12f}".format(metrics["bootstrap"]["one_sided_lower_reduction_percent"] / 100.0)),
        ("v3_minimum_conservative_clearance", "m", "{:.9f}".format(metrics["minimum_conservative_clearance_m_by_method"]["v3_spatial_safe"])),
        ("actual_teb_minimum_conservative_clearance", "m", "{:.9f}".format(metrics["minimum_conservative_clearance_m_by_method"]["actual_teb"])),
        ("contact_state_count_total", "count", str(metrics["contact_state_count_total"])),
        ("maximum_v3_tracking_error", "m", "{:.9f}".format(metrics["maximum_v3_tracking_error_m"])),
        ("minimum_actual_teb_local_plan_count", "count", str(metrics["minimum_actual_teb_local_plan_count"])),
        ("maximum_gate_actor_coordinate_pair_difference", "m", "{:.9f}".format(metrics["maximum_paired_gate_actor_coordinate_difference_m"])),
        ("maximum_gate_actor_speed_pair_difference", "m/s", "{:.9f}".format(metrics["maximum_paired_gate_actor_speed_difference_mps"])),
        ("training_steps", "count", "0"),
    ]
    outputs = {
        SUMMARY: csv_bytes(("metric", "unit", "value"), summary_rows),
    }
    stratum_rows = []
    for key, values in metrics["stratum_metrics"].items():
        scene, actor_policy = key.split("__", 1)
        stratum_rows.append(
            (
                scene,
                actor_policy,
                values["paired_blocks"],
                "{:.9f}".format(values["mean_v3_restricted_elapsed_s"]),
                "{:.9f}".format(values["mean_actual_teb_restricted_elapsed_s"]),
                "{:.9f}".format(values["mean_reduction_percent"] / 100.0),
            )
        )
    outputs[STRATUM] = csv_bytes(
        (
            "scene_id",
            "actor_policy",
            "paired_blocks",
            "v3_mean_restricted_time_s",
            "actual_teb_mean_restricted_time_s",
            "mean_reduction",
        ),
        stratum_rows,
    )
    block_rows = []
    for item in metrics["paired_blocks"]:
        block_rows.append(
            (
                item["block_index"],
                item["seed"],
                item["scene_id"],
                item["actor_policy"],
                "{:.9f}".format(item["v3_restricted_elapsed_s"]),
                "{:.9f}".format(item["actual_teb_restricted_elapsed_s"]),
                "{:.9f}".format(item["v3_minus_teb_s"]),
                "{:.9f}".format(item["block_reduction_percent"] / 100.0),
                str(item["v3_minus_teb_s"] < 0.0).lower(),
            )
        )
    outputs[BLOCK] = csv_bytes(
        (
            "block_index",
            "seed",
            "scene_id",
            "actor_policy",
            "v3_restricted_time_s",
            "actual_teb_restricted_time_s",
            "v3_minus_actual_teb_s",
            "relative_reduction",
            "v3_faster",
        ),
        block_rows,
    )
    outputs[FIGURE] = render_svg(metrics)
    claims = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-12-P1",
        "status": "actual_teb_paper_claim_boundary_frozen",
        "formal_source_stage": "V3-11-F1-ANALYSIS-R1",
        "formal_result": True,
        "performance_improvement_demonstrated": True,
        "qualified_claim": (
            "Within 30 fresh paired blocks from three frozen single-reactive-"
            "actor Gazebo templates and cautious/hesitant policies, V3 "
            "spatial-safe reduced restricted gate-to-goal time by 12.9288% "
            "relative to actual TebLocalPlannerROS 0.8.4; the one-sided 97.5% "
            "paired-bootstrap lower bound was 12.7253%. Both methods achieved "
            "100% goal success and no contact state was observed."
        ),
        "qualified_scope": {
            "paired_blocks": 30,
            "episodes": 60,
            "scenes": [
                "open_crossing_90",
                "wide_corridor_crossing_70",
                "bounded_corridor_crossing_110",
            ],
            "actor_policies": ["cautious", "hesitant"],
            "comparison": "v3_spatial_safe_vs_actual_TebLocalPlannerROS_0_8_4",
            "shared_vehicle": "autolabor_m2_ackermann_simulation",
            "shared_command_boundary": "/cmd_vel_navigation",
            "shared_oracle_actor_track": True,
        },
        "forbidden_extrapolations": {
            "perception_robustness": False,
            "multi_actor_generalization": False,
            "out_of_distribution_generalization": False,
            "formal_collision_free_guarantee": False,
            "learned_policy_or_reinforcement_learning_benefit": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "analysis_repair_disclosure": {
            "preserved": True,
            "type": "namespace_only string-label injection after I0 NameError",
            "statistic_threshold_or_execution_evidence_changed": False,
        },
    }
    outputs[CLAIMS] = yaml_bytes(claims)
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = compute_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    resources = [
        CONTRACT,
        ASSESSMENT,
        REVIEW,
        EVIDENCE_FREEZE,
        TERMINAL_FREEZE,
        EXECUTION_RELEASE,
        ANALYSIS_REPAIR_RELEASE,
        BUILDER,
        REVIEWER,
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path) for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_12_p1_actual_teb_paper_exports_{} "
        "manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
