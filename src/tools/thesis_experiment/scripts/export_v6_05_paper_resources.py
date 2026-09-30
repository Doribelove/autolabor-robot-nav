#!/usr/bin/env python3
"""Export deterministic V5/V6 paper tables, figures, claims and evidence hashes."""

import csv
import hashlib
import json
import os
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np
import yaml


W = Path("/home/robot/robot_ws_base_rl")
OUT = W / "artifacts/v6/paper/v6_05_p1_thesis_export"
V5 = W / "artifacts/v5/formal/v5_03_f1_fast_formal/formal_assessment.json"
V6 = W / "artifacts/v6/formal/v6_02_f1_fresh_formal/formal_assessment.json"
FAMILY = W / "artifacts/v6/formal/v6_02_f1_fresh_formal/family_summary.csv"
VALIDATION = W / "artifacts/v6/training/v6_01_r1_validation/selection_report.json"
TEX = W / "docs/thesis_experiment/thesis_draft/FAM_TEB_V6_PCT_RL_SUPPLEMENT.tex"
V4_PDF = W / "artifacts/v4/paper/v4_06_p1_thesis_export/FAM_TEB_V4_RL_THESIS_DRAFT.pdf"
GAZEBO = [
    ("V6-03-R1", W / "artifacts/v6/gazebo/v6_03_r1_p0_preflight/matrix_summary_recovered_offline.yaml"),
    ("V6-03-R2", W / "artifacts/v6/gazebo/v6_03_r2_p0_preflight/matrix_summary.yaml"),
    ("V6-04", W / "artifacts/v6/gazebo/v6_04_p0_preflight/matrix_summary.yaml"),
]
COLORS = {"analytic": "#667085", "v5": "#E19C24", "v6": "#1769AA", "teb": "#168A72", "negative": "#C74440"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def savefig(fig, stem):
    fig.savefig(OUT / (stem + ".svg"), bbox_inches="tight", metadata={"Date": None})
    fig.savefig(OUT / (stem + ".pdf"), bbox_inches="tight", metadata={"CreationDate": None, "ModDate": None})
    plt.close(fig)


def csv_write(name, fields, rows):
    path = OUT / name
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def architecture_figure():
    fig, ax = plt.subplots(figsize=(13.2, 6.8)); ax.set_xlim(0, 13.2); ax.set_ylim(0, 6.8); ax.axis("off")
    def box(x, y, w, h, text, color, subtitle=""):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04,rounding_size=0.08", facecolor=color + "18", edgecolor=color, linewidth=1.8))
        ax.text(x + w/2, y + h*0.60, text, ha="center", va="center", fontsize=10.5, weight="bold", color=color)
        if subtitle: ax.text(x + w/2, y + h*0.27, subtitle, ha="center", va="center", fontsize=7.8, color="#344054", wrap=True)
    def arrow(x1, y1, x2, y2, color="#475467", style="-"):
        ax.add_patch(FancyArrowPatch((x1,y1),(x2,y2),arrowstyle="-|>",mutation_scale=13,linewidth=1.5,linestyle=style,color=color))
    box(.25, 3.95, 1.75, 1.25, "Real sensors", "#1769AA", "6×72 LaserScan + odometry + local goal")
    box(2.35, 3.95, 1.75, 1.25, "Ackermann bank", "#1769AA", "8 feasible trajectory IDs")
    box(4.45, 4.65, 1.75, 1.25, "Analytic prior", "#667085", "progress, clearance, smoothness")
    box(4.45, 3.15, 1.75, 1.25, "Sensor actor", "#1769AA", "CNN–GRU residual logits")
    box(6.55, 3.95, 1.80, 1.25, "Hard shield", "#C74440", "mask + executable-stop backup")
    box(8.70, 3.95, 1.75, 1.25, "Advantage gate", "#1769AA", "learned override only if Δ≥0.10")
    box(10.80, 3.95, 2.00, 1.25, "Command boundary", "#168A72", "pure pursuit → /cmd_vel_navigation")
    for a in [(2,4.58,2.35,4.58),(4.1,4.58,4.45,5.1),(4.1,4.58,4.45,3.78),(6.2,5.1,6.55,4.7),(6.2,3.78,6.55,4.45),(8.35,4.58,8.70,4.58),(10.45,4.58,10.80,4.58)]: arrow(*a)
    box(.65, .55, 2.05, 1.30, "Simulation privilege", "#7A5AF8", "world state, actor mode, hazard side")
    box(3.15, .55, 2.05, 1.30, "Asymmetric critic", "#7A5AF8", "privilege only in V(s) during PPO")
    box(5.65, .55, 2.25, 1.30, "Counterfactual rank", "#7A5AF8", "training-only target for hard cases")
    box(8.35, .55, 2.20, 1.30, "PPO + PCT loss", "#7A5AF8", "480k new steps; four identities")
    for a in [(2.7,1.2,3.15,1.2),(5.2,1.2,5.65,1.2),(7.9,1.2,8.35,1.2),(9.45,1.85,5.35,3.15)]: arrow(*a,color="#7A5AF8",style="--" if a[-1] > 2 else "-")
    ax.text(6.6, 6.45, "PCT-SCOPE-RL: privileged training, sensor-only candidate evaluation", ha="center", fontsize=15, weight="bold", color="#101828")
    ax.text(12.95, .18, "Purple path is removed from the deployment checkpoint", ha="right", fontsize=8.5, color="#7A5AF8")
    savefig(fig, "figure_v6_architecture")


def results_figure(v5, v6):
    methods = ["Analytic prior", "Frozen V5", "PCT-SCOPE-RL"]
    times = [v6["summaries"]["sensor_analytic_prior"]["mean_failure_inclusive_restricted_time_s"], v6["summaries"]["frozen_v5_full"]["mean_failure_inclusive_restricted_time_s"], v6["summaries"]["pct_scope_rl_v6"]["mean_failure_inclusive_restricted_time_s"]]
    goals = [v6["summaries"]["sensor_analytic_prior"]["goal_count"], v6["summaries"]["frozen_v5_full"]["goal_count"], v6["summaries"]["pct_scope_rl_v6"]["goal_count"]]
    colors = [COLORS["analytic"], COLORS["v5"], COLORS["v6"]]
    fig, axes = plt.subplots(1,2,figsize=(11.5,4.5))
    bars=axes[0].bar(methods,times,color=colors); axes[0].set_ylabel("Failure-inclusive mean time (s)"); axes[0].set_ylim(12.5,14.7); axes[0].set_title("Fresh 80-block formal population")
    for b,v in zip(bars,times): axes[0].text(b.get_x()+b.get_width()/2,v+.04,f"{v:.3f}",ha="center",fontsize=9)
    bars=axes[1].bar(methods,goals,color=colors); axes[1].set_ylabel("Goals reached / 80"); axes[1].set_ylim(45,64); axes[1].set_title("Failure-inclusive task outcome")
    for b,v in zip(bars,goals): axes[1].text(b.get_x()+b.get_width()/2,v+.35,str(v),ha="center",fontsize=9)
    for ax in axes: ax.tick_params(axis="x",rotation=15); ax.grid(axis="y",alpha=.22)
    fig.suptitle("V6 formal result: 8.402% faster than analytic prior", fontsize=14, weight="bold"); fig.tight_layout()
    savefig(fig,"figure_v6_formal_summary")


def effects_figure(v5, v6):
    labels=["V5 vs analytic","V6 vs analytic","V6 vs frozen V5"]
    points=np.array([100*v5["primary_relative_time_reduction"],100*v6["primary_relative_reduction_vs_analytic"],100*v6["secondary_relative_reduction_vs_frozen_v5"]])
    intervals=np.array([[100*x for x in v5["paired_bootstrap_95_percent_interval"]],[100*x for x in v6["primary_two_sided_97_5_percent_interval"]],[100*x for x in v6["secondary_two_sided_95_percent_interval"]]])
    fig,ax=plt.subplots(figsize=(9.5,4.3)); y=np.arange(3)
    ax.errorbar(points,y,xerr=np.vstack((points-intervals[:,0],intervals[:,1]-points)),fmt="o",markersize=8,color=COLORS["v6"],ecolor="#98A2B3",capsize=5)
    ax.axvline(0,color="#344054",lw=1); ax.axvline(8,color=COLORS["negative"],lw=1.5,ls="--",label="8% preregistered gate")
    ax.set_yticks(y,labels); ax.invert_yaxis(); ax.set_xlabel("Relative reduction in restricted mean time (%)"); ax.grid(axis="x",alpha=.22); ax.legend(loc="lower right")
    for p,yy in zip(points,y): ax.text(p+.25,yy-.14,f"{p:.2f}%",fontsize=9)
    ax.set_title("Effect sizes and frozen bootstrap intervals",weight="bold"); fig.tight_layout(); savefig(fig,"figure_v6_effect_intervals")


def family_figure():
    with FAMILY.open(newline="",encoding="utf-8") as stream: rows=list(csv.DictReader(stream))
    families=[]
    for row in rows:
        if row["family"] not in families: families.append(row["family"])
    short={"bidirectional_reactive_crossing":"bidirectional","sequential_two_actor_crossing":"sequential","partially_occluded_crossing":"occluded","narrow_corridor_yield":"narrow"}
    methods=["sensor_analytic_prior","frozen_v5_full","pct_scope_rl_v6"]
    lookup={(r["family"],r["method"]):float(r["mean_failure_inclusive_restricted_time_s"]) for r in rows}
    fig,ax=plt.subplots(figsize=(10.5,4.7)); x=np.arange(len(families)); width=.24
    for idx,(method,label,color) in enumerate(zip(methods,["Analytic","V5","V6"],[COLORS["analytic"],COLORS["v5"],COLORS["v6"]])):
        ax.bar(x+(idx-1)*width,[lookup[(f,method)] for f in families],width,label=label,color=color)
    ax.set_xticks(x,[short[f] for f in families]); ax.set_ylabel("Failure-inclusive mean time (s)"); ax.set_ylim(8,21); ax.grid(axis="y",alpha=.22); ax.legend()
    ax.set_title("Family decomposition: the positive aggregate is concentrated in occlusion",weight="bold"); fig.tight_layout(); savefig(fig,"figure_v6_family_ablation")


def training_figure():
    fig,axes=plt.subplots(1,2,figsize=(11.2,4.2))
    for identity in range(1,5):
        path=W/f"artifacts/v6/training/v6_01_r1/pct_seed950{identity}/training_updates.jsonl"
        rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        steps=np.array([r["global_steps"] for r in rows]); acc=np.array([r["privileged_rank_accuracy"] for r in rows]); reward=np.array([r["mean_rollout_reward"] for r in rows])
        window=8; smooth=lambda x: np.convolve(x,np.ones(window)/window,mode="valid")
        axes[0].plot(steps[window-1:],smooth(acc),label=f"seed950{identity}",lw=1.25)
        axes[1].plot(steps[window-1:],smooth(reward),label=f"seed950{identity}",lw=1.25)
    axes[0].set_ylabel("PCT rank accuracy (8-update mean)"); axes[1].set_ylabel("Rollout reward (8-update mean)")
    for ax in axes: ax.set_xlabel("New environment steps"); ax.grid(alpha=.22)
    axes[0].legend(ncol=2,fontsize=8); fig.suptitle("Four frozen PCT training identities",weight="bold"); fig.tight_layout(); savefig(fig,"figure_v6_training")


def gazebo_figure():
    rows=[]
    for stage,path in GAZEBO:
        data=yaml.safe_load(path.read_bytes())
        methods=data.get("method_summary") or data["methods"]
        for method in ("pct_scope_rl_v6","sensor_analytic_prior","actual_teb"):
            item=methods[method]
            clearance=item.get("minimum_clearance_m",item.get("minimum_conservative_clearance_m"))
            time_s=item.get("mean_restricted_time_s",item.get("restricted_time_s"))
            goal=item.get("goal_successes",int(item.get("goal_reached",False)))
            rows.append({"stage":stage,"method":method,"clearance_m":float(clearance),"restricted_time_s":float(time_s),"goals":int(goal),"episodes":int(item.get("episodes",1)),"contact_count":int(item.get("contact_count",item.get("contact_state_count",0)))})
    csv_write("v6_gazebo_preflight_summary.csv",list(rows[0]),rows)
    fig,ax=plt.subplots(figsize=(10.5,4.6)); stages=[x[0] for x in GAZEBO]; x=np.arange(3); width=.24
    for idx,(method,label,color) in enumerate(zip(("pct_scope_rl_v6","sensor_analytic_prior","actual_teb"),("V6","Analytic","Actual TEB"),(COLORS["v6"],COLORS["analytic"],COLORS["teb"]))):
        values=[next(r["clearance_m"] for r in rows if r["stage"]==stage and r["method"]==method) for stage in stages]
        ax.bar(x+(idx-1)*width,values,width,label=label,color=color)
    ax.axhline(0,color=COLORS["negative"],ls="--",lw=1.4,label="frozen integrity boundary")
    ax.set_xticks(x,stages); ax.set_ylabel("Minimum conservative clearance (m)"); ax.grid(axis="y",alpha=.22); ax.legend(ncol=4,fontsize=8)
    ax.set_title("Gazebo transfer preflights are terminal negative results",weight="bold"); fig.tight_layout(); savefig(fig,"figure_v6_gazebo_boundary")
    return rows


def main():
    if OUT.exists(): raise RuntimeError("paper export root already exists")
    OUT.mkdir(parents=True)
    v5=json.loads(V5.read_text()); v6=json.loads(V6.read_text()); validation=json.loads(VALIDATION.read_text())
    architecture_figure(); results_figure(v5,v6); effects_figure(v5,v6); family_figure(); training_figure(); gazebo_rows=gazebo_figure()
    method_rows=[]
    for method,label in (("sensor_analytic_prior","Analytic prior"),("frozen_v5_full","Frozen V5"),("pct_scope_rl_v6","PCT-SCOPE-RL V6"),("pct_scope_rl_v6_no_history","V6 no history")):
        s=v6["summaries"][method]; method_rows.append({"method":label,"episodes":s["episodes"],"goals":s["goal_count"],"mean_restricted_time_s":s["mean_failure_inclusive_restricted_time_s"],"minimum_clearance_m":s["minimum_physical_clearance_m"],"contacts":s["contact_count"],"mask_violations":s["mask_violation_count"],"mean_latency_ms":s["mean_inference_latency_ms"]})
    csv_write("v6_02_f1_method_summary.csv",list(method_rows[0]),method_rows)
    effect_rows=[
        {"comparison":"V5 vs analytic","point":v5["primary_relative_time_reduction"],"lower":v5["paired_bootstrap_95_percent_interval"][0],"upper":v5["paired_bootstrap_95_percent_interval"][1],"interval":"two-sided 95%","gate":0.08,"pass":False},
        {"comparison":"V6 vs analytic","point":v6["primary_relative_reduction_vs_analytic"],"lower":v6["primary_two_sided_97_5_percent_interval"][0],"upper":v6["primary_two_sided_97_5_percent_interval"][1],"interval":"two-sided 97.5%","gate":0.08,"pass":True},
        {"comparison":"V6 vs frozen V5","point":v6["secondary_relative_reduction_vs_frozen_v5"],"lower":v6["secondary_two_sided_95_percent_interval"][0],"upper":v6["secondary_two_sided_95_percent_interval"][1],"interval":"two-sided 95%","gate":"secondary","pass":True},
    ]
    csv_write("v6_effect_summary.csv",list(effect_rows[0]),effect_rows)
    shutil.copy2(FAMILY,OUT/"v6_02_f1_family_summary.csv")
    shutil.copy2(W/"artifacts/v6/formal/v6_02_f1_fresh_formal/formal_episodes.csv",OUT/"v6_02_f1_formal_episodes.csv")
    shutil.copy2(TEX,OUT/TEX.name)
    claims={
        "status":"v6_05_p1_paper_export_complete",
        "architecture_name":"PCT-SCOPE-RL",
        "allowed_claims":[
            "A deployable sensor-only actor and an asymmetric privileged critic were implemented.",
            "Counterfactual privileged rank distillation improved the fresh fast-population mean by 8.4024% versus the analytic prior with a positive two-sided 97.5% interval.",
            "The primary fast-population gates passed with zero contacts and zero mask violations.",
            "The benefit is concentrated in the partially occluded family.",
            "The current actor history ablation is exactly tied and does not demonstrate temporal-memory benefit.",
            "Three Gazebo transfer preflights failed the frozen conservative-clearance integrity gate and are terminal negative results.",
        ],
        "forbidden_claims":["Gazebo effectiveness","superiority to actual TEB","formal safety","perception or OOD generalization","real-vehicle readiness","temporal-history benefit","deployment readiness"],
        "formal_fast_result":v6,
        "gazebo_qualification_run":False,
        "gazebo_formal_run":False,
        "real_vehicle_used":False,
    }
    (OUT/"v6_05_p1_claims.yaml").write_text(yaml.safe_dump(claims,sort_keys=False,allow_unicode=True),encoding="utf-8")
    sources={
        "ppo":"https://arxiv.org/abs/1707.06347",
        "asymmetric_actor_critic":"https://arxiv.org/abs/1710.06542",
        "learning_by_cheating":"https://proceedings.mlr.press/v100/chen20a.html",
        "shielding":"https://arxiv.org/abs/1708.08611",
        "partial_observability_shielding":"https://arxiv.org/abs/2204.00755",
        "teb":"https://doi.org/10.1016/j.robot.2016.11.007",
        "sensor_level_navigation_rl":"https://arxiv.org/abs/1709.10082",
    }
    (OUT/"literature_primary_sources.yaml").write_text(yaml.safe_dump(sources,sort_keys=False),encoding="utf-8")
    evidence=[V5,V6,FAMILY,VALIDATION,W/"artifacts/v6/gazebo/v6_03_r1_p0_preflight/matrix_summary_recovered_offline.yaml",W/"artifacts/v6/gazebo/v6_03_r2_p0_preflight/matrix_summary.yaml",W/"artifacts/v6/gazebo/v6_04_p0_preflight/matrix_summary.yaml",W/"src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",W/"src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/privileged_distillation.py",W/"src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py"]
    with (OUT/"v6_05_p1_evidence_manifest.sha256").open("w",encoding="utf-8") as stream:
        for path in evidence: stream.write("{}  {}\n".format(sha(path),path.relative_to(W)))
    print("paper_resource_root={}".format(OUT)); print("formal_positive={}".format(v6["method_effectiveness_demonstrated"])); print("gazebo_rows={}".format(len(gazebo_rows)))


if __name__ == "__main__": main()
