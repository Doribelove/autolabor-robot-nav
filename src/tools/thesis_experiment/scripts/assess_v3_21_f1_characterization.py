#!/usr/bin/env python3
"""Assess V3-21 using failure-inclusive paired blocks and frozen statistics."""
import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
from collections import defaultdict
from pathlib import Path
import random
import statistics

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_21_f1_execution"
CONTRACT = W / "config/thesis_experiments/v3/v3_21_a0_ephemeral_port_repair_contract.yaml"
FOUNDATION = W / "artifacts/v3/thesis/v3_21_a0_port_repair_foundation"
SCHEMA_SOURCE = W / "src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py"
OUT = ROOT / "formal_characterization_assessment.yaml"
CANONICAL_JSONL = ROOT / "canonical_episodes.jsonl"
CANONICAL_CSV = ROOT / "canonical_episodes.csv"
METHOD_CSV = ROOT / "method_summary.csv"
COMPARISON_CSV = ROOT / "comparison_summary.csv"
BLOCK_CSV = ROOT / "block_results.csv"
STRATUM_CSV = ROOT / "stratum_results.csv"
MANIFEST = ROOT / "formal_evidence_manifest.sha256"
METHODS = ("v3_spatial_safe", "actual_teb", "fixed_v120_shielded", "v3_no_candidate_conditioning")
ROLES = {"v3_spatial_safe": "main_method", "actual_teb": "primary_baseline", "fixed_v120_shielded": "comparison_method", "v3_no_candidate_conditioning": "single_factor_ablation"}


def sha(data): return hashlib.sha256(data).hexdigest()
def q(values, probability):
    ordered = sorted(float(value) for value in values); position = (len(ordered) - 1) * probability; lower = int(math.floor(position)); upper = int(math.ceil(position))
    return ordered[lower] if lower == upper else ordered[lower] + (position - lower) * (ordered[upper] - ordered[lower])
def csv_bytes(header, rows):
    stream = io.StringIO(newline=""); writer = csv.writer(stream, lineterminator="\n"); writer.writerow(header); writer.writerows(rows); return stream.getvalue().encode("utf-8")
def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))
def schema_module():
    spec=importlib.util.spec_from_file_location("v3_21_schema",SCHEMA_SOURCE);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
def holm(raw):
    ordered=sorted(raw.items(),key=lambda item:item[1]);adjusted={};running=0.0;m=len(ordered)
    for rank,(name,value) in enumerate(ordered):
        running=max(running,min(1.0,(m-rank)*value));adjusted[name]=running
    return adjusted
def comparison(main, comparator, seed, resamples):
    pairs=list(zip(main,comparator));point=(statistics.mean(comparator)-statistics.mean(main))/statistics.mean(comparator);differences=[b-a for a,b in pairs];observed=statistics.mean(differences);centered=[value-observed for value in differences];rng=random.Random(seed);effects=[];null_means=[]
    for _ in range(resamples):
        indices=[rng.randrange(len(pairs)) for __ in pairs];a=[pairs[index][0] for index in indices];b=[pairs[index][1] for index in indices];effects.append((statistics.mean(b)-statistics.mean(a))/statistics.mean(b));null_means.append(statistics.mean(centered[index] for index in indices))
    p=(1+sum(abs(value)>=abs(observed) for value in null_means))/(resamples+1)
    return {"point_relative_reduction":point,"two_sided_95_lower":q(effects,.025),"two_sided_95_upper":q(effects,.975),"mean_comparator_minus_main_s":observed,"raw_two_sided_p":p,"bootstrap_resamples":resamples,"bootstrap_seed":seed}
def method_metrics(rows):
    times=[row["result"]["restricted_elapsed_s"] for row in rows];success=[row["result"]["goal_reached"] for row in rows];successful=[row["result"]["restricted_elapsed_s"] for row in rows if row["result"]["goal_reached"]];clearances=[row["result"]["minimum_conservative_clearance_m"] for row in rows];paths=[row["result"]["path_length_m"] for row in rows];latencies=[row["method_plan"]["latency_ms"] for row in rows if row.get("method_plan")]
    return {"episodes":len(rows),"goal_successes":sum(success),"goal_success_rate":sum(success)/len(rows),"restricted_time_mean_s":statistics.mean(times),"restricted_time_std_s":statistics.stdev(times),"restricted_time_median_s":statistics.median(times),"successful_only_time_mean_s":statistics.mean(successful) if successful else None,"path_length_mean_m":statistics.mean(paths),"minimum_clearance_mean_m":statistics.mean(clearances),"minimum_clearance_min_m":min(clearances),"contact_count":sum(row["result"]["contact_state_count"] for row in rows),"fallback_count":sum(bool((row.get("method_plan") or {}).get("safety_fallback_used")) for row in rows),"fallback_rate":sum(bool((row.get("method_plan") or {}).get("safety_fallback_used")) for row in rows)/len(rows),"execution_integrity_passes":sum(row["execution_integrity_pass"] for row in rows),"planner_latency_p99_ms":q(latencies,.99) if latencies else None}
def outputs():
    contract=yaml.safe_load(CONTRACT.read_bytes());schedule_path=ROOT/"exact_schedule.yaml";journal_path=ROOT/"attempt_journal.jsonl";summary_path=ROOT/"matrix_summary.yaml";release_path=FOUNDATION/"execution_release.yaml";schedule=yaml.safe_load(schedule_path.read_bytes());release_sha=sha(release_path.read_bytes());schema=schema_module();receipts=[];reports=[];resources=[CONTRACT,schedule_path,journal_path,summary_path,release_path,FOUNDATION/"terminal_freeze.sha256",SCHEMA_SOURCE,Path(__file__).resolve(),W/"src/tools/thesis_experiment/scripts/review_v3_21_f1_characterization.py"]
    for path in sorted((ROOT/"episodes").glob("*/matrix_receipt.yaml")):
        receipt=yaml.safe_load(path.read_bytes());report_path=path.with_name("episode_report.yaml")
        if sha(report_path.read_bytes())!=receipt["episode_report_sha256"]:raise RuntimeError("report digest differs")
        row=receipt["row"]
        if row!=schedule["rows"][row["sequence"]-1]:raise RuntimeError("schedule differs")
        receipts.append(receipt);reports.append(receipt["episode_report"]);resources.extend((path,report_path,path.with_name("world.sdf"),path.with_name("roslaunch.log")))
    if len(reports)!=120:raise RuntimeError("matrix incomplete")
    by_method={method:[row for row in reports if row["method"]==method] for method in METHODS};metrics={method:method_metrics(rows) for method,rows in by_method.items()}
    blocks=defaultdict(dict)
    for receipt in receipts:blocks[receipt["row"]["block_index"]][receipt["row"]["method"]]=receipt
    if len(blocks)!=30 or any(set(value)!=set(METHODS) for value in blocks.values()):raise RuntimeError("paired blocks incomplete")
    comparisons={}
    for index,comparator in enumerate(METHODS[1:]):
        name="v3_spatial_safe_vs_{}".format(comparator);main=[blocks[block]["v3_spatial_safe"]["episode_report"]["result"]["restricted_elapsed_s"] for block in sorted(blocks)];other=[blocks[block][comparator]["episode_report"]["result"]["restricted_elapsed_s"] for block in sorted(blocks)];comparisons[name]=comparison(main,other,32020+index,contract["statistics"]["bootstrap_resamples"]);comparisons[name]["main_goal_success_rate"]=metrics["v3_spatial_safe"]["goal_success_rate"];comparisons[name]["comparator_goal_success_rate"]=metrics[comparator]["goal_success_rate"]
    adjusted=holm({name:value["raw_two_sided_p"] for name,value in comparisons.items()})
    for name,value in comparisons.items():value["holm_adjusted_p"]=adjusted[name]
    deviation=[report for report in reports if not report["execution_integrity_pass"]]
    primary=comparisons["v3_spatial_safe_vs_actual_teb"]
    sensitivity_blocks=[block for block in blocks if block!=27];s_main=[blocks[block]["v3_spatial_safe"]["episode_report"]["result"]["restricted_elapsed_s"] for block in sensitivity_blocks];s_teb=[blocks[block]["actual_teb"]["episode_report"]["result"]["restricted_elapsed_s"] for block in sensitivity_blocks]
    assessment={"schema_version":"1.0","stage":"V3-21-F1","status":"complete_failure_inclusive_characterization_with_one_protocol_deviation","scientific_matrix_complete":True,"formal_result":False,"characterization_result_available":True,"performance_improvement_vs_actual_teb_demonstrated":False,"episode_count":120,"paired_block_count":30,"method_metrics":metrics,"comparisons":comparisons,"primary_interpretation":{"result":"no_restricted_time_improvement_vs_actual_teb","point_relative_reduction":primary["point_relative_reduction"],"two_sided_95_interval":[primary["two_sided_95_lower"],primary["two_sided_95_upper"]],"main_success_rate":metrics["v3_spatial_safe"]["goal_success_rate"],"actual_teb_success_rate":metrics["actual_teb"]["goal_success_rate"]},"candidate_conditioning_exploratory_evidence":comparisons["v3_spatial_safe_vs_v3_no_candidate_conditioning"]["two_sided_95_lower"]>0 and comparisons["v3_spatial_safe_vs_v3_no_candidate_conditioning"]["holm_adjusted_p"]<.05,"protocol_deviations":[{"sequence":107,"seed":8227,"method":"v3_spatial_safe","gate_x_m":deviation[0]["warmup"]["gate_pose"]["x_m"],"absolute_gate_error_m":abs(deviation[0]["warmup"]["gate_pose"]["x_m"]+2.6),"tolerance_m":.03,"outcome_retained":True}],"posthoc_sensitivity_excluding_block27":{"block_count":29,"point_relative_reduction":(statistics.mean(s_teb)-statistics.mean(s_main))/statistics.mean(s_teb),"conclusion_changes":False},"integrity":{"all_attempt1":all(row["attempt"]==1 for row in schedule["rows"]),"all_ports_below_ephemeral":all(report["master_port_below_ephemeral_range"] for report in reports),"execution_integrity_pass_rate":sum(report["execution_integrity_pass"] for report in reports)/120,"zero_contact":all(report["result"]["contact_state_count"]==0 for report in reports),"all_nonnegative_clearance":all(report["result"]["minimum_conservative_clearance_m"]>=0 for report in reports),"gate_actor_coordinate_range_max_m":yaml.safe_load(summary_path.read_bytes())["maximum_gate_actor_coordinate_range_m"],"gate_actor_speed_range_max_mps":yaml.safe_load(summary_path.read_bytes())["maximum_gate_actor_speed_range_mps"]},"claim_boundary":{"exploratory_followup":True,"single_reactive_actor_oracle_track_only":True,"perception_ood_formal_safety_deployment_real_vehicle_claims":False},"training_steps":0,"hardware_accessed":False,"real_vehicle_used":False}
    canonical=[];freeze_sha=sha((FOUNDATION/"terminal_freeze.sha256").read_bytes())
    for receipt in receipts:
        identity=receipt["row"];report=receipt["episode_report"];outcome=report["result"];plan=report.get("method_plan") or {};evaluation=plan.get("selected_evaluation") or {}
        row={field:None for field in schema.REQUIRED_FIELDS};row.update({"schema_version":"1.0","stage":"V3-21-F1","sequence":identity["sequence"],"block_id":"v3_21_block_{:02d}".format(identity["block_index"]),"attempt":1,"seed":identity["seed"],"scene_id":identity["scene_id"],"actor_policy":identity["actor_policy"],"method":identity["method"],"method_role":ROLES[identity["method"]],"simulation_only":True,"completed":True,"terminal_reason":"goal_reached" if outcome["goal_reached"] else "certified_stop_timeout","retry_used":False,"resume_used":False,"wall_time_s":receipt["wall_elapsed_s"],"sim_time_s":outcome["elapsed_s"],"goal_success":outcome["goal_reached"],"restricted_time_s":outcome["restricted_elapsed_s"],"path_length_m":outcome["path_length_m"],"final_position_error_m":outcome["final_position_error_m"],"contact_count":outcome["contact_state_count"],"collision":outcome["contact_state_count"]>0,"minimum_clearance_m":outcome["minimum_conservative_clearance_m"],"emergency_count":0,"safety_fallback_used":plan.get("safety_fallback_used"),"decision_count":1 if plan else outcome["local_plan_count"],"selected_candidate_id":plan.get("selected_candidate_id"),"selected_accepted":evaluation.get("accepted"),"certified_stop_available":True if plan else None,"planner_latency_p99_ms":plan.get("latency_ms"),"tracking_error_max_m":outcome["tracking_error_max_m"],"contract_sha256":(report.get("planner_contract") or {}).get("sha256"),"release_sha256":release_sha,"schedule_sha256":sha(schedule_path.read_bytes()),"report_sha256":receipt["episode_report_sha256"],"raw_manifest_sha256":freeze_sha,"source_revision":"v3_21_frozen_release","scientific_inclusion":True,"statistical_unit":"paired_scene_seed_block","oracle_track_used":True,"training_steps":0,"hardware_accessed":False,"real_vehicle_used":False,"execution_integrity_pass":report["execution_integrity_pass"]});schema.validate_row(row);row["row_digest"]=schema.digest({key:value for key,value in row.items() if key!="row_digest"});canonical.append(row)
    jsonl=b"".join(schema.canonical_json(row)+b"\n" for row in canonical);fields=list(canonical[0]);stream=io.StringIO(newline="");writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n");writer.writeheader();writer.writerows(canonical);canonical_csv=stream.getvalue().encode("utf-8")
    method_rows=[(method,)+tuple(metrics[method][key] for key in ("episodes","goal_successes","goal_success_rate","restricted_time_mean_s","restricted_time_std_s","successful_only_time_mean_s","path_length_mean_m","minimum_clearance_mean_m","minimum_clearance_min_m","contact_count","fallback_rate","execution_integrity_passes","planner_latency_p99_ms")) for method in METHODS]
    comparison_rows=[(name,value["point_relative_reduction"],value["two_sided_95_lower"],value["two_sided_95_upper"],value["mean_comparator_minus_main_s"],value["raw_two_sided_p"],value["holm_adjusted_p"],value["main_goal_success_rate"],value["comparator_goal_success_rate"]) for name,value in comparisons.items()]
    block_rows=[]
    for block,mapping in sorted(blocks.items()):
        first=mapping["v3_spatial_safe"]["row"];values=[mapping[method]["episode_report"]["result"] for method in METHODS];block_rows.append((block,first["seed"],first["scene_id"],first["actor_policy"])+tuple(item["restricted_elapsed_s"] for item in values)+tuple(int(item["goal_reached"]) for item in values))
    strata=defaultdict(list)
    for block,mapping in blocks.items():strata[(mapping["v3_spatial_safe"]["row"]["scene_id"],mapping["v3_spatial_safe"]["row"]["actor_policy"])].append(mapping)
    stratum_rows=[]
    for (scene,policy),items in sorted(strata.items()):
        values=[]
        for method in METHODS:
            rows=[mapping[method]["episode_report"]["result"] for mapping in items];values.extend((sum(row["goal_reached"] for row in rows)/len(rows),statistics.mean(row["restricted_elapsed_s"] for row in rows)))
        stratum_rows.append((scene,policy,len(items))+tuple(values))
    generated={OUT:yaml.safe_dump(assessment,sort_keys=False).encode("utf-8"),CANONICAL_JSONL:jsonl,CANONICAL_CSV:canonical_csv,METHOD_CSV:csv_bytes(("method","episodes","goal_successes","goal_success_rate","restricted_time_mean_s","restricted_time_std_s","successful_only_time_mean_s","path_length_mean_m","minimum_clearance_mean_m","minimum_clearance_min_m","contact_count","fallback_rate","execution_integrity_passes","planner_latency_p99_ms"),method_rows),COMPARISON_CSV:csv_bytes(("comparison","point_relative_reduction","ci95_lower","ci95_upper","mean_comparator_minus_main_s","raw_p","holm_p","main_success_rate","comparator_success_rate"),comparison_rows),BLOCK_CSV:csv_bytes(("block","seed","scene","actor_policy")+tuple("{}_time_s".format(method) for method in METHODS)+tuple("{}_success".format(method) for method in METHODS),block_rows),STRATUM_CSV:csv_bytes(("scene","actor_policy","blocks")+tuple(field for method in METHODS for field in ("{}_success_rate".format(method),"{}_mean_time_s".format(method))),stratum_rows)}
    manifest="".join("{}  {}\n".format(sha(path.read_bytes()),path.relative_to(W)) for path in sorted(set(resources)))+"".join("{}  {}\n".format(sha(data),path.relative_to(W)) for path,data in generated.items());generated[MANIFEST]=manifest.encode("utf-8");return generated,assessment
def main():
    parser=argparse.ArgumentParser();parser.add_argument("--check",action="store_true");args=parser.parse_args();generated,assessment=outputs()
    for path,data in generated.items():atomic(path,data,args.check)
    primary=assessment["primary_interpretation"];print("status={} primary={:.6%} ci=[{:.6%},{:.6%}] manifest_sha256={}".format(assessment["status"],primary["point_relative_reduction"],primary["two_sided_95_interval"][0],primary["two_sided_95_interval"][1],sha(generated[MANIFEST])))
if __name__=="__main__":main()

