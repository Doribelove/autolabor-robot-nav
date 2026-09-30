#!/usr/bin/env python3
"""Assess paired V3-09-Q1 performance qualification."""
import argparse,hashlib,importlib.util,json,math
from collections import defaultdict
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1.py"; DISPATCH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_dispatch.py"
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path); value=importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value
def write_once(path,payload):
    if path.exists():
        if path.read_bytes()!=payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)
def assess(v):
    c=v.load_yaml_relative(W,v.CONTRACT_RELATIVE); p=v.load_yaml_relative(W,v.PREREGISTRATION_RELATIVE); v.validate_contract(c,p); schedule=v.load_yaml_relative(W,v.SCHEDULE_RELATIVE); v.require(schedule==v.exact_schedule(c),"Q1 schedule differs")
    planner=v.load_yaml_path(W/c["upstream"]["planner_contract_path"]); model=v.load_yaml_path(W/c["frozen_model"]["path"]); dispatch=load(DISPATCH,"q1_assess_dispatch")
    journal=[json.loads(x) for x in (W/v.JOURNAL_RELATIVE).read_text().splitlines() if x.strip()]; v.require(journal[-1]["event"]=="execution_complete" and journal[-1]["completed_units"]==36 and len([x for x in journal if x["event"]=="unit_complete"])==36,"Q1 journal incomplete")
    methods=["v3_spatial_safe","fixed_v120_shielded"]; rows_by_method=defaultdict(list); blocks=defaultdict(dict); selected=stop=exact=0; min_clear=float("inf"); violations=0; latencies=[]; nonfinite=0; stage_correct=0; evidence=[v.JOURNAL_RELATIVE]
    for row in schedule["rows"]:
        root=v.EXECUTION_RELATIVE/"episodes"/row["unit_id"]; rp=root/"episode_receipt.yaml"; receipt=v.RAW_LOAD(W,rp); v.require(receipt["row"]==row,"Q1 receipt row differs"); stage_correct+=receipt["stage"]=="V3-09-Q1"
        restricted=float(receipt["goal_time_s"]) if receipt["goal_reached"] else 12.5; rows_by_method[row["method"]].append((receipt["goal_reached"],restricted)); blocks[row["block_id"]].setdefault(row["method"],[]).append(restricted)
        for segment in [receipt["prefix"]]+[x["segment"] for x in receipt["decisions"]]:
            clearance=float(segment["minimum_conservative_clearance_m"]); min_clear=min(min_clear,clearance); violations+=clearance<0
        for d in receipt["decisions"]:
            replay=dispatch.plan(d["snapshot_runtime_input_without_policy"],model,planner); exact+=dispatch.digest(replay)==d["planner_result_digest"] and replay["decision"]["decision_digest"]==d["decision_digest"]; selected+=d["selected_candidate_accepted"]; stop+=d["certified_stop_available"]; latencies.append(float(d["planner_latency_ms"])); nonfinite+=sum(not math.isfinite(float(x)) for x in (d["planner_latency_ms"],d["selected_minimum_predicted_clearance_m"]))
        evidence.append(rp); evidence.extend(path.relative_to(W) for path in sorted((W/root).glob("**/*")) if path.is_file() and path.name in ("raw_pose_trace.csv","world.sdf"))
    method_metrics={}
    for method in methods:
        values=rows_by_method[method]; method_metrics[method]={"episode_count":len(values),"goal_success_rate":sum(x[0] for x in values)/len(values),"restricted_goal_time_mean_s":sum(x[1] for x in values)/len(values)}
    block_means={}; reductions=[]
    for block,mapping in sorted(blocks.items()):
        block_means[block]={m:sum(mapping[m])/len(mapping[m]) for m in methods}; reductions.append((block_means[block]["fixed_v120_shielded"]-block_means[block]["v3_spatial_safe"])/block_means[block]["fixed_v120_shielded"])
    point=(method_metrics["fixed_v120_shielded"]["restricted_goal_time_mean_s"]-method_metrics["v3_spatial_safe"]["restricted_goal_time_mean_s"])/method_metrics["fixed_v120_shielded"]["restricted_goal_time_mean_s"]; positive=sum(x>0 for x in reductions)/len(reductions); ordered=sorted(latencies); p99=ordered[min(len(ordered)-1,int(math.ceil(.99*len(ordered)))-1)]; count=len(latencies)
    metrics={"completed_episodes":36,"method_metrics":method_metrics,"point_relative_reduction":point,"positive_block_fraction":positive,"minimum_conservative_clearance_m":min_clear,"conservative_clearance_violation_count":violations,"selected_candidate_accepted_rate":selected/count,"certified_stop_available_rate":stop/count,"exact_core_replay_rate":exact/count,"receipt_stage_identity_rate":stage_correct/36.0,"planner_latency_p99_ms":p99,"nonfinite_metric_count":nonfinite,"decision_count":count,"block_means_restricted_goal_time_s":block_means}
    r=c["qualification_gates"]; gates={"complete_pass":metrics["completed_episodes"]==r["complete_episode_count"],"spatial_goal_pass":method_metrics["v3_spatial_safe"]["goal_success_rate"]>=r["spatial_goal_success_rate_min"],"goal_noninferiority_pass":method_metrics["v3_spatial_safe"]["goal_success_rate"]>=method_metrics["fixed_v120_shielded"]["goal_success_rate"],"point_effect_pass":point>=r["minimum_point_relative_reduction"],"positive_pair_pass":positive>=r["positive_block_fraction_min"],"selection_pass":metrics["selected_candidate_accepted_rate"]>=r["selected_candidate_accepted_rate_min"],"stop_pass":metrics["certified_stop_available_rate"]>=r["certified_stop_available_rate_min"],"replay_pass":metrics["exact_core_replay_rate"]>=r["exact_core_replay_rate_min"],"clearance_pass":violations<=r["conservative_clearance_violation_count_max"],"stage_identity_pass":metrics["receipt_stage_identity_rate"]>=r["receipt_stage_identity_rate_min"],"latency_pass":p99<=r["planner_latency_p99_max_ms"],"finite_pass":nonfinite<=r["nonfinite_metric_count_max"]}
    passed=all(gates.values())
    return {"schema_version":"1.0","architecture_generation":"v3","stage":"V3-09-Q1","status":"paired_performance_qualification_pass" if passed else "paired_performance_qualification_valid_negative","formal_result":False,"execution":{"authorized_units":36,"consumed_units":36,"completed_units":36,"retry_units":0},"metrics":metrics,"gates":gates,"qualified_for_formal_evaluation":passed,"performance_claim_authorized":False,"actual_teb_comparison":False,"runtime_ready":False,"real_vehicle_authorized":False,"overall_pass":passed,"_evidence":evidence}
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--check",action="store_true"); args=ap.parse_args(); v=load(MODULE,"q1assessmodule"); value=assess(v); evidence=value.pop("_evidence"); payload=v.BASE_YAML_BYTES(value)
    if args.check: v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==payload,"Q1 assessment differs")
    else:
        write_once(W/v.ASSESSMENT_RELATIVE,payload); freeze=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/x)),x) for x in evidence+[v.ASSESSMENT_RELATIVE])+"\\n").encode(); write_once(W/v.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} effect={:.6f} assessment_sha256={}".format(value["status"],value["qualified_for_formal_evaluation"],value["metrics"]["point_relative_reduction"],hashlib.sha256(payload).hexdigest()))
if __name__=="__main__": main()
