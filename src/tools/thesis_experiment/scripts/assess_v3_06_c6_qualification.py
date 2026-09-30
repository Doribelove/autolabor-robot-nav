#!/usr/bin/env python3
"""Assess V3-06-C6 dense cached qualification."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

W=Path("/home/robot/robot_ws_base_rl")
V3=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_c6.py"
BASE=W/"src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
RUNNER=W/"src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"


def load(path,name):
    s=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m


def write_once(path,payload):
    if path.exists():
        if path.read_bytes()!=payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)


def assess(v,base,runner):
    runner.apply_selection_method=v.apply_selection_method
    pooled=base.assess(v,runner); evidence=pooled.pop("_evidence_resources")
    contract=v.load_yaml_relative(W,v.CONTRACT_RELATIVE)
    schedule=v.load_yaml_relative(W,v.SCHEDULE_RELATIVE)
    outcomes={x["unit_id"]:x for x in pooled["outcomes"]}
    acc={m:{"times":[],"success":0,"decisions":0,"stop":0,"collision":0}
         for m in schedule["methods"]}
    pairs={}
    for row in schedule["rows"]:
        episode=v.load_yaml_relative(
            W,v.EXECUTION_RELATIVE/"episodes"/row["unit_id"]/"episode_receipt.yaml"
        )
        outcome=outcomes[row["unit_id"]]
        restricted=float(outcome["goal_time_s"]) if outcome["goal_reached"] else 10.0
        key="{}:{}".format(row["block_id"],row["actor_policy_evaluator_only"])
        pairs.setdefault(key,{})[row["method"]]=restricted
        a=acc[row["method"]]; a["times"].append(restricted)
        a["success"]+=outcome["goal_reached"]; a["decisions"]+=outcome["decision_count"]
        a["stop"]+=sum(
            d["selected_candidate_id"]=="certified_stop_v000" for d in episode["decisions"]
        )
        clearances=[float(episode["prefix"]["minimum_conservative_clearance_m"])]+[
            float(d["segment"]["minimum_conservative_clearance_m"])
            for d in episode["decisions"]
        ]
        a["collision"]+=min(clearances)<0.0
    metrics={}
    for method,a in acc.items():
        n=len(a["times"])
        metrics[method]={
            "episode_count":n,"goal_success_rate":a["success"]/n,
            "restricted_goal_time_mean_s":sum(a["times"])/n,
            "decision_count":a["decisions"],
            "certified_stop_decision_count":a["stop"],
            "collision_episode_count":a["collision"],
        }
    effects=[
        (x["sparse_control"]-x["dense_v020_v040"])/x["sparse_control"]
        for x in pairs.values()
    ]
    point=sum(effects)/len(effects)
    positive=sum(x>0.0 for x in effects)/len(effects)
    rules=contract["qualification_gates"]
    treatment=metrics["dense_v020_v040"]; control=metrics["sparse_control"]
    gates={
        "shared_integration_pass":pooled["overall_pass"] is True,
        "point_effect_pass":point>=float(rules["minimum_relative_reduction_vs_control"]),
        "treatment_goal_success_pass":treatment["goal_success_rate"]>=float(
            rules["treatment_goal_success_rate_min"]
        ),
        "goal_success_noninferiority_pass":treatment["goal_success_rate"]>=control["goal_success_rate"],
        "positive_pair_fraction_pass":positive>=float(rules["positive_pair_fraction_min"]),
        "collision_pass":treatment["collision_episode_count"]<=int(rules["collision_count_max"]),
        "selected_candidate_accepted_pass":pooled["metrics"]["selected_candidate_accepted_rate"]>=1.0,
        "exact_replay_pass":pooled["metrics"]["exact_core_replay_rate"]>=1.0,
        "latency_pass":pooled["metrics"]["planner_latency_p99_ms"]<=float(
            rules["planner_latency_p99_max_ms"]
        ),
    }
    qualified=all(gates.values())
    return {
        "schema_version":"1.0","architecture_generation":"v3","stage":"V3-06-C6",
        "assessment_id":"v3_06_c6_qualification_assessment_1",
        "status":(
            "v3_06_c6_dense_cached_qualification_pass"
            if qualified else "v3_06_c6_dense_cached_qualification_fail"
        ),
        "formal_result":False,"execution":pooled["execution"],
        "pooled_integration_metrics":pooled["metrics"],
        "pooled_integration_gates":pooled["gates"],"method_metrics":metrics,
        "paired_effect":{
            "paired_unit_count":len(effects),
            "point_relative_reduction_vs_control":point,
            "positive_pair_fraction":positive,
        },
        "qualification_gates":gates,
        "qualified_for_fresh_formal_evaluation":qualified,
        "overall_pass":qualified,
        "claim":{
            "formal_performance_improvement":False,"actual_teb_superiority":False,
            "general_navigation_safety":False,"deployment_ready":False,
            "real_vehicle_ready":False,
        },
        "_evidence_resources":evidence,
    }


def main():
    p=argparse.ArgumentParser(); p.add_argument("--check",action="store_true"); args=p.parse_args()
    v=load(V3,"c6av"); base=load(BASE,"c6ab"); runner=load(RUNNER,"c6ar")
    value=assess(v,base,runner); evidence=value.pop("_evidence_resources")
    payload=v.yaml_bytes(value); output=W/v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(output.is_file() and output.read_bytes()==payload,"assessment differs")
    else:
        write_once(output,payload)
        freeze=("\n".join(
            "{}  {}".format(v.sha256_bytes(v.read_file_once(W/item)),item.as_posix())
            for item in evidence+[v.ASSESSMENT_RELATIVE]
        )+"\n").encode("utf-8")
        write_once(W/v.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} assessment_sha256={}".format(
        value["status"],value["qualified_for_fresh_formal_evaluation"],
        hashlib.sha256(payload).hexdigest()
    ))


if __name__=="__main__": main()
