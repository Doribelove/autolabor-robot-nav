#!/usr/bin/env python3
"""Run the one-shot V3-16-S1 response-gated support study."""
import argparse,importlib.util,subprocess,sys,time
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl");SCRIPTS=W/"src/tools/thesis_experiment/scripts";sys.path.insert(0,str(SCRIPTS))
import v3_16_response_gated_design as design
def imod(name,path):
 s=importlib.util.spec_from_file_location(name,W/path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
BINDING=imod("v3_16_binding","src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py")
R4=imod("v3_16_r4_summary","src/tools/thesis_experiment/scripts/run_v3_13_q1_r19_r4_prefix_aligned_support.py")
COMMON=R4.COMMON;R1=R4.R2.R1
CONTRACT=W/"config/thesis_experiments/v3/v3_16_s1_response_gated_support_contract.yaml";READINESS=W/"artifacts/v3/multiactor/v3_16_s1_readiness";OUTPUT=W/"artifacts/v3/multiactor/v3_16_s1_response_gated_support";GEOMETRY=W/"config/thesis_experiments/v3/v3_13_q1_r17_goal_aligned_recovery_contract.yaml";LAUNCH="m2_v3_16_response_gated_bank_clone_episode.launch"
def command(row,paths,allowed):
 return ["roslaunch","m2_gazebo",LAUNCH]+BINDING.build_launch_binding_args(row,allowed)+["world:={}".format(paths["world"].resolve()),"pre_action_wait_s:=5","branch_target_sim_s:=12.5","report_path:={}".format(paths["report"].resolve()),"scene_spec_path:={}".format(paths["scene"].resolve()),"actor_specs_path:={}".format(paths["actors"].resolve()),"planner_contract_path:={}".format(COMMON.PLANNER_CONTRACT.resolve()),"geometry_contract_path:={}".format(GEOMETRY.resolve()),"gui:=false"]
def episode(row,scene,root,port,journal,allowed):
 root.mkdir(parents=True,exist_ok=False);paths={k:root/v for k,v in {"world":"world.sdf","scene":"scene_spec.yaml","actors":"actor_specs.yaml","report":"episode_report.yaml","log":"roslaunch.log"}.items()};home=root/"runtime_home";home.mkdir();(home/"roslog").mkdir()
 world,actors=design.render_world(scene,row["actor_policies_evaluator_only"],row["seed"]);COMMON.atomic_bytes(paths["world"],world.encode());COMMON.atomic_yaml(paths["scene"],scene);COMMON.atomic_yaml(paths["actors"],{"schema_version":"1.0","stage":"V3-16-S1","response_enable_sim_s":12.55,"actors":actors})
 if any(x["response_enable_sim_s"]!=12.55 for x in actors):raise RuntimeError("response gate materialization")
 cmd=command(row,paths,allowed);pre=BINDING.verify_preconsumption_binding(row,cmd,allowed);COMMON.append_jsonl(journal,{"event":"unit_binding_verified_preconsumption","sequence":row["sequence"],"binding":pre,"response_enable_sim_s":12.55});COMMON.append_jsonl(journal,{"event":"unit_consumed","sequence":row["sequence"],"row":row})
 started=time.monotonic()
 with paths["log"].open("wb") as stream:
  proc=subprocess.Popen(cmd,cwd=str(W),env=R1.closed_overlay_environment(home,port),stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);timed=False
  try:rc=proc.wait(timeout=190)
  except subprocess.TimeoutExpired:timed=True;rc=COMMON.stop_process(proc)
  else:COMMON.stop_process(proc)
 if not paths["report"].is_file():raise RuntimeError("episode report absent rc={} timeout={} tail={}".format(rc,timed,COMMON.read_bytes(paths["log"])[-4000:].decode(errors="replace")))
 report=yaml.safe_load(COMMON.read_bytes(paths["report"]));runtime=BINDING.verify_runtime_report_binding(row,report,allowed)
 if report["stage"]!="V3-16-S1" or report["single_factor"]["treatment"]!=12.55 or not report["prebranch_actor_response_disabled"]:raise RuntimeError("runtime response gate identity")
 receipt={"schema_version":"1.0","stage":"V3-16-S1","row":row,"preconsumption_binding":pre,"runtime_binding":runtime,"response_gate_binding_pass":True,"episode_report":report,"episode_report_sha256":COMMON.sha(COMMON.read_bytes(paths["report"])),"world_sha256":COMMON.sha(COMMON.read_bytes(paths["world"])),"scene_spec_sha256":COMMON.sha(COMMON.read_bytes(paths["scene"])),"actor_specs_sha256":COMMON.sha(COMMON.read_bytes(paths["actors"])),"roslaunch_log_sha256":COMMON.sha(COMMON.read_bytes(paths["log"])),"roslaunch_return_code":rc,"wall_elapsed_s":time.monotonic()-started,"timed_out":timed,"scientific_inclusion_binding_pass":True,"hardware_accessed":False,"real_vehicle_used":False};COMMON.atomic_yaml(root/"matrix_receipt.yaml",receipt);COMMON.append_jsonl(journal,{"event":"unit_complete","sequence":row["sequence"],"row":row,"runtime_binding":runtime,"response_gate_binding_pass":True,"episode_report_sha256":receipt["episode_report_sha256"],"collection_pass":report["episode_collection_pass"]});return receipt
def summarize(receipts,contract_sha,c,phase):
 v=R4.summarize(receipts,contract_sha,c);expected=5 if phase=="preflight" else 40;v.update({"stage":"V3-16-S1","phase":phase,"authorized_units":expected,"consumed_units":len(receipts),"completed_units":len(receipts),"forfeited_units":0,"single_factor":{"name":"reactive_actor_response_enable_sim_s","control":0.0,"treatment":12.55},"predictor_fit_authorized":False,"actual_TEB_execution_authorized":False,"hardware_accessed":False,"real_vehicle_used":False});v["gate_checks"]["completed_units_exact"]=len(receipts)==expected;v["gate_checks"]["all_action_and_response_bindings"]=all(r["scientific_inclusion_binding_pass"] and r["response_gate_binding_pass"] for r in receipts);v["collection_gate_pass"]=all(v["gate_checks"].values());v["status"]="v3_16_s1_{}_{}".format(phase,"pass" if v["collection_gate_pass"] else "fail");return v
def main():
 p=argparse.ArgumentParser();p.add_argument("--output-root",required=True);p.add_argument("--contract-sha256",required=True);p.add_argument("--release-sha256",required=True);p.add_argument("--review-sha256",required=True);p.add_argument("--execute",action="store_true",required=True);a=p.parse_args()
 if Path(a.output_root).resolve()!=OUTPUT.resolve() or OUTPUT.exists():raise RuntimeError("canonical output differs or exists")
 payloads=[COMMON.read_bytes(CONTRACT),COMMON.read_bytes(READINESS/"execution_release.yaml"),COMMON.read_bytes(READINESS/"independent_review.yaml")]
 for b,e,l in zip(payloads,(a.contract_sha256,a.release_sha256,a.review_sha256),("contract","release","review")):
  if COMMON.sha(b)!=e:raise RuntimeError(l+" SHA differs")
 release,review=yaml.safe_load(payloads[1]),yaml.safe_load(payloads[2])
 if not release["execution_authorized"] or not review["execution_ready"]:raise RuntimeError("not ready")
 rows=yaml.safe_load(COMMON.read_bytes(READINESS/"exact_schedule.yaml"))["rows"];sc=yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT));scenes={x["scene_id"]:x for x in sc["scene_families"]};c=yaml.safe_load(payloads[0]);allowed=tuple(c["population"]["actions"])
 OUTPUT.mkdir(parents=True);COMMON.atomic_yaml(OUTPUT/"exact_schedule.yaml",{"schema_version":"1.0","stage":"V3-16-S1","contract_sha256":a.contract_sha256,"row_count":45,"rows":rows});journal=OUTPUT/"attempt_journal.jsonl";journal.touch(exist_ok=False);COMMON.append_jsonl(journal,{"event":"execution_opened","authorized_units":45,"release_sha256":a.release_sha256});got={"preflight":[],"evidence":[]}
 for row in rows:
  root=OUTPUT/"episodes"/"{:03d}_{}_b{:02d}_{}_{}".format(row["sequence"],row["phase"],row["block_index"],row["seed"],row["forced_candidate_id"])
  try:r=episode(row,scenes[row["scene_id"]],root,32500+2*row["sequence"],journal,allowed)
  except Exception as err:COMMON.append_jsonl(journal,{"event":"terminal_failure","sequence":row["sequence"],"row":row,"error":"{}:{}".format(type(err).__name__,err),"remaining_units_forfeited":45-row["sequence"]});raise
  got[row["phase"]].append(r);rep=r["episode_report"];print("[V3-16-S1] {}/45 {} b{} {} bind=pass collect={} ego={:.6f}".format(row["sequence"],row["phase"],row["block_index"],row["forced_candidate_id"],rep["episode_collection_pass"],rep["common_stop_prefix"]["decision_pose"]["x_m"]),flush=True)
  if row["sequence"]==5:
   s=summarize(got["preflight"],a.contract_sha256,c,"preflight");COMMON.atomic_yaml(OUTPUT/"preflight_summary.yaml",s)
   if not s["collection_gate_pass"]:COMMON.atomic_yaml(OUTPUT/"terminal_result.yaml",{"schema_version":"1.0","stage":"V3-16-S1","status":"terminal_negative_preflight","consumed_units":5,"completed_units":5,"forfeited_units":40,"scientific_episode_count":0,"predictor_fit_authorized":False,"hardware_accessed":False,"real_vehicle_used":False});return
   COMMON.append_jsonl(journal,{"event":"preflight_gate_passed","evidence_units_released":40,"summary_sha256":COMMON.sha(COMMON.read_bytes(OUTPUT/"preflight_summary.yaml"))})
 s=summarize(got["evidence"],a.contract_sha256,c,"evidence");COMMON.atomic_yaml(OUTPUT/"evidence_summary.yaml",s);terminal={"schema_version":"1.0","stage":"V3-16-S1","status":"fresh_response_gated_support_pass" if s["collection_gate_pass"] else "terminal_negative_response_gated_support","authorized_units":45,"consumed_units":45,"completed_units":45,"forfeited_units":0,"scientific_episode_count":40,"all_action_and_response_bindings_pass":True,"evidence_collection_gate_pass":s["collection_gate_pass"],"predictor_fit_authorized":s["collection_gate_pass"],"actual_TEB_execution_authorized":False,"perception_execution_authorized":False,"training_authorized":False,"hardware_accessed":False,"real_vehicle_used":False};COMMON.atomic_yaml(OUTPUT/"terminal_result.yaml",terminal);COMMON.append_jsonl(journal,{"event":"execution_complete","status":terminal["status"],"consumed_units":45,"scientific_episode_count":40,"terminal_result_sha256":COMMON.sha(COMMON.read_bytes(OUTPUT/"terminal_result.yaml"))});print("status={} evidence_summary_sha256={}".format(terminal["status"],COMMON.sha(COMMON.read_bytes(OUTPUT/"evidence_summary.yaml"))))
if __name__=="__main__":main()
