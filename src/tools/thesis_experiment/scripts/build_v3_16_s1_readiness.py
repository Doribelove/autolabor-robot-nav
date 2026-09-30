#!/usr/bin/env python3
"""Build the immutable V3-16-S1 response-gated simulation release."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml

W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(0, str(W / "src/tools/thesis_experiment/scripts"))
import v3_16_response_gated_design as design

CONTRACT = Path("config/thesis_experiments/v3/v3_16_s1_response_gated_support_contract.yaml")
AUTH = Path("experiments/manifests/v3/v3_16_s1_simulation_authorization.yaml")
ROOT = Path("artifacts/v3/multiactor/v3_16_s1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"; DEPENDENCIES = ROOT / "dependency_freeze.yaml"; RELEASE = ROOT / "execution_release.yaml"; MANIFEST = ROOT / "release_manifest.sha256"
BASE_PATH = W / "src/tools/thesis_experiment/scripts/build_v3_14_s1_readiness.py"
spec = importlib.util.spec_from_file_location("v3_16_schedule_base", BASE_PATH); base = importlib.util.module_from_spec(spec); spec.loader.exec_module(base)
RESOURCES = [CONTRACT, AUTH,
 Path("artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution/attribution_report.yaml"),
 Path("artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution/independent_review.yaml"),
 Path("artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution/source_snapshot/snapshot_manifest.sha256"),
 Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"), Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
 Path("src/simulation/m2_gazebo/tests/test_ackermann_kinematics.cpp"), Path("devel/lib/libv3_reactive_actor_plugin.so"),
 Path("src/tools/thesis_experiment/scripts/v3_16_response_gated_design.py"), Path("src/tools/thesis_experiment/tests/test_v3_16_response_gated_design.py"),
 Path("src/application/v3_scope_planner/scripts/v3_scope_response_gated_tolerance_repaired_clone_episode.py"),
 Path("src/simulation/m2_gazebo/launch/m2_v3_16_response_gated_bank_clone_episode.launch"),
 Path("src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"),
 Path("src/tools/thesis_experiment/scripts/build_v3_16_s1_readiness.py"), Path("src/tools/thesis_experiment/scripts/review_v3_16_s1_readiness.py"),
 Path("src/tools/thesis_experiment/scripts/run_v3_16_s1_response_gated_support.py"), Path("src/tools/thesis_experiment/scripts/assess_v3_16_s1_response_gated_support.py"),
 Path("src/tools/thesis_experiment/scripts/review_v3_16_s1_result.py"), Path("config/thesis_experiments/v3/v3_13_d0_multiactor_foundation_contract.yaml"),
 Path("config/thesis_experiments/v3/v3_13_q1_r17_goal_aligned_recovery_contract.yaml"), Path("devel/lib/libm2_ackermann_plugin.so")]
def read(p): return (W / p).read_bytes()
def sha(p): return hashlib.sha256(p).hexdigest()
def ybytes(v): return yaml.safe_dump(v, sort_keys=False, allow_unicode=True).encode("utf-8")
def atomic(p,b,check):
 t=W/p
 if check:
  if not t.is_file() or t.read_bytes()!=b: raise RuntimeError("{} differs".format(p))
  return
 if t.exists(): raise RuntimeError("{} exists".format(p))
 t.parent.mkdir(parents=True,exist_ok=True); q=t.with_name(t.name+".tmp")
 with q.open("wb") as s: s.write(b);s.flush();os.fsync(s.fileno())
 os.replace(str(q),str(t))
def build():
 c=yaml.safe_load(read(CONTRACT)); a=yaml.safe_load(read(AUTH)); rows=base.schedule_rows(c)
 if len(rows)!=45 or {r["seed"] for r in rows}!={7741,*range(7742,7750)} or min(r["seed"] for r in rows)<=7728: raise RuntimeError("schedule/seed roster")
 actions=set(c["population"]["actions"])
 binding_spec=importlib.util.spec_from_file_location("v3_16_binding",W/"src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"); binding=importlib.util.module_from_spec(binding_spec);binding_spec.loader.exec_module(binding)
 scene_contract=yaml.safe_load(read(Path("config/thesis_experiments/v3/v3_13_d0_multiactor_foundation_contract.yaml"))); scenes={x["scene_id"]:x for x in scene_contract["scene_families"]}
 for r in rows:
  cmd=["roslaunch","m2_gazebo","m2_v3_16_response_gated_bank_clone_episode.launch"]+binding.build_launch_binding_args(r,tuple(actions));binding.verify_preconsumption_binding(r,cmd,tuple(actions))
  text,specs=design.render_world(scenes[r["scene_id"]],r["actor_policies_evaluator_only"],r["seed"])
  values=[float(x.text) for x in ET.fromstring(text).findall(".//plugin/response_enable_time")]
  if values!=[12.55,12.55] or any(x["response_enable_sim_s"]!=12.55 for x in specs): raise RuntimeError("response gate materialization")
 up=c["upstream"]
 for p,k in ((RESOURCES[2],"v3_16_d1_attribution_report_sha256"),(RESOURCES[3],"v3_16_d1_review_sha256"),(RESOURCES[4],"v3_16_d1_source_snapshot_manifest_sha256")):
  if sha(read(p))!=up[k]: raise RuntimeError("upstream drift")
 if not a["execution_authorized"] or a["authorized_scope"]["exact_units"]!=45: raise RuntimeError("authority")
 schedule={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-16-S1","status":"immutable_exact_schedule","row_count":45,"preflight_units":5,"evidence_units":40,"rows":rows}; sb=ybytes(schedule)
 deps={"schema_version":"1.0","stage":"V3-16-S1","status":"closed_response_gated_dependency_freeze","resource_count":len(RESOURCES),"resources":[{"path":str(p),"sha256":sha(read(p))} for p in RESOURCES]}; db=ybytes(deps)
 release={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-16-S1","status":"fresh_bounded_response_gated_simulation_release","contract_sha256":sha(read(CONTRACT)),"authorization_sha256":sha(read(AUTH)),"exact_schedule_sha256":sha(sb),"dependency_freeze_sha256":sha(db),"authorized_units":45,"fresh_seeds":[7741,*range(7742,7750)],"attempts_per_identity":1,"single_factor":{"name":"reactive_actor_response_enable_sim_s","control":0.0,"treatment":12.55},"retry_resume_replacement_or_selective_exclusion_authorized":False,"execution_authorized":True,"predictor_fit_authorized":False,"actual_TEB_execution_authorized":False,"perception_execution_authorized":False,"training_authorized":False,"hardware_device_access_authorized":False,"real_vehicle_authorized":False}; rb=ybytes(release)
 out={SCHEDULE:sb,DEPENDENCIES:db,RELEASE:rb}; manifest="".join("{}  {}\n".format(sha(read(p)),p) for p in RESOURCES)+"".join("{}  {}\n".format(sha(b),p) for p,b in out.items());out[MANIFEST]=manifest.encode();return out
def main():
 p=argparse.ArgumentParser();p.add_argument("--check",action="store_true");args=p.parse_args();out=build()
 for path,b in out.items():atomic(path,b,args.check)
 print("release_sha256={} manifest_sha256={}".format(sha(out[RELEASE]),sha(out[MANIFEST])))
if __name__=="__main__":main()

