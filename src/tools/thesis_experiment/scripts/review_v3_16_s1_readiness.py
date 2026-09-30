#!/usr/bin/env python3
"""Independent review of V3-16-S1 response-gated release."""
import argparse,hashlib,importlib.util,os
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl");ROOT=W/"artifacts/v3/multiactor/v3_16_s1_readiness";OUT=ROOT/"independent_review.yaml"
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument("--check",action="store_true");a=p.parse_args();sp=ROOT/"exact_schedule.yaml";dp=ROOT/"dependency_freeze.yaml";rp=ROOT/"execution_release.yaml"
 s=yaml.safe_load(sp.read_bytes());d=yaml.safe_load(dp.read_bytes());r=yaml.safe_load(rp.read_bytes());rows=s["rows"]
 checks={"release_binds_schedule":r["exact_schedule_sha256"]==sha(sp.read_bytes()),"release_binds_dependencies":r["dependency_freeze_sha256"]==sha(dp.read_bytes()),"dependencies_rehash":all(sha((W/x["path"]).read_bytes())==x["sha256"] for x in d["resources"]),"exact_45_rows":len(rows)==45 and [x["sequence"] for x in rows]==list(range(1,46)),"fresh_seed_roster":{x["seed"] for x in rows}=={7741,*range(7742,7750)},"single_factor_exact":r["single_factor"]=={"name":"reactive_actor_response_enable_sim_s","control":0.0,"treatment":12.55},"bounded_sim_only":r["execution_authorized"] and not any(r[k] for k in ("predictor_fit_authorized","actual_TEB_execution_authorized","perception_execution_authorized","training_authorized","hardware_device_access_authorized","real_vehicle_authorized"))}
 value={"schema_version":"1.0","stage":"V3-16-S1","status":"v3_16_s1_independent_release_review_pass" if all(checks.values()) else "v3_16_s1_independent_release_review_fail","execution_release_sha256":sha(rp.read_bytes()),"checks":checks,"execution_ready":all(checks.values()),"hardware_device_access_authorized":False,"real_vehicle_authorized":False};b=yaml.safe_dump(value,sort_keys=False).encode()
 if a.check:
  if not OUT.is_file() or OUT.read_bytes()!=b:raise RuntimeError("review differs")
 else:
  if OUT.exists():raise RuntimeError("review exists")
  q=OUT.with_name(OUT.name+".tmp");q.write_bytes(b);os.replace(str(q),str(OUT))
 print("status={} review_sha256={}".format(value["status"],sha(b)))
if __name__=="__main__":main()

