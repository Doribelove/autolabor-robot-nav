#!/usr/bin/env python3
"""Independent review of the V3-21 safe port release."""
import argparse,hashlib,os
from collections import Counter
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl");ROOT=W/"artifacts/v3/thesis/v3_21_a0_port_repair_foundation";OUT=ROOT/"independent_release_review.yaml";FREEZE=ROOT/"terminal_freeze.sha256"
def sha(data):return hashlib.sha256(data).hexdigest()
def atomic(path,data,check):
    if check:
        if not path.is_file() or path.read_bytes()!=data:raise RuntimeError("{} differs".format(path))
        return
    if path.exists():raise RuntimeError("{} exists".format(path))
    temporary=path.with_name(path.name+".tmp")
    with temporary.open("wb") as stream:stream.write(data);stream.flush();os.fsync(stream.fileno())
    os.replace(str(temporary),str(path))
def main():
    parser=argparse.ArgumentParser();parser.add_argument("--check",action="store_true");args=parser.parse_args();manifest=ROOT/"foundation_manifest.sha256";release_path=ROOT/"execution_release.yaml";auth_path=ROOT/"simulation_authorization.yaml";schedule_path=ROOT/"formal_exact_schedule.yaml";release=yaml.safe_load(release_path.read_bytes());auth=yaml.safe_load(auth_path.read_bytes());schedule=yaml.safe_load(schedule_path.read_bytes());terminal=yaml.safe_load((W/"artifacts/v3/thesis/v3_20_f1_execution/terminal_failure_assessment.yaml").read_bytes());entries=[line.split("  ",1) for line in manifest.read_text().splitlines() if line];ports=[row["master_port"] for row in schedule["rows"]]+[row["gazebo_master_port"] for row in schedule["rows"]];per_block=Counter(row["block_index"] for row in schedule["rows"]);per_stratum=Counter((row["scene_id"],row["actor_policy"]) for row in schedule["rows"] if row["method"]=="v3_spatial_safe")
    checks={"manifest_rehash":all((W/path).is_file() and sha((W/path).read_bytes())==digest for digest,path in entries),"v3_20_terminal_preserved":terminal["formal_or_inferential_result"] is False and terminal["seed_range_8101_8130_reuse_forbidden"] is True,"fresh_seed_range":min(row["seed"] for row in schedule["rows"])==8201 and max(row["seed"] for row in schedule["rows"])==8230,"balanced_120_schedule":len(schedule["rows"])==120 and set(per_block.values())=={4} and set(per_stratum.values())=={5},"ports_unique_and_below_ephemeral":len(ports)==len(set(ports)) and min(ports)==24002 and max(ports)==24241 and max(ports)<release["linux_ephemeral_start"],"only_infrastructure_factor_changed":yaml.safe_load((W/"config/thesis_experiments/v3/v3_21_a0_ephemeral_port_repair_contract.yaml").read_bytes())["single_infrastructure_factor"]["method_scene_actor_vehicle_or_statistic_changed"] is False,"runtime_bindings_rehash":all(sha((W/item["path"]).read_bytes())==item["sha256"] for item in release["runtime_bindings"]),"simulation_authority_bounded":auth["execution_authorized"] is True and auth["training_authorized"] is False and auth["hardware_access_authorized"] is False and auth["real_vehicle_authorized"] is False};value={"schema_version":"1.0","stage":"V3-21-A0","status":"v3_21_a0_independent_port_repair_release_review_pass" if all(checks.values()) else "v3_21_a0_independent_port_repair_release_review_fail","checks":checks,"release_sha256":sha(release_path.read_bytes()),"manifest_sha256":sha(manifest.read_bytes()),"execution_ready":all(checks.values()),"formal_units":120,"training_authorized":False,"hardware_access_authorized":False,"real_vehicle_authorized":False};body=yaml.safe_dump(value,sort_keys=False).encode("utf-8");atomic(OUT,body,args.check);paths=(manifest,release_path,auth_path,schedule_path,OUT);freeze="".join("{}  {}\n".format(sha(path.read_bytes()),path.relative_to(W)) for path in paths).encode("utf-8");atomic(FREEZE,freeze,args.check);print("status={} release_sha256={} review_sha256={}".format(value["status"],value["release_sha256"],sha(body)))
if __name__=="__main__":main()

