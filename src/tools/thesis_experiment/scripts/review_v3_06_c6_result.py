#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path

W=Path("/home/robot/robot_ws_base_rl")
V=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_06_c6.py"
A=W/"src/tools/thesis_experiment/scripts/assess_v3_06_c6_qualification.py"
B=W/"src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
R=W/"src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def sha(x): return hashlib.sha256(x).hexdigest()
def write_once(p,x):
    if p.exists():
        if p.read_bytes()!=x: raise RuntimeError("refusing overwrite")
    else:p.write_bytes(x)


def main():
    p=argparse.ArgumentParser(); p.add_argument("--check",action="store_true"); args=p.parse_args()
    v=load(V,"c6rv"); a=load(A,"c6ra"); b=load(B,"c6rb"); r=load(R,"c6rr")
    value=a.assess(v,b,r); value.pop("_evidence_resources")
    stored=v.read_file_once(W/v.ASSESSMENT_RELATIVE)
    v.require(v.yaml_bytes(value)==stored,"assessment replay differs")
    freeze=v.read_file_once(W/v.EVIDENCE_FREEZE_RELATIVE)
    for line in freeze.decode().splitlines():
        digest,path=line.split("  ",1); v.require(sha(v.read_file_once(W/path))==digest,"evidence differs")
    q=value["qualified_for_fresh_formal_evaluation"]
    review={
        "schema_version":"1.0","architecture_generation":"v3","stage":"V3-06-C6",
        "status":(
            "v3_06_c6_qualification_review_pass" if q
            else "v3_06_c6_qualification_failure_review_pass"
        ),
        "assessment_sha256":sha(stored),"evidence_freeze_sha256":sha(freeze),
        "completed_units":36,"qualified_for_fresh_formal_evaluation":q,
        "formal_result":False,"simulation_authorized":False,
        "training_authorized":False,"real_vehicle_authorized":False,
    }
    payload=v.yaml_bytes(review); output=W/v.INDEPENDENT_RESULT_REVIEW_RELATIVE
    terminal=(
        "{}  {}\n{}  {}\n{}  {}\n".format(
            sha(stored),v.ASSESSMENT_RELATIVE.as_posix(),
            sha(freeze),v.EVIDENCE_FREEZE_RELATIVE.as_posix(),
            sha(payload),v.INDEPENDENT_RESULT_REVIEW_RELATIVE.as_posix()
        )
    ).encode()
    if args.check:
        v.require(output.read_bytes()==payload,"review differs")
        v.require(v.read_file_once(W/v.TERMINAL_FREEZE_RELATIVE)==terminal,"terminal differs")
    else:
        write_once(output,payload); write_once(W/v.TERMINAL_FREEZE_RELATIVE,terminal)
    print("status={} qualified={} review_sha256={}".format(review["status"],q,sha(payload)))


if __name__=="__main__":main()
