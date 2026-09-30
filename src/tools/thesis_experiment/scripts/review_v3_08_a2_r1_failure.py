#!/usr/bin/env python3
import hashlib
import importlib.util
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl"); BUILDER=W/"src/tools/thesis_experiment/scripts/analyze_v3_08_a2_r1_failure.py"
REVIEW=Path("artifacts/v3/analysis/v3_08_a2/v3_08_a2_independent_review.yaml"); TERMINAL=Path("artifacts/v3/analysis/v3_08_a2/v3_08_a2_terminal_freeze.sha256")
def main():
    spec=importlib.util.spec_from_file_location("v3_08_a2_review",BUILDER); b=importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
    report=b.build(); report.pop("_evidence"); payload=b.encode(report)
    if (W/b.REPORT).read_bytes()!=payload: raise RuntimeError("A2 replay differs")
    review={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-08-A2","status":"v3_08_a2_independent_attribution_review_pass" if report["overall_pass"] else "v3_08_a2_independent_attribution_review_fail","report_sha256":hashlib.sha256(payload).hexdigest(),"duration_only_future_design_supported":report["overall_pass"],"receipt_stage_identity_repair_required":True,"simulation_authorized":False,"performance_claim_authorized":False}
    review_bytes=yaml.safe_dump(review,sort_keys=False).encode(); b.write_once(W/REVIEW,review_bytes)
    resources=[b.REPORT,b.FREEZE,REVIEW]; terminal=("\\n".join("{}  {}".format(hashlib.sha256((W/x).read_bytes()).hexdigest(),x) for x in resources)+"\\n").encode(); b.write_once(W/TERMINAL,terminal)
    print("status={} review_sha256={}".format(review["status"],hashlib.sha256(review_bytes).hexdigest()))
if __name__=="__main__": main()
