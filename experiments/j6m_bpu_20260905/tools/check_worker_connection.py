#!/usr/bin/env python3
"""One saved-image SSH/BPU integration check; never starts a camera or ROS."""
import os
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
from live_protocol import header, receive_json, write_all

if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
    raise RuntimeError("Use optimized.sh run")
lab = Path(__file__).resolve().parents[1]
run_id = "wire_test_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
remote = "/map/robot_j6m_optimized_20260905/bpu_lab_20260905"
process = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                            "root@192.168.10.100", "exec nice -n 10 python3 -u " + remote +
                            "/tools/bpu_preview_worker.py " + run_id],
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
try:
    print(json.dumps(receive_json(process.stdout)))
    stamp = time.time_ns()
    payload = (lab / "inputs/bus/y.bin").read_bytes() + (lab / "inputs/bus/uv.bin").read_bytes()
    write_all(process.stdin, header(1, stamp, 810, 1080) + payload)
    result = receive_json(process.stdout)
    if result["sequence"] != 1 or result["stamp_ns"] != stamp or result["motion_eligible"] is not False:
        raise RuntimeError("Frame identity or eligibility mismatch")
    expected = json.loads((lab / "results/fcos_bus/detections_demo.json").read_text())["detections"]
    if len(result["detections"]) != len(expected):
        raise RuntimeError("Saved-image detection count changed")
    for actual, reference in zip(result["detections"], expected):
        if actual["class_id"] != reference["class_id"] or abs(actual["confidence"] - reference["confidence"]) > 1e-5:
            raise RuntimeError("Saved-image class/score changed")
    print(json.dumps(result, indent=2))
    process.stdin.close()
    if process.wait(timeout=5) != 0:
        raise RuntimeError("Worker did not exit cleanly on EOF")
    print("PASS: frame identity, saved-image detections and worker EOF cleanup")
finally:
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=5)
