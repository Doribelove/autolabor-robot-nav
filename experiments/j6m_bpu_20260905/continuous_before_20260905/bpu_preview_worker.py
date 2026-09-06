#!/usr/bin/env python3
"""J6M visual-demo worker. No ROS, TCP listener, CUDA, CAN or motion outputs.

Uses the already verified vendor CLI per frame, intentionally limited by the
client to 1 Hz. Scratch dumps are bounded/reused, not a production inference API.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
import fcntl
import hashlib
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time

LAB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB / "vendor/python311"))
import numpy as np
from decode_fcos_demo import decode, SIZES
from live_protocol import HEADER, PAYLOAD_BYTES, Y_BYTES, read_exact, send_json, unpack


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_id")
    parser.add_argument("--display-with-navigation", action="store_true",
                        help="Explicit opt-in to display-only BPU work alongside an existing ROS master")
    args = parser.parse_args()
    if LAB != Path("/map/robot_j6m_optimized_20260905/bpu_lab_20260905"):
        raise RuntimeError("Worker must run inside the private J6M laboratory")
    if not re.fullmatch(r"[a-z0-9_]{1,64}", args.run_id):
        raise ValueError("Invalid run identifier")
    with socket.socket() as probe:
        probe.settimeout(0.3)
        if probe.connect_ex(("127.0.0.1", 11311)) == 0 and not args.display_with_navigation:
            raise RuntimeError("Navigation master is present; refuse concurrent preview")
    if subprocess.run(["pgrep", "-x", "hrt_model_exec"], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError("Another inference process is present")
    model = LAB / "models/fcos_nash_m/model.hbm"
    if hashlib.sha256(model.read_bytes()).hexdigest() != "1daacc7dfde181b65872c47f4fe5db84fd31f19d3ad56aa1fa1d902bb5ffd7fa":
        raise RuntimeError("Reference model hash changed")
    lock_file = (LAB / "preview_worker.lock").open("a")
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    directory = LAB / "live" / args.run_id
    directory.mkdir(parents=True, exist_ok=False)
    scratch = directory / "scratch"
    scratch.mkdir()
    dump = scratch / "dump"
    dump.mkdir()
    env = dict(os.environ, LD_LIBRARY_PATH=str(LAB / "vendor/runtime_4_9_2/lib") + ":/usr/hobot/lib")
    command = [str(LAB / "vendor/runtime_4_9_2/bin/hrt_model_exec"), "infer",
               "--model_file=" + str(model),
               "--input_file=" + str(scratch / "y.bin") + "," + str(scratch / "uv.bin"),
               "--input_stride=802816,896,1,1;401408,896,2,1", "--core_id=1", "--frame_count=1",
               "--enable_dump=true", "--dequantize_process=true", "--remove_padding_process=true",
               "--dump_format=bin", "--dump_path=" + str(dump)]
    paths = [dump / ("model_infer_output_%d__output_%d.bin" % (i, i)) for i in range(15)]
    reader = os.fdopen(os.dup(sys.stdin.fileno()), "rb", buffering=0)
    writer = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    send_json(writer, {"ready": True, "demo_only": True, "pid": os.getpid(), "numpy": np.__version__})
    previous = 0
    deadline = time.monotonic() + 660
    with (directory / "inference.log").open("x") as log:
        while time.monotonic() < deadline:
            try:
                data = read_exact(reader, HEADER.size, timeout=10)
            except EOFError:
                break
            sequence, stamp, width, height = unpack(data)
            if sequence <= previous:
                raise ValueError("Non-increasing frame sequence")
            payload = read_exact(reader, PAYLOAD_BYTES)
            if shutil.disk_usage(LAB).free < 512 * 1024 * 1024:
                raise RuntimeError("Insufficient free space for bounded demo tensors")
            started = time.monotonic()
            (scratch / "y.bin").write_bytes(payload[:Y_BYTES])
            (scratch / "uv.bin").write_bytes(payload[Y_BYTES:])
            # Only discard this worker's consumed temporary output tensors;
            # never consume a preceding frame if a dump was not regenerated.
            for path in paths:
                if path.exists():
                    path.unlink()
            result = subprocess.run(command, cwd=str(directory), env=env, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=8, check=False)
            log.write("\nFRAME %d STAMP_NS %d\n" % (sequence, stamp))
            log.write(result.stdout.decode(errors="replace"))
            log.flush()
            result.check_returncode()
            outputs = []
            for index, path in enumerate(paths):
                channels = 80 if index < 5 else 4 if index < 10 else 1
                size = SIZES[index % 5]
                if path.stat().st_size != channels * size * size * 4:
                    raise ValueError("Malformed output dump")
                outputs.append(np.fromfile(str(path), dtype="<f4").reshape(channels, size, size))
            detections = decode(outputs)
            match = re.search(rb"Infer time: ([0-9.]+) ms", result.stdout)
            send_json(writer, {"sequence": sequence, "stamp_ns": stamp, "width": width, "height": height,
                               "detections": detections, "demo_only": True, "motion_eligible": False,
                               "vendor_infer_ms": float(match[1]) if match else None,
                               "worker_ms": (time.monotonic() - started) * 1000})
            previous = sequence
    lock_file.close()


if __name__ == "__main__":
    main()
