#!/usr/bin/env python3
"""J6M visual-demo worker. No ROS, TCP listener, CUDA, CAN or motion outputs.

The default engine keeps the model and tensors resident. The old CLI engine is
retained explicitly for golden comparisons and rollback. One request at a time.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
from contextlib import ExitStack
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
from live_protocol import HEADER, Y_BYTES, read_exact, read_payload, send_json, unpack
from preview_lifecycle import BoundedLog, within_lifetime


class CliEngine:
    """The original per-frame CLI path, retained for explicit A/B comparisons."""
    def __init__(self, model, scratch, directory, log):
        self.scratch, self.directory, self.log = scratch, directory, log
        self.env = dict(os.environ, LD_LIBRARY_PATH=str(LAB / "vendor/runtime_4_9_2/lib") + ":/usr/hobot/lib")
        self.command = [str(LAB / "vendor/runtime_4_9_2/bin/hrt_model_exec"), "infer",
                        "--model_file=" + str(model),
                        "--input_file=" + str(scratch / "y.bin") + "," + str(scratch / "uv.bin"),
                        "--input_stride=802816,896,1,1;401408,896,2,1", "--core_id=1", "--frame_count=1",
                        "--enable_dump=true", "--dequantize_process=true", "--remove_padding_process=true",
                        "--dump_format=bin", "--dump_path=" + str(scratch / "dump")]
        self.paths = [scratch / "dump" / ("model_infer_output_%d__output_%d.bin" % (i, i)) for i in range(15)]
        self.model_load_ms = None

    def infer(self, payload):
        (self.scratch / "y.bin").write_bytes(payload[:Y_BYTES])
        (self.scratch / "uv.bin").write_bytes(payload[Y_BYTES:])
        for path in self.paths:
            if path.exists():
                path.unlink()  # Only this session's consumed tensor scratch.
        result = subprocess.run(self.command, cwd=str(self.directory), env=self.env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=8, check=False)
        self.log.write(result.stdout.decode(errors="replace"))
        result.check_returncode()
        outputs = []
        for index, path in enumerate(self.paths):
            channels = 80 if index < 5 else 4 if index < 10 else 1
            size = SIZES[index % 5]
            if path.stat().st_size != channels * size * size * 4:
                raise ValueError("Malformed output dump")
            outputs.append(np.fromfile(str(path), dtype="<f4").reshape(channels, size, size))
        match = re.search(rb"Infer time: ([0-9.]+) ms", result.stdout)
        return outputs, dict(vendor_infer_ms=float(match[1]) if match else None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_id")
    parser.add_argument("--engine", choices=("resident", "cli"), default="resident")
    parser.add_argument("--display-with-navigation", action="store_true",
                        help="Explicit opt-in to display-only BPU work alongside an existing ROS master")
    parser.add_argument("--continuous", action="store_true",
                        help="No demo lifetime; EOF, idle and inference timeouts remain active")
    args = parser.parse_args()
    if LAB != Path("/map/robot_j6m_optimized_20260905/bpu_lab_20260905"):
        raise RuntimeError("Worker must run inside the private J6M laboratory")
    # Dependency resolution happens when the process loads native libraries.
    # Re-exec with only this worker's existing private runtime; never change the system.
    runtime_path = str(LAB / "vendor/runtime_4_9_2/lib") + ":/usr/hobot/lib"
    if args.engine == "resident" and os.environ.get("LD_LIBRARY_PATH") != runtime_path:
        os.execve(sys.executable, [sys.executable, "-u"] + sys.argv,
                  dict(os.environ, LD_LIBRARY_PATH=runtime_path, PYTHONDONTWRITEBYTECODE="1"))
    if not re.fullmatch(r"[a-z0-9_]{1,64}", args.run_id):
        raise ValueError("Invalid run identifier")
    if args.continuous and not args.display_with_navigation:
        raise ValueError("Continuous worker requires explicit navigation display-only opt-in")
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
    scratch = directory / "scratch"
    dump = scratch / "dump"
    for path in (directory, scratch, dump):
        if path.resolve() != path:
            raise RuntimeError("Refuse symlinked preview scratch directory")
        path.mkdir(parents=True, exist_ok=args.continuous)
    reader = os.fdopen(os.dup(sys.stdin.fileno()), "rb", buffering=0)
    writer = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    # Keep the duplicated protocol FD clean even when the vendor uses printf(1).
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    previous = 0
    started_session = time.monotonic()
    seconds = None if args.continuous else 660
    with ExitStack() as cleanup:
        log = cleanup.enter_context(BoundedLog(directory / "inference.log"))
        if args.engine == "resident":
            from resident_fcos import ResidentFCOS
            engine = cleanup.enter_context(ResidentFCOS(LAB, model))
        else:
            engine = CliEngine(model, scratch, directory, log)
        send_json(writer, {"ready": True, "demo_only": True, "motion_eligible": False,
                           "continuous": args.continuous, "engine": args.engine,
                           "pid": os.getpid(), "numpy": np.__version__, "model_load_ms": engine.model_load_ms})
        log.write("WORKER_START pid=%d continuous=%s engine=%s" % (os.getpid(), args.continuous, args.engine))
        next_disk_check = 0.0
        while within_lifetime(started_session, time.monotonic(), seconds):
            try:
                data = read_exact(reader, HEADER.size, timeout=10)
            except EOFError:
                break
            sequence, stamp, width, height = unpack(data)
            if sequence <= previous:
                raise ValueError("Non-increasing frame sequence")
            payload = read_payload(reader, data)
            if time.monotonic() >= next_disk_check:
                if shutil.disk_usage(LAB).free < 512 * 1024 * 1024:
                    raise RuntimeError("Insufficient free space for bounded demo tensors")
                next_disk_check = time.monotonic() + 5.0
            started = time.monotonic()
            outputs, timing = engine.infer(payload)
            decode_started = time.monotonic()
            detections = decode(outputs)
            timing["decode_ms"] = (time.monotonic() - decode_started) * 1000
            send_json(writer, {"sequence": sequence, "stamp_ns": stamp, "width": width, "height": height,
                               "detections": detections, "demo_only": True, "motion_eligible": False,
                               "engine": args.engine, **timing,
                               "worker_ms": (time.monotonic() - started) * 1000})
            previous = sequence
    lock_file.close()


if __name__ == "__main__":
    main()
