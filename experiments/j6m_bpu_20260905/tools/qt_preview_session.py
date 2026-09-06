#!/usr/bin/env python3
"""Bounded, owned camera + navigation Qt display test, without a chassis node."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

LAB = Path(__file__).resolve().parents[1]
MASTER = "http://127.0.0.1:11571"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=int, default=180)
    parser.add_argument("--max-fps", type=float, default=30.0)
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--engine", choices=("resident", "cli"), default="resident")
    parser.add_argument("--stop", action="store_true")
    args = parser.parse_args()
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1" or os.environ.get("ROS_MASTER_URI") != MASTER:
        raise RuntimeError("Use the isolated Qt preview launcher")
    pointer = LAB / "qt_preview_current.json"
    if args.stop:
        state = json.loads(pointer.read_text())
        directory = Path(state["directory"])
        if directory.resolve().parent != LAB / "live":
            raise ValueError("Unexpected preview directory")
        (directory / "STOP").touch()
        print("Qt preview stop requested:", directory)
        return
    if not 10 <= args.seconds <= 600:
        raise ValueError("Preview duration must be 10..600 seconds")
    from qt_bpu_bridge import frame_period
    frame_period(args.max_fps)
    if not 1 <= args.camera_fps <= 30:
        raise ValueError("Camera frequency must be within 1..30 Hz")
    lock = (LAB / "qt_session.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    for unit in ("autolabor-dual-host.service", "autolabor-optimized-20260905.service"):
        state = subprocess.check_output(["systemctl", "--user", "show", unit,
                                         "-p", "ActiveState", "--value"], text=True).strip()
        if state not in ("inactive", "failed"):
            raise RuntimeError("Navigation supervisor is active: " + unit)
    for host, port in (("127.0.0.1", 11311), ("127.0.0.1", 11571), ("192.168.10.100", 11311)):
        with socket.socket() as probe:
            probe.settimeout(1)
            if probe.connect_ex((host, port)) == 0:
                raise RuntimeError("Existing ROS master: %s:%d; do not take over camera" % (host, port))
    # The client lock is probed before opening the camera, not used to terminate another owner.
    with (LAB / "preview_client.lock").open("a") as client_lock:
        fcntl.flock(client_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    directory = LAB / "live" / ("qt_session_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    directory.mkdir(parents=True, exist_ok=False)
    status = dict(pid=os.getpid(), directory=str(directory), active=True, master=MASTER,
                  camera_only=True, motion_eligible=False)
    pointer.write_text(json.dumps(status, indent=2) + "\n")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    children = []
    started = time.monotonic()
    reason = "time_limit"
    print("QT_SESSION_START " + json.dumps(status), flush=True)
    try:
        with (directory / "qt_camera.log").open("x") as gui_log, (directory / "bridge.log").open("x") as bridge_log:
            launch = subprocess.Popen(["roslaunch", "-p", "11571", str(LAB / "qt_preview.launch"),
                                       "camera_fps:=" + str(args.camera_fps)],
                                      stdout=gui_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(launch)
            while True:
                if launch.poll() is not None:
                    raise RuntimeError("Camera/Qt launch exited; see qt_camera.log")
                with socket.socket() as probe:
                    probe.settimeout(0.2)
                    if probe.connect_ex(("127.0.0.1", 11571)) == 0:
                        break
                if time.monotonic() - started > 15 or stop.wait(0.1):
                    raise TimeoutError("Private master startup timeout")
            bridge = subprocess.Popen(["/home/slam/robot_ws/.venv/fod_yolo/bin/python3", "-u",
                                       str(LAB / "tools/qt_bpu_bridge.py"), "--seconds", str(args.seconds),
                                       "--max-fps", str(args.max_fps), "--engine", args.engine,
                                       "--image-topic", "/bpu_preview_zed/zed_node/rgb/image_rect_color"],
                                      stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(bridge)
            bridge_reported = False
            while time.monotonic() - started < args.seconds and not stop.wait(0.2):
                if (directory / "STOP").exists():
                    reason = "stop_requested"
                    break
                if launch.poll() is not None:
                    reason = "qt_camera_closed"
                    break
                if bridge.poll() is not None and not bridge_reported:
                    print("BPU bridge exited; retaining Qt live camera. Inspect bridge.log", flush=True)
                    bridge_reported = True
    except Exception as error:
        reason = repr(error)
        raise
    finally:
        if stop.is_set() and reason == "time_limit":
            reason = "stop_signal"
        # Only our exact process groups, bridge first. Never stop a shared stack here.
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait(timeout=2)
        status.update(active=False, elapsed_seconds=time.monotonic() - started, reason=reason)
        (directory / "summary.json").write_text(json.dumps(status, indent=2) + "\n")
        pointer.write_text(json.dumps(status, indent=2) + "\n")
        print("QT_SESSION_STOP " + json.dumps(status), flush=True)


if __name__ == "__main__":
    main()
