#!/usr/bin/env python3
"""Optional Qt display bridge; reuses ZED, never starts a camera or a control node.

Only /fod/bpu_preview/image and /fod/bpu_preview/status are published. The
reference COCO model is deliberately not a production FOD backend. One fresh
source frame in flight, configurable up to 30 Hz; idle when no viewer exists.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
from collections import deque
from contextlib import ExitStack
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import signal
import shutil
import subprocess
import threading
import time

from live_preview import image_bgr, LAB, REMOTE
from live_protocol import frame_packet, receive_json, write_all
from preview_lifecycle import (BoundedLog, drain_stderr, master_proxy, request_stop,
                               session_identity, within_lifetime)

IMAGE_TOPIC = "/fod/bpu_preview/image"
STATUS_TOPIC = "/fod/bpu_preview/status"
FRAME_ID = "bpu_preview_demo_only"


def frame_period(fps):
    if not math.isfinite(fps) or not 0.5 <= fps <= 30:
        raise ValueError("Preview frequency must be finite and within 0.5..30 Hz")
    return 1.0 / fps


def prepare(frame):
    return NV12Preprocessor().prepare(frame)


class NV12Preprocessor:
    """Reuse fixed preprocessing buffers; returned bytes own their storage."""
    def __init__(self):
        import numpy as np
        self.canvas = np.zeros((896, 896, 3), np.uint8)
        self.i420 = np.empty((1344, 896), np.uint8)
        self.nv12 = np.empty(896 * 896 * 3 // 2, np.uint8)
        self.source_shape = None

    def prepare(self, frame):
        import cv2
        height, width = frame.shape[:2]
        if not 64 <= width <= 1920 or not 64 <= height <= 1080:
            raise ValueError("Source dimensions outside preview bounds")
        scale = min(896 / width, 896 / height)
        out_width, out_height = round(width * scale), round(height * scale)
        if self.source_shape != (height, width):
            self.canvas.fill(0)
            self.source_shape = (height, width)
        cv2.resize(frame, (out_width, out_height), dst=self.canvas[:out_height, :out_width])
        cv2.cvtColor(self.canvas, cv2.COLOR_BGR2YUV_I420, dst=self.i420)
        i420, count = self.i420.reshape(-1), 896 * 896
        self.nv12[:count] = i420[:count]
        self.nv12[count::2] = i420[count:count * 5 // 4]
        self.nv12[count + 1::2] = i420[count * 5 // 4:]
        return self.nv12.tobytes(), (out_width / width, out_height / height)


def annotate(frame, scales, response, sequence, stamp_ns, classes):
    import cv2
    import numpy as np
    if (response.get("sequence") != sequence or response.get("stamp_ns") != stamp_ns or
            response.get("demo_only") is not True or response.get("motion_eligible") is not False or
            response.get("width") != frame.shape[1] or response.get("height") != frame.shape[0]):
        raise ValueError("BPU response identity or display-only contract mismatch")
    detections = response.get("detections")
    if not isinstance(detections, list) or len(detections) > 100:
        raise ValueError("Invalid result count")
    canvas = frame.copy()
    for index, detection in enumerate(detections):
        box = np.array(detection["bbox_model_px"], dtype=float)
        confidence, label = detection["confidence"], detection["class_id"]
        if (box.shape != (4,) or not np.isfinite(box).all() or
                not isinstance(confidence, (int, float)) or not 0 <= confidence <= 1 or
                type(label) is not int or label not in classes):
            raise ValueError("Malformed detection")
        box[[0, 2]] = np.clip(box[[0, 2]] / scales[0], 0, frame.shape[1] - 1)
        box[[1, 3]] = np.clip(box[[1, 3]] / scales[1], 0, frame.shape[0] - 1)
        x1, y1, x2, y2 = np.rint(box).astype(int)
        if x2 <= x1 or y2 <= y1:
            continue
        detection["bbox_source_px"] = box.tolist()
        detection["class_name"] = classes[label]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 220, 0), 2)
        cv2.putText(canvas, "%s %.2f" % (classes[label], confidence),
                    (x1, min(frame.shape[0] - 5, max(18 + index % 3 * 18, y1 - 5))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return canvas


def fresh(stamp_seconds, now_seconds):
    age = now_seconds - stamp_seconds
    return stamp_seconds > 0 and math.isfinite(age) and -0.02 <= age <= 3.0


def stop_worker(worker):
    if worker is None:
        return
    if worker.stdin:
        worker.stdin.close()  # EOF releases the worker lock and preserves its log.
    try:
        worker.wait(timeout=3)
    except subprocess.TimeoutExpired:
        worker.terminate()  # Only this bridge's exact SSH child, never a broad kill.
        try:
            worker.wait(timeout=3)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.wait(timeout=2)
    worker.stdout.close()
    if hasattr(worker, 'preview_stderr_thread'):
        worker.preview_stderr_thread.join(timeout=1)
        worker.stderr.close()


def main():
    parser = argparse.ArgumentParser()
    duration = parser.add_mutually_exclusive_group()
    duration.add_argument("--seconds", type=int)
    duration.add_argument("--continuous", action="store_true")
    duration.add_argument("--stop", action="store_true")
    parser.add_argument("--display-with-navigation", action="store_true")
    parser.add_argument("--max-fps", type=float, default=30.0,
                        help="Fresh-frame processing ceiling (0.5..30); never repeats source frames")
    parser.add_argument("--engine", choices=("resident", "cli"), default="resident")
    parser.add_argument("--image-topic", default="/fod_camera/image_raw",
                        choices=("/fod_camera/image_raw", "/bpu_preview_zed/zed_node/rgb/image_rect_color"))
    args = parser.parse_args()
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise RuntimeError("Use the isolated candidate launcher")
    if args.stop:
        print(request_stop(LAB))
        return
    period = frame_period(args.max_fps)
    seconds = None if args.continuous else (args.seconds if args.seconds is not None else 600)
    if seconds is not None and not 10 <= seconds <= 600:
        raise ValueError("Demo duration must be 10..600 seconds")
    master = os.environ.get("ROS_MASTER_URI", "")
    if master not in ("http://127.0.0.1:11571", "http://192.168.10.100:11311"):
        raise ValueError("Unsupported ROS master")
    if master != "http://127.0.0.1:11571" and not args.display_with_navigation:
        raise ValueError("Existing navigation requires explicit display-only opt-in")
    if args.continuous and (master != "http://192.168.10.100:11311" or
                            not args.display_with_navigation):
        raise ValueError("Continuous preview requires the existing candidate navigation session")
    master_client = master_proxy(master) if args.continuous else None
    owned_session = session_identity(master_client) if args.continuous else None
    lock = (LAB / "preview_client.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import cv2
    import rospy
    import yaml
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    cv2.setNumThreads(1)
    preprocessor = NV12Preprocessor()
    rospy.init_node("j6m_bpu_qt_bridge", disable_signals=True)
    image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1, latch=False)
    status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=False)
    classes = yaml.safe_load(Path(
        "/home/slam/yolo11/yolo11_GAM/ultralytics/cfg/datasets/coco.yaml").read_text())["names"]
    run_id = "qt_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory = LAB / "live" / run_id
    directory.mkdir(parents=True, exist_ok=False)
    status = dict(pid=os.getpid(), run_id=run_id, directory=str(directory), master=master,
                  active=True, demo_only=True, motion_eligible=False,
                  continuous=args.continuous, duration_seconds=seconds,
                  engine=args.engine, max_fps=args.max_fps,
                  started_wall_local=datetime.now().isoformat())
    (LAB / "qt_bridge_current.json").write_text(json.dumps(status, indent=2) + "\n")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    latest, latest_lock = [None], threading.Lock()
    frame_available = threading.Event()
    def callback(message):
        with latest_lock:
            latest[0] = message
            frame_available.set()
    worker = subscriber = None
    sequence = frames = generations = 0
    next_frame = last_stamp = 0
    next_health = 0
    next_status = next_snapshot = 0
    completed = deque(maxlen=120)
    worker_starts = []
    started = time.monotonic()
    reason = "ros_shutdown" if args.continuous else "time_limit"
    print("QT_BRIDGE_START " + json.dumps(status), flush=True)
    try:
        with ExitStack() as cleanup:
            log = cleanup.enter_context(BoundedLog(directory / "results.jsonl"))
            remote_log = cleanup.enter_context(BoundedLog(directory / "remote.log"))
            # Drain the owned SSH child's stderr before closing its rotating log.
            cleanup.callback(lambda: stop_worker(worker))
            while not stop.is_set() and not rospy.is_shutdown() and within_lifetime(started, time.monotonic(), seconds):
                if (directory / "STOP").exists():
                    reason = "stop_requested"
                    break
                now = time.monotonic()
                if now >= next_health:
                    if args.continuous and session_identity(master_client) != owned_session:
                        raise RuntimeError("Navigation master/Qt session changed; preview stopped")
                    if shutil.disk_usage(LAB).free < 512 * 1024 * 1024:
                        raise RuntimeError("Insufficient free space for preview diagnostics")
                    status.update(frames=frames, generations=generations,
                                  elapsed_seconds=now - started)
                    (LAB / "qt_bridge_current.json").write_text(json.dumps(status, indent=2) + "\n")
                    next_health = now + 5.0
                if worker is not None and worker.preview_stderr_error:
                    raise RuntimeError("Preview diagnostic stream failed: " + worker.preview_stderr_error)
                if image_pub.get_num_connections() == 0:
                    if subscriber is not None:
                        subscriber.unregister()
                        subscriber = None
                    stop_worker(worker)
                    worker = None
                    with latest_lock:
                        latest[0] = None
                    completed.clear()
                    status["effective_fps"] = 0.0
                    stop.wait(0.5)
                    continue
                if subscriber is None:
                    subscriber = rospy.Subscriber(args.image_topic, Image, callback,
                                                  queue_size=1, buff_size=10 * 1024 * 1024)
                with latest_lock:
                    frame_available.clear()
                    message = latest[0]
                if message is None:
                    status_pub.publish(String(data="等待现有 ZED 原图；本桥接器不启动相机"))
                    stop.wait(0.5)
                    continue
                if not fresh(message.header.stamp.to_sec(), rospy.Time.now().to_sec()):
                    raise TimeoutError("ZED source timestamp stale/future; preview stopped")
                remaining = next_frame - time.monotonic()
                if remaining > 0:
                    stop.wait(min(remaining, 0.025))
                    continue
                if message.header.stamp.to_nsec() <= last_stamp:
                    frame_available.wait(0.025)
                    continue
                if worker is None:
                    generations += 1
                    worker_starts = [stamp for stamp in worker_starts if now - stamp < 600]
                    if len(worker_starts) >= 30:
                        raise RuntimeError("Preview worker start rate budget reached")
                    worker_starts.append(now)
                    # Continuous hide/show reuses only this session's bounded scratch/logs.
                    remote_run = run_id if args.continuous else run_id + "_%02d" % generations
                    remote_command = ("exec nice -n 10 python3 -u " + REMOTE +
                                      "/tools/bpu_preview_worker.py " + remote_run + " --engine " + args.engine)
                    if args.display_with_navigation:
                        remote_command += " --display-with-navigation"
                    if args.continuous:
                        remote_command += " --continuous"
                    worker = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                                               "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                                               "root@192.168.10.100", remote_command],
                                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                              bufsize=0, start_new_session=True)
                    drain_stderr(worker, remote_log)
                    ready = receive_json(worker.stdout)
                    if (ready.get("ready") is not True or ready.get("demo_only") is not True or
                            ready.get("motion_eligible") is not False or ready.get("engine") != args.engine or
                            (args.continuous and ready.get("continuous") is not True)):
                        raise ValueError("Unexpected worker handshake")
                    print("BPU_READY " + json.dumps(ready), flush=True)
                    # Use the newest frame after SSH/model handshake, not the earlier reference.
                    continue
                prepared = time.monotonic()
                frame = image_bgr(message)
                payload, scales = preprocessor.prepare(frame)
                prepare_ms = (time.monotonic() - prepared) * 1000
                sequence += 1
                last_stamp = message.header.stamp.to_nsec()
                encode_started = time.monotonic()
                packet = frame_packet(sequence, last_stamp, message.width, message.height, payload)
                encode_ms = (time.monotonic() - encode_started) * 1000
                sent = time.monotonic()
                next_frame = prepared + period
                write_all(worker.stdin, packet)
                response = receive_json(worker.stdout)
                if not fresh(last_stamp / 1e9, rospy.Time.now().to_sec()):
                    raise TimeoutError("BPU result stale; preview stopped")
                annotated = annotate(frame, scales, response, sequence, last_stamp, classes)
                response["roundtrip_ms"] = (time.monotonic() - sent) * 1000
                response["source_age_ms"] = (rospy.Time.now().to_sec() - last_stamp / 1e9) * 1000
                response["prepare_ms"] = prepare_ms
                response["wire_bytes"] = len(packet)
                response["transport_encode_ms"] = encode_ms
                response["completed_monotonic"] = time.monotonic()
                response["generation"] = generations
                completed.append(response["completed_monotonic"])
                effective_fps = ((len(completed) - 1) / (completed[-1] - completed[0])
                                 if len(completed) > 1 else 0.0)
                status.update(effective_fps=effective_fps, last_source_age_ms=response["source_age_ms"],
                              last_roundtrip_ms=response["roundtrip_ms"], last_stamp_ns=last_stamp)
                log.write(json.dumps(response, allow_nan=False) + "\n")
                log.flush()
                frames += 1
                if image_pub.get_num_connections() and not stop.is_set():
                    output = Image()
                    output.header.stamp = message.header.stamp
                    output.header.seq = sequence
                    output.header.frame_id = FRAME_ID
                    output.width, output.height = message.width, message.height
                    output.encoding, output.step = "bgr8", message.width * 3
                    output.data = annotated.tobytes()
                    image_pub.publish(output)
                    if time.monotonic() >= next_status:
                        status_pub.publish(String(data=("FCOS %s · %.1f Hz / 上限 %.0f · 往返 %.0f ms · 源帧 %.0f ms · 仅显示" %
                            ("常驻" if args.engine == "resident" else "CLI", effective_fps, args.max_fps,
                             response["roundtrip_ms"], response["source_age_ms"]))))
                        next_status = time.monotonic() + 1.0
                # Evidence rate is wall-clock bounded, independent of inference FPS.
                if time.monotonic() >= next_snapshot:
                    cv2.imwrite(str(directory / "latest_raw.png"), frame)
                    cv2.imwrite(str(directory / "latest_detection.png"), annotated)
                    next_snapshot = time.monotonic() + 5.0
                    print("FRAME %d fps=%.2f roundtrip_ms=%.1f age_ms=%.1f" %
                          (frames, effective_fps, response["roundtrip_ms"], response["source_age_ms"]), flush=True)
    except Exception as error:
        reason = repr(error)
        status_pub.publish(String(data=("BPU 预览已停止：" + reason)[:200]))
        print("QT_BRIDGE_ABORT " + reason, flush=True)
        raise
    finally:
        if stop.is_set() and reason in ("time_limit", "ros_shutdown"):
            reason = "stop_signal"
        if subscriber is not None:
            subscriber.unregister()
        status.update(active=False, frames=frames, generations=generations, reason=reason,
                      elapsed_seconds=time.monotonic() - started)
        status_pub.publish(String(data=("BPU 预览已停止：" + reason)[:200]))
        (directory / "summary.json").write_text(json.dumps(status, indent=2) + "\n")
        (LAB / "qt_bridge_current.json").write_text(json.dumps(status, indent=2) + "\n")
        rospy.signal_shutdown(reason)
        print("QT_BRIDGE_STOP " + json.dumps(status), flush=True)


if __name__ == "__main__":
    main()
