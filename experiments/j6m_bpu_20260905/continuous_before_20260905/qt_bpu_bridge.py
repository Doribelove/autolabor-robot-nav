#!/usr/bin/env python3
"""Optional Qt display bridge; reuses ZED, never starts a camera or a control node.

Only /fod/bpu_preview/image and /fod/bpu_preview/status are published. The
reference COCO model is deliberately not a production FOD backend. One source
frame in flight, at most 0.5 Hz, with no inference when no image viewer exists.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time

from live_preview import image_bgr, LAB, REMOTE
from live_protocol import header, receive_json, write_all

IMAGE_TOPIC = "/fod/bpu_preview/image"
STATUS_TOPIC = "/fod/bpu_preview/status"
FRAME_ID = "bpu_preview_demo_only"


def prepare(frame):
    import cv2
    import numpy as np
    height, width = frame.shape[:2]
    if not 64 <= width <= 1920 or not 64 <= height <= 1080:
        raise ValueError("Source dimensions outside preview bounds")
    scale = min(896 / width, 896 / height)
    out_width, out_height = round(width * scale), round(height * scale)
    canvas = np.zeros((896, 896, 3), np.uint8)
    canvas[:out_height, :out_width] = cv2.resize(frame, (out_width, out_height))
    i420 = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    count = 896 * 896
    uv = np.stack((i420[count:count * 5 // 4], i420[count * 5 // 4:]), axis=-1).reshape(-1)
    return i420[:count].tobytes() + uv.tobytes(), (out_width / width, out_height / height)


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=int, default=600)
    parser.add_argument("--display-with-navigation", action="store_true")
    parser.add_argument("--image-topic", default="/fod_camera/image_raw",
                        choices=("/fod_camera/image_raw", "/bpu_preview_zed/zed_node/rgb/image_rect_color"))
    args = parser.parse_args()
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1":
        raise RuntimeError("Use the isolated candidate launcher")
    if not 10 <= args.seconds <= 600:
        raise ValueError("Demo duration must be 10..600 seconds")
    master = os.environ.get("ROS_MASTER_URI", "")
    if master not in ("http://127.0.0.1:11571", "http://192.168.10.100:11311"):
        raise ValueError("Unsupported ROS master")
    if master != "http://127.0.0.1:11571" and not args.display_with_navigation:
        raise ValueError("Existing navigation requires explicit display-only opt-in")
    lock = (LAB / "preview_client.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import cv2
    import rospy
    import yaml
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    cv2.setNumThreads(1)
    rospy.init_node("j6m_bpu_qt_bridge", disable_signals=True)
    image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1, latch=False)
    status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=False)
    classes = yaml.safe_load(Path(
        "/home/slam/yolo11/yolo11_GAM/ultralytics/cfg/datasets/coco.yaml").read_text())["names"]
    run_id = "qt_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory = LAB / "live" / run_id
    directory.mkdir(parents=True, exist_ok=False)
    status = dict(pid=os.getpid(), run_id=run_id, directory=str(directory), master=master,
                  active=True, demo_only=True, motion_eligible=False)
    (LAB / "qt_bridge_current.json").write_text(json.dumps(status, indent=2) + "\n")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    latest, latest_lock = [None], threading.Lock()
    def callback(message):
        with latest_lock:
            latest[0] = message
    worker = subscriber = None
    sequence = frames = generations = 0
    next_frame = last_stamp = 0
    started = time.monotonic()
    reason = "time_limit"
    print("QT_BRIDGE_START " + json.dumps(status), flush=True)
    try:
        with (directory / "results.jsonl").open("x") as log, (directory / "remote.log").open("x") as remote_log:
            while not stop.is_set() and not rospy.is_shutdown() and time.monotonic() - started < args.seconds:
                if (directory / "STOP").exists():
                    reason = "stop_requested"
                    break
                if image_pub.get_num_connections() == 0:
                    if subscriber is not None:
                        subscriber.unregister()
                        subscriber = None
                    stop_worker(worker)
                    worker = None
                    with latest_lock:
                        latest[0] = None
                    stop.wait(0.5)
                    continue
                if subscriber is None:
                    subscriber = rospy.Subscriber(args.image_topic, Image, callback,
                                                  queue_size=1, buff_size=10 * 1024 * 1024)
                with latest_lock:
                    message = latest[0]
                if message is None:
                    status_pub.publish(String(data="等待现有 ZED 原图；本桥接器不启动相机"))
                    stop.wait(0.5)
                    continue
                if not fresh(message.header.stamp.to_sec(), rospy.Time.now().to_sec()):
                    raise TimeoutError("ZED source timestamp stale/future; preview stopped")
                status_pub.publish(String(data="FCOS 通用物体 · 仅预览 · ≤0.5 Hz · 无运动资格"))
                if time.monotonic() < next_frame or message.header.stamp.to_nsec() <= last_stamp:
                    stop.wait(0.2)
                    continue
                if worker is None:
                    generations += 1
                    if generations > 30:
                        raise RuntimeError("Demo worker restart budget reached")
                    remote_command = "exec nice -n 10 python3 -u " + REMOTE + "/tools/bpu_preview_worker.py " + run_id + "_%02d" % generations
                    if args.display_with_navigation:
                        remote_command += " --display-with-navigation"
                    worker = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                                               "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                                               "root@192.168.10.100", remote_command],
                                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=remote_log,
                                              bufsize=0, start_new_session=True)
                    ready = receive_json(worker.stdout)
                    if ready.get("ready") is not True or ready.get("demo_only") is not True:
                        raise ValueError("Unexpected worker handshake")
                    print("BPU_READY " + json.dumps(ready), flush=True)
                    # Use the newest frame after SSH/model handshake, not the earlier reference.
                    continue
                frame = image_bgr(message)
                payload, scales = prepare(frame)
                sequence += 1
                last_stamp = message.header.stamp.to_nsec()
                sent = time.monotonic()
                next_frame = sent + 2.0
                write_all(worker.stdin, header(sequence, last_stamp, message.width, message.height) + payload)
                response = receive_json(worker.stdout)
                if not fresh(last_stamp / 1e9, rospy.Time.now().to_sec()):
                    raise TimeoutError("BPU result stale; preview stopped")
                annotated = annotate(frame, scales, response, sequence, last_stamp, classes)
                response["roundtrip_ms"] = (time.monotonic() - sent) * 1000
                response["source_age_ms"] = (rospy.Time.now().to_sec() - last_stamp / 1e9) * 1000
                response["generation"] = generations
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
                    status_pub.publish(String(data="BPU 帧 %d · 往返 %.0f ms · 仅显示" % (frames, response["roundtrip_ms"])))
                # Bounded evidence: one snapshot per 10 results, not per UI repaint.
                if frames == 1 or frames % 10 == 0:
                    cv2.imwrite(str(directory / "latest_raw.png"), frame)
                    cv2.imwrite(str(directory / "latest_detection.png"), annotated)
                print("FRAME %d roundtrip_ms=%.1f age_ms=%.1f" %
                      (frames, response["roundtrip_ms"], response["source_age_ms"]), flush=True)
    except Exception as error:
        reason = repr(error)
        status_pub.publish(String(data=("BPU 预览已停止：" + reason)[:200]))
        print("QT_BRIDGE_ABORT " + reason, flush=True)
        raise
    finally:
        if stop.is_set() and reason == "time_limit":
            reason = "stop_signal"
        if subscriber is not None:
            subscriber.unregister()
        stop_worker(worker)
        status.update(active=False, frames=frames, generations=generations, reason=reason,
                      elapsed_seconds=time.monotonic() - started)
        (directory / "summary.json").write_text(json.dumps(status, indent=2) + "\n")
        (LAB / "qt_bridge_current.json").write_text(json.dumps(status, indent=2) + "\n")
        rospy.signal_shutdown(reason)
        print("QT_BRIDGE_STOP " + json.dumps(status), flush=True)


if __name__ == "__main__":
    main()
