#!/usr/bin/env python3
"""Camera-only stationary preview: ZED -> private SSH -> BPU -> local window.

No production publications, navigation, control, torch or local NN inference.
The isolated camera launch and SSH worker are children owned by this process.
"""
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
import argparse
from datetime import datetime
import fcntl
import json
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time
from live_protocol import header, receive_json, write_all

LAB = Path(__file__).resolve().parents[1]
REMOTE = "/map/robot_j6m_optimized_20260905/bpu_lab_20260905"
MASTER = "http://127.0.0.1:11571"
TOPIC = "/bpu_preview_zed/zed_node/rgb/image_rect_color"
WINDOW = "ZED LIVE + J6M BPU RESULT | DEMO ONLY | Q / Esc to stop"


def image_bgr(message):
    import numpy as np
    channels = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(message.encoding)
    if (not channels or not 1 <= message.width <= 1920 or not 1 <= message.height <= 1080 or
            message.step < message.width * channels or len(message.data) != message.step * message.height):
        raise ValueError("Unexpected ZED image layout")
    pixels = np.ndarray((message.height, message.width, channels), np.uint8,
                        buffer=message.data, strides=(message.step, channels, 1))
    frame = pixels[:, :, :3].copy()
    return frame[:, :, ::-1].copy() if message.encoding.startswith("rgb") else frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=int, default=300)
    parser.add_argument("--stop", action="store_true")
    args = parser.parse_args()
    if os.environ.get("ROBOT_OPTIMIZED_SANDBOX") != "1" or LAB != Path(
            "/home/slam/robot_j6m_ws_optimized_20260905/experiments/j6m_bpu_20260905"):
        raise RuntimeError("Use the isolated candidate preview launcher")
    if args.stop:
        status = json.loads((LAB / "live_preview_current.json").read_text())
        directory = Path(status["directory"])
        if directory.resolve().parent != LAB / "live":
            raise RuntimeError("Unexpected preview directory")
        (directory / "STOP").touch(exist_ok=True)
        print("Stop requested for", directory)
        return
    if not 10 <= args.seconds <= 600 or os.environ.get("ROS_MASTER_URI") != MASTER:
        raise ValueError("Preview requires private master and a 10..600 second lifetime")
    for port in (11311, 11571):
        with socket.socket() as probe:
            probe.settimeout(0.2)
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError("ROS port already occupied: " + str(port))
    for unit in ("autolabor-dual-host.service", "autolabor-optimized-20260905.service"):
        state = subprocess.check_output(["systemctl", "--user", "show", unit,
                                         "-p", "ActiveState", "--value"], text=True).strip()
        if state not in ("inactive", "failed"):
            raise RuntimeError("Navigation supervisor is not stopped: " + unit)
    lock = (LAB / "preview_client.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    import cv2
    import numpy as np
    import rospy
    import yaml
    from sensor_msgs.msg import Image
    cv2.setNumThreads(1)
    run_id = "preview_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    directory = LAB / "live" / run_id
    directory.mkdir(parents=True, exist_ok=False)
    status = {"pid": os.getpid(), "run_id": run_id, "directory": str(directory),
              "master": MASTER, "demo_only": True, "motion_eligible": False, "active": True}
    (LAB / "live_preview_current.json").write_text(json.dumps(status, indent=2) + "\n")
    with Path("/home/slam/yolo11/yolo11_GAM/ultralytics/cfg/datasets/coco.yaml").open() as stream:
        classes = yaml.safe_load(stream)["names"]
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    latest, latest_lock = [None], threading.Lock()
    outgoing, incoming = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
    camera_log = (directory / "camera.log").open("x")
    remote_log = (directory / "remote.log").open("x")
    result_log = (directory / "results.jsonl").open("x")
    camera = remote = None
    started = time.monotonic()
    processed = 0
    last_snapshot_frame = -1
    reason = "time_limit"

    def callback(message):
        with latest_lock:
            latest[0] = message

    def exchange():
        try:
            ready = receive_json(remote.stdout)
            if ready.get("ready") is not True or ready.get("demo_only") is not True:
                raise ValueError("Worker handshake mismatch")
            print("BPU_READY " + json.dumps(ready), flush=True)
            while not stop.is_set():
                try:
                    item = outgoing.get(timeout=0.2)
                except queue.Empty:
                    continue
                seq, stamp_ns, frame, payload, scales, received = item
                write_all(remote.stdin, header(seq, stamp_ns, frame.shape[1], frame.shape[0]) + payload)
                response = receive_json(remote.stdout)
                if (response.get("sequence") != seq or response.get("stamp_ns") != stamp_ns or
                        response.get("demo_only") is not True or response.get("motion_eligible") is not False or
                        response.get("width") != frame.shape[1] or response.get("height") != frame.shape[0]):
                    raise ValueError("BPU response frame identity mismatch")
                incoming.put((frame, scales, response, received), timeout=1)
        except Exception as error:
            if stop.is_set():
                return  # Expected EOF while this supervisor stops its worker.
            try:
                incoming.put(error, timeout=0.5)
            except queue.Full:
                pass
            print("BPU_WORKER_ERROR " + repr(error), flush=True)

    try:
        camera = subprocess.Popen(["roslaunch", "-p", "11571", str(LAB / "live_camera.launch")],
                                  stdout=camera_log, stderr=subprocess.STDOUT, start_new_session=True)
        master_deadline = time.monotonic() + 15
        while True:
            if camera.poll() is not None:
                raise RuntimeError("Camera launch failed before master startup")
            with socket.socket() as probe:
                probe.settimeout(0.2)
                if probe.connect_ex(("127.0.0.1", 11571)) == 0:
                    break
            if time.monotonic() >= master_deadline or stop.wait(0.1):
                raise TimeoutError("Isolated ROS master did not start")
        rospy.init_node("bpu_preview_display", anonymous=False, disable_signals=True)
        subscriber = rospy.Subscriber(TOPIC, Image, callback, queue_size=1, buff_size=3 * 1024 * 1024)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 1200, 480)
        cv2.moveWindow(WINDOW, 100, 80)
        background = np.zeros((360, 640, 3), np.uint8)
        live_frame = background.copy()
        live_stamp = 0
        displayed_stamp, next_frame, busy, sequence, last_source_stamp = 0, 0, False, 0, 0
        print("PREVIEW_START " + json.dumps(status), flush=True)
        while not stop.is_set() and time.monotonic() - started < args.seconds:
            now = time.monotonic()
            if (directory / "STOP").exists():
                reason = "stop_requested"
                break
            if camera.poll() is not None:
                raise RuntimeError("Camera launch exited; see camera.log")
            with latest_lock:
                message = latest[0]
            if message is None:
                if now - started > 55:
                    raise TimeoutError("No fresh ZED image within startup limit")
            else:
                age = time.time() - message.header.stamp.to_sec()
                if not -0.2 <= age <= 3:
                    raise TimeoutError("ZED image timestamp is stale or in the future")
                # Refresh the camera panel independently of BPU completion.
                # The detection panel always keeps its own exact source frame.
                if message.header.stamp.to_nsec() != live_stamp:
                    live_frame = image_bgr(message)
                    live_stamp = message.header.stamp.to_nsec()
                if remote is None:
                    remote = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                                               "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                                               "root@192.168.10.100", "exec nice -n 10 python3 -u " +
                                               REMOTE + "/tools/bpu_preview_worker.py " + run_id],
                                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=remote_log,
                                              bufsize=0, start_new_session=True)
                    threading.Thread(target=exchange, daemon=True).start()
                if not busy and now >= next_frame and message.header.stamp.to_nsec() > last_source_stamp:
                    frame = live_frame.copy()
                    scale = min(896 / message.width, 896 / message.height)
                    width, height = round(message.width * scale), round(message.height * scale)
                    prepared = np.zeros((896, 896, 3), np.uint8)
                    prepared[:height, :width] = cv2.resize(frame, (width, height))
                    i420 = cv2.cvtColor(prepared, cv2.COLOR_BGR2YUV_I420).reshape(-1)
                    count = 896 * 896
                    uv = np.stack((i420[count:count * 5 // 4], i420[count * 5 // 4:]), axis=-1).reshape(-1)
                    sequence += 1
                    last_source_stamp = message.header.stamp.to_nsec()
                    outgoing.put_nowait((sequence, last_source_stamp, frame, i420[:count].tobytes() + uv.tobytes(),
                                         (width / message.width, height / message.height), now))
                    busy, next_frame = True, now + 1.0
            try:
                item = incoming.get_nowait()
            except queue.Empty:
                item = None
            if isinstance(item, Exception):
                raise item
            if item is not None:
                frame, scales, response, received = item
                busy = False
                displayed_stamp = response["stamp_ns"] / 1e9
                if time.time() - displayed_stamp > 3:
                    raise TimeoutError("BPU preview result is stale; not displayed")
                background = frame.copy()
                detections = response["detections"]
                if not isinstance(detections, list) or len(detections) > 100:
                    raise ValueError("Invalid result count")
                for index, detection in enumerate(detections):
                    box = np.array(detection["bbox_model_px"], dtype=float)
                    confidence = detection["confidence"]
                    label_id = detection["class_id"]
                    if box.shape != (4,) or not np.isfinite(box).all() or not 0 <= confidence <= 1 or label_id not in classes:
                        raise ValueError("Malformed detection")
                    box[[0, 2]] = np.clip(box[[0, 2]] / scales[0], 0, frame.shape[1] - 1)
                    box[[1, 3]] = np.clip(box[[1, 3]] / scales[1], 0, frame.shape[0] - 1)
                    x1, y1, x2, y2 = np.rint(box).astype(int)
                    if x2 <= x1 or y2 <= y1:
                        continue
                    detection["bbox_source_px"] = box.tolist()
                    detection["class_name"] = classes[label_id]
                    cv2.rectangle(background, (x1, y1), (x2, y2), (0, 220, 0), 2)
                    cv2.putText(background, "%s %.2f" % (classes[label_id], confidence),
                                (x1, min(frame.shape[0] - 5, max(18 + index % 3 * 18, y1 - 5))),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                response["local_roundtrip_ms"] = (time.monotonic() - received) * 1000
                response["display_age_ms"] = (time.time() - displayed_stamp) * 1000
                result_log.write(json.dumps(response, allow_nan=False) + "\n")
                result_log.flush()
                processed += 1
                print("FRAME %d boxes=%d roundtrip_ms=%.1f age_ms=%.1f" %
                      (processed, len(detections), response["local_roundtrip_ms"], response["display_age_ms"]), flush=True)
                cv2.imwrite(str(directory / "latest_raw.png"), frame)
                cv2.imwrite(str(directory / "latest_detection.png"), background)
            age = time.time() - displayed_stamp if displayed_stamp else None
            left = cv2.copyMakeBorder(live_frame, 70, 0, 0, 0, cv2.BORDER_CONSTANT)
            right = cv2.copyMakeBorder(background, 70, 0, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(left, "LIVE CAMERA | RGB only | NO MOTION", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.53, (0, 220, 255), 1)
            live_age = time.time() - live_stamp / 1e9 if live_stamp else None
            camera_label = "Waiting for ZED..." if live_age is None else "Camera age %.2fs | Q/Esc: stop | auto stop %ds" % (
                live_age, max(0, args.seconds - (now - started)))
            cv2.putText(left, camera_label, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            cv2.putText(right, "J6M BPU RESULT | matching source frame | DEMO", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1)
            result_label = "Waiting for BPU..." if age is None else "Result frame %d | age %.2fs | NOT the live left frame" % (processed, age)
            color = (0, 160, 255) if age is not None and age > 2 else (255, 255, 255)
            cv2.putText(right, result_label, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            if left.shape != right.shape:
                right = cv2.resize(right, (left.shape[1], left.shape[0]))
            canvas = np.concatenate((left, right), axis=1)
            cv2.imshow(WINDOW, canvas)
            if processed and processed != last_snapshot_frame and (
                    not (directory / "preview_window.png").exists() or processed % 10 == 0):
                cv2.imwrite(str(directory / "preview_window.png"), canvas)
                last_snapshot_frame = processed
            if cv2.waitKey(40) & 0xff in (27, ord("q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                reason = "window_closed"
                break
        subscriber.unregister()
    except Exception as error:
        reason = repr(error)
        print("PREVIEW_ABORT " + reason, flush=True)
        raise
    finally:
        stop.set()
        # Signal only process groups started by this exact preview session.
        if remote is not None and remote.poll() is None:
            remote.terminate()
            try:
                remote.wait(timeout=5)
            except subprocess.TimeoutExpired:
                remote.kill()
                remote.wait(timeout=3)
        if camera is not None and camera.poll() is None:
            os.killpg(camera.pid, signal.SIGINT)
            try:
                camera.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(camera.pid, signal.SIGTERM)
                camera.wait(timeout=8)
        cv2.destroyAllWindows()
        status.update(active=False, frames=processed, reason=reason,
                      elapsed_seconds=time.monotonic() - started)
        (directory / "summary.json").write_text(json.dumps(status, indent=2) + "\n")
        (LAB / "live_preview_current.json").write_text(json.dumps(status, indent=2) + "\n")
        camera_log.close()
        remote_log.close()
        result_log.close()
        print("PREVIEW_STOP " + json.dumps(status), flush=True)


if __name__ == "__main__":
    main()
