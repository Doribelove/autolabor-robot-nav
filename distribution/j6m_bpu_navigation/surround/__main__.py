"""Offline tools and an explicitly invoked observation-only hardware service."""
import argparse
from dataclasses import fields
import importlib.util
import json
from pathlib import Path
import signal
import threading
import time

import cv2

from . import SCHEMA
from .capture import capture_issues, NativeCapture
from .contracts import Calibration, Frame, atomic_json, read_json, sha256
from .display import PreviewWriter, Renderer
from .geometry import Pose, PoseBuffer
from .inference import BpuEngine, FixtureEngine, ROOT
from .pipeline import Pipeline
from .scheduler import Scheduler

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config/surround.json"


def load_config(path, root):
    c = read_json(path)
    if c.get("schema") != "j6m.surround.config.v1":
        raise ValueError("Unsupported configuration schema")
    def resolve(obj, key):
        if obj.get(key) and not Path(obj[key]).is_absolute():
            obj[key] = str(root / obj[key])
    for key in ("calibration", "pose_input", "motion_input"):
        resolve(c, key)
    for key in ("library", "vio_library", "hbmem_library", "vpm_config", "camera_config", "exclusive_lock"):
        resolve(c["capture"], key)
    for camera in c["capture"]["cameras"].values():
        for key in ("gdc_bin", "clock_mapping_file"):
            resolve(camera, key)
    for key in ("library", "exclusive_lock"):
        resolve(c["inference"], key)
    for kind in ("detector_zed", "classifier"):
        resolve(c["inference"][kind], "path")
    infer = c["inference"]
    if (type(infer["max_crops"]) is not int or not 0 <= infer["max_crops"] <= 8
            or not 0 < infer["classify_budget_ms"] <= 50
            or not 0 <= infer["confidence"] <= 1):
        raise ValueError("Invalid detector/classification budget")
    if not 0 <= c["pipeline"]["fusion_window_ms"] <= 100:
        raise ValueError("Fusion window must not exceed 100 ms")
    Scheduler(**c["scheduler"])
    return c


def doctor(config):
    cal = Calibration(read_json(config["calibration"]))
    models = {}
    for kind in ("detector_zed", "classifier"):
        item = config["inference"][kind]
        p = Path(item["path"])
        models[kind] = dict(path=str(p), exists=p.is_file(),
                            sha256_ok=p.is_file() and sha256(p) == item["sha256"])
    issues = capture_issues(config["capture"], calibration=cal) + cal.issues()
    for name in ("numpy", "cv2", "PIL"):
        if importlib.util.find_spec(name) is None:
            issues.append("Python dependency missing: " + name)
    inference = config["inference"]
    if not inference.get("library") or not inference.get("library_sha256"):
        issues.append("BPU executor/runtime ABI must be verified against camera image")
    if not config.get("pose_input"):
        issues.append("Source-time localization pose feed not configured")
    if not config["pipeline"]["overlap_pairs"]:
        issues.append("Camera overlap graph not measured; cross-camera fusion disabled")
    if not all(m["sha256_ok"] for m in models.values()):
        issues.append("Pinned model file missing or SHA256 mismatch")
    result = dict(schema=SCHEMA, models=models, pending=issues,
                  live_rectification="gdc_required", cpu_rectification_fallback=False,
                  offline_framework_available=True, hardware_ready=not issues,
                  production_ready=False, motion_eligible=False)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 2 if issues else 0


def replay(events_file, calibration_file, config, output):
    events_file = Path(events_file)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    calibration = Calibration(read_json(calibration_file))
    engine, poses = FixtureEngine(), PoseBuffer()
    pipeline = Pipeline(calibration, engine, poses, config["pipeline"], session="offline-replay",
                        offline_reference=True)
    names = {f.name for f in fields(Frame)} - {"image"}
    now = None
    seen = set()
    processed = rejected = 0
    with (output / "results.jsonl").open("w") as log, events_file.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event["type"] == "pose":
                poses.put(Pose.from_dict(event))
                continue
            if event["type"] == "task_status":
                pipeline.store.set_status(event["target_id"], event["status"])
                continue
            if event["type"] != "frame":
                raise ValueError("Unknown replay event")
            source = {k: event[k] for k in names if k in event}
            path = (events_file.parent / event["image"]).resolve()
            if events_file.parent.resolve() not in path.parents:
                raise ValueError("Replay image must remain inside dataset directory")
            frame = Frame(image=cv2.imread(str(path)), **source)
            if frame.key in seen:
                raise ValueError("Repeated replay frame")
            seen.add(frame.key)
            now = event["now_ns"]
            engine.objects = event["detections"]
            try:
                result = pipeline.process(frame, now)
                processed += 1
            except ValueError as e:
                result = dict(source=source, error=str(e))
                rejected += 1
            log.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
    if now is None:
        raise ValueError("No frames in replay")
    while pipeline.pending:
        pipeline.tick(now, flush=True)
    snapshot = pipeline.tick(now)
    snapshot.update(replay=True, inference_backend="fixture_not_inference",
                    processed_frames=processed, rejected_frames=rejected)
    atomic_json(output / "state.json", snapshot)
    image = Renderer(calibration, offline_reference=True).render(snapshot, pipeline.latest_images, now)
    cv2.imwrite(str(output / "preview.jpg"), image)
    atomic_json(output / "preview_meta.json",
                dict(produced_ns=now, synthetic=calibration.synthetic, session=pipeline.session))
    print(json.dumps(dict(output=str(output.resolve()), processed=processed, rejected=rejected,
                          targets=len(snapshot["targets"]), synthetic=calibration.synthetic,
                          production_ready=False, motion_eligible=False), ensure_ascii=False))
    return 0


def live(config, root, output, duration):
    calibration = Calibration(read_json(config["calibration"]))
    issues = capture_issues(config["capture"], calibration=calibration)
    if issues or calibration.synthetic:
        raise ValueError("; ".join(issues or ["Synthetic calibration is offline only"]))
    if not config["inference"].get("library"):
        raise ValueError("Configure and verify the BPU library before live inference")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    scheduler, poses = Scheduler(**config["scheduler"]), PoseBuffer()
    engine = BpuEngine(config["inference"], root)
    pipeline = Pipeline(calibration, engine, poses, config["pipeline"])
    capture = preview = None
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    previous_handlers = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    begin, last_output, last_pose = time.monotonic(), 0, None
    feed_errors = {}
    preview_frames, preview_lock = {}, threading.Lock()
    def received(frame, now_ns):
        scheduler.put(frame, now_ns)
        # Display still receives all four cameras when rear inference is disabled.
        with preview_lock:
            preview_frames[frame.camera] = (frame.stamp_ns, frame.image, frame.rectified)
    try:
        capture = NativeCapture(config["capture"], calibration, received)
        capture.start()
        preview = PreviewWriter(output, calibration)
        with (output / "results.jsonl").open("w", buffering=1) as log:
            while not stopping and (duration <= 0 or time.monotonic() - begin < duration):
                now = time.time_ns()
                if config.get("pose_input"):
                    try:
                        d = read_json(config["pose_input"])
                        identity = (d["map_id"], d["localization_session"], d["stamp_ns"])
                        if identity != last_pose:
                            poses.put(Pose.from_dict(d))
                            last_pose = identity
                        feed_errors.pop("pose", None)
                    except (OSError, ValueError, KeyError) as e:
                        feed_errors["pose"] = str(e)
                if config.get("motion_input"):
                    try:
                        d = read_json(config["motion_input"])
                        if not 0 <= d["valid_until_ns"] - now <= 500_000_000:
                            raise ValueError("Motion context stale or validity too long")
                        scheduler.set_motion(d["direction"], d["turning"], d.get("target_camera"),
                                             d["valid_until_ns"])
                        feed_errors.pop("motion", None)
                    except (OSError, ValueError, KeyError) as e:
                        feed_errors["motion"] = str(e)
                frame = scheduler.take(now)
                if frame:
                    try:
                        result = pipeline.process(frame)
                    except ValueError as e:
                        result = dict(source=frame.identity(), error=str(e), produced_ns=time.time_ns())
                    log.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
                else:
                    time.sleep(.005)
                if time.monotonic() - last_output >= .2:
                    stamp = time.time_ns()
                    state = pipeline.tick(stamp)
                    state.update(scheduler=scheduler.snapshot(), capture_errors=dict(capture.errors),
                                 live_rectification="gdc_required", cpu_rectification_fallback=False,
                                 feed_errors=feed_errors.copy(), preview_error=preview.error)
                    atomic_json(output / "state.json", state)
                    with preview_lock:
                        display_images = dict(preview_frames)
                    # Draw boxes only on their exact inferred source image.
                    # Unscheduled cameras retain their fresh capture-only view.
                    for camera, value in pipeline.latest_images.items():
                        if 0 <= stamp - value[0] <= 500_000_000:
                            display_images[camera] = value
                    preview.offer(state, display_images, stamp)
                    last_output = time.monotonic()
    finally:
        if preview:
            preview.close()
        try:
            if capture:
                capture.close()
        finally:
            engine.close()
            for s, handler in previous_handlers.items():
                signal.signal(s, handler)
            state = pipeline.tick(time.time_ns())
            state.update(service_stopped=True, production_ready=False, motion_eligible=False)
            atomic_json(output / "state.json", state)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--root", default=str(ROOT))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor", help="Read-only prerequisites; exit 2 means pending hardware/calibration")
    demo = sub.add_parser("demo", help="Generate synthetic frames and replay without ROS or hardware")
    demo.add_argument("--output", required=True)
    rep = sub.add_parser("replay", help="Replay images with supplied detections; does not benchmark BPU")
    rep.add_argument("--events", required=True)
    rep.add_argument("--calibration", required=True)
    rep.add_argument("--output", required=True)
    run = sub.add_parser("run", help="Explicit hardware camera+BPU observation session")
    run.add_argument("--output", required=True)
    run.add_argument("--duration", type=float, default=0)
    ui = sub.add_parser("view", help="Read a session directory in Qt")
    ui_source = ui.add_mutually_exclusive_group(required=True)
    ui_source.add_argument("--input")
    ui_source.add_argument("--url")
    server = sub.add_parser("serve-preview", help="Read-only preview transport; no device start")
    server.add_argument("--input", required=True)
    server.add_argument("--bind", default="127.0.0.1")
    server.add_argument("--port", type=int, default=18776)
    acquire = sub.add_parser("calibration-frames", help="Explicit raw camera acquisition without BPU/ROS")
    acquire.add_argument("--output", required=True)
    acquire.add_argument("--count", type=int, default=20)
    acquire.add_argument("--interval", type=float, default=3)
    intrinsic = sub.add_parser("calibrate-intrinsics")
    intrinsic.add_argument("--images", required=True)
    intrinsic.add_argument("--columns", type=int, required=True)
    intrinsic.add_argument("--rows", type=int, required=True)
    intrinsic.add_argument("--square-m", type=float, required=True)
    intrinsic.add_argument("--output", required=True)
    extrinsic = sub.add_parser("solve-extrinsics")
    extrinsic.add_argument("--points", required=True)
    extrinsic.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.root).resolve()
    config = load_config(args.config, root)
    if args.command == "doctor":
        return doctor(config)
    if args.command == "demo":
        from .demo import make_demo
        directory = Path(args.output)
        events, cal = make_demo(directory / "input", read_json(config["calibration"]))
        config["pipeline"]["overlap_pairs"] = [["front", "left"], ["front", "right"],
                                                ["rear", "left"], ["rear", "right"]]
        return replay(events, cal, config, directory / "result")
    if args.command == "replay":
        return replay(args.events, args.calibration, config, args.output)
    if args.command == "run":
        return live(config, root, args.output, args.duration)
    if args.command == "view":
        from .display import show
        return show(args.input, args.url)
    if args.command == "serve-preview":
        from .preview_server import serve
        serve(args.input, args.bind, args.port)
        return 0
    if args.command == "calibration-frames":
        from .acquire_calibration import acquire
        acquire(config, args.output, args.count, args.interval)
        return 0
    from . import calibrate
    if Path(args.output).exists():
        raise ValueError("Calibration output exists: choose a new version")
    if args.command == "calibrate-intrinsics":
        calibrate.intrinsics(args.images, args.columns, args.rows, args.square_m, args.output)
    elif args.command == "solve-extrinsics":
        calibrate.extrinsics(args.points, args.output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError, RuntimeError) as e:
        raise SystemExit("Surround refused: " + str(e))
