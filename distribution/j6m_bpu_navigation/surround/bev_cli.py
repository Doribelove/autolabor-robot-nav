"""Standalone BEV commands: offline images, producer files, or explicit J6M capture."""
import argparse
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import signal
import shutil
import sys
import threading
import time
import uuid

import cv2

from . import SCHEMA
from .bev import BevComposer, preview
from .contracts import Calibration, Frame, atomic_json, read_json

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
LOCAL = ROOT / "runtime/surround_bev_config"


def local_or_template(name, template):
    local = LOCAL / name
    return str(local if local.is_file() else HERE / "config" / template)


def read_frames(filename):
    filename = Path(filename).resolve()
    data = read_json(filename)
    if data.get("schema") != "j6m.surround.bev.frames.v1":
        raise ValueError("Unknown BEV input manifest")
    if type(data.get("now_ns")) is not int or data["now_ns"] <= 0:
        raise ValueError("Offline manifest requires a source-domain now_ns")
    names = {f.name for f in fields(Frame)} - {"image"}
    frames = {}
    for name, entry in data["frames"].items():
        path = (filename.parent / entry["image"]).resolve()
        if filename.parent not in path.parents:
            raise ValueError("Input image escapes its manifest directory")
        frame = Frame(image=cv2.imread(str(path)), **{k: entry[k] for k in names if k in entry})
        if name != frame.camera:
            raise ValueError("Manifest camera identity mismatch")
        frames[name] = frame
    return frames, data["now_ns"]


def write_image(path, image):
    path = Path(path)
    temp = path.with_name(path.stem + ".tmp" + path.suffix)
    if not cv2.imwrite(str(temp), image):
        raise RuntimeError("Could not encode " + str(path))
    temp.replace(path)


class Output:
    def __init__(self, directory, calibration, offline):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=False)
        self.calibration, self.offline = calibration, offline
        self.session = uuid.uuid4().hex
        self.counter = 0

    def state(self, valid, error="", stopped=False):
        state = dict(schema=SCHEMA, service_kind="bev_only", session=self.session,
                     produced_ns=time.time_ns(), synthetic=self.calibration.synthetic,
                     offline=self.offline, replay=self.offline, bev_valid=valid,
                     error=error, service_stopped=stopped, frames_produced=self.counter,
                     calibration_issues=self.calibration.issues(), localization_valid=False,
                     targets=[], observations=[], production_ready=False, motion_eligible=False)
        atomic_json(self.path / "state.json", state)
        return state

    def save(self, bird, coverage, meta, frames):
        produced = time.time_ns()
        meta.update(produced_ns=produced, offline=self.offline, session=self.session)
        if not self.offline and not 0 <= produced - meta["source_oldest_ns"] <= (
                meta["max_source_age_ms"] * 1e6):
            raise ValueError("Sources expired during BEV calculation")
        write_image(self.path / "bev.png", bird)
        write_image(self.path / "coverage.png", coverage)
        write_image(self.path / "preview.jpg", preview(bird, frames, meta))
        meta["bev_png_sha256"] = hashlib.sha256((self.path / "bev.png").read_bytes()).hexdigest()
        atomic_json(self.path / "bev.json", meta)
        atomic_json(self.path / "preview_meta.json", dict(
            produced_ns=produced, synthetic=self.calibration.synthetic,
            offline=self.offline, session=self.session))
        self.counter += 1
        self.state(True)
        with (self.path / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps({k: meta[k] for k in (
                "source_oldest_ns", "source_newest_ns", "exposure_skew_ms",
                "processing_ms", "coverage_fraction", "synthetic", "offline")}) + "\n")


class Inbox:
    """At most four immutable frame references; never queues camera work."""
    def __init__(self):
        self.frames = {}
        self.lock = threading.Lock()

    def put(self, frame, now_ns):
        frame.validate(now_ns)
        if frame.synthetic:
            raise ValueError("Synthetic frame cannot enter a live BEV session")
        with self.lock:
            old = self.frames.get(frame.camera)
            if old and (old.session != frame.session or frame.sequence <= old.sequence
                        or frame.stamp_ns <= old.stamp_ns):
                raise ValueError("Camera session reset or non-increasing source identity")
            self.frames[frame.camera] = frame

    def snapshot(self):
        with self.lock:
            return dict(self.frames)


def run_stream(composer, output, duration, getter, errors=None):
    if composer.calibration.synthetic:
        raise ValueError("Synthetic calibration is offline only")
    out = Output(output, composer.calibration, False)
    stop = threading.Event()
    handlers = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    begin, last_signature = time.monotonic(), None
    failure = ""
    out.state(False, "Waiting for camera frames")
    try:
        while not stop.is_set() and (duration <= 0 or time.monotonic() - begin < duration):
            start = time.monotonic()
            try:
                if errors and errors():
                    raise ValueError(json.dumps(errors()))
                frames = getter()
                now = time.time_ns()
                for f in frames.values():
                    f.validate(now, int(composer.config["max_source_age_ms"] * 1e6))
                signature = tuple(sorted((k, f.key, f.stamp_ns) for k, f in frames.items()))
                if signature != last_signature:
                    bird, coverage, meta = composer.compose(frames, now)
                    out.save(bird, coverage, meta, frames)
                    last_signature = signature
                    failure = ""
            except (ValueError, KeyError, TypeError, OSError) as e:
                failure = str(e)
                out.state(False, failure)
            stop.wait(max(0, 1 / composer.config["output_hz"] - (time.monotonic() - start)))
    finally:
        for s, handler in handlers.items():
            signal.signal(s, handler)
        out.state(False, failure or "BEV service stopped", stopped=True)
    print(json.dumps(dict(output=str(out.path.resolve()), frames_produced=out.counter,
                          service_stopped=True, last_error=failure)))
    return 0 if out.counter else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bev-config", default=local_or_template("bev.json", "bev.json"))
    sub = p.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init-config", help="Copy pending configuration, without filling hardware values")
    init.add_argument("--directory", default=str(LOCAL))
    demo = sub.add_parser("demo", help="Synthetic fisheye images; no devices")
    demo.add_argument("--output", required=True)
    images = sub.add_parser("images", help="One offline four-camera image set")
    images.add_argument("--frames", required=True)
    images.add_argument("--calibration", required=True)
    images.add_argument("--output", required=True)
    for name in ("watch", "camera"):
        live = sub.add_parser(name, help="Explicit live input; no BPU or ROS")
        live.add_argument("--calibration", default=local_or_template("calibration.json", "calibration.template.json"))
        live.add_argument("--output", required=True)
        live.add_argument("--duration", type=float, default=0)
        if name == "watch":
            live.add_argument("--frames", required=True)
        else:
            live.add_argument("--capture-config", default=local_or_template("capture.json", "surround.json"))
    view = sub.add_parser("view")
    source = view.add_mutually_exclusive_group(required=True)
    source.add_argument("--input")
    source.add_argument("--url")
    transport = sub.add_parser("serve-preview")
    transport.add_argument("--input", required=True)
    transport.add_argument("--bind", default="127.0.0.1")
    transport.add_argument("--port", type=int, default=18776)
    a = p.parse_args()
    cv2.setNumThreads(2)
    if a.command == "init-config":
        directory = Path(a.directory).resolve()
        directory.mkdir(parents=True, exist_ok=False)
        for source, dest in (("bev.json", "bev.json"), ("calibration.template.json", "calibration.json"),
                             ("clock.template.json", "clock.template.json")):
            shutil.copy2(HERE / "config" / source, directory / dest)
        capture = read_json(HERE / "config/surround.json")
        capture["calibration"] = str(directory / "calibration.json")
        native = ROOT / capture["capture"]["library"]
        if native.is_file():
            capture["capture"]["library"] = str(native)
        atomic_json(directory / "capture.json", capture)
        print(directory)
        return 0
    if a.command == "view":
        # Optional system Qt is read-only and scoped to this viewer process.
        # Core numpy/OpenCV/Pillow have already loaded from the private venv.
        try:
            import PyQt5  # noqa: F401
        except ImportError:
            sys.path.append("/usr/lib/python3/dist-packages")
        from .display import show
        return show(a.input, a.url)
    if a.command == "serve-preview":
        from .preview_server import serve
        serve(a.input, a.bind, a.port)
        return 0
    if a.command == "demo":
        from .bev_demo import make_scene
        a.frames, a.calibration = make_scene(Path(a.output) / "input")
        a.output = str(Path(a.output) / "result")
    cal = Calibration(read_json(a.calibration))
    composer = BevComposer(cal, read_json(a.bev_config), perspective_only=a.command in ("watch", "camera"))
    if a.command in ("images", "demo"):
        frames, now = read_frames(a.frames)
        bird, coverage, meta = composer.compose(frames, now)
        out = Output(a.output, cal, True)
        out.save(bird, coverage, meta, frames)
        print(json.dumps(dict(output=str(out.path.resolve()), coverage_fraction=meta["coverage_fraction"],
                              processing_ms=meta["processing_ms"], synthetic=cal.synthetic,
                              offline=True, hardware_opened=False)))
        return 0
    if cal.synthetic:
        raise ValueError("Synthetic calibration cannot start live input")
    if a.command == "watch":
        inbox, last_file = Inbox(), None
        def get():
            nonlocal last_file
            info = Path(a.frames).stat()
            identity = (info.st_ino, info.st_mtime_ns, info.st_size)
            if identity != last_file:
                incoming, _ = read_frames(a.frames)
                current = inbox.snapshot()
                for name, f in incoming.items():
                    old = current.get(name)
                    if old is None or (f.key, f.stamp_ns) != (old.key, old.stamp_ns):
                        inbox.put(f, time.time_ns())
                last_file = identity
            return inbox.snapshot()
        return run_stream(composer, a.output, a.duration, get)
    from .capture import NativeCapture, capture_issues
    # Reuse the existing path resolver but never create its detector or pose service.
    from .__main__ import load_config
    config = load_config(a.capture_config, ROOT)["capture"]
    issues = capture_issues(config, calibration=cal)
    if issues:
        raise ValueError("; ".join(issues))
    inbox = Inbox()
    capture = NativeCapture(config, cal, inbox.put)
    try:
        capture.start()
        return run_stream(composer, a.output, a.duration, inbox.snapshot,
                          lambda: dict(capture.errors))
    finally:
        capture.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, OSError) as e:
        print("BEV: " + str(e), file=sys.stderr)
        raise SystemExit(2)
