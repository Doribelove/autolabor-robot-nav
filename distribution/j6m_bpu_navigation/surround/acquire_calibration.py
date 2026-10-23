"""Explicit future hardware acquisition, with no inference and no ROS publisher."""
from pathlib import Path
import threading
import time

import cv2

from . import CAMERAS
from .capture import NativeCapture, capture_issues
from .contracts import Calibration, atomic_json, read_json, sha256


def acquire(config, output, count, interval):
    if not 1 <= count <= 200 or not 1 <= interval <= 60:
        raise ValueError("Use 1..200 sets with 1..60 seconds between sets")
    problems = capture_issues(config["capture"], require_clock=False, require_gdc=False)
    if problems:
        raise ValueError("; ".join(problems))
    calibration = Calibration(read_json(config["calibration"]))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    latest, lock = {}, threading.Lock()
    def receive(frame, _):
        # stamp_ns intentionally remains zero; these frames are never admitted
        # to the live pipeline. Preserve the hardware counter in the sidecar.
        with lock:
            latest[frame.camera] = frame
    capture = NativeCapture(config["capture"], calibration, receive, calibration_only=True)
    saved = []
    try:
        capture.start()
        for index in range(count):
            deadline = time.monotonic() + max(10, interval)
            frames = None
            while time.monotonic() < deadline:
                with lock:
                    now = time.time_ns()
                    if all(name in latest and 0 <= now - latest[name].received_ns <= 500_000_000
                           for name in CAMERAS):
                        frames = dict(latest)
                        break
                time.sleep(.05)
            if frames is None:
                raise RuntimeError("Four fresh cameras unavailable: " + str(capture.errors))
            for name, frame in frames.items():
                filename = "%s_%03d.png" % (name, index)
                if not cv2.imwrite(str(output / filename), frame.image):
                    raise RuntimeError("Could not save " + filename)
                saved.append(dict(file=filename, sha256=sha256(output / filename),
                                  source=frame.identity(), exposure_time_verified=False))
            atomic_json(output / "manifest.json",
                        dict(kind="raw_intrinsic_calibration_images", images=saved,
                             production_ready=False, motion_eligible=False))
            print("Saved calibration set %d/%d" % (index + 1, count), flush=True)
            if index + 1 < count:
                time.sleep(interval)
    finally:
        capture.close()
