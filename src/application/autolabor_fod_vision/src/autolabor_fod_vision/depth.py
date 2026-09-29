"""Registered stereo depth and current-observation IDs (no held detections)."""

from collections import deque
from dataclasses import replace
import math
import threading

import numpy as np


class RegisteredDepthFusion:
    def __init__(self, sync_tolerance_sec=0.06):
        self.tolerance = float(sync_tolerance_sec)
        self.frames = deque(maxlen=8)
        self.camera = None
        self.lock = threading.Lock()

    def set_camera(self, stamp, frame, width, height, projection):
        fx, fy, cx, cy = (projection[i] for i in (0, 5, 2, 6))
        if not all(math.isfinite(v) for v in (stamp, fx, fy, cx, cy)) or min(fx, fy) <= 0:
            return
        with self.lock:
            self.camera = (stamp, frame, width, height, fx, fy, cx, cy)

    def add_depth(self, stamp, frame, depth, encoding):
        if encoding not in ("32FC1", "16UC1") or depth.ndim != 2:
            raise ValueError("registered depth must be 32FC1 metres or 16UC1 millimetres")
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError("depth requires a valid source timestamp")
        scale = 0.001 if encoding == "16UC1" else 1.0
        with self.lock:
            self.frames.append((stamp, frame, depth, scale))

    def enrich(self, detections, stamp, frame, width, height):
        # No waiting in the inference worker: use a temporally matching depth
        # already received while inference ran, or explicitly mark it missing.
        output = [replace(d, depth_valid=False, depth_m=0.0, range_m=0.0) for d in detections]
        with self.lock:
            frames, camera = tuple(self.frames), self.camera
        if not frames or camera is None or width <= 1 or height <= 1:
            return output
        cstamp, cframe, cw, ch, fx, fy, cx, cy = camera
        if cframe != frame or (cw, ch) != (width, height) or abs(cstamp - stamp) > 0.6:
            return output
        frames = [f for f in frames if f[1] == frame and abs(f[0] - stamp) <= self.tolerance]
        if not frames:
            return output
        _, _, depth, scale = min(frames, key=lambda f: abs(f[0] - stamp))
        dh, dw = depth.shape
        if min(dw, dh) < 2 or abs(dw / float(dh) - width / float(height)) > 0.01:
            return output
        for i, d in enumerate(output):
            u, v = (d.xmin + d.xmax) * 0.5, (d.ymin + d.ymax) * 0.5
            # Median of the inner half-box avoids noisy depth at object edges.
            # Never expand a tiny FOD box into the surrounding ground.
            half_w, half_h = max(0.5, (d.xmax-d.xmin)*0.25), max(0.5, (d.ymax-d.ymin)*0.25)
            x1 = max(0, min(dw, int(math.floor((u-half_w)*dw/width))))
            x2 = max(0, min(dw, int(math.ceil((u+half_w)*dw/width))))
            y1 = max(0, min(dh, int(math.floor((v-half_h)*dh/height))))
            y2 = max(0, min(dh, int(math.ceil((v+half_h)*dh/height))))
            patch = depth[y1:y2, x1:x2]
            valid = patch[np.isfinite(patch) & (patch > 0)]
            if valid.size < max(3, math.ceil(patch.size * 0.20)):
                continue
            z = float(np.median(valid)) * scale
            distance = z * math.sqrt(1.0 + ((u-cx)/fx)**2 + ((v-cy)/fy)**2)
            if math.isfinite(distance) and distance > 0:
                output[i] = replace(d, depth_valid=True, depth_m=z, range_m=distance)
        return output


class DetectionTracker:
    """Associate all objects before choosing one; publish only actual YOLO hits.

    IDs are never reused within a process. Competing existing tracks take part
    in association, so a visible bystander keeps its own ID when the selected
    object disappears. Missing depth may retain an ID but cannot acquire a new
    recovery target. This state is independent of preview smoothing/holds.
    """

    def __init__(self):
        self.tracks = {}
        self.next_id = 1
        self.last_stamp = None
        self.geometry = None

    def update(self, detections, stamp, geometry):
        if geometry != self.geometry or (self.last_stamp is not None and stamp <= self.last_stamp):
            self.tracks.clear()
        self.geometry, self.last_stamp = geometry, stamp
        self.tracks = {i:t for i,t in self.tracks.items() if 0 <= stamp-t[1] <= 2.0}
        pairs = []
        for track_id, (old, old_stamp, old_range, range_stamp) in self.tracks.items():
            ow, oh = old.xmax-old.xmin, old.ymax-old.ymin
            for index, d in enumerate(detections):
                nw, nh = d.xmax-d.xmin, d.ymax-d.ymin
                ratio = nw*nh / max(1.0, ow*oh)
                if not 0.3 <= ratio <= 3.0:
                    continue
                shift = math.hypot((old.xmin+old.xmax-d.xmin-d.xmax)*0.5,
                                   (old.ymin+old.ymax-d.ymin-d.ymax)*0.5)
                pixel_limit = max(30.0, 0.65*max(math.hypot(ow,oh), math.hypot(nw,nh)))
                if shift > pixel_limit:
                    continue
                depth_cost = 0.0
                if d.depth_valid and old_range is not None:
                    # The object approaches with the chassis; tolerate its
                    # measured motion over a dropout, not arbitrary depth jumps.
                    limit = 0.20 + 0.8 * max(0.0, stamp-range_stamp)
                    delta = abs(d.range_m-old_range)
                    if delta > limit:
                        continue
                    depth_cost = delta/limit
                cost = shift/pixel_limit + depth_cost + 0.1*(stamp-old_stamp)
                pairs.append((cost, track_id, index))
        used, assigned = set(), {}
        for _, track_id, index in sorted(pairs):
            if track_id not in used and index not in assigned:
                used.add(track_id)
                assigned[index] = track_id
        output = []
        for index, d in enumerate(detections):
            track_id = assigned.get(index)
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
            d = replace(d, track_id=track_id)
            old = self.tracks.get(track_id)
            distance = d.range_m if d.depth_valid else (old[2] if old else None)
            range_stamp = stamp if d.depth_valid else (old[3] if old else stamp)
            self.tracks[track_id] = (d, stamp, distance, range_stamp)
            output.append(d)
        return output
