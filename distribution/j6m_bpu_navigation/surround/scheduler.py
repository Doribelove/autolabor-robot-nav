"""One latest frame per camera; exactly one consumer owns the BPU."""
from collections import Counter
import threading

from . import CAMERAS


class Scheduler:
    def __init__(self, primary_hz=15., side_hz=3., rear_turn_hz=2., stale_ms=500):
        if not 0 < rear_turn_hz <= side_hz <= primary_hz <= 60 or not 0 < stale_ms <= 500:
            raise ValueError("Invalid scheduler budgets")
        self.period = dict(primary=int(1e9 / primary_hz), side=int(1e9 / side_hz),
                           rear=int(1e9 / rear_turn_hz))
        self.stale_ns = int(stale_ms * 1e6)
        self.slots, self.last_run, self.last_frame = {}, {}, {}
        self.stats = Counter()
        self.lock = threading.RLock()
        self.motion = ("unknown", False, None, 0)

    def set_motion(self, direction, turning, target_camera, valid_until_ns):
        if direction not in ("forward", "reverse", "stopped", "unknown"):
            raise ValueError("Unknown travel direction")
        if target_camera is not None and target_camera not in CAMERAS:
            raise ValueError("Invalid target camera")
        with self.lock:
            self.motion = (direction, bool(turning), target_camera, valid_until_ns)

    def put(self, frame, now_ns):
        frame.validate(now_ns, self.stale_ns)
        with self.lock:
            previous = self.last_frame.get(frame.camera)
            if previous and (previous[0] != frame.session or frame.sequence <= previous[1]
                             or frame.stamp_ns <= previous[2]):
                # A new capture process needs a fresh scheduler/tracker instance.
                self.stats["identity_or_time_rejected"] += 1
                raise ValueError("Camera restart/reordered frame: start a new surround session")
            self.last_frame[frame.camera] = (frame.session, frame.sequence, frame.stamp_ns)
            self.stats["received_" + frame.camera] += 1
            if frame.camera in self.slots:
                self.stats["overwritten_" + frame.camera] += 1
            self.slots[frame.camera] = frame

    def take(self, now_ns):
        with self.lock:
            direction, turning, target, until = self.motion
            if now_ns > until:
                direction, turning, target = "unknown", False, None
            primary = "rear" if direction == "reverse" else "front"
            eligible = []
            for camera, frame in list(self.slots.items()):
                if not 0 <= now_ns - frame.stamp_ns <= self.stale_ns:
                    del self.slots[camera]
                    self.stats["stale_" + camera] += 1
                    continue
                if camera == primary or camera == target:
                    priority, period = 0, self.period["primary"]
                elif camera in ("left", "right", "front"):
                    priority, period = 1, self.period["side"]
                elif turning or direction in ("unknown", "stopped"):
                    priority, period = 2, self.period["rear"]
                else:
                    del self.slots[camera]
                    self.stats["direction_skip_" + camera] += 1
                    continue
                due = self.last_run.get(camera, -10**18) + period
                if now_ns >= due:
                    eligible.append((priority, due, camera))
            if not eligible:
                return None
            _, _, camera = min(eligible)
            self.last_run[camera] = now_ns
            self.stats["scheduled_" + camera] += 1
            return self.slots.pop(camera)

    def snapshot(self):
        with self.lock:
            return dict(self.stats, pending=len(self.slots))
